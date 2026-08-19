"""Sim bring-up: unitree_mujoco + control.launch.py.

See README.md for the operating procedure, the domain/DDS story and the sim-bridge safety
banner.
"""

import os
import shutil

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown, matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

BRINGUP_SHARE = get_package_share_directory("g1_bringup")
STATE_SHARE = get_package_share_directory("g1_state_estimation")
RELAY_SHARE = get_package_share_directory("g1_sensor_relay")

UNITREE_MUJOCO_BIN = "/opt/unitree_robotics/unitree_mujoco/simulate/build/unitree_mujoco"
# Vendored G1 model directory, derived from the binary location. MuJoCo resolves <include>
# relative to the staged file, so scenes are copied in here rather than read from the share.
G1_MODEL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(UNITREE_MUJOCO_BIN), "..", "..", "unitree_robots", "g1")
)
STAGED_SCENE_NAME = "g1_grove_scene.staged.xml"
STAGED_WALK_BASE = "g1_walk_base.staged.xml"

# So the simulator loads its own CycloneDDS build, not ROS's ABI-incompatible one.
UNITREE_ROBOTICS_LIB = "/opt/unitree_robotics/lib"

# Managed as a launch action: xvfb-run orphans its children, leaving the sim running after
# launch exits.
XVFB_DISPLAY = ":133"

# Delays only the simulator, so the bridge and controller_manager are DDS-ready before the
# first physics tick. Too short also crashes headless GLFW startup; do not set to 0.
SIM_START_DELAY_S = 2.0

# FAST-LIO estimates gravity from its first ten IMU samples and the robot free-falls at spawn:
# an init that catches the drop bakes a wrong gravity in and the estimate diverges.
FASTLIO_EXTRA_DELAY_S = 10.0

WORLDS = ("navigation", "perception", "manipulation", "lio")
ODOMETRY_SOURCES = ("ground_truth", "fast_lio")

EXPECTED_RMW = "rmw_fastrtps_cpp"
EXPECTED_DOMAIN_ID = "1"


# --- environment ----------------------------------------------------------------------------


def _read_text(path):
    try:
        with open(path, encoding="utf-8") as handle:
            return handle.read()
    except OSError:
        return ""


def _cyclonedds_problems(uri):
    """Checks what the config says rather than how it was named: CycloneDDS accepts a file://
    URI, a bare path or inline XML, and a bare path to the hardware profile would otherwise
    walk past this and put rt/lowcmd on the LAN."""
    if not uri:
        return [
            "CYCLONEDDS_URI is unset -- expected the container-baked cyclonedds.xml pinning "
            "the 'lo' interface (see .devcontainer/Dockerfile)."
        ]

    if uri.lstrip().startswith("<"):
        config = uri
    else:
        path = uri[len("file://"):] if uri.startswith("file://") else uri
        if not os.path.isfile(path):
            # An unreadable URI is only a warning to CycloneDDS, which then falls back to
            # defaults and, with network_mode: host, binds the real NIC.
            return [
                f"CYCLONEDDS_URI points at {path!r}, which does not exist -- CycloneDDS would "
                "silently fall back to defaults and bind the host NIC instead of 'lo'."
            ]
        config = _read_text(path)

    if config and 'NetworkInterface name="lo"' not in config:
        return [
            f"the CycloneDDS config in use ({uri!r}) does not pin the 'lo' interface. The "
            "simulator must never publish rt/lowcmd anywhere a real robot can hear it."
        ]
    return []


def _check_environment(context, *args, **kwargs):
    """Fails the launch before anything starts. An empty ROS graph from a silent RMW/domain
    mismatch is much harder to debug than an explicit error."""
    problems = []

    # ROS must not load a second CycloneDDS: the hardware component reaches the wire through
    # unitree_sdk2's own, and the two ship libddsc.so.0 with different ABIs.
    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if rmw != EXPECTED_RMW:
        problems.append(f"RMW_IMPLEMENTATION={rmw!r}, expected {EXPECTED_RMW!r}.")

    problems += _cyclonedds_problems(os.environ.get("CYCLONEDDS_URI"))

    domain_id = os.environ.get("ROS_DOMAIN_ID")
    if domain_id != EXPECTED_DOMAIN_ID:
        problems.append(
            f"ROS_DOMAIN_ID={domain_id!r}, expected {EXPECTED_DOMAIN_ID!r} -- the sim-first "
            "milestone's dedicated domain (see README.md)."
        )

    if problems:
        raise RuntimeError(
            "g1_bringup/sim.launch.py: refusing to start, environment precondition(s) "
            "failed:\n"
            + "\n".join(f"  - {p}" for p in problems)
            + "\nThese are set container-wide by .devcontainer/Dockerfile; a fresh "
            "'manage.sh recreate' is the usual fix."
        )
    return []


# --- scene staging --------------------------------------------------------------------------


def _scene_files(world, sensors, pin_pelvis):
    """The overlay to stage, and the base it <include>s, if any.

    Unpinned always wraps the chosen world in the walk overlay: its pelvis weld is what holds
    the robot up until the control stack has driven every motor, and an unwrapped scene leaves
    it prone by the time the policy activates.
    """
    if not pin_pelvis:
        base = f"g1_{world}_scene.xml" if sensors else "g1_flat_scene.xml"
        return "g1_walk_scene.xml", [(base, STAGED_WALK_BASE)]
    if sensors:
        # The pinned sensor overlay includes the plain scene of the same world.
        return f"g1_{world}_pinned_scene.xml", [
            (f"g1_{world}_scene.xml", f"g1_{world}_scene.staged.xml")
        ]
    return "g1_pinned_scene.xml", []


def _stage_scene(world, sensors, pin_pelvis):
    """Copies the scene and its base into the model directory. Returns every staged path so
    shutdown can remove them."""
    mjcf_dir = os.path.join(BRINGUP_SHARE, "mjcf")
    overlay, bases = _scene_files(world, sensors, pin_pelvis)

    staged = os.path.join(G1_MODEL_DIR, STAGED_SCENE_NAME)
    shutil.copyfile(os.path.join(mjcf_dir, overlay), staged)

    staged_paths = [staged]
    for source, staged_name in bases:
        target = os.path.join(G1_MODEL_DIR, staged_name)
        shutil.copyfile(os.path.join(mjcf_dir, source), target)
        staged_paths.append(target)
    return staged_paths


def _cleanup_on_shutdown(staged_paths):
    def _remove(context, *a, **k):
        for path in staged_paths:
            try:
                os.remove(path)
            except OSError:
                pass
        return []

    return RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_remove)]))


# --- sensors and odometry -------------------------------------------------------------------


def _sensor_nodes(sim_env, want_rviz):
    """The relay owns the ROS side. Start order does not matter: it listens whenever it comes
    up and the simulator retries connecting every cycle."""
    # The patched unitree_mujoco starts its sensor thread only when this names a config, so
    # the stock code path is what runs unless sensors are asked for explicitly.
    sensor_config = os.path.join(BRINGUP_SHARE, "config", "sim_sensors.yaml")
    sim_env["GROVE_G1_SENSOR_CONFIG"] = sensor_config
    with open(sensor_config) as handle:
        socket_path = yaml.safe_load(handle)["socket_path"]

    actions = [
        Node(
            package="g1_sensor_relay",
            executable="g1_sensor_relay",
            name="g1_sensor_relay",
            output="both",
            parameters=[
                os.path.join(RELAY_SHARE, "config", "g1_sensor_relay.yaml"),
                # One source of truth for the socket: the simulator reads it from the file
                # above, so the relay is told what that file said.
                {"socket_path": socket_path},
            ],
        )
    ]

    if want_rviz:
        # Inside this branch because the shipped config's displays are all sensor topics.
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", os.path.join(BRINGUP_SHARE, "config", "g1_sensors.rviz")],
            )
        )
    return actions


def _bring_up(node):
    """Configure on start, activate once inactive."""

    def _transition(transition_id):
        return EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node), transition_id=transition_id
            )
        )

    return [
        RegisterEventHandler(
            OnProcessStart(
                target_action=node, on_start=[_transition(Transition.TRANSITION_CONFIGURE)]
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                goal_state="inactive",
                entities=[_transition(Transition.TRANSITION_ACTIVATE)],
            )
        ),
    ]


def _odometry_actions(odometry, sim_start_delay_s):
    """Exactly one branch runs. Two writers on odom -> base_footprint is the failure the
    `odometry` argument exists to make impossible."""
    if odometry == "fast_lio":
        return [
            TimerAction(
                period=sim_start_delay_s + FASTLIO_EXTRA_DELAY_S,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(STATE_SHARE, "launch", "fastlio_odometry.launch.py")
                        ),
                        launch_arguments={"sim": "true"}.items(),
                    )
                ],
            )
        ]

    node = LifecycleNode(
        package="g1_state_estimation",
        executable="g1_odometry_publisher",
        name="g1_odometry_publisher",
        namespace="",
        output="both",
        parameters=[
            os.path.join(STATE_SHARE, "config", "g1_odometry_publisher_converged.yaml")
        ],
        remappings=[("~/base_state", "/g1_sensor_relay/base_state")],
    )
    return [node] + _bring_up(node)


# --- simulator ------------------------------------------------------------------------------


def _simulator(sim_env, headless, sim_start_delay_s):
    actions = []
    if headless:
        actions.append(
            ExecuteProcess(
                cmd=["Xvfb", XVFB_DISPLAY, "-screen", "0", "1280x1024x24", "-nolisten", "tcp"],
                name="xvfb",
                output="screen",
            )
        )
        sim_env["DISPLAY"] = XVFB_DISPLAY

    sim_process = ExecuteProcess(
        cmd=[UNITREE_MUJOCO_BIN, "-r", "g1", "-s", STAGED_SCENE_NAME],
        name="unitree_mujoco",
        output="screen",
        env=sim_env,
    )
    actions.append(TimerAction(period=sim_start_delay_s, actions=[sim_process]))
    # Tear down the whole launch if the sim dies.
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=sim_process,
                on_exit=[EmitEvent(event=Shutdown(reason="unitree_mujoco exited"))],
            )
        )
    )
    return actions


# --- assembly -------------------------------------------------------------------------------


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _launch_setup(context, *args, **kwargs):
    headless = _flag(context, "headless")
    pin_pelvis = _flag(context, "pin_pelvis")
    sensors = _flag(context, "sensors")
    world = LaunchConfiguration("world").perform(context)
    odometry = LaunchConfiguration("odometry").perform(context)
    sim_start_delay_s = float(LaunchConfiguration("sim_start_delay_s").perform(context))

    if world not in WORLDS:
        raise RuntimeError(
            f"world:={world!r} is not a scene. Use 'navigation' (the multi-room facility), "
            "'perception' (the small room the geometry test measures against), "
            "'manipulation' (one object on a pedestal at arm's length, for the skill tests) "
            "or 'lio' (the walled, asymmetric room for scoring LiDAR-inertial odometry)."
        )
    # Checked even when sensors are off, so a typo is caught where it was made rather than
    # silently selecting the other source.
    if odometry not in ODOMETRY_SOURCES:
        raise RuntimeError(
            f"odometry:={odometry!r} is not an odometry source. Use 'ground_truth' (exact "
            "MuJoCo state, simulation only) or 'fast_lio' (LiDAR-inertial, what the robot runs)."
        )

    sim_env = dict(os.environ)
    sim_env["LD_LIBRARY_PATH"] = UNITREE_ROBOTICS_LIB + ":" + sim_env.get("LD_LIBRARY_PATH", "")

    actions = []
    if sensors:
        actions += _sensor_nodes(sim_env, _flag(context, "rviz"))
        actions += _odometry_actions(odometry, sim_start_delay_s)

    actions.append(_cleanup_on_shutdown(_stage_scene(world, sensors, pin_pelvis)))
    actions += _simulator(sim_env, headless, sim_start_delay_s)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(BRINGUP_SHARE, "launch", "control.launch.py")
            )
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run unitree_mujoco against our own managed Xvfb (no GUI window).",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="navigation",
            description="Which room to stage, when sensors are on. 'navigation' is the "
            "multi-room facility; 'perception' is the bare room test_lidar_geometry measures "
            "against; 'manipulation' is one object at arm's length; 'lio' is the walled room "
            "for scoring odometry.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open config/g1_sensors.rviz. Needs sensors:=true.",
        ),
        DeclareLaunchArgument(
            "sensors",
            default_value="false",
            description="Stage the sensor scene, run the LiDAR sweep inside the simulator and "
            "start g1_sensor_relay. Off by default, provisionally: one timing-sensitive test "
            "regressed with sensors on, measured on a throttled CPU and never re-measured.",
        ),
        DeclareLaunchArgument(
            "odometry",
            default_value="fast_lio",
            description="Which source publishes odom -> base_footprint. 'fast_lio' is the "
            "pipeline the robot runs; 'ground_truth' is exact MuJoCo state and exists to "
            "isolate a fault to 'not the odometry'. Either needs sensors:=true.",
        ),
        DeclareLaunchArgument(
            "pin_pelvis",
            default_value="false",
            description="SIM-ONLY: weld the pelvis AND disable the walking policy, so the arm "
            "bridge can be exercised with nothing else driving the legs.",
        ),
        DeclareLaunchArgument(
            "sim_start_delay_s",
            default_value=str(SIM_START_DELAY_S),
            description="Seconds to delay unitree_mujoco relative to the rest of the launch, "
            "so the bridge and controller_manager are DDS-ready before the first physics "
            "tick. Raise it if the robot topples on startup; do not set to 0.",
        ),
        OpaqueFunction(function=_check_environment),
        OpaqueFunction(function=_launch_setup),
    ])
