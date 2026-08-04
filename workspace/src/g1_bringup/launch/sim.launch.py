"""Sim bring-up: unitree_mujoco + motion_service_sim + control.launch.py + loco.launch.py.

See README.md for the full operating procedure (sim.launch.py ->
activate_arm.launch.py -> command -> deactivate_arm.launch.py -> stop),
the domain/DDS story, and the sim-bridge safety banner.
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

UNITREE_MUJOCO_BIN = "/opt/unitree_robotics/unitree_mujoco/simulate/build/unitree_mujoco"

# Vendored G1 model directory. Derived from the binary location.
G1_MODEL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(UNITREE_MUJOCO_BIN), "..", "..", "unitree_robots", "g1")
)

# Scene overlay staged into G1_MODEL_DIR at launch (relative includes require it).
STAGED_SCENE_NAME = "g1_grove_scene.staged.xml"

# Prepend unitree_robotics/lib so the sim loads its own CycloneDDS build
# instead of ROS's ABI-incompatible /opt/ros/humble/lib/libddsc.so.0.
UNITREE_ROBOTICS_LIB = "/opt/unitree_robotics/lib"

# Manage Xvfb directly as a launch action — xvfb-run orphans its children,
# leaving the sim running after launch exits.
XVFB_DISPLAY = ":133"

# Delay the sim start so the bridge and controller_manager are DDS-ready
# before the first physics tick. Too short can also crash headless GLFW startup.
SIM_START_DELAY_S = 2.0


def _check_environment(context, *args, **kwargs):
    """Fails the whole launch immediately, before anything starts, if the
    container's DDS/domain env isn't what the sim-first milestone assumes
    (see README.md's domain/DDS story). An empty ROS graph from a silent
    RMW/domain mismatch is much harder to debug than this explicit error.
    """
    problems = []

    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if rmw != "rmw_cyclonedds_cpp":
        problems.append(
            f"RMW_IMPLEMENTATION={rmw!r}, expected 'rmw_cyclonedds_cpp' -- "
            "the Unitree SDK/sim only speak CycloneDDS."
        )

    cyclone_uri = os.environ.get("CYCLONEDDS_URI")
    if not cyclone_uri:
        problems.append(
            "CYCLONEDDS_URI is unset -- expected the container-baked cyclonedds.xml "
            "pinning the 'lo' interface (see .devcontainer/Dockerfile)."
        )
    elif cyclone_uri.startswith("file://"):
        # Existence, not just non-emptiness. CycloneDDS treats an unreadable URI as a
        # warning on stderr and falls back to its defaults, which with the compose file's
        # network_mode: host means binding the real NIC instead of lo. The sim's
        # rt/lowcmd and rt/arm_sdk would then be on the LAN on domain 1, within reach of a
        # real G1. The simulator relies on this file too: its own config sets an empty
        # interface so the SDK reads CYCLONEDDS_URI (see patches/unitree_mujoco/002).
        cyclone_path = cyclone_uri[len("file://") :]
        if not os.path.isfile(cyclone_path):
            problems.append(
                f"CYCLONEDDS_URI points at {cyclone_path!r}, which does not exist -- "
                "CycloneDDS would silently fall back to defaults and bind the host NIC "
                "instead of 'lo'."
            )

    domain_id = os.environ.get("ROS_DOMAIN_ID")
    if domain_id != "1":
        problems.append(
            f"ROS_DOMAIN_ID={domain_id!r}, expected '1' -- the sim-first milestone's "
            "dedicated domain (see README.md)."
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


def _launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    pin_pelvis = LaunchConfiguration("pin_pelvis").perform(context).lower() == "true"

    sim_env = dict(os.environ)
    sim_env["LD_LIBRARY_PATH"] = UNITREE_ROBOTICS_LIB + ":" + sim_env.get("LD_LIBRARY_PATH", "")

    actions = []
    if headless:
        xvfb_process = ExecuteProcess(
            cmd=["Xvfb", XVFB_DISPLAY, "-screen", "0", "1280x1024x24", "-nolisten", "tcp"],
            name="xvfb",
            output="screen",
        )
        actions.append(xvfb_process)
        sim_env["DISPLAY"] = XVFB_DISPLAY

    # Stage one of our scenes (bare floor). The vendored obstacle course spawns the robot
    # among obstacles that look like balance failures.
    #
    # The perception scene is the same floor plus a room with known geometry, which is what
    # the sensor assertions measure against. Selected separately from pin_pelvis because a
    # pinned pelvis and a walking robot both want sensors eventually.
    sensors = LaunchConfiguration("sensors").perform(context).lower() == "true"
    # World and pinning compose: pinning is orthogonal to which room the robot is in, and
    # the geometry test wants both at once (a known robot pose in a known room).
    world = LaunchConfiguration("world").perform(context)
    if world not in ("navigation", "perception"):
        raise RuntimeError(
            f"world:={world!r} is not a scene. Use 'navigation' (the multi-room facility) "
            "or 'perception' (the small room the geometry test measures against)."
        )
    if sensors:
        suffix       = "_pinned" if pin_pelvis else ""
        overlay_name = f"g1_{world}{suffix}_scene.xml"
    else:
        overlay_name = "g1_pinned_scene.xml" if pin_pelvis else "g1_flat_scene.xml"
    mjcf_dir     = os.path.join(get_package_share_directory("g1_bringup"), "mjcf")
    staged_path  = os.path.join(G1_MODEL_DIR, STAGED_SCENE_NAME)
    shutil.copyfile(os.path.join(mjcf_dir, overlay_name), staged_path)

    # The pinned scenes are overlays that <include> their base, so the base has to be staged
    # beside them under the name the overlay asks for. MuJoCo resolves includes relative to
    # the staged file, not to the package share.
    staged_bases = []
    if sensors and pin_pelvis:
        base_name   = f"g1_{world}_scene.staged.xml"
        base_staged = os.path.join(G1_MODEL_DIR, base_name)
        shutil.copyfile(os.path.join(mjcf_dir, f"g1_{world}_scene.xml"), base_staged)
        staged_bases.append(base_staged)
    sim_cmd = [UNITREE_MUJOCO_BIN, "-r", "g1", "-s", STAGED_SCENE_NAME]

    # The patched unitree_mujoco starts its sensor thread only when this names a config,
    # so the stock code path is what runs unless sensors are asked for explicitly.
    sensor_config = os.path.join(
        get_package_share_directory("g1_bringup"), "config", "sim_sensors.yaml"
    )
    if sensors:
        sim_env["GROVE_G1_SENSOR_CONFIG"] = sensor_config

        # The relay owns the ROS side. Start order does not matter: it listens whenever it
        # comes up and the simulator retries connecting every cycle, so either process can
        # start, die or restart independently.
        with open(sensor_config) as sensor_file:
            socket_path = yaml.safe_load(sensor_file)["socket_path"]
        actions.append(
            Node(
                package="g1_sensor_relay",
                executable="g1_sensor_relay",
                name="g1_sensor_relay",
                output="both",
                parameters=[{
                    "socket_path": socket_path,
                    "frame_id": "mid360_link",
                    "depth_frame_id": "camera_depth_optical_frame",
                    "color_frame_id": "camera_color_optical_frame",
                    "world_frame_id": "odom",
                }],
            )
        )

        # rviz only makes sense with sensors up, which is why it lives inside this branch:
        # the shipped config's displays are all sensor topics.
        if LaunchConfiguration("rviz").perform(context).lower() == "true":
            actions.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="log",
                    arguments=[
                        "-d",
                        os.path.join(
                            get_package_share_directory("g1_bringup"), "config", "g1_sensors.rviz"
                        ),
                    ],
                )
            )

        # odom -> base_footprint -> pelvis. Tunables live in the package's own converged-track
        # config rather than inline here, so there is one place to change them.
        odometry_node = LifecycleNode(
            package="g1_state_estimation",
            executable="g1_odometry_publisher",
            name="g1_odometry_publisher",
            namespace="",
            output="both",
            parameters=[
                os.path.join(
                    get_package_share_directory("g1_state_estimation"),
                    "config",
                    "g1_odometry_publisher_converged.yaml",
                )
            ],
            remappings=[
                ("~/sport_state", "/sportmodestate"),
                ("~/imu_state", "/lowstate"),
            ],
        )
        actions.append(odometry_node)
        actions.append(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=odometry_node,
                    on_start=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(odometry_node),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ],
                )
            )
        )
        actions.append(
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=odometry_node,
                    goal_state="inactive",
                    entities=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(odometry_node),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        )
                    ],
                )
            )
        )

    def _remove_staged_scene(context, *a, **k):
        try:
            os.remove(staged_path)
            for base in staged_bases:
                if os.path.exists(base):
                    os.remove(base)
        except OSError:
            pass
        return []

    actions.append(
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=_remove_staged_scene)])
        )
    )

    sim_process = ExecuteProcess(
        cmd=sim_cmd,
        name="unitree_mujoco",
        output="screen",
        env=sim_env,
    )
    # Delay only the sim; everything else starts immediately.
    sim_start_delay_s = float(LaunchConfiguration("sim_start_delay_s").perform(context))
    actions.append(TimerAction(period=sim_start_delay_s, actions=[sim_process]))

    # Pelvis weld and walking policy are mutually exclusive.
    motion_service_sim_share = get_package_share_directory("g1_motion_service_sim")
    motion_service_sim_node  = Node(
        package="g1_motion_service_sim",
        executable="motion_service_sim",
        name="motion_service_sim",
        output="screen",
        parameters=[
            os.path.join(motion_service_sim_share, "config", "motion_service_sim.yaml"),
            os.path.join(motion_service_sim_share, "config", "walk_policy.yaml"),
            # Completes pelvis -> torso_link so the sensor frames are not stranded in their
            # own TF tree. Only when sensors run: it costs work on the 1 kHz /lowstate path.
            {"publish_lower_joint_states": sensors},
            {"walk_policy.enabled": not pin_pelvis},
        ],
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "control.launch.py")
        )
    )

    # LocoClient bridge — talks to motion_service_sim's /api/sport/* responder.
    loco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "loco.launch.py")
        )
    )

    # Tear down the whole launch if the sim dies.
    shutdown_on_sim_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=sim_process,
            on_exit=[EmitEvent(event=Shutdown(reason="unitree_mujoco exited"))],
        )
    )

    actions.extend([motion_service_sim_node, control_launch, loco_launch, shutdown_on_sim_exit])
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=_check_environment),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run unitree_mujoco against our own managed Xvfb (no GUI window).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="navigation",
                description="Which room to stage, when sensors are on. 'navigation' is the "
                "multi-room facility with furniture, shelving, pillars and a ramp. "
                "'perception' is the small bare room whose wall positions test_lidar_geometry "
                "asserts against; use it for tests, not for looking at things.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open RViz on config/g1_sensors.rviz with the cloud, TF, robot "
                "model and odometry already set up.",
            ),
            DeclareLaunchArgument(
                "sensors",
                default_value="false",
                description="Stage the perception scene (a room with known geometry), run "
                "the LiDAR sweep inside the simulator, and start g1_sensor_relay to publish "
                "it.\n"
                "OFF by default, provisionally. test_arm_command missed its slew-limited "
                "convergence window with sensors on, but that was measured on a CPU pinned "
                "at ~14% of its peak clock, and the test is timing-sensitive. Treat the "
                "result as unproven, not as a property of the sensor stack. Opt-in until it "
                "is re-measured on an unthrottled machine. test_lidar_geometry, which is "
                "pure geometry, turns sensors on explicitly and passes.",
            ),
            DeclareLaunchArgument(
                "pin_pelvis",
                default_value="false",
                description="SIM-ONLY debugging aid: weld the pelvis to the world AND disable the "
                "walking policy, so the arm bridge can be exercised with nothing else driving the "
                "legs. Default false -- the policy balances the robot itself.",
            ),
            DeclareLaunchArgument(
                "sim_start_delay_s",
                default_value=str(SIM_START_DELAY_S),
                description="Seconds to delay unitree_mujoco's start relative to the rest of "
                "the launch, so the bridge and controller_manager are DDS-ready before the "
                "sim's first physics tick. Raise this if the robot still topples on startup "
                "(slower discovery); do not set to 0.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
