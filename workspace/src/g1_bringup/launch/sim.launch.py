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
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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

    if not os.environ.get("CYCLONEDDS_URI"):
        problems.append(
            "CYCLONEDDS_URI is unset -- expected the container-baked cyclonedds.xml "
            "pinning the 'lo' interface (see .devcontainer/Dockerfile)."
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
    # The two options compose: pinning is orthogonal to whether sensors run, and the
    # geometry tests want both at once (a known robot pose in a known room).
    if sensors:
        overlay_name = (
            "g1_perception_pinned_scene.xml" if pin_pelvis else "g1_perception_scene.xml"
        )
    else:
        overlay_name = "g1_pinned_scene.xml" if pin_pelvis else "g1_flat_scene.xml"
    staged_path  = os.path.join(G1_MODEL_DIR, STAGED_SCENE_NAME)
    overlay_src  = os.path.join(
        get_package_share_directory("g1_bringup"), "mjcf", overlay_name
    )
    shutil.copyfile(overlay_src, staged_path)
    sim_cmd = [UNITREE_MUJOCO_BIN, "-r", "g1", "-s", STAGED_SCENE_NAME]

    # The patched unitree_mujoco starts its sensor thread only when this names a config,
    # so the stock code path is what runs unless sensors are asked for explicitly.
    sensor_config = os.path.join(
        get_package_share_directory("g1_bringup"), "config", "sim_sensors.yaml"
    )
    if sensors:
        # The patched binary starts its sensor thread only when this names a config, so the
        # stock code path is what runs unless sensors are asked for explicitly.
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
                parameters=[{"socket_path": socket_path, "frame_id": "mid360_link"}],
            )
        )

    def _remove_staged_scene(context, *a, **k):
        try:
            os.remove(staged_path)
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
    motion_service_sim_node = Node(
        package="g1_bringup",
        executable="motion_service_sim",
        name="motion_service_sim",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("g1_bringup"), "config", "motion_service_sim.yaml"
            ),
            os.path.join(get_package_share_directory("g1_bringup"), "config", "walk_policy.yaml"),
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
                "sensors",
                default_value="false",
                description="Stage the perception scene (a room with known geometry) and "
                "run the sensor thread. Off by default so the locomotion suites keep their "
                "existing bare-floor conditions until the timing gate says otherwise.",
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
