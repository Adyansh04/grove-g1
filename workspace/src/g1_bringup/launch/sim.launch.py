"""Sim bring-up: unitree_mujoco + arm_sdk_sim_bridge + control.launch.py.

See README.md for the full operating procedure (sim.launch.py ->
activate_arm.launch.py -> command -> deactivate_arm.launch.py -> stop),
the domain/DDS story, and the sim-bridge safety banner.
"""

import os

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
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

UNITREE_MUJOCO_BIN = "/opt/unitree_robotics/unitree_mujoco/simulate/build/unitree_mujoco"

# unitree_mujoco is a native unitree_sdk2 DDS app, not a ROS node, and links
# its own build of CycloneDDS from here. Sourcing a ROS environment (as any
# shell launching `ros2 launch` already has) prepends /opt/ros/humble/lib*
# to LD_LIBRARY_PATH ahead of this directory; since the binary's own rpath
# is a RUNPATH (checked *after* LD_LIBRARY_PATH), that shadows this build
# with ROS's separately-built libddsc.so.0 and crashes on the first DDS
# write with a heap-corruption/assertion abort -- confirmed directly
# (gdb backtrace landed inside /opt/ros/humble/lib/.../libddsc.so.0, an
# ABI-incompatible copy). Prepending this path here, rather than replacing
# LD_LIBRARY_PATH outright, keeps everything else (X11/GL, etc.) intact.
UNITREE_ROBOTICS_LIB = "/opt/unitree_robotics/lib"

# Headless mode runs its own Xvfb as a directly-tracked launch action rather
# than wrapping the sim in the `xvfb-run` script. `xvfb-run` spawns Xvfb and
# the wrapped binary as untracked grandchildren that never receive launch's
# SIGTERM: confirmed directly that both `unitree_mujoco` and `Xvfb` were
# still running -- still holding the DDS graph -- well after `ros2 launch`
# itself had exited cleanly, which risks a second sim instance colliding
# with the next launch (the same dual-writer hazard as two publishers on
# one low-level channel). Managing Xvfb ourselves means launch's normal shutdown handling
# (which does correctly signal every process it started directly) tears it
# down too.
XVFB_DISPLAY = ":133"

# The sim's first physics tick applies zero actuator torque until
# arm_sdk_sim_bridge captures its first /lowstate sample and starts
# publishing /lowcmd -- with no balance controller, a humanoid stance
# collapses within the first few unactuated ticks. Delaying the sim's start
# (not the rest of the stack) gives the bridge's /lowstate subscription and
# controller_manager time to come up and DDS-match first, so the sim's very
# first physics step is already commanded. This must apply unconditionally,
# not just in headless mode: it was originally sized for Xvfb's startup
# alone, and headless mode's incidental head start over the ROS graph is
# what masked this race in validation -- headless:=false has zero delay by
# default and can free-fall before the bridge is ready.
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

    sim_process = ExecuteProcess(
        cmd=[UNITREE_MUJOCO_BIN, "-r", "g1"],
        name="unitree_mujoco",
        output="screen",
        env=sim_env,
    )
    # Delay only the sim's start, not the rest of the stack -- Xvfb (headless
    # only), the bridge, and control.launch.py can all start immediately and
    # use the head start to get their /lowstate subscription DDS-matched
    # before the sim's first physics tick (see SIM_START_DELAY_S above).
    sim_start_delay_s = float(LaunchConfiguration("sim_start_delay_s").perform(context))
    actions.append(TimerAction(period=sim_start_delay_s, actions=[sim_process]))

    arm_sdk_sim_bridge_node = Node(
        package="g1_bringup",
        executable="arm_sdk_sim_bridge",
        name="arm_sdk_sim_bridge",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("g1_bringup"), "config", "arm_sdk_sim_bridge.yaml"
            )
        ],
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "control.launch.py")
        )
    )

    # A dead sim leaves the controllers commanding nothing and the bridge
    # holding onto a stale world -- tear down the whole launch rather than
    # limp on -- the same "no dangling control authority" rule that governs
    # a live sim applies here too).
    shutdown_on_sim_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=sim_process,
            on_exit=[EmitEvent(event=Shutdown(reason="unitree_mujoco exited"))],
        )
    )

    actions.extend([arm_sdk_sim_bridge_node, control_launch, shutdown_on_sim_exit])
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
