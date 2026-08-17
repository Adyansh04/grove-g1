"""Composition-pure control stack: robot_state_publisher + controller_manager.

No simulator here -- this is the launch file that carries
over unchanged to hardware bring-up (see README.md's domain/DDS story).
Included by sim.launch.py for the simulation milestone.

`control_stack` picks which hardware component owns the motors. The two are mutually exclusive
by construction: `arm_sdk` blends our arm targets under the onboard balance controller, `lowcmd`
takes the whole body and leaves no balance running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Workaround: `ros2 run` CLI instead of launch_ros `Node` — under `Node`,
# arm_trajectory_controller's params reliably declare empty. `ros2 run` works
# correctly every time.
# The shell wrapper forwards SIGTERM/INT to the `ros2 run` subprocess (which
# would otherwise be orphaned) and re-waits so launch sees a clean exit.
_SIGNAL_FORWARDING_WRAPPER = (
    "set -m; {command} & child=$!; "
    "trap 'kill -TERM -$child 2>/dev/null; wait $child' TERM INT; "
    "wait $child"
)


def _launch_setup(context, *args, **kwargs):
    g1_description_share = get_package_share_directory("g1_description")
    g1_bringup_share = get_package_share_directory("g1_bringup")

    control_stack = LaunchConfiguration("control_stack").perform(context)
    if control_stack not in ("arm_sdk", "lowcmd"):
        raise RuntimeError(
            f"control_stack must be 'arm_sdk' or 'lowcmd', got '{control_stack}'"
        )

    rmw_for_stack = []
    if control_stack == "lowcmd":
        rmw_for_stack = [SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")]

    if control_stack == "lowcmd":
        xacro_path = os.path.join(g1_description_share, "urdf", "g1_lowcmd.urdf.xacro")
        controllers_yaml = os.path.join(
            get_package_share_directory("g1_controllers"), "config", "lowcmd_controllers.yaml"
        )
        # Every body joint must be claimed from the start: one the component sees unclaimed is
        # one it leaves unpowered. The policy and the safety controller it writes through must
        # activate in one switch, hence --activate-as-group: a chainable controller's reference
        # interfaces only become claimable as it enters chained mode, within that same switch.
        #
        # The three that load inactive are the ones something else switches in later: the arm
        # and hands wait for the arm bracket, and the locomotion freeze is the safety
        # controller's emergency target, which can only be activated if it is already loaded.
        controller_spawners = [
            ExecuteProcess(
                cmd=["ros2", "run", "controller_manager", "spawner", *names, *extra],
                name=f"{names[0]}_spawner",
                output="screen",
            )
            for names, extra in (
                (["joint_state_broadcaster"], []),
                (["imu_sensor_broadcaster"], []),
                (["waist_freeze_controller"], []),
                (["arm_freeze_controller"], []),
                (
                    ["locomotion_safety_controller", "agile_controller"],
                    ["--activate-as-group"],
                ),
                (["locomotion_freeze_controller"], ["--inactive"]),
                (["arm_trajectory_controller"], ["--inactive"]),
                (["left_hand_controller"], ["--inactive"]),
                (["right_hand_controller"], ["--inactive"]),
            )
        ]
    else:
        xacro_path = os.path.join(g1_description_share, "urdf", "g1_arm_sdk.urdf.xacro")
        controllers_yaml = os.path.join(g1_bringup_share, "config", "controllers.yaml")
        controller_spawners = None

    robot_description_content = Command([FindExecutable(name="xacro"), " ", xacro_path])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # robot_description delivered via ~/robot_description topic, remapped to global.
    control_node = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            _SIGNAL_FORWARDING_WRAPPER.format(
                command=(
                    "ros2 run controller_manager ros2_control_node --ros-args "
                    "-r '~/robot_description:=/robot_description' "
                    f"--params-file {controllers_yaml}"
                )
            ),
        ],
        name="ros2_control_node",
        output="screen",
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"],
        name="joint_state_broadcaster_spawner",
        output="screen",
    )

    # Loaded inactive — hardware component must activate first (see README).
    arm_trajectory_controller_spawner = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "arm_trajectory_controller",
            "--inactive",
        ],
        name="arm_trajectory_controller_spawner",
        output="screen",
    )

    # Same story as the arm: loaded inactive, activated once the hand component is.
    hand_controller_spawners = [
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "controller_manager",
                "spawner",
                f"{side}_hand_controller",
                "--inactive",
            ],
            name=f"{side}_hand_controller_spawner",
            output="screen",
        )
        for side in ("left", "right")
    ]

    # Tear down the whole launch if controller_manager dies.
    shutdown_on_control_node_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
        )
    )

    if controller_spawners is None:
        controller_spawners = [
            joint_state_broadcaster_spawner,
            arm_trajectory_controller_spawner,
            *hand_controller_spawners,
        ]

    return [
        *rmw_for_stack,
        robot_state_publisher_node,
        control_node,
        *controller_spawners,
        shutdown_on_control_node_exit,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "control_stack",
                default_value="arm_sdk",
                description="Which hardware component owns the motors. 'arm_sdk' blends arm "
                "targets under the onboard balance controller; 'lowcmd' takes the whole body "
                "and runs no balance underneath it.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
