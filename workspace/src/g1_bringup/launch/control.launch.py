"""Composition-pure control stack: robot_state_publisher + controller_manager.

No sim, no motion_service_sim here -- this is the launch file that carries
over unchanged to hardware bring-up (see README.md's domain/DDS story).
Included by sim.launch.py for the simulation milestone.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable
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


def generate_launch_description():
    g1_description_share = get_package_share_directory("g1_description")
    g1_bringup_share = get_package_share_directory("g1_bringup")

    xacro_path = os.path.join(g1_description_share, "urdf", "g1_arm_sdk.urdf.xacro")
    controllers_yaml = os.path.join(g1_bringup_share, "config", "controllers.yaml")

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

    return LaunchDescription(
        [
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner,
            arm_trajectory_controller_spawner,
            *hand_controller_spawners,
            shutdown_on_control_node_exit,
        ]
    )
