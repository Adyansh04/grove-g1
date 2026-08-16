"""Acquire the arms and hands (see README).

Must run after sim.launch.py is up. On arm_sdk this activates G1ArmSdkSystem before
arm_trajectory_controller, which is the mandatory order. On lowcmd the body component is
already active, so it trades arm_freeze_controller for arm_trajectory_controller in one switch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "control_stack",
                default_value="arm_sdk",
                description="Which control stack is running. Must match sim.launch.py's.",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "g1_bringup",
                    "activate_arm",
                    "--stack",
                    LaunchConfiguration("control_stack"),
                ],
                name="activate_arm",
                output="screen",
            ),
        ]
    )
