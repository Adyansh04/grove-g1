"""Hand the arms and hands back (reverse of activate_arm).

On arm_sdk the component's on_deactivate ramp blocks for blend_ramp_down_s by design. On lowcmd
there is no component to deactivate and arm_freeze_controller takes the arms back instead.
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
                    "deactivate_arm",
                    "--stack",
                    LaunchConfiguration("control_stack"),
                ],
                name="deactivate_arm",
                output="screen",
            ),
        ]
    )
