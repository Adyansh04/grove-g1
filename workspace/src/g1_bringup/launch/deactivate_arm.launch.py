"""Deactivate arm controller and hardware component (reverse of activate_arm).

The component's on_deactivate ramp blocks for blend_ramp_down_s by design."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["ros2", "run", "g1_bringup", "deactivate_arm"],
                name="deactivate_arm",
                output="screen",
            ),
        ]
    )
