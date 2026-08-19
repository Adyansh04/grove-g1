"""Hand the arms and hands back. Reverse of activate_arm (see README)."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "run", "g1_bringup", "deactivate_arm"],
            name="deactivate_arm",
            output="screen",
        ),
    ])
