"""Hand the arms and hands back (reverse of activate_arm).

There is no component to deactivate: arm_freeze_controller takes the arms back instead.
"""

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
