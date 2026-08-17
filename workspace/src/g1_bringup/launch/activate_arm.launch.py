"""Acquire the arms and hands (see README).

Must run after sim.launch.py is up. The body component is already active by then, so this
trades arm_freeze_controller for arm_trajectory_controller in one switch.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["ros2", "run", "g1_bringup", "activate_arm"],
                name="activate_arm",
                output="screen",
            ),
        ]
    )
