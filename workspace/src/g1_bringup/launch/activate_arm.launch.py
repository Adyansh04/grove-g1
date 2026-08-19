"""Acquire the arms and hands. Run after sim.launch.py is up (see README)."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "run", "g1_bringup", "activate_arm"],
            name="activate_arm",
            output="screen",
        ),
    ])
