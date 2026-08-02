"""Activate the arm hardware component and controller (see README).

Must run after sim.launch.py is up. Activates G1ArmSdkSystem before
arm_trajectory_controller (mandatory order in Humble)."""

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
