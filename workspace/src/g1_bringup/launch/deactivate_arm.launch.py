"""Explicit release entry point (see README.md's operating procedure).

Runs scripts/deactivate_arm, which deactivates arm_trajectory_controller
*before* G1ArmSdkSystem -- the reverse of activate_arm.launch.py. The
component's own on_deactivate ramp runs synchronously inside that step and
blocks for roughly blend_ramp_down_s by design (see g1_hardware_interface's
README): this is the documented clean stop, expected to take a couple of
seconds, not a hang.
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
