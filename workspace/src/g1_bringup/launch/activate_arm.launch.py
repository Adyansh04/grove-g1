"""Explicit activation entry point (see README.md's operating procedure).

Runs scripts/activate_arm, which waits for /lowstate to be flowing and then
activates G1ArmSdkSystem *before* arm_trajectory_controller -- the mandatory
order (Humble ties command-interface availability to hardware component
state; the reverse order can fail the controller switch). Run only after
sim.launch.py is up.
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
