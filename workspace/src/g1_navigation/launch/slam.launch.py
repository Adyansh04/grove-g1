"""slam_toolbox, in whichever mode the caller asked for.

Mapping builds a new map and owns map -> odom. Does not include the scan pipeline;
scan.launch.py does, and every mode needs it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# slam_toolbox ships a separate executable per mode rather than one node with a flag, so the
# mode argument has to pick both the binary and the parameter file.
MODES = {
    "mapping": ("async_slam_toolbox_node", "slam_mapping.yaml"),
}


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODES:
        raise RuntimeError(
            f"mode:={mode!r} is not available. Known modes: {sorted(MODES)}."
        )
    executable, params_file = MODES[mode]
    share = get_package_share_directory("g1_navigation")

    return [
        Node(
            package="slam_toolbox",
            executable=executable,
            name="slam_toolbox",
            output="both",
            parameters=[os.path.join(share, "config", params_file)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="mapping",
            description="'mapping' builds a new map of the facility.",
        ),
        OpaqueFunction(function=_setup),
    ])
