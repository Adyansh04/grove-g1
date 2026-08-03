"""slam_toolbox building a map, owning map -> odom while it does.

Mapping only. Localization went to map_server + AMCL (localization.launch.py) because
slam_toolbox's own localization mode loads a serialized pose graph, which for this map is 33 MB
against 130 KB for the grid -- see maps/README.md.

Does not include the scan pipeline; scan.launch.py does, and both modes need it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(
            get_package_share_directory("g1_navigation"), "config", "slam_mapping.yaml"
        )
    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="both",
            parameters=[params_file],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value="",
            description="Override config/slam_mapping.yaml. Empty uses the shipped one. "
            "Same escape hatch slam_toolbox's own launch files offer.",
        ),
        OpaqueFunction(function=_setup),
    ])
