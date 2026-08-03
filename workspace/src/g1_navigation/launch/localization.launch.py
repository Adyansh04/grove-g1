"""Localization against the committed map: map_server serves it, AMCL owns map -> odom.

The alternative to slam.launch.py, never run alongside it -- both would broadcast map -> odom.
Neither includes the scan pipeline; scan.launch.py does, and both need it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIFECYCLE_NODES = ["map_server", "amcl"]


def generate_launch_description():
    share = get_package_share_directory("g1_navigation")
    params = os.path.join(share, "config", "localization.yaml")

    map_yaml = LaunchConfiguration("map")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(share, "maps", "facility.yaml"),
            description="Occupancy grid to localize against. Re-map if g1_bringup's "
            "g1_navigation_scene.xml has changed; nothing checks that this still matches.",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="both",
            # The map path is a launch argument, so it overrides the empty yaml_filename in
            # the params file rather than being duplicated there.
            parameters=[params, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="both",
            parameters=[params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="both",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": LIFECYCLE_NODES,
            }],
        ),
    ])
