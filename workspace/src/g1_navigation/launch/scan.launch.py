"""Flattens the 3D LiDAR sweep into the 2D scan slam_toolbox needs.

Shared by mapping and localization, which is why it is its own file rather than part of
either. See config/scan.yaml for what every parameter is protecting against.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("g1_navigation"), "config", "scan.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/livox/lidar",
            description="PointCloud2 to flatten. g1_sensor_relay's default on the converged "
            "track; a real Mid360 driver publishes the same topic.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="LaserScan output. slam_toolbox's scan_topic must match.",
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="both",
            parameters=[config],
            remappings=[
                ("cloud_in", LaunchConfiguration("cloud_topic")),
                ("scan", LaunchConfiguration("scan_topic")),
            ],
        ),
    ])
