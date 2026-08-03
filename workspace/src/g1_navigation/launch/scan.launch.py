"""Flattens the 3D LiDAR sweep into the 2D scan slam_toolbox and AMCL need.

Shared by mapping and localization, which is why it is its own file rather than part of
either. See config/scan.yaml for what every parameter is protecting against.

Composed by default, into the same container as AMCL. That shares a process and gives each
component its own executor; it is not zero-copy, because nothing here sets
use_intra_process_comms and nav2_bringup does not either -- /map is transient-local, which
Humble's intra-process path does not support.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("g1_navigation"), "config", "scan.yaml"
    )
    use_composition = LaunchConfiguration("use_composition")
    remappings = [
        ("cloud_in", LaunchConfiguration("cloud_topic")),
        ("scan", LaunchConfiguration("scan_topic")),
    ]

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
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Load into a shared container instead of its own process. Requires "
            "that container to already exist -- nav_sim.launch.py creates it.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),

        Node(
            condition=UnlessCondition(use_composition),
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="both",
            parameters=[config],
            remappings=remappings,
        ),
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=LaunchConfiguration("container_name"),
            composable_node_descriptions=[
                ComposableNode(
                    package="pointcloud_to_laserscan",
                    plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                    name="pointcloud_to_laserscan",
                    parameters=[config],
                    remappings=remappings,
                ),
            ],
        ),
    ])
