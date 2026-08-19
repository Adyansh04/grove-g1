"""Flattens the 3D LiDAR sweep into the 2D scan slam_toolbox and AMCL need.

Shared by mapping and localization, so it is its own file rather than part of either.
Composed by default, into the same container as AMCL.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

PARAMS_FILE = os.path.join(
    get_package_share_directory("g1_navigation"), "config", "scan.yaml"
)


def _arguments():
    return [
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/livox/lidar",
            description="PointCloud2 to flatten. Both g1_sensor_relay and a real Mid360 "
            "driver publish this topic.",
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
            "that container to already exist; nav_stack.launch.py creates it.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
    ]


def generate_launch_description():
    use_composition = LaunchConfiguration("use_composition")
    remappings = [
        ("cloud_in", LaunchConfiguration("cloud_topic")),
        ("scan", LaunchConfiguration("scan_topic")),
    ]

    standalone = Node(
        condition=UnlessCondition(use_composition),
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="both",
        parameters=[PARAMS_FILE],
        remappings=remappings,
    )

    composed = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=LaunchConfiguration("container_name"),
        composable_node_descriptions=[
            ComposableNode(
                package="pointcloud_to_laserscan",
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name="pointcloud_to_laserscan",
                parameters=[PARAMS_FILE],
                remappings=remappings,
            ),
        ],
    )

    return LaunchDescription(_arguments() + [standalone, composed])
