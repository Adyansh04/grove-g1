"""Localization against the committed map: map_server serves it, AMCL owns map -> odom.

The alternative to slam.launch.py, never run alongside it: both would broadcast map -> odom.
Does not include the scan pipeline; scan.launch.py does, and both modes need it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

SHARE = get_package_share_directory("g1_navigation")
PARAMS_FILE = os.path.join(SHARE, "config", "localization.yaml")


def _arguments():
    return [
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(SHARE, "maps", "facility.yaml"),
            description="Occupancy grid to localize against. Re-map if the scene has "
            "changed; nothing checks that this still matches.",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Load into a shared container instead of one process per node. "
            "Requires that container to already exist; nav_stack.launch.py creates it.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
    ]


def _map_server_params():
    # Overrides the empty yaml_filename in the params file rather than duplicating the path.
    return [PARAMS_FILE, {"yaml_filename": LaunchConfiguration("map")}]


def _standalone():
    return GroupAction(
        condition=UnlessCondition(LaunchConfiguration("use_composition")),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="both",
                parameters=_map_server_params(),
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="both",
                parameters=[PARAMS_FILE],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="both",
                parameters=[PARAMS_FILE],
            ),
        ],
    )


def _composed():
    return LoadComposableNodes(
        condition=IfCondition(LaunchConfiguration("use_composition")),
        target_container=LaunchConfiguration("container_name"),
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=_map_server_params(),
            ),
            ComposableNode(
                package="nav2_amcl",
                plugin="nav2_amcl::AmclNode",
                name="amcl",
                parameters=[PARAMS_FILE],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[PARAMS_FILE],
            ),
        ],
    )


def generate_launch_description():
    return LaunchDescription(_arguments() + [_standalone(), _composed()])
