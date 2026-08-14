"""Localization against the committed map: map_server serves it, AMCL owns map -> odom.

The alternative to slam.launch.py, never run alongside it -- both would broadcast map -> odom.
Neither includes the scan pipeline; scan.launch.py does, and both need it.

Structure mirrors nav2_bringup's own localization_launch.py, including the composed and
non-composed branches, so this file's servers drop into the same shared container as the rest
of the navigation stack.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

LIFECYCLE_NODES = ["map_server", "amcl"]


def generate_launch_description():
    share = get_package_share_directory("g1_navigation")
    params = os.path.join(share, "config", "localization.yaml")

    map_yaml = LaunchConfiguration("map")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")

    # The map path is a launch argument so it can be overridden per run; it overrides the
    # empty yaml_filename in the params file rather than being duplicated there.
    map_server_params = [params, {"yaml_filename": map_yaml}]
    manager_params = [{
        "use_sim_time": False,
        "autostart": True,
        "node_names": LIFECYCLE_NODES,
    }]

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(share, "maps", "facility.yaml"),
            description="Occupancy grid to localize against. Re-map if g1_bringup's "
            "g1_navigation_scene.xml has changed; nothing checks that this still matches.",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Load into a shared container instead of one process per node. "
            "Requires that container to already exist -- nav_sim.launch.py creates it.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),

        GroupAction(
            condition=UnlessCondition(use_composition),
            actions=[
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="both",
                    parameters=map_server_params,
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
                    parameters=manager_params,
                ),
            ],
        ),

        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package="nav2_map_server",
                    plugin="nav2_map_server::MapServer",
                    name="map_server",
                    parameters=map_server_params,
                ),
                ComposableNode(
                    package="nav2_amcl",
                    plugin="nav2_amcl::AmclNode",
                    name="amcl",
                    parameters=[params],
                ),
                ComposableNode(
                    package="nav2_lifecycle_manager",
                    plugin="nav2_lifecycle_manager::LifecycleManager",
                    name="lifecycle_manager_localization",
                    parameters=manager_params,
                ),
            ],
        ),
    ])
