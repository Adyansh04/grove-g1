"""The navigation stack without a simulator under it.

The container, the scan pipeline, SLAM or localization, optionally Nav2 itself, and with
tether_filter:=true a crop box that keeps a real robot's tether out of the costmaps and the scan.
Nothing here is sim-specific, so this is the file that carries to hardware unchanged.

Stages NO simulator and no RViz: both callers own those decisions, and a second simulator would
put two publishers on rt/lowcmd. test_launch_threading asserts the absence.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

SHARE = get_package_share_directory("g1_navigation")
LAUNCH_DIR = os.path.join(SHARE, "launch")
TETHER_FILTER_PARAMS = os.path.join(SHARE, "config", "tether_filter.yaml")
RAW_CLOUD = "/livox/lidar"
TETHER_FILTERED_CLOUD = "/livox/lidar/tether_filtered"

MODES = ("mapping", "localization")

# Isolated rather than the plain container: each component keeps its own single-threaded
# executor, so one blocking callback cannot stall the others.
CONTAINER_EXECUTABLE = "component_container_isolated"


def _include(name, **launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, name)),
        launch_arguments=launch_args.items(),
    )


def _tether_filter(use_composition, container_name):
    """pcl_ros's CropBox between the LiDAR and the costmaps and scan; see tether_filter.yaml."""
    remappings = [("input", RAW_CLOUD), ("output", TETHER_FILTERED_CLOUD)]
    return [
        Node(
            condition=UnlessCondition(use_composition),
            package="pcl_ros",
            executable="filter_crop_box_node",
            name="tether_filter",
            output="both",
            parameters=[TETHER_FILTER_PARAMS],
            remappings=remappings,
        ),
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package="pcl_ros",
                    plugin="pcl_ros::CropBox",
                    name="tether_filter",
                    parameters=[TETHER_FILTER_PARAMS],
                    remappings=remappings,
                ),
            ],
        ),
    ]


def _check_mode(mode):
    if mode not in MODES:
        raise RuntimeError(
            f"mode:={mode!r} is not a mode. 'mapping' builds a new map of the facility; "
            f"'localization' runs against the committed one, which is what a repeatable "
            f"goal pose needs."
        )


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    _check_mode(mode)

    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    tether = LaunchConfiguration("tether_filter").perform(context).lower() == "true"
    cloud = TETHER_FILTERED_CLOUD if tether else RAW_CLOUD

    actions = [
        # Created here rather than in the leaf launches, so one container serves all of them.
        Node(
            condition=IfCondition(use_composition),
            name=container_name,
            package="rclcpp_components",
            executable=CONTAINER_EXECUTABLE,
            output="both",
        ),
        _include(
            "scan.launch.py",
            cloud_topic=cloud,
            use_composition=use_composition,
            container_name=container_name,
        ),
    ]
    if tether:
        actions += _tether_filter(use_composition, container_name)

    if mode == "mapping":
        # slam_toolbox stays out of the container: its 40 MB stack requirement for map
        # serialization is not something to hand a shared process.
        actions.append(_include("slam.launch.py"))
    else:
        actions.append(
            _include(
                "localization.launch.py",
                use_composition=use_composition,
                container_name=container_name,
            )
        )

    if LaunchConfiguration("nav").perform(context).lower() == "true":
        # With mapping, Nav2 plans on the map slam_toolbox is still growing, as exploration needs;
        # a goal can shift a little under a loop closure. A fixed goal pose wants localization.
        # Explicit, because an included file inherits the parent's configurations and Nav2 must
        # run uncomposed. The scan and localization nodes above still compose.
        actions.append(
            _include(
                "nav2.launch.py",
                use_composition="false",
                collision_high_cloud=LaunchConfiguration("collision_high_cloud"),
                marking_cloud=cloud,
                route_graph=LaunchConfiguration("route_graph"),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="mapping",
                description="'mapping' builds a new map with slam_toolbox; 'localization' runs "
                "map_server + AMCL against maps/facility.",
            ),
            DeclareLaunchArgument(
                "route_graph",
                default_value="",
                description="Passed to nav2.launch.py: the route graph, empty for no route server.",
            ),
            DeclareLaunchArgument(
                "collision_high_cloud",
                default_value="/livox/lidar",
                description="Passed to nav2.launch.py: the cloud collision_monitor's high band "
                "reads, /filtered_cloud when MoveIt runs.",
            ),
            DeclareLaunchArgument(
                "tether_filter",
                default_value="false",
                choices=["true", "false"],
                description="On the real robot with a tether: the costmaps and the scan read the "
                "LiDAR less a box behind the robot where the tether hangs (tether_filter.yaml); "
                "collision_monitor still reads all of it. Off in simulation, which has no tether.",
            ),
            DeclareLaunchArgument(
                "nav",
                default_value="false",
                description="Start the Nav2 servers and the base approach. With mode:=mapping they "
                "plan on the map slam_toolbox is building, which is what exploration does.",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="true",
                description="Run the navigation nodes in one component container, each with its "
                "own executor. Set false for one process per node when a single node is crashing.",
            ),
            DeclareLaunchArgument("container_name", default_value="nav2_container"),
            OpaqueFunction(function=_setup),
        ]
    )
