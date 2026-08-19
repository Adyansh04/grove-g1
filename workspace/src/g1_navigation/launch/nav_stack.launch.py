"""The navigation stack without a simulator under it.

The container, the scan pipeline, SLAM or localization, and optionally Nav2 itself. Nothing here
is sim-specific, so this is the file that carries to hardware unchanged.

Stages NO simulator and no RViz: both callers own those decisions, and a second simulator would
put two publishers on rt/lowcmd. test_launch_threading asserts the absence.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LAUNCH_DIR = os.path.join(get_package_share_directory("g1_navigation"), "launch")

MODES = ("mapping", "localization")

# Isolated rather than the plain container: each component keeps its own single-threaded
# executor, so one blocking callback cannot stall the others.
CONTAINER_EXECUTABLE = "component_container_isolated"


def _include(name, **launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, name)),
        launch_arguments=launch_args.items(),
    )


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

    actions = [
        # Created here rather than in the leaf launches, so one container serves all of them.
        Node(
            condition=IfCondition(use_composition),
            name=container_name,
            package="rclcpp_components",
            executable=CONTAINER_EXECUTABLE,
            output="both",
        ),
        _include("scan.launch.py", use_composition=use_composition, container_name=container_name),
    ]

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
        if mode != "localization":
            raise RuntimeError(
                "nav:=true needs mode:=localization. Navigating against a map slam_toolbox is "
                "still building means the goal pose moves under the planner."
            )
        # Explicit, because an included file inherits the parent's configurations and Nav2 must
        # run uncomposed. The scan and localization nodes above still compose.
        actions.append(_include("nav2.launch.py", use_composition="false"))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="mapping",
            description="'mapping' builds a new map with slam_toolbox; 'localization' runs "
            "map_server + AMCL against maps/facility.",
        ),
        DeclareLaunchArgument(
            "nav",
            default_value="false",
            description="Start the Nav2 servers and the base approach. Requires "
            "mode:=localization. Off by default: mapping runs need none of it.",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Run the navigation nodes in one component container, each with its "
            "own executor. Set false for one process per node when a single node is crashing.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
        OpaqueFunction(function=_setup),
    ])
