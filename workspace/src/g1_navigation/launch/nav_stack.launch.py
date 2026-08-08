"""The navigation stack without a simulator under it.

The `control.launch.py` analogue for navigation: the shared component container, the scan
pipeline, SLAM or localization, and optionally Nav2 itself. Nothing here is sim-specific, so
this is the file that carries to hardware unchanged, and it is what a bring-up includes.

Named for the stack, not for Nav2. `nav2.launch.py` is one of the pieces this file composes:
the Nav2 servers alone, and only when nav:=true. This file is the assembly around them, and it
runs a mapping session that never loads Nav2 at all.

It stages NO simulator, deliberately. Both callers -- `nav_sim.launch.py` here and
`g1_bringup`'s `bringup.launch.py` -- stage exactly one themselves. A simulator include in this
file would give whichever caller already had one a second `motion_service_sim`, and that is two
publishers on `/lowcmd` fighting for the same joints. The robot would
collapse with nothing in the logs to explain it, so `test_launch_threading` asserts the absence
rather than trusting it.

RViz is not here either. It is a per-operator choice about one window, not part of the stack,
and both callers already own that decision.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = ("mapping", "localization")

# Isolated rather than the plain container: each component keeps its own single-threaded
# executor, so one blocking callback cannot stall the others.
CONTAINER_EXECUTABLE = "component_container_isolated"


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODES:
        raise RuntimeError(
            f"mode:={mode!r} is not a mode. 'mapping' builds a new map of the facility; "
            f"'localization' runs against the committed one, which is what a repeatable "
            f"goal pose needs."
        )
    launch_dir = os.path.join(get_package_share_directory("g1_navigation"), "launch")

    def include(name, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=launch_args.items(),
        )

    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")

    actions = [
        # The container has to exist before anything tries to load into it. Created here
        # rather than in the leaf launches so one container serves all of them -- which is
        # the point, and is what the costmaps and controller join.
        Node(
            condition=IfCondition(use_composition),
            name=container_name,
            package="rclcpp_components",
            executable=CONTAINER_EXECUTABLE,
            output="both",
        ),
        include(
            "scan.launch.py",
            use_composition=use_composition,
            container_name=container_name,
        ),
    ]

    if mode == "mapping":
        # slam_toolbox stays out of the container even though it ships a component:
        # nav2_bringup's own slam_launch.py runs it as a plain node, and its 40 MB stack
        # requirement for map serialization is not something to hand a shared process.
        actions.append(include("slam.launch.py"))
    else:
        actions.append(
            include(
                "localization.launch.py",
                use_composition=use_composition,
                container_name=container_name,
            )
        )

    # Nav2 itself. Off by default so a mapping run starts nothing it does not need, and the
    # navigation invocation stays explicit.
    if LaunchConfiguration("nav").perform(context).lower() == "true":
        if mode != "localization":
            raise RuntimeError(
                "nav:=true needs mode:=localization. Navigating against a map slam_toolbox is "
                "still building means the goal pose moves under the planner."
            )
        # Passed explicitly as false, and it has to be explicit: an included launch file
        # INHERITS the parent's launch configurations, so nav2.launch.py's own
        # DeclareLaunchArgument default never applies against this file's use_composition.
        # Nav2 must run uncomposed because composition does not deliver the nested costmap
        # parameters; see nav2.launch.py's docstring. The scan and localization nodes above
        # still compose, and still get theirs.
        actions.append(include("nav2.launch.py", use_composition="false"))

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
            description="Start Nav2, the gait shaper and the locomotion authority bracket. "
            "Requires mode:=localization. Off by default: mapping runs need none of it.",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Run the navigation nodes in one component container, each with its "
            "own executor. Set false to get one process per node, which is what you want when "
            "a single node is crashing and you need to see which.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
        OpaqueFunction(function=_setup),
    ])
