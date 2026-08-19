"""Standalone wrapper: the simulator, the navigation stack, and optionally RViz.

One command for running navigation on its own, and what the integration suites launch. The
stack itself is nav_stack.launch.py; this file only stages a simulator under it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("g1_navigation")
BRINGUP_LAUNCH = os.path.join(
    get_package_share_directory("g1_bringup"), "launch", "sim.launch.py"
)
NAV_STACK_LAUNCH = os.path.join(SHARE, "launch", "nav_stack.launch.py")


def _include(path, **launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=launch_args.items()
    )


def _setup(context, *args, **kwargs):
    # Resolved BEFORE the sim include, which sets rviz=false for its own scope and leaks that
    # back here. Without capturing it first, rviz:=true silently gets you no RViz at all.
    want_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"

    actions = [
        # sensors:=true is not optional: it gates the LiDAR sweep, the relay, the
        # odom -> base_footprint -> pelvis chain and the waist joint states.
        # rviz stays false, or sim.launch.py opens a second window on the sensor config.
        _include(
            BRINGUP_LAUNCH,
            sensors="true",
            rviz="false",
            odometry=LaunchConfiguration("odometry"),
            world=LaunchConfiguration("world"),
            headless=LaunchConfiguration("headless"),
            sim_start_delay_s=LaunchConfiguration("sim_start_delay_s"),
        ),
        # Every argument forwarded explicitly, including ones matching this file's own
        # defaults: the child's default never fires for a name the parent also declares.
        _include(
            NAV_STACK_LAUNCH,
            mode=LaunchConfiguration("mode"),
            nav=LaunchConfiguration("nav"),
            use_composition=LaunchConfiguration("use_composition"),
            container_name=LaunchConfiguration("container_name"),
        ),
    ]

    if want_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", os.path.join(SHARE, "config", "g1_navigation.rviz")],
            )
        )
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
            "mode:=localization.",
        ),
        DeclareLaunchArgument(
            "odometry",
            default_value="fast_lio",
            description="Forwarded to sim.launch.py. 'ground_truth' isolates a fault to "
            "'not the odometry'.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="navigation",
            description="Which g1_bringup scene to stage. The committed map was built from "
            "'navigation'; localization against any other world will not converge.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open config/g1_navigation.rviz. Its fixed frame is map, so nothing "
            "renders until slam_toolbox or AMCL has published map -> odom.",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Run the navigation nodes in one component container. Set false for "
            "one process per node when a single node is crashing.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="false shows the MuJoCo viewer. Its Reload button is fatal with "
            "sensors on; see g1_bringup's README.",
        ),
        DeclareLaunchArgument(
            "sim_start_delay_s",
            default_value="4.0",
            description="Longer than g1_bringup's own default: navigation starts more nodes "
            "before the first physics tick, and the robot topples at spawn if discovery is "
            "still in progress.",
        ),
        OpaqueFunction(function=_setup),
    ])
