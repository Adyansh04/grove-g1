"""Standalone wrapper: the simulator, the navigation stack, and optionally RViz.

One command for running navigation on its own, which is what the integration suites launch and
what a nav-only debugging session wants. The stack itself lives in nav_stack.launch.py; this
file only stages a simulator under it and attaches a window.

Includes g1_bringup rather than the other way round. Navigation sits above bring-up, so the
higher layer composes the lower one; a nav:=true argument on sim.launch.py would give
g1_bringup a dependency on Nav2 and slam_toolbox.

g1_bringup's bringup.launch.py composes the same two pieces for the operator entry point. That
leaves exactly one fact stated twice -- sensors:=true on the simulator -- because each file
stages its own simulator and there is nowhere shared to put it. test_launch_threading asserts
it on both.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    share = get_package_share_directory("g1_navigation")
    launch_dir = os.path.join(share, "launch")
    # Resolved BEFORE the sim include below, which sets rviz=false for its own scope and leaks
    # that back here -- without capturing it first, asking for rviz:=true silently gets you no
    # RViz at all. Same launch-configuration inheritance that bites use_composition.
    want_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"

    actions = [
        # sensors:=true is not optional: it gates the LiDAR sweep, the relay, the
        # odom -> base_footprint -> pelvis chain and the waist joint states. Passed explicitly
        # rather than by flipping the bringup default, which is still provisional on an
        # unthrottled re-measurement of test_arm_command.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("g1_bringup"), "launch", "sim.launch.py"
                )
            ),
            launch_arguments={
                "sensors": "true",
                "world": LaunchConfiguration("world"),
                "headless": LaunchConfiguration("headless"),
                "sim_start_delay_s": LaunchConfiguration("sim_start_delay_s"),
                # Explicitly false, and it has to be. An included launch file inherits the
                # parent's configurations, so without this sim.launch.py sees this file's
                # rviz:=true and opens a SECOND RViz on the sensor config -- two windows, and
                # the sensor one carries a RobotModel display that cannot render.
                "rviz": "false",
            }.items(),
        ),
        # Every argument forwarded explicitly, including the ones whose values match this
        # file's own defaults: the child's DeclareLaunchArgument default never fires for a name
        # this file also declares.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav_stack.launch.py")),
            launch_arguments={
                "mode": LaunchConfiguration("mode"),
                "nav": LaunchConfiguration("nav"),
                "use_composition": LaunchConfiguration("use_composition"),
                "container_name": LaunchConfiguration("container_name"),
            }.items(),
        ),
    ]

    if want_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", os.path.join(share, "config", "g1_navigation.rviz")],
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
            "world",
            default_value="navigation",
            description="Which g1_bringup scene to stage. 'navigation' is the facility the "
            "committed map was built from -- localization against any other world will not "
            "converge.",
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
            description="Run the navigation nodes in one component container, each with its "
            "own executor. Set false to get one process per node, which is what you want when "
            "a single node is crashing and you need to see which.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
        DeclareLaunchArgument(
            "nav",
            default_value="false",
            description="Start Nav2, the gait shaper and the locomotion authority bracket. "
            "Requires mode:=localization. Off by default: mapping runs need none of it.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="false shows the MuJoCo viewer. Its Reload button is fatal with "
            "sensors on; see g1_bringup's README.",
        ),
        DeclareLaunchArgument(
            "sim_start_delay_s",
            default_value="4.0",
            description="Longer than g1_bringup's own default. Navigation starts more nodes "
            "before the first physics tick, and the robot topples at spawn if discovery is "
            "still in progress.",
        ),
        OpaqueFunction(function=_setup),
    ])
