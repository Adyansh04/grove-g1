"""Runs a behavior tree against a stack that is already up.

Deliberately starts nothing else. The mission needs the simulator, Nav2, MoveIt and the
manipulation skills already running, and staging any of them here would put a second writer on
a low-level channel the moment someone ran both. `g1_bringup`'s entry point composes them:

    ros2 launch g1_bringup bringup.launch.py \\
        mode:=localization nav:=true moveit:=true manipulation:=true
    ros2 launch g1_orchestration mission.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    trees_dir = os.path.join(get_package_share_directory("g1_orchestration"), "trees")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tree",
                default_value="pick_and_place.xml",
                description="Which tree in g1_orchestration/trees to run.",
            ),
            DeclareLaunchArgument(
                "groot2_port",
                default_value="1667",
                description=(
                    "ZeroMQ port Groot2 connects to for live monitoring, or 0 to disable. "
                    "The container uses host networking, so the editor reaches it on "
                    "localhost. The free tier monitors at most 20 nodes."
                ),
            ),
            DeclareLaunchArgument(
                "tick_rate_hz",
                default_value="10.0",
                description="How often the tree is ticked.",
            ),
            Node(
                package="g1_orchestration",
                executable="g1_bt_executor",
                name="g1_bt_executor",
                output="screen",
                parameters=[
                    {
                        "tree_file": PathJoinSubstitution(
                            [trees_dir, LaunchConfiguration("tree")]
                        ),
                        "groot2_port": LaunchConfiguration("groot2_port"),
                        "tick_rate_hz": LaunchConfiguration("tick_rate_hz"),
                    }
                ],
            ),
        ]
    )
