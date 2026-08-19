"""Runs a behavior tree against a stack that is already up.

Starts nothing else on purpose: staging the sim, Nav2 or MoveIt here would put a second writer
on a low-level channel the moment someone ran both. Compose them with g1_bringup instead.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

SHARE = get_package_share_directory("g1_orchestration")
PARAMS_FILE = os.path.join(SHARE, "config", "g1_bt_executor.yaml")
TREES_DIR = os.path.join(SHARE, "trees")


def _defaults():
    """Argument defaults come from the config file, so it stays the one place to edit."""
    with open(PARAMS_FILE) as params:
        return yaml.safe_load(params)["g1_bt_executor"]["ros__parameters"]


def generate_launch_description():
    defaults = _defaults()

    arguments = [
        DeclareLaunchArgument(
            "tree",
            default_value="pick_and_place.xml",
            description="Which tree in g1_orchestration/trees to run.",
        ),
        DeclareLaunchArgument(
            "tick_rate_hz",
            default_value=str(defaults["tick_rate_hz"]),
            description="How often the tree is ticked.",
        ),
        DeclareLaunchArgument(
            "groot2_port",
            default_value=str(defaults["groot2_port"]),
            description="ZeroMQ port for Groot2 monitoring, or 0 to disable.",
        ),
    ]

    executor = Node(
        package="g1_orchestration",
        executable="g1_bt_executor",
        name="g1_bt_executor",
        output="screen",
        parameters=[
            PARAMS_FILE,
            {
                # Built from the `tree` argument, so it has no place in the config file.
                "tree_file": PathJoinSubstitution([TREES_DIR, LaunchConfiguration("tree")]),
                "tick_rate_hz": LaunchConfiguration("tick_rate_hz"),
                "groot2_port": LaunchConfiguration("groot2_port"),
            },
        ],
    )

    return LaunchDescription(arguments + [executor])
