"""Starts RViz on a caller-supplied config.

Deliberately knows nothing about navigation. The caller has already decided which mode it
is in, so it also knows which config file to hand over; duplicating that conditional here
would give this file a reason to resolve g1_navigation's share directory, which is exactly
what bringup.launch.py exists to keep in one place.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            description="Absolute path to an .rviz file. Required -- there is no default, "
            "because the right one depends on whether navigation is running and this file "
            "has no way to know that.",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="rviz2",
            # Overridable because a mission run shows two windows at once, and MoveIt's own
            # launcher already claims 'rviz2'. Two nodes under one name is a graph the tooling
            # cannot address unambiguously.
            description="Node name. Give the second window its own so it does not collide "
            "with MoveIt's.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name=LaunchConfiguration("node_name"),
            output="log",
            arguments=["-d", rviz_config],
        ),
    ])
