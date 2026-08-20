"""RViz on a caller-supplied config.

Knows nothing about navigation: the caller picks the config, so only the caller needs to know
which mode it is in.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            description="Absolute path to an .rviz file. Required: the right one depends on "
            "whether navigation is running, which this file cannot know.",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="rviz2",
            description="Node name. A second window needs its own, so it does not collide "
            "with MoveIt's.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name=LaunchConfiguration("node_name"),
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
