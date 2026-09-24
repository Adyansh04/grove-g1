"""RViz on a caller-supplied config; the caller knows which mode is running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            description="Absolute path to an .rviz file. Required.",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="rviz2",
            description="Node name. A second window next to MoveIt's needs its own.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name=LaunchConfiguration("node_name"),
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
