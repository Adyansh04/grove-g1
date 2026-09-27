"""canopy's world model on the G1: its cameras, odometry and LiDAR topics.

    ros2 launch g1_bringup world_model.launch.py world_dir:=/root/data/worlds/apartment rviz:=true
    ros2 launch g1_bringup world_model.launch.py cameras:=head,chest detector:=true describe:=true

cameras takes bringup's names: head is the RealSense driver's /camera, any other name
/<name>_camera, as the relay publishes them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def canopy(context):
    cameras = [
        name.strip()
        for name in LaunchConfiguration("cameras").perform(context).split(",")
        if name.strip()
    ]
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("canopy"), "launch", "world_model.launch.py"
                )
            ),
            launch_arguments={
                "cameras": ",".join(
                    f"{name}=/camera" if name == "head" else f"{name}=/{name}_camera"
                    for name in cameras
                ),
                "odom_topic": "/g1_odometry_publisher/odom",
                "cloud_topic": "/livox/lidar",
                "world_dir": LaunchConfiguration("world_dir"),
                "detector": LaunchConfiguration("detector"),
                "describe": LaunchConfiguration("describe"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_dir", default_value=""),
            DeclareLaunchArgument(
                "cameras",
                default_value="head",
                description="Comma-separated cameras to read: head, chest.",
            ),
            DeclareLaunchArgument("detector", default_value="false"),
            DeclareLaunchArgument("describe", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=canopy),
        ]
    )
