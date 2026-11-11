"""The robot's own sensors, on its Orin: the head D435i and the Mid360's odometry chain.

Runs beside control.launch.py hardware:=true, whose robot_state_publisher and IMU broadcaster the
odometry needs:

    ros2 launch g1_bringup robot_sensors.launch.py
    ros2 launch g1_bringup robot_sensors.launch.py lidar:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

BRINGUP_SHARE = get_package_share_directory("g1_bringup")
STATE_ESTIMATION_SHARE = get_package_share_directory("g1_state_estimation")


def generate_launch_description():
    # A container, so consumers of full-size frames can join the camera's process later.
    camera = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="realsense2_camera",
                plugin="realsense2_camera::RealSenseNodeFactory",
                name="camera",
                namespace="",
                parameters=[os.path.join(BRINGUP_SHARE, "config", "robot_camera.yaml")],
            )
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("camera")),
    )
    # The hardware front end: our Livox driver points the Mid360 at this machine.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(STATE_ESTIMATION_SHARE, "launch", "fastlio_odometry.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("lidar")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera",
                default_value="true",
                choices=["true", "false"],
                description="Start the head D435i (config/robot_camera.yaml).",
            ),
            DeclareLaunchArgument(
                "lidar",
                default_value="true",
                choices=["true", "false"],
                description="Start the Mid360 driver, FAST-LIO and the odometry publisher.",
            ),
            camera,
            lidar,
        ]
    )
