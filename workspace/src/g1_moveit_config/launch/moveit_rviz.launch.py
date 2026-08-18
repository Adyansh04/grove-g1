"""RViz with the MotionPlanning panel, for dragging the arms around by hand.

A dedicated launcher rather than g1_bringup's rviz.launch.py, which takes only an rviz_config
and passes no parameters. RViz resolves robot_description from robot_state_publisher's latched
topic, but robot_description_semantic and robot_description_kinematics have no publisher
anywhere -- move_group holds them as its own parameters. Without them the MotionPlanning panel
loads and then sits there with no planning groups, which reads as a broken install rather than
as missing configuration. So this file hands RViz the same config move_group got.

Run it alongside a stack that is already up:

    ros2 launch g1_moveit_config moveit_sim.launch.py pin_pelvis:=true headless:=false
    ros2 launch g1_moveit_config moveit_rviz.launch.py

Planning works immediately. Executing needs the arm acquired first, which is a separate,
deliberate step:

    ros2 launch g1_bringup activate_arm.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("g1_description")
    config_share = get_package_share_directory("g1_moveit_config")

    # Same builder call as move_group.launch.py. RViz needs the semantic and kinematics
    # descriptions; it does not need the controller or planning-pipeline configuration.
    #
    # The shared description rather than the stack's own: RViz reads links and joints and never
    # the ros2_control block, so this stays correct however the hardware side is configured.
    moveit_config = (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        .robot_description(file_path=os.path.join(description_share, "urdf", "g1_common.xacro"))
        .robot_description_semantic(file_path=os.path.join(config_share, "config", "g1.srdf"))
        .robot_description_kinematics(
            file_path=os.path.join(config_share, "config", "kinematics.yaml")
        )
        .joint_limits(file_path=os.path.join(config_share, "config", "joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(config_share, "config", "g1_moveit.rviz"),
            description="RViz config to open. The default starts on the both_arms group.",
        ),
        rviz_node,
    ])
