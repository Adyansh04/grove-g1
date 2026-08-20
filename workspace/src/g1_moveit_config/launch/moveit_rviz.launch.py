"""RViz with the MotionPlanning panel, for dragging the arms around by hand.

Its own launcher rather than g1_bringup's rviz.launch.py because RViz needs the semantic and
kinematics descriptions as parameters, which nothing publishes, and without them the panel
loads with no planning groups. Run alongside a stack that is already up; executing also needs
`ros2 launch g1_bringup activate_arm.launch.py`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

DESCRIPTION_SHARE = get_package_share_directory("g1_description")
CONFIG_SHARE = get_package_share_directory("g1_moveit_config")


def _config(name):
    return os.path.join(CONFIG_SHARE, "config", name)


def _moveit_config():
    """The shared description, not the stack's own: RViz reads links and joints and never the
    ros2_control block, so this stays correct however the hardware side is configured."""
    return (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        .robot_description(file_path=os.path.join(DESCRIPTION_SHARE, "urdf", "g1_common.xacro"))
        .robot_description_semantic(file_path=_config("g1.srdf"))
        .robot_description_kinematics(file_path=_config("kinematics.yaml"))
        .joint_limits(file_path=_config("joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def generate_launch_description():
    moveit_config = _moveit_config()

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=_config("g1_moveit.rviz"),
            description="RViz config to open. The default starts on the both_arms group.",
        ),
        Node(
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
        ),
    ])
