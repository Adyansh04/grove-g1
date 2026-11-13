"""RViz with the MotionPlanning panel, for dragging the arms around by hand.

Its own launcher rather than g1_bringup's rviz.launch.py because the panel needs MoveIt's
kinematics and planning parameters on its own node. Run alongside a stack that is already up;
executing also needs `ros2 launch g1_bringup activate_arm.launch.py`.

    ros2 launch g1_moveit_config moveit_rviz.launch.py hand_model:=inspire
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

DESCRIPTION_SHARE = get_package_share_directory("g1_description")
CONFIG_SHARE = get_package_share_directory("g1_moveit_config")


def _config(name):
    return os.path.join(CONFIG_SHARE, "config", name)


def _moveit_config(hand_model):
    """The shared description, not the stack's own: RViz reads links and joints and never the
    ros2_control block, so this stays correct however the hardware side is configured."""
    mappings = {"hand_model": hand_model}
    return (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        .robot_description(
            file_path=os.path.join(DESCRIPTION_SHARE, "urdf", "g1_common.xacro"),
            mappings=mappings,
        )
        .robot_description_semantic(file_path=_config("g1.srdf.xacro"), mappings=mappings)
        .robot_description_kinematics(file_path=_config("kinematics.yaml"))
        .joint_limits(file_path=_config("joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def _launch_setup(context, *args, **kwargs):
    moveit_config = _moveit_config(LaunchConfiguration("hand_model").perform(context))
    return [
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
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=_config("g1_moveit.rviz"),
                description="RViz config to open. The default starts on the both_arms group.",
            ),
            DeclareLaunchArgument(
                "hand_model",
                default_value=EnvironmentVariable("G1_HAND_MODEL", default_value="inspire"),
                choices=["dex3", "inspire"],
                description="The hands the robot wears, 'dex3' or 'inspire', as move_group was "
                "started with. Defaults to G1_HAND_MODEL, else inspire.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
