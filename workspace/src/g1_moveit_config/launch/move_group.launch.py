"""move_group on its own: planning, no simulator.

The control.launch.py analogue for manipulation -- nothing here is sim-specific, so this file is
what carries over to hardware unchanged. moveit_sim.launch.py composes it with the simulator.

move_group starts whether or not the arm is active. Planning only needs joint states, which
joint_state_broadcaster publishes from the moment bring-up runs; executing needs the hardware
component and the controller, which g1_bringup/scripts/activate_arm acquires in that order.
Nothing in this file activates anything, and moveit_controllers.yaml sets
moveit_manage_controllers: false so MoveIt will not either.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Every path is passed explicitly. MoveItConfigsBuilder will otherwise guess file names from
    # the robot name and silently carry on when one is missing, which surfaces much later as an
    # empty planning pipeline rather than as an error here.
    description_share = get_package_share_directory("g1_description")
    config_share = get_package_share_directory("g1_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        # The same xacro control.launch.py feeds robot_state_publisher, so move_group plans
        # against exactly the model the rest of the stack is running.
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "g1_arm_sdk.urdf.xacro")
        )
        .robot_description_semantic(file_path=os.path.join(config_share, "config", "g1.srdf"))
        .robot_description_kinematics(
            file_path=os.path.join(config_share, "config", "kinematics.yaml")
        )
        .joint_limits(file_path=os.path.join(config_share, "config", "joint_limits.yaml"))
        .trajectory_execution(
            file_path=os.path.join(config_share, "config", "moveit_controllers.yaml")
        )
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # Publishes the SRDF on a topic, so RViz's MotionPlanning display picks it up
            # instead of every consumer having to be handed the same parameter.
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription([move_group_node])
