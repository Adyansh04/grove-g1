"""move_group on its own: planning, no simulator.

Nothing here is sim-specific, so this file carries to hardware unchanged; moveit_sim.launch.py
composes it with the simulator. Starts whether or not the arm is acquired, since planning needs only
joint states, and nothing here activates a controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

DESCRIPTION_SHARE = get_package_share_directory("g1_description")
CONFIG_SHARE = get_package_share_directory("g1_moveit_config")


def _config(name):
    return os.path.join(CONFIG_SHARE, "config", name)


def _moveit_config():
    """Every path explicit: the builder otherwise guesses names from the robot name and
    silently carries on when one is missing, which surfaces later as an empty planning
    pipeline. sensors_3d is worse, being guarded by an exists() check, so a wrong path is a
    silent no-op with no octomap."""
    return (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        # The same xacro control.launch.py feeds robot_state_publisher, so move_group plans
        # against exactly the model the rest of the stack is running.
        .robot_description(
            file_path=os.path.join(DESCRIPTION_SHARE, "urdf", "g1_lowcmd.urdf.xacro")
        )
        .robot_description_semantic(file_path=_config("g1.srdf"))
        .robot_description_kinematics(file_path=_config("kinematics.yaml"))
        .joint_limits(file_path=_config("joint_limits.yaml"))
        .trajectory_execution(file_path=_config("moveit_controllers.yaml"))
        # Named, or the builder assembles every pipeline it knows and pilz fails the launch
        # wanting a pilz_cartesian_limits.yaml this config does not ship.
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .sensors_3d(file_path=_config("sensors_3d.yaml"))
        .to_moveit_configs()
    )


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                _moveit_config().to_dict(),
                # Puts the SRDF on a topic, so RViz's MotionPlanning display picks it up
                # instead of every consumer being handed the same parameter.
                {"publish_robot_description_semantic": True},
            ],
        ),
    ])
