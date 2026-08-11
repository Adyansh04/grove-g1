"""Pick and place skills, and the object-pose source they read.

No simulator and no move_group: both are composed alongside this by g1_bringup, the same way
nav_stack.launch.py and move_group.launch.py are. Only the object source is simulation-specific,
and it says so itself -- its `hardware` default refuses to configure.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    manipulation_share = get_package_share_directory("g1_manipulation")
    description_share = get_package_share_directory("g1_description")
    moveit_share = get_package_share_directory("g1_moveit_config")

    # The same assembly move_group.launch.py uses. MoveGroupInterface needs the robot
    # description, its semantics and the kinematics solvers in its own node's parameters --
    # without them it constructs against an empty model and every plan fails for a reason
    # that names neither this file nor the missing parameter.
    moveit_config = (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "g1_arm_sdk.urdf.xacro")
        )
        .robot_description_semantic(file_path=os.path.join(moveit_share, "config", "g1.srdf"))
        .robot_description_kinematics(
            file_path=os.path.join(moveit_share, "config", "kinematics.yaml")
        )
        .joint_limits(file_path=os.path.join(moveit_share, "config", "joint_limits.yaml"))
        # Named explicitly for the same reason move_group.launch.py names it: left to itself
        # the builder assembles every pipeline it knows, and pilz wants a
        # pilz_cartesian_limits.yaml this config does not ship, which fails the launch.
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    object_source = LifecycleNode(
        package="g1_manipulation",
        executable="g1_object_pose_source",
        name="g1_object_pose_source",
        namespace="",
        output="screen",
        parameters=[
            os.path.join(manipulation_share, "config", "g1_object_pose_source.yaml"),
            {"object_source": LaunchConfiguration("object_source")},
        ],
        remappings=[
            ("~/object_poses", "/g1_sensor_relay/object_poses"),
            ("~/objects", "/objects"),
            ("~/object_markers", "/object_markers"),
        ],
    )

    manipulation_server = Node(
        package="g1_manipulation",
        executable="g1_manipulation_server",
        name="g1_manipulation_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            os.path.join(manipulation_share, "config", "g1_manipulation_server.yaml"),
        ],
    )

    # Driven by launch event handlers rather than a lifecycle manager, matching how
    # g1_navigation drives g1_loco_authority: there is one node here, and adding a manager
    # for it would be more moving parts than the thing it manages.
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(object_source),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=object_source,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(object_source),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "object_source",
                default_value="sim_ground_truth",
                description=(
                    "Where object poses come from. 'sim_ground_truth' reads MuJoCo bodies "
                    "through g1_sensor_relay; 'hardware' refuses to configure, because no "
                    "object-detection pipeline exists yet."
                ),
            ),
            object_source,
            activate_on_configured,
            configure,
            manipulation_server,
        ]
    )
