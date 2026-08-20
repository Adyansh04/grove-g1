"""Pick and place skills, and the object-pose source they read.

No simulator and no move_group: g1_bringup composes both alongside this. Only the object source
is simulation-specific, and it says so itself: its `hardware` setting refuses to configure.
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

SHARE = get_package_share_directory("g1_manipulation")
DESCRIPTION_SHARE = get_package_share_directory("g1_description")
MOVEIT_SHARE = get_package_share_directory("g1_moveit_config")


def _config(share, name):
    return os.path.join(share, "config", name)


def _moveit_config():
    """MoveGroupInterface needs the description, its semantics and the kinematics solvers in
    its own node's parameters; without them it builds against an empty model and every plan
    fails for a reason that names neither this file nor the missing parameter."""
    return (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        .robot_description(
            file_path=os.path.join(DESCRIPTION_SHARE, "urdf", "g1_lowcmd.urdf.xacro")
        )
        .robot_description_semantic(file_path=_config(MOVEIT_SHARE, "g1.srdf"))
        .robot_description_kinematics(file_path=_config(MOVEIT_SHARE, "kinematics.yaml"))
        .joint_limits(file_path=_config(MOVEIT_SHARE, "joint_limits.yaml"))
        # Named, or the builder assembles every pipeline it knows and pilz fails the launch.
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def _object_source():
    return LifecycleNode(
        package="g1_manipulation",
        executable="g1_object_pose_source",
        name="g1_object_pose_source",
        namespace="",
        output="screen",
        parameters=[
            _config(SHARE, "g1_object_pose_source.yaml"),
            {"object_source": LaunchConfiguration("object_source")},
        ],
        remappings=[
            ("~/object_poses", "/g1_sensor_relay/object_poses"),
            ("~/objects", "/objects"),
            ("~/object_markers", "/object_markers"),
        ],
    )


def _bring_up(node):
    """Launch event handlers rather than a lifecycle manager: there is one node here, and a
    manager would be more moving parts than the thing it manages."""

    def _transition(transition_id):
        return EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node), transition_id=transition_id
            )
        )

    return [
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                goal_state="inactive",
                entities=[_transition(Transition.TRANSITION_ACTIVATE)],
            )
        ),
        _transition(Transition.TRANSITION_CONFIGURE),
    ]


def generate_launch_description():
    object_source = _object_source()

    server = Node(
        package="g1_manipulation",
        executable="g1_manipulation_server",
        name="g1_manipulation_server",
        output="screen",
        parameters=[
            _moveit_config().to_dict(),
            _config(SHARE, "g1_manipulation_server.yaml"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "object_source",
                default_value="sim_ground_truth",
                description="Where object poses come from. 'sim_ground_truth' reads MuJoCo "
                "bodies through g1_sensor_relay; 'hardware' refuses to configure, because no "
                "object-detection pipeline exists yet.",
            ),
            object_source,
        ]
        + _bring_up(object_source)
        + [server]
    )
