"""The grasp skill and the policy engine that feeds it.

Needs move_group and the controllers already running; g1_bringup composes them. The engine is
chosen here, so the server only ever sees one service.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

SHARE = get_package_share_directory("g1_vla")
DESCRIPTION_SHARE = get_package_share_directory("g1_description")
MOVEIT_SHARE = get_package_share_directory("g1_moveit_config")

ENGINE_SERVICE = "/g1_vla_engine/get_action_chunk"


def _config(share, name):
    return os.path.join(share, "config", name)


def _moveit_config():
    """Robot model for the server, which loads it from its own parameters for the groups and
    hand links. joint_limits is this package's own: a policy chunk may move faster than a
    planned motion is timed at."""
    return (
        MoveItConfigsBuilder("g1", package_name="g1_moveit_config")
        .robot_description(
            file_path=os.path.join(DESCRIPTION_SHARE, "urdf", "g1_lowcmd.urdf.xacro")
        )
        .robot_description_semantic(file_path=_config(MOVEIT_SHARE, "g1.srdf.xacro"))
        .joint_limits(file_path=_config(SHARE, "joint_limits.yaml"))
        # Named, or the builder assembles every pipeline it knows and pilz fails the launch.
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def _server():
    return Node(
        package="g1_vla",
        executable="g1_vla_server",
        name="g1_vla_server",
        output="screen",
        parameters=[
            _moveit_config().to_dict(),
            _config(SHARE, "g1_vla_server.yaml"),
            {
                "engine_service": ENGINE_SERVICE,
                "execution_mode": LaunchConfiguration("execution_mode"),
            },
        ],
    )


def _mock_engine():
    return Node(
        package="g1_vla",
        executable="g1_vla_mock_engine",
        name="g1_vla_mock_engine",
        output="screen",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("engine"), "mock")),
        parameters=[_config(SHARE, "g1_vla_mock_engine.yaml")],
        remappings=[("~/get_action_chunk", ENGINE_SERVICE)],
    )


def _groot_engine():
    return Node(
        package="g1_vla",
        executable="g1_vla_groot_adapter",
        name="g1_vla_groot_adapter",
        output="screen",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("engine"), "groot")),
        parameters=[_config(SHARE, "g1_vla_groot_adapter.yaml")],
        remappings=[("~/get_action_chunk", ENGINE_SERVICE)],
    )


def _refuse_other_hands(context, *args, **kwargs):
    """The policy and its adapter speak the Dex3's seven finger joints, and nothing else."""
    hand_model = LaunchConfiguration("hand_model").perform(context)
    if hand_model != "dex3":
        raise RuntimeError(
            f"g1_vla drives the Dex3 hands: GR00T's G1 embodiment and g1_vla's adapter map their "
            f"seven finger joints. hand_model:={hand_model} has no policy here."
        )
    return []


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand_model",
                default_value=EnvironmentVariable("G1_HAND_MODEL", default_value="inspire"),
                choices=["dex3", "inspire"],
                description="The hands the robot wears. Only 'dex3' has a policy; any other is "
                "refused. Defaults to G1_HAND_MODEL, else inspire.",
            ),
            OpaqueFunction(function=_refuse_other_hands),
            DeclareLaunchArgument(
                "execution_mode",
                default_value="trajectory",
                choices=["trajectory", "servo"],
                description="How a validated chunk is executed. 'servo' streams it as jog "
                "commands and needs a servo_node running.",
            ),
            DeclareLaunchArgument(
                "engine",
                default_value="mock",
                choices=["mock", "groot"],
                description="Which policy engine answers the server. 'mock' walks the arm "
                "toward a fixed target and needs no model; 'groot' talks to a policy server "
                "running outside the container.",
            ),
            _server(),
            _mock_engine(),
            _groot_engine(),
        ]
    )
