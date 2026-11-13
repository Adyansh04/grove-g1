"""move_group on its own: planning, no simulator.

Nothing here is sim-specific, so it carries to hardware unchanged. Planning needs only joint
states, and nothing here activates a controller.

    ros2 launch g1_moveit_config move_group.launch.py hand_model:=inspire
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

DESCRIPTION_SHARE = get_package_share_directory("g1_description")
CONFIG_SHARE = get_package_share_directory("g1_moveit_config")

# Both arms: matches arm_trajectory_controller's 14 joints, and servo.yaml's move_group_name.
SERVO_GROUP = "both_arms"


def _config(name):
    return os.path.join(CONFIG_SHARE, "config", name)


def _model(builder, hand_model):
    """The description and its semantics for the hands the robot wears, the same xacro
    control.launch.py feeds robot_state_publisher, so move_group plans against exactly the model
    the rest of the stack is running."""
    mappings = {"hand_model": hand_model}
    return builder.robot_description(
        file_path=os.path.join(DESCRIPTION_SHARE, "urdf", "g1_lowcmd.urdf.xacro"),
        mappings=mappings,
    ).robot_description_semantic(file_path=_config("g1.srdf.xacro"), mappings=mappings)


def _hand_controllers(config, hand_model):
    """Lays the hand controllers of a hand model other than the Dex3 over moveit_controllers.yaml,
    whose own are the Dex3's."""
    overlay = _config(f"moveit_hand_controllers_{hand_model}.yaml")
    if not os.path.isfile(overlay):
        return config
    with open(overlay, encoding="utf-8") as handle:
        hands = yaml.safe_load(handle)["moveit_simple_controller_manager"]
    config.trajectory_execution["moveit_simple_controller_manager"].update(hands)
    return config


def _moveit_config(hand_model):
    """Every path explicit: the builder otherwise guesses file names and silently skips a missing
    one, and a wrong sensors_3d path is a silent no-op with no octomap."""
    builder = _model(MoveItConfigsBuilder("g1", package_name="g1_moveit_config"), hand_model)
    return _hand_controllers(
        builder.robot_description_kinematics(file_path=_config("kinematics.yaml"))
        .joint_limits(file_path=_config("joint_limits.yaml"))
        .trajectory_execution(file_path=_config("moveit_controllers.yaml"))
        # Named, or the builder assembles every pipeline it knows and pilz fails the launch
        # wanting a pilz_cartesian_limits.yaml this config does not ship.
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .sensors_3d(file_path=_config("sensors_3d.yaml"))
        .to_moveit_configs(),
        hand_model,
    )


def _servo_config(hand_model):
    """Without sensors_3d: servo would start a second octomap updater on the same sensor, one that
    can disagree with move_group's. It reads move_group's scene over /monitored_planning_scene."""
    builder = _model(MoveItConfigsBuilder("g1", package_name="g1_moveit_config"), hand_model)
    return (
        builder.robot_description_kinematics(file_path=_config("kinematics.yaml"))
        .joint_limits(file_path=_config("joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def _servo(hand_model):
    """Off unless asked for. Servo streams onto the same controller topic move_group executes
    through, so running it alongside a planned motion would put two writers on one controller."""
    return Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("servo")),
        parameters=[
            _servo_config(hand_model).to_dict(),
            # Servo reads its own settings from under this key, not from the node root.
            {
                "moveit_servo": ParameterBuilder("g1_moveit_config")
                .yaml("config/servo.yaml")
                .to_dict()
            },
            # The acceleration-limiting smoother is a separate plugin and takes these at the
            # node root; without them it fails to configure and servo starts with no smoothing.
            {"update_period": 0.01, "planning_group_name": SERVO_GROUP},
        ],
    )


def _launch_setup(context, *args, **kwargs):
    hand_model = LaunchConfiguration("hand_model").perform(context)
    return [
        # No `name=`: it would remap every node in the process, and move_group's own helper
        # nodes would all become move_group. The executable names itself.
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                _moveit_config(hand_model).to_dict(),
                # Puts the SRDF on a topic, so RViz's MotionPlanning display picks it up
                # instead of every consumer being handed the same parameter.
                {"publish_robot_description_semantic": True},
                # Off by default, and servo's secondary scene monitor gets its world and robot
                # state from this topic alone; without it servo silently never moves.
                {"publish_planning_scene": True},
                {"publish_geometry_updates": True},
                {"publish_state_updates": True},
                {"publish_transforms_updates": True},
            ],
        ),
        _servo(hand_model),
    ]


def _hand_model_argument():
    """The hands the robot wears, by default from G1_HAND_MODEL, which robot.sh sets on the robot."""
    return DeclareLaunchArgument(
        "hand_model",
        default_value=EnvironmentVariable("G1_HAND_MODEL", default_value="inspire"),
        choices=["dex3", "inspire"],
        description="The hands the robot wears: 'dex3', Unitree's Dex3-1, or 'inspire', "
        "Inspire's RH56DFQ. Picks the description, the SRDF's hand groups and the hand "
        "controllers. Defaults to G1_HAND_MODEL, else inspire.",
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo",
                default_value="false",
                description="Also start MoveIt Servo, the streaming command path for the arms. "
                "Off by default: it and a planned motion would both drive the same controller.",
            ),
            _hand_model_argument(),
            OpaqueFunction(function=_launch_setup),
        ]
    )
