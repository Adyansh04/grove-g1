"""Runs a detector and the node that turns its masks into object poses.

`mock` cuts masks from simulator ground truth and needs no GPU; `vision` asks the host vision
server. Both publish the same mask topic, so the geometry node cannot tell them apart.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("g1_perception")

MASK_TOPIC = "/g1_perception/instance_masks"
COLOR_IMAGE = "/camera/color/image_raw"
COLOR_INFO = "/camera/color/camera_info"
DEPTH_IMAGE = "/camera/aligned_depth_to_color/image_raw"
DEPTH_INFO = "/camera/aligned_depth_to_color/camera_info"
GROUND_TRUTH = "/g1_sensor_relay/object_poses"
TRACKED_MASKS = "/g1_object_geometry/tracked_masks"
OBJECT_POSES = "/g1_object_geometry/object_poses"
# The same name whichever generator answers.
GRASP_SERVICE = "/g1_grasp_engine/generate_grasps"


def _config(name):
    return os.path.join(SHARE, "config", name)


def _nodes(context, *args, **kwargs):
    # Comma separated, since launch arguments are strings.
    phrases = [
        phrase.strip()
        for phrase in LaunchConfiguration("phrases").perform(context).split(",")
        if phrase.strip()
    ]
    detector = LaunchConfiguration("detector")

    mock = Node(
        package="g1_perception",
        executable="g1_mock_detector",
        # The real detector's name, since only one runs: trees write `phrases` on g1_detector.
        name="g1_detector",
        output="screen",
        condition=IfCondition(EqualsSubstitution(detector, "mock")),
        parameters=[
            _config("g1_mock_detector.yaml"),
            {
                "phrases": phrases,
                "mock_latency_s": LaunchConfiguration("mock_latency_s"),
                "mock_rate_hz": LaunchConfiguration("mock_rate_hz"),
                "mock_margin_m": LaunchConfiguration("mock_margin_m"),
            },
        ],
        remappings=[
            ("object_poses", GROUND_TRUTH),
            ("depth/image_raw", DEPTH_IMAGE),
            ("camera_info", COLOR_INFO),
            ("~/instance_masks", MASK_TOPIC),
        ],
    )

    vision = Node(
        package="g1_perception",
        executable="g1_detector",
        name="g1_detector",
        output="screen",
        condition=IfCondition(EqualsSubstitution(detector, "vision")),
        parameters=[_config("g1_detector.yaml"), {"phrases": phrases}],
        remappings=[("color/image_raw", COLOR_IMAGE), ("~/instance_masks", MASK_TOPIC)],
    )

    geometry = Node(
        package="g1_perception",
        executable="g1_object_geometry",
        name="g1_object_geometry",
        output="screen",
        parameters=[_config("g1_object_geometry.yaml")],
        remappings=[
            ("~/instance_masks", MASK_TOPIC),
            ("depth/image_raw", DEPTH_IMAGE),
            ("depth/camera_info", DEPTH_INFO),
        ],
    )
    grounder = Node(
        package="g1_perception",
        executable="g1_instruction_grounder",
        name="g1_instruction_grounder",
        output="screen",
        condition=IfCondition(LaunchConfiguration("grounding")),
        parameters=[_config("g1_instruction_grounder.yaml")],
        remappings=[("color/image_raw", COLOR_IMAGE), ("~/ground", "/ground_instruction")],
    )

    grasp_engine = LaunchConfiguration("grasp_engine")
    mock_grasps = Node(
        package="g1_perception",
        executable="g1_mock_grasp_source",
        name="g1_mock_grasp_source",
        output="screen",
        condition=IfCondition(EqualsSubstitution(grasp_engine, "mock")),
        parameters=[
            _config("g1_mock_grasp_source.yaml"),
            {"only_from_below": LaunchConfiguration("only_from_below")},
        ],
        remappings=[("objects", "/objects"), ("~/generate_grasps", GRASP_SERVICE)],
    )

    graspgen = Node(
        package="g1_perception",
        executable="g1_graspgen_adapter",
        name="g1_graspgen_adapter",
        output="screen",
        condition=IfCondition(EqualsSubstitution(grasp_engine, "graspgen")),
        parameters=[_config("g1_graspgen_adapter.yaml")],
        remappings=[
            ("tracked_masks", TRACKED_MASKS),
            ("depth/image_raw", DEPTH_IMAGE),
            ("depth/camera_info", DEPTH_INFO),
            ("~/generate_grasps", GRASP_SERVICE),
        ],
    )

    visualizer = Node(
        package="g1_perception",
        executable="g1_perception_visualizer",
        name="g1_perception_visualizer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("visualization")),
        # Perception only runs in simulation today, so the relay's ground truth is always there.
        parameters=[_config("g1_perception_visualizer.yaml"), {"ground_truth_topic": GROUND_TRUTH}],
        remappings=[
            ("color/image_raw", COLOR_IMAGE),
            ("color/camera_info", COLOR_INFO),
            ("tracked_masks", TRACKED_MASKS),
            ("object_poses", OBJECT_POSES),
        ],
    )
    return [mock, vision, geometry, grounder, mock_grasps, graspgen, visualizer]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector",
                default_value="mock",
                choices=["mock", "vision"],
                description="mock cuts masks from simulator ground truth and needs no GPU; "
                "vision asks the host server started by scripts/vision_server.py.",
            ),
            DeclareLaunchArgument(
                "phrases",
                default_value="red block,green cylinder,blue sphere,yellow box,white cup",
                description="Comma separated objects to look for. Empty means idle, and "
                "`ros2 param set` on the detector changes it while running.",
            ),
            DeclareLaunchArgument(
                "mock_latency_s",
                default_value="0.0",
                description="How far behind the camera the mock detector's masks are. Set 1.5 "
                "to make the stack face what the real detector costs.",
            ),
            DeclareLaunchArgument(
                "mock_rate_hz",
                default_value="10.0",
                description="How often the mock detector answers. The real one manages 0.7.",
            ),
            DeclareLaunchArgument(
                "mock_margin_m",
                default_value="0.005",
                description="How far past an object's own box the mock's mask may spill. "
                "Positive simulates a sloppy segmenter.",
            ),
            DeclareLaunchArgument(
                "grounding",
                default_value="false",
                description="Runs the instruction grounder, which turns a sentence into the "
                "phrases the detector looks for. Needs the vision server started with --vlm.",
            ),
            DeclareLaunchArgument(
                "only_from_below",
                default_value="false",
                description="Makes the stand-in generator offer nothing but the grasp reaching "
                "up through the table, so a filter that accepts everything is visible.",
            ),
            DeclareLaunchArgument(
                "grasp_engine",
                default_value="none",
                choices=["none", "mock", "graspgen"],
                description="Who answers for six-degree-of-freedom grasps: nobody, a stand-in "
                "that needs no GPU, or the GraspGenX server on the host.",
            ),
            DeclareLaunchArgument(
                "visualization",
                default_value="false",
                description="Runs g1_perception_visualizer, which draws the masks and boxes on "
                "the camera image and ground truth for RViz. false starts nothing.",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
