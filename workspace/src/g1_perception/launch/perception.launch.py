"""Runs a detector and the node that turns its masks into object poses.

`mock` cuts masks from simulator ground truth and needs no GPU; `vision` segments with SAM 3.1 in
TensorRT, sam3_inference's node, which it starts; `server` asks a host model server instead, such
as Grounded SAM 2. All publish the same mask topic, so the geometry node cannot tell them apart.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, EqualsSubstitution, LaunchConfiguration
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
    asks = detector.perform(context)
    hand_model = LaunchConfiguration("hand_model").perform(context)

    mock = Node(
        package="canopy_perception",
        executable="mock_detector",
        # The real detector's name, since only one runs: trees write `phrases` on g1_detector.
        name="g1_detector",
        output="screen",
        condition=IfCondition(EqualsSubstitution(detector, "mock")),
        parameters=[
            os.path.join(
                get_package_share_directory("canopy_perception"), "config", "mock_detector.yaml"
            ),
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

    # The segmentation service g1_detector.yaml's segment_service names.
    sam3 = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sam3_inference"), "launch", "sam3_segmentation.launch.py"
        ),
        condition=IfCondition(EqualsSubstitution(detector, "vision")),
    )
    vision = Node(
        package="canopy_perception",
        executable="detector",
        name="g1_detector",
        output="screen",
        condition=IfCondition("true" if asks in ("vision", "server") else "false"),
        parameters=[
            _config("g1_detector.yaml"),
            # A host server segments in place of sam3_inference's service.
            {"phrases": phrases, **({"segment_service": ""} if asks == "server" else {})},
        ],
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
        parameters=[
            _config("g1_graspgen_adapter.yaml"),
            # The hand's own description where it is not the Dex3, whose is the base file's.
            *(
                [_config(f"g1_graspgen_adapter_{hand_model}.yaml")]
                if os.path.isfile(_config(f"g1_graspgen_adapter_{hand_model}.yaml"))
                else []
            ),
        ],
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
    return [mock, sam3, vision, geometry, grounder, mock_grasps, graspgen, visualizer]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector",
                default_value="mock",
                choices=["mock", "vision", "server"],
                description="mock cuts masks from simulator ground truth and needs no GPU; "
                "vision segments with SAM 3.1 in TensorRT (sam3_inference), which needs the "
                "ONNX graphs in data/models/sam3; server asks a host model server on port "
                "5561, such as Grounded SAM 2.",
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
            DeclareLaunchArgument(
                "hand_model",
                default_value=EnvironmentVariable("G1_HAND_MODEL", default_value="inspire"),
                choices=["dex3", "inspire"],
                description="The hands, 'dex3' or 'inspire': which gripper description the grasp "
                "generator is asked for. Defaults to G1_HAND_MODEL, else inspire.",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
