"""canopy's world model on the G1: its cameras, odometry and LiDAR topics.

    ros2 launch g1_bringup world_model.launch.py world_dir:=/root/data/worlds/apartment rviz:=true
    ros2 launch g1_bringup world_model.launch.py cameras:=head,chest detector:=true describe:=true
    ros2 launch g1_bringup world_model.launch.py cameras:=head,chest detector:=mock
    ros2 launch g1_bringup world_model.launch.py cameras:=head,chest segmenter:=true

cameras takes bringup's names: head is the RealSense driver's /camera, any other name
/<name>_camera, as the relay publishes them. detector:=true asks canopy's semantic server on the
host; detector:=mock cuts the masks from the simulator's ground truth instead, for every label in
worlds/<world>.truth.yaml. segmenter:=true adds canopy's segmenter, a service that segments one
camera's newest frame by a text prompt against the vision server on the host.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("g1_bringup")


def namespace_of(camera):
    return "/camera" if camera == "head" else f"/{camera}_camera"


def mock_detectors(cameras, world):
    """A mock per camera, named as canopy names that camera's detector so canopy reads its masks."""
    with open(os.path.join(SHARE, "worlds", f"{world}.truth.yaml")) as handle:
        truth = yaml.safe_load(handle)
    # The labels are the bodies' class names, which is what the mock matches them against.
    phrases = sorted({item["label"] for item in truth["objects"]})
    return [
        Node(
            package="canopy_perception",
            executable="mock_detector",
            name=f"detector_{camera}",
            output="log",
            parameters=[
                os.path.join(
                    get_package_share_directory("canopy_perception"), "config", "mock_detector.yaml"
                ),
                os.path.join(SHARE, "config", "mock_detector.yaml"),
                {"phrases": phrases},
            ],
            remappings=[
                (
                    "object_poses",
                    "/g1_sensor_relay/object_poses"
                    if camera == "head"
                    else f"/g1_sensor_relay/{camera}/object_poses",
                ),
                ("depth/image_raw", f"{namespace_of(camera)}/aligned_depth_to_color/image_raw"),
                ("camera_info", f"{namespace_of(camera)}/color/camera_info"),
            ],
        )
        for camera in cameras
    ]


def canopy(context):
    cameras = [
        name.strip()
        for name in LaunchConfiguration("cameras").perform(context).split(",")
        if name.strip()
    ]
    # Read before the include: its arguments leak into this file's configurations, and it hands
    # canopy rviz:=false, and detector:=false for the mock.
    rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")
    detector = LaunchConfiguration("detector").perform(context)
    mock = detector.lower() == "mock"
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("canopy"), "launch", "world_model.launch.py"
                )
            ),
            launch_arguments={
                "cameras": ",".join(f"{name}={namespace_of(name)}" for name in cameras),
                "odom_topic": "/g1_odometry_publisher/odom",
                "cloud_topic": "/livox/lidar",
                # Always the G1's own: an including launch's params_file, Nav2's say, would
                # otherwise reach canopy under the same name.
                "params_file": os.path.join(SHARE, "config", "world_model.yaml"),
                "world_dir": LaunchConfiguration("world_dir"),
                # canopy's detector is the real one; this package starts the mock.
                "detector": "false" if mock else detector,
                "describe": LaunchConfiguration("describe"),
                # canopy's own view has no panel for the chest camera; this package opens one
                # that does.
                "rviz": "false",
            }.items(),
        )
    ]
    if mock:
        actions += mock_detectors(cameras, LaunchConfiguration("world").perform(context))
    if LaunchConfiguration("segmenter").perform(context).lower() in ("true", "1"):
        actions.append(
            Node(
                package="canopy_perception",
                executable="segmenter",
                name="segmenter",
                output="log",
                parameters=[
                    os.path.join(SHARE, "config", "segmenter.yaml"),
                    {
                        "cameras": [
                            f"{name}={namespace_of(name)}/color/image_raw" for name in cameras
                        ]
                    },
                ],
            )
        )
    if rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="world_model_rviz",
                arguments=["-d", os.path.join(SHARE, "config", "world_model.rviz")],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_dir", default_value=""),
            DeclareLaunchArgument(
                "cameras",
                default_value="head",
                description="Comma-separated cameras to read: head, chest.",
            ),
            DeclareLaunchArgument(
                "detector",
                default_value="false",
                description="true: canopy's detector against the host semantic server; mock: "
                "masks from the simulator's ground truth.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="apartment",
                description="The world whose worlds/<world>.truth.yaml names what the mock finds.",
            ),
            DeclareLaunchArgument("describe", default_value="false"),
            DeclareLaunchArgument(
                "segmenter",
                default_value="false",
                description="true: canopy's segmenter, text-prompted masks on request.",
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=canopy),
        ]
    )
