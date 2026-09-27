"""The world model node, wired to the cameras, their detectors' masks, the map and odometry.

    ros2 launch canopy world_model.launch.py world_dir:=/data/worlds/home rviz:=true
    ros2 launch canopy world_model.launch.py cameras:=head=/camera,chest=/chest_camera \
        odom_topic:=/odom cloud_topic:=/lidar/points detector:=true describe:=true

world_dir keeps the rooms, objects and camera coverage between runs, with the map they were built
on (map.pgm, map.yaml) and a picture of both (semantic_map.png); empty keeps them in memory.

cameras lists name=namespace pairs, a bare name meaning /<name>. Each camera's driver is read
under its namespace as a RealSense driver names it (color/image_raw,
aligned_depth_to_color/image_raw and its camera_info), and its masks on
/detector_<name>/instance_masks: detector:=true starts that detector, or any node of that name
may publish them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SHARE = get_package_share_directory("canopy")


def cameras_of(context):
    """(name, namespace) per camera, from name=namespace entries."""
    pairs = []
    for entry in LaunchConfiguration("cameras").perform(context).split(","):
        name, _, namespace = entry.strip().partition("=")
        if name:
            pairs.append((name, namespace or f"/{name}"))
    return pairs


def nodes(context):
    cameras = cameras_of(context)
    remappings = []
    detectors = []
    for name, namespace in cameras:
        masks = f"/detector_{name}/instance_masks"
        remappings += [
            (f"{name}/depth/image_raw", f"{namespace}/aligned_depth_to_color/image_raw"),
            (f"{name}/depth/camera_info", f"{namespace}/aligned_depth_to_color/camera_info"),
            (f"{name}/color/image_raw", f"{namespace}/color/image_raw"),
            (f"{name}/instance_masks", masks),
        ]
        detectors.append(
            Node(
                package="canopy_perception",
                executable="detector",
                name=f"detector_{name}",
                output="log",
                condition=IfCondition(LaunchConfiguration("detector")),
                parameters=[
                    os.path.join(
                        get_package_share_directory("canopy_perception"), "config", "detector.yaml"
                    )
                ],
                remappings=[("color/image_raw", f"{namespace}/color/image_raw")],
            )
        )
    parameters = [
        os.path.join(SHARE, "config", "canopy.yaml"),
        {
            "world_dir": LaunchConfiguration("world_dir"),
            "describe": ParameterValue(LaunchConfiguration("describe"), value_type=bool),
            "room_types_file": os.path.join(SHARE, "config", "room_types.yaml"),
            "cameras": [name for name, _ in cameras],
        },
    ]
    robot = LaunchConfiguration("params_file").perform(context)
    if robot:
        parameters.append(robot)
    return [
        Node(
            package="canopy",
            executable="world_model",
            name="canopy",
            output="both",
            parameters=parameters,
            remappings=remappings
            + [
                ("map", LaunchConfiguration("map_topic")),
                ("odom", LaunchConfiguration("odom_topic")),
                ("cloud", LaunchConfiguration("cloud_topic")),
                ("descriptions", "/object_describer/descriptions"),
            ],
        ),
        *detectors,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_dir",
                default_value="",
                description="Directory the world is saved to and resumed from; empty for none.",
            ),
            DeclareLaunchArgument(
                "cameras",
                default_value="camera=/camera",
                description="Comma-separated name=namespace cameras; a bare name is /<name>.",
            ),
            DeclareLaunchArgument(
                "detector",
                default_value="false",
                description="Start canopy_perception's detector per camera, against the host "
                "semantic server.",
            ),
            DeclareLaunchArgument(
                "describe",
                default_value="false",
                description="Name objects and type rooms through canopy_perception's describer.",
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/points",
                description="A LiDAR's PointCloud2, for the walls above the furniture.",
            ),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="The robot's own values, loaded over config/canopy.yaml.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open RViz on the map, rooms, objects, camera coverage and the next "
                "viewpoint.",
            ),
            OpaqueFunction(function=nodes),
            Node(
                package="rviz2",
                executable="rviz2",
                name="canopy_rviz",
                arguments=["-d", os.path.join(SHARE, "config", "canopy.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            # Names objects through the host semantic server (servers/semantic_server.py).
            Node(
                package="canopy_perception",
                executable="object_describer",
                name="object_describer",
                output="both",
                condition=IfCondition(LaunchConfiguration("describe")),
                remappings=[("describe_requests", "/canopy/describe_requests")],
            ),
        ]
    )
