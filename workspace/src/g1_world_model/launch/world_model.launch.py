"""The world model node, wired to the cameras, the detector's masks, the map and odometry.

    ros2 launch g1_world_model world_model.launch.py world_dir:=/root/data/worlds/apartment rviz:=true
    ros2 launch g1_world_model world_model.launch.py cameras:=head,chest

world_dir keeps the rooms, objects and camera coverage between runs, with the map they were built
on (map.pgm, map.yaml) and a picture of both (semantic_map.png); empty keeps them in memory.
cameras lists the cameras read, each with its detector's masks on /g1_perception/instance_masks
(head) or /g1_perception/<camera>/instance_masks; bringup's cameras:= picks which ones render.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SHARE = get_package_share_directory("g1_world_model")

ODOMETRY = "/g1_odometry_publisher/odom"
LIDAR = "/livox/lidar"


def camera_topics(name):
    """A camera's driver namespace and its detector's masks, as the relay and detectors name them.

    The head camera keeps the RealSense driver's /camera namespace the manipulation stack uses.
    """
    if name == "head":
        return "/camera", "/g1_perception/instance_masks"
    return f"/{name}_camera", f"/g1_perception/{name}/instance_masks"


def world_model(context):
    cameras = [
        name.strip()
        for name in LaunchConfiguration("cameras").perform(context).split(",")
        if name.strip()
    ]
    remappings = []
    for name in cameras:
        namespace, masks = camera_topics(name)
        remappings += [
            (f"{name}/depth/image_raw", f"{namespace}/aligned_depth_to_color/image_raw"),
            (f"{name}/depth/camera_info", f"{namespace}/aligned_depth_to_color/camera_info"),
            (f"{name}/color/image_raw", f"{namespace}/color/image_raw"),
            (f"{name}/instance_masks", masks),
        ]
    return [
        Node(
            package="g1_world_model",
            executable="g1_world_model",
            name="g1_world_model",
            output="both",
            parameters=[
                os.path.join(SHARE, "config", "g1_world_model.yaml"),
                {
                    "world_dir": LaunchConfiguration("world_dir"),
                    "describe": ParameterValue(LaunchConfiguration("describe"), value_type=bool),
                    "room_types_file": os.path.join(SHARE, "config", "room_types.yaml"),
                    "cameras": cameras,
                },
            ],
            remappings=remappings
            + [
                ("map", "/map"),
                ("odom", ODOMETRY),
                ("cloud", LIDAR),
                ("descriptions", "/object_describer/descriptions"),
            ],
        )
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
                "describe",
                default_value="false",
                description="Ask canopy_perception's object_describer to name objects from their best crop.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open RViz on the map, rooms, objects, camera coverage and the next "
                "viewpoint.",
            ),
            DeclareLaunchArgument(
                "cameras",
                default_value="head",
                description="Comma-separated cameras to read: head, chest.",
            ),
            OpaqueFunction(function=world_model),
            Node(
                package="rviz2",
                executable="rviz2",
                name="g1_world_model_rviz",
                arguments=["-d", os.path.join(SHARE, "config", "g1_world_model.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            # Names objects through the host semantic server (scripts/semantic_server.py).
            Node(
                package="canopy_perception",
                executable="object_describer",
                name="object_describer",
                output="both",
                condition=IfCondition(LaunchConfiguration("describe")),
                remappings=[("describe_requests", "/g1_world_model/describe_requests")],
            ),
        ]
    )
