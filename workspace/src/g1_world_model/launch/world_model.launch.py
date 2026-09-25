"""The world model node, wired to the camera, the detector's masks, the map and odometry.

    ros2 launch g1_world_model world_model.launch.py world_dir:=/root/data/worlds/apartment

world_dir keeps the rooms, objects and camera coverage between runs; empty keeps them in memory.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SHARE = get_package_share_directory("g1_world_model")

# The relay's topics; see g1_sensor_relay/config/g1_sensor_relay.yaml.
DEPTH_IMAGE = "/camera/aligned_depth_to_color/image_raw"
DEPTH_INFO = "/camera/aligned_depth_to_color/camera_info"
COLOR_IMAGE = "/camera/color/image_raw"
MASK_TOPIC = "/g1_perception/instance_masks"
ODOMETRY = "/g1_odometry_publisher/odom"
LIDAR = "/livox/lidar"


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
                description="Ask g1_object_describer to name objects from their best crop.",
            ),
            Node(
                package="g1_world_model",
                executable="g1_world_model",
                name="g1_world_model",
                output="both",
                parameters=[
                    os.path.join(SHARE, "config", "g1_world_model.yaml"),
                    {
                        "world_dir": LaunchConfiguration("world_dir"),
                        "describe": ParameterValue(
                            LaunchConfiguration("describe"), value_type=bool
                        ),
                        "room_types_file": os.path.join(SHARE, "config", "room_types.yaml"),
                    },
                ],
                remappings=[
                    ("map", "/map"),
                    ("depth/image_raw", DEPTH_IMAGE),
                    ("depth/camera_info", DEPTH_INFO),
                    ("color/image_raw", COLOR_IMAGE),
                    ("instance_masks", MASK_TOPIC),
                    ("odom", ODOMETRY),
                    ("cloud", LIDAR),
                    ("descriptions", "/g1_object_describer/descriptions"),
                ],
            ),
            # Names objects through the host semantic server (scripts/semantic_server.py).
            Node(
                package="g1_perception",
                executable="g1_object_describer",
                name="g1_object_describer",
                output="both",
                condition=IfCondition(LaunchConfiguration("describe")),
                remappings=[("describe_requests", "/g1_world_model/describe_requests")],
            ),
        ]
    )
