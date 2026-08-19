"""The simulator plus move_group: one command for arm planning in sim.

Includes g1_bringup rather than the other way round, because manipulation sits above bring-up. Every
argument is forwarded explicitly: an included file inherits the parent's configurations, so a
child's default never fires for a name declared here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

BRINGUP_LAUNCH = os.path.join(
    get_package_share_directory("g1_bringup"), "launch", "sim.launch.py"
)
MOVE_GROUP_LAUNCH = os.path.join(
    get_package_share_directory("g1_moveit_config"), "launch", "move_group.launch.py"
)


def _include(path, **launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=launch_args.items()
    )


def _setup(context, *args, **kwargs):
    return [
        _include(
            BRINGUP_LAUNCH,
            rviz="false",
            sensors=LaunchConfiguration("sensors"),
            headless=LaunchConfiguration("headless"),
            world=LaunchConfiguration("world"),
            pin_pelvis=LaunchConfiguration("pin_pelvis"),
            sim_start_delay_s=LaunchConfiguration("sim_start_delay_s"),
        ),
        _include(MOVE_GROUP_LAUNCH),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="false shows the MuJoCo viewer.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="navigation",
            description="Which scene to stage. Only matters for what the robot can bump into.",
        ),
        DeclareLaunchArgument(
            "sensors",
            default_value="false",
            description="LiDAR, camera and the odom chain. Required for the octomap: without "
            "them there is nothing to build a planning scene from. Off by default because it "
            "costs sim performance.",
        ),
        DeclareLaunchArgument(
            "pin_pelvis",
            default_value="false",
            description="SIM-ONLY: weld the pelvis and disable the walking policy, so the arms "
            "can be exercised with nothing driving the legs. Worth it for planning work -- a "
            "balancing robot sways, and the arm chain hangs off that.",
        ),
        DeclareLaunchArgument(
            "sim_start_delay_s",
            default_value="4.0",
            description="Seconds to delay the simulator's start. Higher than sim.launch.py's "
            "own default because move_group starts alongside and the machine is busier.",
        ),
        OpaqueFunction(function=_setup),
    ])
