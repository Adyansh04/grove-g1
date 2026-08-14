"""The simulator plus move_group: one command for arm planning in sim.

Includes g1_bringup rather than the other way round, the same direction as g1_navigation's
nav_sim.launch.py. Manipulation sits above bring-up, and a moveit:=true argument on
sim.launch.py would give g1_bringup a dependency on all of MoveIt.

Every argument is forwarded explicitly, including ones whose values match this file's own
defaults. An included launch file inherits the parent's configurations, so a child's
DeclareLaunchArgument default never fires for anything declared here -- relying on it is how
this stack has already shipped two silent bugs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    bringup_launch = os.path.join(
        get_package_share_directory("g1_bringup"), "launch", "sim.launch.py"
    )
    moveit_launch = os.path.join(
        get_package_share_directory("g1_moveit_config"), "launch", "move_group.launch.py"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                # Not optional. MoveIt refuses to plan until every active joint has a state,
                # and the arms hang off three waist joints joint_state_broadcaster does not
                # own -- so without this the model has no idea where the torso is pointing.
                "non_arm_joint_states": "true",
                # Off by default for the same reason as ever: g1_bringup's README records that
                # sensors:=true cost test_arm_command its slew-limited convergence window, and
                # manipulation executes the same kind of trajectory. Turning it on is what feeds
                # the octomap -- there is no depth image without it.
                "sensors": LaunchConfiguration("sensors"),
                "headless": LaunchConfiguration("headless"),
                "world": LaunchConfiguration("world"),
                "pin_pelvis": LaunchConfiguration("pin_pelvis"),
                "waist_hold_rad": LaunchConfiguration("waist_hold_rad"),
                "sim_start_delay_s": LaunchConfiguration("sim_start_delay_s"),
                "rviz": "false",
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_launch)),
        # No depth-to-cloud converter here. The octomap consumes /livox/lidar directly, which
        # is already a PointCloud2 with QoS the updater matches; the camera path needs
        # depth_image_proc and that node cannot receive from our best-effort relay. See
        # config/sensors_3d.yaml.
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
            description="Which scene to stage. Only matters for what the robot can bump into; "
            "nothing here reads the map.",
        ),
        DeclareLaunchArgument(
            "sensors",
            default_value="false",
            description="LiDAR, camera and the odom chain. Required for the octomap: without a "
            "depth image there is nothing to build a planning scene from. Off by default "
            "because it costs sim performance and manipulation does not otherwise need it.",
        ),
        DeclareLaunchArgument(
            "pin_pelvis",
            default_value="false",
            description="SIM-ONLY: weld the pelvis and disable the walking policy, so the arms "
            "can be exercised with nothing driving the legs. Worth it for planning work -- a "
            "balancing robot sways, and the arm chain hangs off that.",
        ),
        DeclareLaunchArgument(
            "waist_hold_rad",
            default_value="",
            description="SIM-ONLY: three comma-separated radians (yaw,roll,pitch) to stand the "
            "waist at. Requires pin_pelvis:=true. Use it to check arm planning against a torso "
            "that is not square to the pelvis.",
        ),
        DeclareLaunchArgument(
            "sim_start_delay_s",
            default_value="4.0",
            description="Seconds to delay the simulator's start. Higher than sim.launch.py's "
            "own default because move_group starts alongside and the machine is busier.",
        ),
        OpaqueFunction(function=_setup),
    ])
