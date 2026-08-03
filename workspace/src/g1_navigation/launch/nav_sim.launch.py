"""Top level: the simulator, the scan pipeline, and either SLAM or localization.

Includes g1_bringup rather than the other way round. Navigation sits above bring-up, so the
higher layer composes the lower one; a nav:=true argument on sim.launch.py would give
g1_bringup a dependency on Nav2 and slam_toolbox.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = ("mapping", "localization")

# Isolated rather than the plain container: each component keeps its own single-threaded
# executor, so one blocking callback cannot stall the others. Same choice nav2_bringup makes.
CONTAINER_EXECUTABLE = "component_container_isolated"


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODES:
        raise RuntimeError(
            f"mode:={mode!r} is not a mode. 'mapping' builds a new map of the facility; "
            f"'localization' runs against the committed one, which is what a repeatable "
            f"goal pose needs."
        )
    share = get_package_share_directory("g1_navigation")
    launch_dir = os.path.join(share, "launch")

    def include(name, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=launch_args.items(),
        )

    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")

    actions = [
        # sensors:=true is not optional: it gates the LiDAR sweep, the relay, the
        # odom -> base_footprint -> pelvis chain and the waist joint states. Passed explicitly
        # rather than by flipping the bringup default, which is still provisional on an
        # unthrottled re-measurement of test_arm_command.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("g1_bringup"), "launch", "sim.launch.py"
                )
            ),
            launch_arguments={
                "sensors": "true",
                "world": LaunchConfiguration("world"),
                "headless": LaunchConfiguration("headless"),
                "sim_start_delay_s": LaunchConfiguration("sim_start_delay_s"),
            }.items(),
        ),
        # The container has to exist before anything tries to load into it. Created here
        # rather than in the leaf launches so one container serves all of them -- which is
        # the point, and is what PR B's costmaps and controller will join.
        Node(
            condition=IfCondition(use_composition),
            name=container_name,
            package="rclcpp_components",
            executable=CONTAINER_EXECUTABLE,
            output="both",
        ),
        include(
            "scan.launch.py",
            use_composition=use_composition,
            container_name=container_name,
        ),
    ]

    if mode == "mapping":
        # slam_toolbox stays out of the container even though it ships a component:
        # nav2_bringup's own slam_launch.py runs it as a plain node, and its 40 MB stack
        # requirement for map serialization is not something to hand a shared process.
        actions.append(include("slam.launch.py", mode="mapping"))
    else:
        actions.append(
            include(
                "localization.launch.py",
                use_composition=use_composition,
                container_name=container_name,
            )
        )

    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", os.path.join(share, "config", "g1_navigation.rviz")],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="mapping",
            description="'mapping' builds a new map with slam_toolbox; 'localization' runs "
            "map_server + AMCL against maps/facility.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="navigation",
            description="Which g1_bringup scene to stage. 'navigation' is the facility the "
            "committed map was built from -- localization against any other world will not "
            "converge.",
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Run the navigation nodes in one component container so they talk "
            "intra-process. Set false to get one process per node, which is what you want "
            "when a single node is crashing and you need to see which.",
        ),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="false shows the MuJoCo viewer. Its Reload button is fatal with "
            "sensors on; see g1_bringup's README.",
        ),
        DeclareLaunchArgument(
            "sim_start_delay_s",
            default_value="4.0",
            description="Longer than g1_bringup's own default. Navigation starts more nodes "
            "before the first physics tick, and the robot topples at spawn if discovery is "
            "still in progress.",
        ),
        OpaqueFunction(function=_setup),
    ])
