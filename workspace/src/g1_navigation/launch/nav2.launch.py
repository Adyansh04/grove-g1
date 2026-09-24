"""The Nav2 servers, plus g1_base_approach for the last half metre Nav2's tolerance leaves.

Runs uncomposed: composed, the nested costmap parameters never arrive and controller_server
hangs in Activating.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

SHARE = get_package_share_directory("g1_navigation")
LOCO_SHARE = get_package_share_directory("g1_locomotion")

TF_REMAPPINGS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def _reject_composition(context, *args, **kwargs):
    """Fails fast on use_composition:=true, which otherwise hangs the bring-up."""
    if LaunchConfiguration("use_composition").perform(context).lower() == "true":
        raise RuntimeError(
            "use_composition:=true does not work for the Nav2 servers: the nested costmap "
            "sections never reach /local_costmap/local_costmap, so it comes up on "
            "Costmap2DROS defaults (base_link, map) and controller_server hangs forever in "
            "Activating. Run uncomposed."
        )
    return []


def _arguments():
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Passed to g1_base_approach only; the servers read params_file. Keep "
            "false: there is no /clock on this track.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(SHARE, "config", "nav2_params.yaml"),
            description="Parameters for every server launched here.",
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "use_composition",
            default_value="false",
            description="Rejected if true: composed, the nested costmap parameters never arrive.",
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]


def _servers():
    params = ParameterFile(LaunchConfiguration("params_file"), allow_substs=True)
    log_args = ["--ros-args", "--log-level", LaunchConfiguration("log_level")]
    # The params file cannot name its own share. Both trees: bt_navigator loads every
    # navigator's tree on activate regardless of `navigators`.
    bt_xml = {
        "default_nav_to_pose_bt_xml": os.path.join(SHARE, "config", "navigate_to_pose.xml"),
        "default_nav_through_poses_bt_xml": os.path.join(
            SHARE, "config", "navigate_through_poses.xml"
        ),
    }

    def server(package, executable, name, parameters, remappings):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=parameters,
            arguments=log_args,
            remappings=remappings,
        )

    return GroupAction(actions=[
        # controller_server subscribes to relative `odom`; unremapped it hears nothing and
        # treats the robot as stationary.
        server(
            "nav2_controller", "controller_server", "controller_server", [params],
            TF_REMAPPINGS + [("cmd_vel", "/cmd_vel"), ("odom", "/g1_odometry_publisher/odom")],
        ),
        server("nav2_planner", "planner_server", "planner_server", [params], TF_REMAPPINGS),
        server("nav2_behaviors", "behavior_server", "behavior_server", [params], TF_REMAPPINGS),
        server(
            "nav2_bt_navigator", "bt_navigator", "bt_navigator", [params, bt_xml], TF_REMAPPINGS
        ),
        server(
            "nav2_lifecycle_manager", "lifecycle_manager", "lifecycle_manager_navigation",
            [params, {"autostart": LaunchConfiguration("autostart")}], [],
        ),
    ])


def _base_approach():
    """Launched unconditionally; without /objects its goals just fail.

    A manipulation flag here would couple this package to one it knows nothing about. It writes
    /cmd_vel like Nav2, so the mission tree runs the two strictly in sequence.
    """
    return Node(
        package="g1_locomotion",
        executable="g1_base_approach",
        name="g1_base_approach",
        output="both",
        parameters=[
            os.path.join(LOCO_SHARE, "config", "g1_base_approach.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("objects", "/objects")],
    )


def generate_launch_description():
    return LaunchDescription(
        [SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")]
        + _arguments()
        # After the declarations, or use_composition does not exist yet on a plain launch.
        + [OpaqueFunction(function=_reject_composition), _servers(), _base_approach()]
    )
