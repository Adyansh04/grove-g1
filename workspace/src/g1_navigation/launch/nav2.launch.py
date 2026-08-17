"""Nav2 servers, plus the one G1-specific node that makes their output usable.

Adapted from nav2_bringup's own navigation_launch.py (Humble 1.1.20), keeping its structure:
the same composed/non-composed split and the same /tf remappings. Differences from upstream:

  * velocity_smoother, smoother_server and waypoint_follower are NOT launched: path smoothing is
    for a robot with more than two motion primitives, and nothing calls waypoint_follower.
  * controller_server's cmd_vel goes to /cmd_vel rather than upstream's cmd_vel_nav: the walking
    policy subscribes there directly, so there is nothing in between to name.
  * controller_server also REMAPS odom. Its OdomSmoother is built with the C++ default topic,
    and controller_server declares no odom_topic parameter on Humble -- setting one is silently
    ignored, leaving Nav2 believing the robot is permanently stationary.
  * g1_base_approach is added; it does not exist upstream.

use_composition defaults to FALSE here, unlike scan.launch.py and localization.launch.py:
composed, the nested costmap parameter sections never reach the ComposableNode being loaded --
they resolve against differently-named nodes instead -- so Costmap2DROS comes up on its own
built-in defaults (base_link/map/0.1) instead of this package's config (base_footprint/odom/
0.45), and controller_server hangs forever in Activating, waiting on a transform from a frame
that does not exist. Uncomposed, the identical file delivers every value correctly. Composition
here is an optimisation; correctness is not negotiable for it.
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

# Upstream's list minus smoother_server, waypoint_follower and velocity_smoother. See the
# module docstring for why each is out.
LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def _reject_composition(context, *args, **kwargs):
    """Composition is known broken here, so say so instead of hanging.

    The branch below is kept because it is the shape upstream uses and the shape this will take
    again once the nested-parameter problem is understood. But left merely available it fails as
    a bringup that never finishes, which is a much worse thing to debug than a refusal.
    """
    if LaunchConfiguration("use_composition").perform(context).lower() == "true":
        raise RuntimeError(
            "use_composition:=true does not work for the Nav2 servers on Humble 1.1.20: the "
            "nested costmap sections never reach /local_costmap/local_costmap, so it comes up "
            "on Costmap2DROS defaults (base_link, map) and controller_server hangs forever in "
            "Activating. See this file's docstring. Run uncomposed."
        )
    return []


def generate_launch_description():
    share = get_package_share_directory("g1_navigation")
    loco_share = get_package_share_directory("g1_locomotion")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    # Nav2's controller publishes here; the shaper reduces it onto the gait's achievable
    # motions and forwards to the bridge. It is the LOW-priority of two sources, and the
    # shaper is what decides between them.
    controller_remappings = remappings + [
        ("cmd_vel", "/cmd_vel"),
        ("odom", "/g1_odometry_publisher/odom"),
    ]

    # Upstream additionally wraps this in RewrittenYaml to substitute use_sim_time and autostart
    # per namespace. Dropped: this package uses no namespaces, use_sim_time is already false
    # throughout the file, and autostart is set on the lifecycle manager directly.
    configured_params = ParameterFile(params_file, allow_substs=True)
    # The params file cannot name its own package share, so the BT path is injected here.
    # Both trees, not just the one we use. bt_navigator loads every navigator's tree on
    # activate regardless of the navigators parameter (accepted but not honoured on Humble
    # 1.1.20), so leaving the through-poses tree at upstream's copy makes it call BackUp, whose
    # action server no longer exists, and bt_navigator fails to activate.
    bt_xml = {
        "default_nav_to_pose_bt_xml": os.path.join(share, "config", "navigate_to_pose.xml"),
        "default_nav_through_poses_bt_xml": os.path.join(
            share, "config", "navigate_through_poses.xml"
        ),
    }

    # Unconditional: _reject_composition aborts before this is reached if composition was asked
    # for, so there is no second branch to select between.
    load_nodes = GroupAction(
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=controller_remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params, bt_xml],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": LIFECYCLE_NODES,
                }],
            ),
        ],
    )

    # Reads /objects, so it does nothing useful without manipulation:=true. Launched
    # unconditionally anyway: its goals simply fail with "no fresh pose" when nothing publishes
    # objects, and gating it on a navigation argument that knows nothing about manipulation is
    # the kind of cross-package coupling this file already avoids elsewhere.
    #
    # It writes /cmd_vel directly, like Nav2 does. Nothing arbitrates between them because
    # nothing has to: the mission tree runs NavigateToPose and ApproachObject in sequence, so
    # only one of the two is ever driving.
    base_approach = Node(
        package="g1_locomotion",
        executable="g1_base_approach",
        name="g1_base_approach",
        output="both",
        parameters=[os.path.join(loco_share, "config", "g1_base_approach.yaml")],
        remappings=[("objects", "/objects")],
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="There is no /clock on this track -- the simulator links no ROS at all. "
            "A true here gives every Nav2 server a clock that never advances.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "nav2_params.yaml"),
            description="Full path to the parameters file for all launched nodes.",
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "use_composition",
            default_value="false",
            description="Load the Nav2 servers into the shared container. DEFAULT FALSE, unlike "
            "the rest of this package -- composition does not deliver the nested costmap "
            "parameters. See the module docstring.",
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
        # After the declarations, not before: entities run in order, and evaluating
        # use_composition ahead of its DeclareLaunchArgument fails with "does not exist" on a
        # plain `ros2 launch` of this file.
        OpaqueFunction(function=_reject_composition),
        load_nodes,
        base_approach,
    ])
