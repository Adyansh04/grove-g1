"""Nav2 servers, plus the two G1-specific nodes that make their output usable.

Adapted from nav2_bringup's own navigation_launch.py (Humble 1.1.20), keeping its structure:
the same composed/non-composed split and the same /tf remappings. Differences from upstream:

  * velocity_smoother, smoother_server and waypoint_follower are NOT launched: smoothing toward
    zero decelerates this robot into the gait's dead zone and stops it dead, path smoothing is
    for a robot with more than two motion primitives, and nothing calls waypoint_follower.
  * controller_server's cmd_vel goes to /cmd_vel rather than upstream's cmd_vel_nav, because
    g1_gait_shaper sits between Nav2 and the robot here, in place of velocity_smoother, and also
    arbitrates Nav2 against g1_base_approach's higher-priority /cmd_vel_approach.
  * controller_server also REMAPS odom. Its OdomSmoother is built with the C++ default topic,
    and controller_server declares no odom_topic parameter on Humble -- setting one is silently
    ignored, leaving Nav2 believing the robot is permanently stationary.
  * g1_gait_shaper, g1_loco_authority and g1_base_approach are added; none exists upstream.

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
    EmitEvent,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

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
    # unconditionally anyway: it is the shaper's sibling on the velocity path, its goals simply
    # fail with "no fresh pose" when nothing publishes objects, and gating it on a navigation
    # argument that knows nothing about manipulation is the kind of cross-package coupling this
    # file already avoids elsewhere.
    base_approach = Node(
        package="g1_locomotion",
        executable="g1_base_approach",
        name="g1_base_approach",
        output="both",
        parameters=[os.path.join(loco_share, "config", "g1_base_approach.yaml")],
        remappings=[("objects", "/objects")],
    )

    # Not composed: making it a component would add rclcpp_components to g1_locomotion, the
    # package that has to survive to hardware, for no benefit -- nothing here is intra-process.
    gait_shaper = Node(
        package="g1_locomotion",
        executable="g1_gait_shaper",
        name="g1_gait_shaper",
        output="both",
        parameters=[os.path.join(loco_share, "config", "g1_gait_shaper.yaml")],
        remappings=[
            ("cmd_vel_in", "/cmd_vel"),
            ("cmd_vel_override", "/cmd_vel_approach"),
            ("cmd_vel_out", "/g1_loco_bridge/cmd_vel"),
        ],
    )

    # Its own lifecycle manager, not Nav2's: a plain rclcpp_lifecycle node creates no bond, and
    # deriving from nav2_util::LifecycleNode would drag Nav2 into g1_locomotion.
    # bond_timeout: 0.0 is what makes an unbonded node manageable -- verified empirically.
    loco_authority = LifecycleNode(
        package="g1_locomotion",
        executable="g1_loco_authority",
        name="g1_loco_authority",
        namespace="",
        output="both",
        parameters=[os.path.join(loco_share, "config", "g1_loco_authority.yaml")],
        remappings=[("status", "/g1_loco_bridge/status")],
    )
    # Driven by launch event handlers, NOT nav2_lifecycle_manager. Measured: the manager
    # declared the activate failed 1.6 ms after requesting it, while the node legitimately takes
    # ~2.5 s (the measured gait settle happens inside on_activate). The manager then aborts the
    # whole bringup over a transition that in fact succeeds moments later. Its service-call
    # budget is not exposed as a parameter in Humble, so there is nothing to widen.
    #
    # This is also the pattern g1_bringup/launch/sim.launch.py already uses to bring up
    # g1_odometry_publisher, so it is the house style rather than a workaround invented here.
    configure_authority = RegisterEventHandler(
        OnProcessStart(
            target_action=loco_authority,
            on_start=[
                # Delayed, and it has to be. OnProcessStart fires when the PROCESS spawns, not
                # when the node's lifecycle services are up, so emitting CONFIGURE immediately
                # races them and launch reports "Failed to make transition TRANSITION_CONFIGURE"
                # while the node itself is perfectly healthy. Observed on this stack.
                TimerAction(
                    period=2.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(loco_authority),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ],
                )
            ],
        )
    )
    activate_authority = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=loco_authority,
            # start_state pins this to the configuring -> inactive edge specifically. Matching
            # goal_state alone re-fires on every later entry to inactive, including a deliberate
            # deactivate, which then bounces the node straight back to active and floods the log
            # with "No transition matching 3 found for current state active".
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(loco_authority),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # If the node holding locomotion authority dies, Nav2 and the shaper keep publishing, the
    # bridge keeps re-issuing SET_VELOCITY, its 1 s dead-man never expires, and the robot walks
    # on with nobody supervising authority. Stopping the shaper breaks that chain: cmd_vel
    # ceases, the bridge's staleness path emits one zero, and the robot stands balanced.
    #
    # This covers ONLY a dead authority node. A robot that is fully healthy but whose gait has
    # stopped responding is a different failure with a different mitigation -- see the README.
    stop_shaper_if_authority_dies = RegisterEventHandler(
        OnProcessExit(
            target_action=loco_authority,
            on_exit=[
                EmitEvent(
                    event=ShutdownProcess(process_matcher=matches_action(gait_shaper))
                ),
            ],
        )
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
        gait_shaper,
        loco_authority,
        configure_authority,
        activate_authority,
        stop_shaper_if_authority_dies,
    ])
