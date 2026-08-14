"""The operator entry point: bare sim, mapping, localization, Nav2, MoveIt, or a combination.

    ros2 launch g1_bringup bringup.launch.py                                  # bare sim
    ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true         # + SLAM
    ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true
    ros2 launch g1_bringup bringup.launch.py moveit:=true pin_pelvis:=true rviz:=true
    ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true moveit:=true

sim.launch.py and control.launch.py are unchanged and still work standalone; this file sits
above them and adds nothing of its own except the composition.

================================================================================
g1_navigation and g1_moveit_config are referenced WITHOUT being declared as
dependencies. On purpose.
================================================================================

Both already declare <exec_depend>g1_bringup</exec_depend>, because each composes bring-up and
not the other way round. Adding a reciprocal dependency here is not
merely untidy, it does not build -- colcon refuses outright:

    ERROR:colcon:colcon list: Unable to order packages topologically:
    g1_bringup: ['g1_navigation']
    g1_navigation: ['g1_bringup']

So both references below are launch-time path lookups and nothing else. Consequences worth
knowing before touching them:

  * g1_bringup builds, installs and runs with either package absent from the workspace. Only
    mode:=mapping / mode:=localization name g1_navigation, and only moveit:=true names
    g1_moveit_config; _navigation_share() and _moveit_share() turn an absence into an
    actionable message rather than a raw ament search-path dump.
  * colcon and rosdep cannot see these edges. Nothing will warn you if either package renames
    a launch file; the launch fails at runtime instead. test_launch_threading in each of those
    packages is the compensating check, and neither can live here for the same reason the
    dependency cannot.
  * Do not "fix" this by adding the dependency. It is load-bearing that it stays absent.

This file composes independent pieces; it does not delegate to one bundled entry point per
package. Three pieces, each conditional and each unaware of the others:

  * the simulator -- `_simulator()`, the only place it is named anywhere in this file, and
    exactly once per launch. Two would put two motion_service_sim processes on /lowcmd, and
    CONTROL_MODES.md puts that failure first for a reason. Swapping in a hardware bring-up
    later means replacing that function's body; the branches only decide what goes in its
    arguments.
  * navigation -- g1_navigation's nav_stack.launch.py, which stages no simulator of its own.
  * manipulation -- g1_moveit_config's move_group.launch.py, sim-free by the same design.

The direction of composition is unchanged and still navigation-over-bringup. What changed is
the granularity: bring-up reaches genuinely sim-independent pieces of those packages directly,
rather than wrappers that bundle a simulator in with them. A package may expose several such
pieces, and bring-up may reach whichever it needs.

nav_sim.launch.py and moveit_sim.launch.py still exist and still run standalone, each composing
the same pieces for a single-package session, and they are what those packages' integration
suites launch. That leaves two facts stated in two files each -- sensors:=true and
non_arm_joint_states:=true on the simulator -- because every one of these files stages its own
simulator and there is nowhere shared to put them. test_launch_threading asserts both copies.

A note on what is NOT forwarded, since the reflex here is to forward everything. The hazard is
inheritance: a child's DeclareLaunchArgument default never fires for a name the PARENT also
declares, so a name declared in both places must be forwarded explicitly or the parent's value
silently wins. That requires the same name on both sides. Names this file never declares --
nav_stack's use_composition and container_name, moveit_rviz's rviz_config on the bare MoveIt
branch -- have nothing to inherit from and resolve to the child's own default, which is the
intended value. So they are deliberately left alone, and an operator who needs to set them runs
the standalone wrapper, which does declare them.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 'none' keeps g1_navigation entirely out of the picture; the other two bring it in.
MODES = ("none", "mapping", "localization")

# A bare simulator wants 2.0; anything that starts a stack alongside it wants 4.0, because more
# nodes come up before the first physics tick and the robot topples at spawn if discovery is
# still in progress. Declaring either as this file's default would silently override whichever
# branch was right, because an included launch file inherits the parent's configurations.
# The empty sentinel means "let the branch decide", and a concrete value is always forwarded.
DEFAULT_SIM_START_DELAY_S = {"bare": "2.0", "loaded": "4.0"}


def _navigation_share():
    """g1_navigation's share directory, or a message an operator can act on."""
    try:
        return get_package_share_directory("g1_navigation")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "bringup.launch.py: mode:=mapping and mode:=localization need the g1_navigation "
            "package, which is not on the ament prefix path.\n"
            "  - Build it:  colcon build --packages-select g1_navigation\n"
            "  - Then source install/setup.bash again in this shell.\n"
            "g1_bringup deliberately does not declare g1_navigation as a dependency (the two "
            "would form a colcon dependency cycle -- see this file's docstring), so nothing "
            "builds it for you.\n"
            "mode:=none needs none of this and runs the simulator on its own."
        ) from exc


def _moveit_share():
    """g1_moveit_config's share directory, or a message an operator can act on."""
    try:
        return get_package_share_directory("g1_moveit_config")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "bringup.launch.py: moveit:=true needs the g1_moveit_config package, which is not "
            "on the ament prefix path.\n"
            "  - Build it:  colcon build --packages-select g1_moveit_config\n"
            "  - Then source install/setup.bash again in this shell.\n"
            "g1_bringup deliberately does not declare g1_moveit_config as a dependency (the "
            "two would form a colcon dependency cycle -- see this file's docstring), so "
            "nothing builds it for you.\n"
            "moveit:=false, the default, needs none of this."
        ) from exc


def _manipulation_share():
    """g1_manipulation's share directory, or a message an operator can act on."""
    try:
        return get_package_share_directory("g1_manipulation")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "bringup.launch.py: manipulation:=true needs the g1_manipulation package, which "
            "is not on the ament prefix path.\n"
            "  - Build it:  colcon build --packages-select g1_manipulation\n"
            "  - Then source install/setup.bash again in this shell.\n"
            "manipulation:=false, the default, needs none of this."
        ) from exc


def _simulator(sim_args):
    """The one simulator this file stages, and the only place it is named.

    Isolated deliberately. Everything above composes around it, so swapping in a hardware
    bring-up later replaces this function's body and nothing else: the branches only decide
    what goes in sim_args. There is no platform:= argument because there is no second
    implementation yet, and one would be a knob with a single position.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments=sim_args.items(),
    )


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODES:
        raise RuntimeError(
            f"mode:={mode!r} is not a mode. 'none' is the simulator on its own; 'mapping' "
            f"builds a map with slam_toolbox; 'localization' runs map_server + AMCL against "
            f"the committed one, and is what nav:=true requires."
        )

    navigating = mode != "none"
    want_nav = LaunchConfiguration("nav").perform(context).lower() == "true"
    want_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    want_moveit = LaunchConfiguration("moveit").perform(context).lower() == "true"
    want_manipulation = LaunchConfiguration("manipulation").perform(context).lower() == "true"
    pin_pelvis = LaunchConfiguration("pin_pelvis").perform(context).lower() == "true"

    if want_nav and mode != "localization":
        raise RuntimeError(
            f"nav:=true needs mode:=localization, not mode:={mode!r}. Navigating against a "
            "map slam_toolbox is still building means the goal pose moves under the planner."
        )
    if want_manipulation and not want_moveit:
        raise RuntimeError(
            "manipulation:=true needs moveit:=true. The skills plan and execute through "
            "move_group, so without it every goal fails on a planning pipeline that is not "
            "there."
        )
    if pin_pelvis and navigating:
        raise RuntimeError(
            "pin_pelvis:=true welds the pelvis to the world and disables the walking policy, "
            "so the robot cannot drive anywhere. It is a bare-sim debugging aid; use it with "
            "mode:=none."
        )

    delay = LaunchConfiguration("sim_start_delay_s").perform(context)
    if not delay:
        delay = DEFAULT_SIM_START_DELAY_S["loaded" if navigating or want_moveit else "bare"]

    # Every argument below is forwarded EXPLICITLY, including the ones whose values match
    # this file's own defaults. Included launch files inherit the parent's configurations,
    # so a child's DeclareLaunchArgument default never fires for anything declared here --
    # relying on it is how this stack has already shipped two silent bugs (a second RViz
    # window, and use_composition ignoring its own default).
    sim_args = {
        # Not optional on the navigation modes, whatever the operator asked for: sensors gates
        # the LiDAR sweep, the relay, the odom -> base_footprint -> pelvis chain and the waist
        # joint states, and navigation is dead without all four. Forced here rather than by
        # flipping sim.launch.py's default, which is still provisional on an unthrottled
        # re-measurement of test_arm_command.
        #
        # manipulation:=true forces it for a different reason: object ground truth leaves the
        # simulator over the sensor relay's own socket, so without sensors the pose source
        # comes up healthy, subscribes, and never receives anything.
        "sensors": "true"
        if navigating or want_manipulation
        else LaunchConfiguration("sensors"),
        "world": LaunchConfiguration("world"),
        "odometry": LaunchConfiguration("odometry"),
        "headless": LaunchConfiguration("headless"),
        "pin_pelvis": "true" if pin_pelvis else "false",
        "waist_hold_rad": LaunchConfiguration("waist_hold_rad"),
        "sim_start_delay_s": delay,
        # We own RViz, below. Without this the simulator opens its own on the wrong config.
        "rviz": "false",
    }

    if want_moveit:
        # MoveIt refuses to plan until every active joint has a state, and the arms hang off
        # three waist joints joint_state_broadcaster does not own. Not exposed as an operator
        # argument: moveit:=true with this off is a move_group that comes up healthy and then
        # silently never plans, which is a worse failure than not having the knob.
        sim_args["non_arm_joint_states"] = "true"

    actions = [_simulator(sim_args)]

    if navigating:
        # nav_stack.launch.py stages no simulator of its own, which is what makes including it
        # next to _simulator() safe; two would be two writers on /lowcmd.
        #
        # use_composition and container_name are deliberately NOT declared by this file and
        # NOT forwarded, so nav_stack's own defaults are the ones that apply. That is safe for
        # the same reason the explicit forwarding above is necessary: inheritance only leaks
        # through a name the parent also declares. No name here, nothing to leak, and an
        # operator who wants to decompose the container runs nav_sim.launch.py, which exposes
        # both.
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(_navigation_share(), "launch", "nav_stack.launch.py")
                ),
                launch_arguments={
                    "mode": mode,
                    "nav": "true" if want_nav else "false",
                }.items(),
            )
        )

    if want_moveit:
        # move_group.launch.py declares no arguments, so there is nothing to forward. It is
        # sim-free by design -- planning needs joint states, which bring-up publishes from the
        # moment it runs -- and it activates nothing: executing a plan still needs the ordered
        # acquire in scripts/activate_arm.
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(_moveit_share(), "launch", "move_group.launch.py")
                )
            )
        )

    if want_manipulation:
        # The pick/place skills and the object-pose source they read. Composed beside
        # move_group rather than including it, for the same reason nav_stack.launch.py stages
        # no simulator: both are already here, and a second copy of either would be a second
        # writer.
        #
        # The object source is the simulation-specific half and says so itself -- its
        # `hardware` default refuses to configure, so this argument is what opts into ground
        # truth rather than something a hardware bring-up could inherit by accident.
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(_manipulation_share(), "launch", "manipulation.launch.py")
                ),
                launch_arguments={
                    "object_source": LaunchConfiguration("object_source"),
                }.items(),
            )
        )

    if want_moveit and LaunchConfiguration("activate_arm").perform(context).lower() == "true":
        # Off by default, and it stays off by default. Acquiring the arm is a deliberate act:
        # on hardware this is the moment /arm_sdk starts driving real joints, and a stack that
        # goes live on `ros2 launch` is a stack that goes live when someone launches it to look
        # at something else. This argument is a sim convenience, nothing more.
        #
        # Delayed rather than sequenced on an event: the component only accepts activation once
        # controller_manager has loaded it and /lowstate is flowing, and neither emits anything
        # this file can wait on. scripts/activate_arm still enforces the component-then-
        # controller order, and still fails loudly if it runs too early.
        #
        # The principled version of this now exists: g1_orchestration's executor brackets a
        # whole mission, acquiring before the tree runs and releasing on success, failure and
        # SIGINT alike -- what CONTROL_MODES.md rule 4 actually asks for. This argument stays
        # for the case it was written for, which is driving the arm by hand from RViz.
        actions.append(
            TimerAction(
                period=float(LaunchConfiguration("activate_arm_delay_s").perform(context)),
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "run", "g1_bringup", "activate_arm"],
                        name="activate_arm",
                        output="screen",
                    )
                ],
            )
        )

    if want_rviz:
        actions.extend(_rviz(navigating, want_nav, want_moveit))

    return actions


def _rviz(navigating, want_nav, want_moveit):
    """The RViz windows that match what is running.

    MoveIt's own launcher rather than this package's generic rviz.launch.py, which takes only
    an rviz_config and passes no parameters: the MotionPlanning panel needs
    robot_description_semantic and robot_description_kinematics as node parameters, and without
    them it loads with no planning groups, which reads as a broken install.

    With MoveIt and Nav2 BOTH running this returns two windows: the MoveIt one for the arm and
    a second on g1_navigation.rviz for the map, costmaps and plan. A combined single-window view
    is NOT available: merging the two configs segfaults rviz2 on load once the navigation stack
    is up.

    The second window keys off nav, not mode: mode=localization without nav has a map but no
    planner, and MoveIt's view is the only one worth opening.
    """
    bringup_share = get_package_share_directory("g1_bringup")
    generic_rviz  = os.path.join(bringup_share, "launch", "rviz.launch.py")
    windows       = []

    if want_moveit:
        windows.append(
            IncludeLaunchDescription(
                # Bare MoveIt leaves rviz_config unset on purpose: this file never declares that
                # name, so there is nothing for the child's default to inherit from and
                # g1_moveit.rviz is what applies. The one case where a child default is safe.
                PythonLaunchDescriptionSource(
                    os.path.join(_moveit_share(), "launch", "moveit_rviz.launch.py")
                )
            )
        )

    # The nav config carries a nav2_rviz_plugins display, so it ships from g1_navigation; on a
    # run that never navigates g1_navigation is not named at all, the same rule the launch
    # includes above follow.
    if want_moveit:
        config = os.path.join(_navigation_share(), "config", "g1_navigation.rviz") if want_nav \
            else None
    elif navigating:
        config = os.path.join(_navigation_share(), "config", "g1_navigation.rviz")
    else:
        config = os.path.join(bringup_share, "config", "g1_sensors.rviz")

    if config is not None:
        args = {"rviz_config": config}
        if want_moveit:
            # MoveIt's launcher already runs a node called rviz2.
            args["node_name"] = "rviz2_navigation"
        windows.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(generic_rviz), launch_arguments=args.items()
            )
        )

    return windows


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="none",
            description="'none' runs the simulator alone and never touches g1_navigation. "
            "'mapping' adds the scan pipeline and slam_toolbox. 'localization' adds the scan "
            "pipeline, map_server and AMCL, and is the mode nav:=true requires.",
        ),
        DeclareLaunchArgument(
            "nav",
            default_value="false",
            description="Start Nav2, the gait shaper and the locomotion authority bracket. "
            "Requires mode:=localization.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open RViz. mode:=none uses g1_bringup's sensor config (fixed frame "
            "odom); the navigation modes use g1_navigation's, which adds the Nav2 display "
            "group and is fixed on map.",
        ),
        DeclareLaunchArgument(
            "moveit",
            default_value="false",
            description="Start move_group for arm planning. Works with any mode. Planning is "
            "available immediately; executing a plan still needs activate_arm.launch.py.",
        ),
        DeclareLaunchArgument(
            "manipulation",
            default_value="false",
            description="Start the pick and place skills and the object-pose source. Needs "
            "moveit:=true, since the skills plan through move_group.",
        ),
        DeclareLaunchArgument(
            "object_source",
            default_value="sim_ground_truth",
            description="Where object poses come from with manipulation:=true. "
            "'sim_ground_truth' reads MuJoCo bodies; 'hardware' refuses to configure, because "
            "no object-detection pipeline exists yet.",
        ),
        DeclareLaunchArgument(
            "activate_arm",
            default_value="false",
            description="SIM CONVENIENCE: run scripts/activate_arm automatically once the stack "
            "is up, instead of as a separate command. Needs moveit:=true. Off by default -- "
            "acquiring the arm is deliberate, and on hardware it is the moment /arm_sdk starts "
            "driving real joints.",
        ),
        DeclareLaunchArgument(
            "activate_arm_delay_s",
            default_value="25.0",
            description="Seconds to wait before the automatic activation. The component has to "
            "be loaded and /lowstate flowing first; too early and activate_arm fails loudly.",
        ),
        DeclareLaunchArgument(
            "waist_hold_rad",
            default_value="",
            description="SIM-ONLY: three comma-separated radians (yaw,roll,pitch) to stand the "
            "waist at. Requires pin_pelvis:=true, which sim.launch.py enforces. Use it to "
            "check arm planning against a torso that is not square to the pelvis.",
        ),
        DeclareLaunchArgument(
            "sensors",
            default_value="false",
            description="LiDAR sweep, the relay and the odom -> base_footprint -> pelvis "
            "chain. Only meaningful with mode:=none -- the navigation modes need it and turn "
            "it on themselves.",
        ),
        DeclareLaunchArgument(
            "odometry",
            default_value="sportmodestate",
            description="Which source publishes odom -> base_footprint. 'sportmodestate' is "
            "exact MuJoCo state and is what the mission is tuned against. 'fast_lio' runs the "
            "LiDAR-inertial pipeline the real robot uses, over the simulated Mid360.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="navigation",
            description="Which scene to stage. 'navigation' is the facility the committed map "
            "was built from; localization against any other world will not converge. "
            "'manipulation' is one object on a pedestal at arm's length, for exercising a "
            "pick without navigating to the workbench first.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="false shows the MuJoCo viewer. Its Reload button is fatal with "
            "sensors on; see the README.",
        ),
        DeclareLaunchArgument(
            "pin_pelvis",
            default_value="false",
            description="SIM-ONLY debugging aid: weld the pelvis and disable the walking "
            "policy, to exercise the arm bridge with nothing driving the legs. mode:=none only.",
        ),
        DeclareLaunchArgument(
            "sim_start_delay_s",
            default_value="",
            description="Seconds to delay the simulator's start. Empty means the branch's own "
            "default: 2.0 for mode:=none, 4.0 for the navigation modes, which start more nodes "
            "before the first physics tick.",
        ),
        OpaqueFunction(function=_setup),
    ])
