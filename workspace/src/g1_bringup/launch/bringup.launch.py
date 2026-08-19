"""The operator entry point: bare sim, mapping, localization, Nav2, MoveIt, or a combination.

    ros2 launch g1_bringup bringup.launch.py                                  # bare sim
    ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true         # + SLAM
    ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true
    ros2 launch g1_bringup bringup.launch.py moveit:=true pin_pelvis:=true rviz:=true
    ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true moveit:=true

sim.launch.py and control.launch.py still work standalone; this file only composes them with
g1_navigation, g1_moveit_config and g1_manipulation.

Those three are reached by path lookup and are deliberately NOT dependencies: each already
depends on g1_bringup, and a reciprocal edge is a colcon cycle that refuses to build. Every
argument to an included file is forwarded explicitly, because a child's default never fires for
a name this file also declares.
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

BRINGUP_SHARE = get_package_share_directory("g1_bringup")

# 'none' keeps g1_navigation entirely out of the picture; the other two bring it in.
MODES = ("none", "mapping", "localization")

# A bare simulator wants 2.0; anything starting a stack alongside it wants 4.0, because more
# nodes come up before the first physics tick. Declaring either as this file's default would
# override whichever branch was right, so the argument defaults to an empty sentinel instead.
SIM_START_DELAY_S = {"bare": "2.0", "loaded": "4.0"}

# Which argument pulls each optional package in, for the not-installed message.
_ENABLED_BY = {
    "g1_navigation": "mode:=mapping and mode:=localization",
    "g1_moveit_config": "moveit:=true",
    "g1_manipulation": "manipulation:=true",
}


def _share(package):
    """Share directory, or a message an operator can act on. Nothing builds these packages for
    us: they are not dependencies, because the reverse edge already exists and a cycle does not
    build."""
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"bringup.launch.py: {_ENABLED_BY[package]} needs the {package} package, which is "
            "not on the ament prefix path.\n"
            f"  - Build it:  colcon build --packages-select {package}\n"
            "  - Then source install/setup.bash again in this shell.\n"
            f"g1_bringup deliberately does not depend on {package} (the two would form a "
            "colcon cycle), so nothing builds it for you."
        ) from exc


def _include(path, **launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=launch_args.items()
    )


# --- validation -----------------------------------------------------------------------------


def _validate(mode, want_nav, want_moveit, want_manipulation, pin_pelvis):
    if mode not in MODES:
        raise RuntimeError(
            f"mode:={mode!r} is not a mode. 'none' is the simulator on its own; 'mapping' "
            f"builds a map with slam_toolbox; 'localization' runs map_server + AMCL against "
            f"the committed one, and is what nav:=true requires."
        )
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
    if pin_pelvis and mode != "none":
        raise RuntimeError(
            "pin_pelvis:=true welds the pelvis to the world and disables the walking policy, "
            "so the robot cannot drive anywhere. It is a bare-sim debugging aid; use it with "
            "mode:=none."
        )


# --- the pieces -----------------------------------------------------------------------------


def _simulator(sim_args):
    """The one simulator this file stages, and the only place it is named: two would be two
    writers on rt/lowcmd. The branches only decide what goes in sim_args."""
    return _include(os.path.join(BRINGUP_SHARE, "launch", "sim.launch.py"), **sim_args)


def _sim_args(context, navigating, want_manipulation, want_moveit, pin_pelvis):
    delay = LaunchConfiguration("sim_start_delay_s").perform(context)
    if not delay:
        delay = SIM_START_DELAY_S["loaded" if navigating or want_moveit else "bare"]

    return {
        # Forced on for navigation, which needs the sweep, the relay, the odom chain and the
        # waist joint states, and for manipulation, whose object ground truth arrives over the
        # relay's socket.
        "sensors": (
            "true" if navigating or want_manipulation else LaunchConfiguration("sensors")
        ),
        "world": LaunchConfiguration("world"),
        "odometry": LaunchConfiguration("odometry"),
        "headless": LaunchConfiguration("headless"),
        "pin_pelvis": "true" if pin_pelvis else "false",
        "sim_start_delay_s": delay,
        # We own RViz below; without this the simulator opens its own on the wrong config.
        "rviz": "false",
    }


def _navigation(mode, want_nav):
    """nav_stack.launch.py stages no simulator of its own, which is what makes including it
    beside _simulator() safe. use_composition and container_name are not declared here and not
    forwarded, so nav_stack's own defaults apply."""
    return _include(
        os.path.join(_share("g1_navigation"), "launch", "nav_stack.launch.py"),
        mode=mode,
        nav="true" if want_nav else "false",
    )


def _moveit():
    """Sim-free by design, and it activates nothing: executing a plan still needs the ordered
    acquire in scripts/activate_arm."""
    return _include(os.path.join(_share("g1_moveit_config"), "launch", "move_group.launch.py"))


def _manipulation():
    return _include(
        os.path.join(_share("g1_manipulation"), "launch", "manipulation.launch.py"),
        object_source=LaunchConfiguration("object_source"),
    )


def _activate_arm(delay_s):
    """Delayed rather than sequenced on an event: the component only accepts activation once
    controller_manager has loaded it and /lowstate is flowing, and neither emits anything this
    file can wait on. scripts/activate_arm still fails loudly if it runs too early."""
    return TimerAction(
        period=delay_s,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "g1_bringup", "activate_arm"],
                name="activate_arm",
                output="screen",
            )
        ],
    )


def _rviz(navigating, want_nav, want_moveit):
    """The windows that match what is running. Two, never one: merging the two configs
    segfaults rviz2 on load once the navigation stack is up."""
    windows = []

    if want_moveit:
        # MoveIt's own launcher, because the MotionPlanning panel needs the semantic and
        # kinematics descriptions as node parameters. rviz_config is left unset on purpose:
        # this file never declares that name, so the child's own default applies.
        windows.append(
            _include(os.path.join(_share("g1_moveit_config"), "launch", "moveit_rviz.launch.py"))
        )

    # Keyed off nav, not mode: localization without a planner has a map and nothing to show.
    # The nav config carries a nav2_rviz_plugins display, so it is only named on a run that
    # navigates.
    if want_moveit:
        config = (
            os.path.join(_share("g1_navigation"), "config", "g1_navigation.rviz")
            if want_nav
            else None
        )
    elif navigating:
        config = os.path.join(_share("g1_navigation"), "config", "g1_navigation.rviz")
    else:
        config = os.path.join(BRINGUP_SHARE, "config", "g1_sensors.rviz")

    if config is not None:
        args = {"rviz_config": config}
        if want_moveit:
            # MoveIt's launcher already runs a node called rviz2.
            args["node_name"] = "rviz2_navigation"
        windows.append(_include(os.path.join(BRINGUP_SHARE, "launch", "rviz.launch.py"), **args))

    return windows


# --- assembly -------------------------------------------------------------------------------


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    want_nav = _flag(context, "nav")
    want_rviz = _flag(context, "rviz")
    want_moveit = _flag(context, "moveit")
    want_manipulation = _flag(context, "manipulation")
    pin_pelvis = _flag(context, "pin_pelvis")
    navigating = mode != "none"

    _validate(mode, want_nav, want_moveit, want_manipulation, pin_pelvis)

    actions = [
        _simulator(_sim_args(context, navigating, want_manipulation, want_moveit, pin_pelvis))
    ]
    if navigating:
        actions.append(_navigation(mode, want_nav))
    if want_moveit:
        actions.append(_moveit())
    if want_manipulation:
        actions.append(_manipulation())
    if want_moveit and _flag(context, "activate_arm"):
        actions.append(
            _activate_arm(float(LaunchConfiguration("activate_arm_delay_s").perform(context)))
        )
    if want_rviz:
        actions.extend(_rviz(navigating, want_nav, want_moveit))
    return actions


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
            description="Start the Nav2 servers and the base approach. Requires "
            "mode:=localization.",
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
            description="SIM CONVENIENCE: run scripts/activate_arm automatically once the "
            "stack is up. Needs moveit:=true. Off by default -- acquiring the arm is "
            "deliberate, and on hardware it is the moment MoveIt starts driving real joints.",
        ),
        DeclareLaunchArgument(
            "activate_arm_delay_s",
            default_value="25.0",
            description="Seconds to wait before the automatic activation. The component has to "
            "be loaded and /lowstate flowing first; too early and activate_arm fails loudly.",
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
            default_value="fast_lio",
            description="Which source publishes odom -> base_footprint. 'fast_lio' is the "
            "pipeline the real robot runs, over the simulated Mid360. 'ground_truth' is exact "
            "MuJoCo state, for isolating a fault to 'not the odometry'.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="navigation",
            description="Which scene to stage. 'navigation' is the facility the committed map "
            "was built from; localization against any other world will not converge. "
            "'manipulation' is one object at arm's length, for a pick without navigating "
            "to the workbench first.",
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
