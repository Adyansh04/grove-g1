"""The operator entry point: one command for bare sim, mapping, localization or full Nav2.

    ros2 launch g1_bringup bringup.launch.py                                  # bare sim
    ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true         # + SLAM
    ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true

sim.launch.py and control.launch.py are unchanged and still work standalone; this file sits
above them and adds nothing of its own except the routing.

================================================================================
g1_navigation is referenced WITHOUT being declared as a dependency. On purpose.
================================================================================

g1_navigation already declares <exec_depend>g1_bringup</exec_depend>, because navigation
composes bring-up and not the other way round (docs/notes, M6). Adding the reciprocal
dependency here is not merely untidy, it does not build -- colcon refuses outright:

    ERROR:colcon:colcon list: Unable to order packages topologically:
    g1_bringup: ['g1_navigation']
    g1_navigation: ['g1_bringup']

So the reference below is a launch-time path lookup and nothing else. Consequences worth
knowing before touching it:

  * g1_bringup builds, installs and runs with g1_navigation absent from the workspace. Only
    the mode:=mapping / mode:=localization branches ever name it, and _navigation_share()
    turns its absence into an actionable message rather than a raw ament search-path dump.
  * colcon and rosdep cannot see this edge. Nothing will warn you if g1_navigation's launch
    file names change; the launch fails at runtime instead.
  * Do not "fix" this by adding the dependency. It is load-bearing that it stays absent.

This file composes independent pieces; it does not delegate to one bundled entry point per
package. It stages exactly one simulator itself -- `_simulator()`, the only place the simulator
is named anywhere in this file -- and the navigation modes add g1_navigation's
nav_stack.launch.py, which stages none of its own. Exactly one simulator per launch, always:
two would put two motion_service_sim processes on /lowcmd, and CONTROL_MODES.md puts that
failure first for a reason.

The direction of composition is unchanged and still navigation-over-bringup. What changed is
the granularity: bring-up reaches a genuinely sim-independent piece of g1_navigation directly,
rather than a wrapper that bundles a simulator in with it.

nav_sim.launch.py still exists and still runs standalone, composing the same two pieces for a
nav-only session, and it is what the g1_navigation integration suites launch. That leaves
exactly one fact stated in two files -- sensors:=true on the simulator -- because each stages
its own and there is nowhere shared to put it. test_launch_threading asserts it on both.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
    pin_pelvis = LaunchConfiguration("pin_pelvis").perform(context).lower() == "true"

    if want_nav and mode != "localization":
        raise RuntimeError(
            f"nav:=true needs mode:=localization, not mode:={mode!r}. Navigating against a "
            "map slam_toolbox is still building means the goal pose moves under the planner."
        )
    if pin_pelvis and navigating:
        raise RuntimeError(
            "pin_pelvis:=true welds the pelvis to the world and disables the walking policy, "
            "so the robot cannot drive anywhere. It is a bare-sim debugging aid; use it with "
            "mode:=none."
        )

    delay = LaunchConfiguration("sim_start_delay_s").perform(context)
    if not delay:
        delay = DEFAULT_SIM_START_DELAY_S["loaded" if navigating else "bare"]

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
        "sensors": "true" if navigating else LaunchConfiguration("sensors"),
        "world": LaunchConfiguration("world"),
        "headless": LaunchConfiguration("headless"),
        "pin_pelvis": "true" if pin_pelvis else "false",
        "sim_start_delay_s": delay,
        # We own RViz, below. Without this the simulator opens its own on the wrong config.
        "rviz": "false",
    }

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

    if want_rviz:
        # The nav config carries a nav2_rviz_plugins display, so it ships from g1_navigation.
        # On the bare branch this resolves g1_bringup's own share and g1_navigation is never
        # named -- same rule as the launch include above.
        if navigating:
            rviz_config = os.path.join(_navigation_share(), "config", "g1_navigation.rviz")
        else:
            rviz_config = os.path.join(
                get_package_share_directory("g1_bringup"), "config", "g1_sensors.rviz"
            )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("g1_bringup"), "launch", "rviz.launch.py"
                    )
                ),
                launch_arguments={"rviz_config": rviz_config}.items(),
            )
        )

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
            "sensors",
            default_value="false",
            description="LiDAR sweep, the relay and the odom -> base_footprint -> pelvis "
            "chain. Only meaningful with mode:=none -- the navigation modes need it and turn "
            "it on themselves.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="navigation",
            description="Which scene to stage. 'navigation' is the facility the committed map "
            "was built from; localization against any other world will not converge.",
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
