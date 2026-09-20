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
from launch.substitutions import EqualsSubstitution, LaunchConfiguration

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
    "g1_perception": "perception:=true",
    "g1_vla": "vla:=true",
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


def _validate(mode, want_nav, want_moveit, want_manipulation, want_perception, want_vla,
              pin_pelvis):
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
    if want_perception and not want_manipulation:
        raise RuntimeError(
            "perception:=true needs manipulation:=true. What perception measures reaches the "
            "skills through the object-pose source, and that comes with manipulation."
        )
    if want_vla and not want_manipulation:
        raise RuntimeError(
            "vla:=true needs manipulation:=true. The grasp skill validates against "
            "move_group's planning scene and measures its result off /objects, and the "
            "object-pose source comes with manipulation."
        )
    if pin_pelvis and mode != "none":
        raise RuntimeError(
            "pin_pelvis:=true welds the pelvis and freezes the legs, so the robot cannot "
            "drive anywhere. It is a bare-sim debugging aid; use it with mode:=none."
        )


# --- the pieces -----------------------------------------------------------------------------


def _simulator(sim_args):
    """The one simulator this file stages, and the only place it is named: two would be two
    writers on rt/lowcmd. The branches only decide what goes in sim_args."""
    return _include(os.path.join(BRINGUP_SHARE, "launch", "sim.launch.py"), **sim_args)


def _sim_args(context, navigating, want_manipulation, want_perception, want_moveit, pin_pelvis):
    delay = LaunchConfiguration("sim_start_delay_s").perform(context)
    if not delay:
        delay = SIM_START_DELAY_S["loaded" if navigating or want_moveit else "bare"]

    return {
        # Forced on for navigation, which needs the sweep, the relay, the odom chain and the
        # waist joint states, for manipulation, whose object ground truth arrives over the
        # relay's socket, and for perception, which has nothing to look at without the camera.
        "sensors": (
            "true"
            if navigating or want_manipulation or want_perception
            else LaunchConfiguration("sensors")
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
    return _include(
        os.path.join(_share("g1_moveit_config"), "launch", "move_group.launch.py"),
        servo=EqualsSubstitution(LaunchConfiguration("vla_execution_mode"), "servo"),
    )


def _manipulation(want_perception, visualization):
    # Perception owns the object stream when it runs, and its poses arrive seconds late.
    #
    # 8 s, not 4. Measured in sim, where the detector shares a GPU with MuJoCo's rendering:
    # /objects comes out at 0.39 Hz, a cycle every 2.6 s, and stretches past 4 s once the arm is
    # planning too. Every lookup was then refused on age with "No usable object poses ... 4.17 s
    # old", which surfaces to a tree as the surface having vanished. The scenes this runs on are
    # static, so an 8 s old pose of a bench is still exactly where the bench is; a moving object
    # on hardware, off its own camera, wants the shorter number back.
    return _include(
        os.path.join(_share("g1_manipulation"), "launch", "manipulation.launch.py"),
        object_source="perception" if want_perception else LaunchConfiguration("object_source"),
        object_timeout_ms="8000.0" if want_perception else "1000.0",
        grasp_source=LaunchConfiguration("grasp_source"),
        grasp_offset=LaunchConfiguration("grasp_offset"),
        visualization=visualization,
    )


def _perception(visualization):
    return _include(
        os.path.join(_share("g1_perception"), "launch", "perception.launch.py"),
        visualization=visualization,
        detector=LaunchConfiguration("detector"),
        grasp_engine=LaunchConfiguration("grasp_engine"),
        only_from_below=LaunchConfiguration("only_from_below"),
        grounding=LaunchConfiguration("grounding"),
        phrases=LaunchConfiguration("phrases"),
        mock_latency_s=LaunchConfiguration("mock_latency_s"),
        mock_rate_hz=LaunchConfiguration("mock_rate_hz"),
        mock_margin_m=LaunchConfiguration("mock_margin_m"),
    )


def _vla():
    return _include(
        os.path.join(_share("g1_vla"), "launch", "vla.launch.py"),
        engine=LaunchConfiguration("vla_engine"),
        execution_mode=LaunchConfiguration("vla_execution_mode"),
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


def _visualization(context, want_rviz):
    """Resolved here and forwarded as a literal: the children read it as a bool, and launch
    configurations are shared, so the empty default would reach them unresolved."""
    value = LaunchConfiguration("visualization").perform(context)
    shown = value.lower() == "true" if value else want_rviz
    return "true" if shown else "false"


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    want_nav = _flag(context, "nav")
    want_rviz = _flag(context, "rviz")
    want_moveit = _flag(context, "moveit")
    want_manipulation = _flag(context, "manipulation")
    want_perception = _flag(context, "perception")
    want_vla = _flag(context, "vla")
    pin_pelvis = _flag(context, "pin_pelvis")
    visualization = _visualization(context, want_rviz)
    navigating = mode != "none"

    _validate(mode, want_nav, want_moveit, want_manipulation, want_perception, want_vla,
              pin_pelvis)

    actions = [
        _simulator(
            _sim_args(context, navigating, want_manipulation, want_perception, want_moveit,
                      pin_pelvis)
        )
    ]
    if navigating:
        actions.append(_navigation(mode, want_nav))
    if want_moveit:
        actions.append(_moveit())
    if want_perception:
        actions.append(_perception(visualization))
    if want_manipulation:
        actions.append(_manipulation(want_perception, visualization))
    if want_vla:
        actions.append(_vla())
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
            "visualization",
            default_value="",
            description="Everything drawn only for RViz: the annotated camera image, ground "
            "truth, /object_markers and the grasp plan. false starts none of it. Empty follows "
            "rviz, so an RViz opened by hand later needs visualization:=true.",
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
            "vla",
            default_value="false",
            description="Start the learned-grasp skill and its policy engine. Needs "
            "manipulation:=true.",
        ),
        DeclareLaunchArgument(
            "vla_engine",
            default_value="mock",
            description="Which policy engine answers with vla:=true. 'mock' needs no model; "
            "'groot' talks to a policy server running outside the container.",
        ),
        DeclareLaunchArgument(
            "vla_execution_mode",
            default_value="trajectory",
            choices=["trajectory", "servo"],
            description="How the grasp skill executes a validated chunk. 'servo' streams it "
            "through MoveIt Servo, which adds proximity slowdown while the arm is moving, and "
            "starts a servo_node alongside move_group.",
        ),
        DeclareLaunchArgument(
            "object_source",
            default_value="sim_ground_truth",
            description="Where object poses come from with manipulation:=true, when perception "
            "is off. 'sim_ground_truth' reads MuJoCo bodies; 'hardware' refuses to configure, "
            "because the robot has no detector of its own. perception:=true overrides this.",
        ),
        DeclareLaunchArgument(
            "perception",
            default_value="false",
            description="Detect objects from the camera instead of reading them out of the "
            "simulator. Requires manipulation:=true and forces sensors:=true.",
        ),
        DeclareLaunchArgument(
            "detector",
            default_value="mock",
            choices=["mock", "vision"],
            description="Which detector perception runs: 'mock' cuts masks from simulator "
            "ground truth and needs no GPU, 'vision' asks the host vision server.",
        ),
        DeclareLaunchArgument(
            "grasp_source",
            default_value="fixed_top_down",
            choices=["fixed_top_down", "generated"],
            description="Where a pick's grasp comes from. 'generated' needs grasp_engine set to "
            "something that answers.",
        ),
        DeclareLaunchArgument(
            "grounding",
            default_value="false",
            description="Runs the instruction grounder beside the detector. Needs the host "
            "vision server started with --vlm.",
        ),
        DeclareLaunchArgument(
            "only_from_below",
            default_value="false",
            description="Makes the stand-in grasp generator offer nothing but a grasp reaching "
            "up through the table, for testing that the filter refuses it.",
        ),
        DeclareLaunchArgument(
            "grasp_offset",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            description="The grasp generator's gripper frame to this robot's grasp frame, xyz "
            "then rpy. Measure it against the candidates in RViz before trusting it.",
        ),
        DeclareLaunchArgument(
            "grasp_engine",
            default_value="none",
            choices=["none", "mock", "graspgen"],
            description="Who answers for six-degree-of-freedom grasps: nobody, a stand-in that "
            "needs no GPU, or the GraspGenX server on the host.",
        ),
        DeclareLaunchArgument(
            "phrases",
            default_value="red block,blue block,green cylinder,blue sphere,yellow box,white cup,brown box",
            description="Comma separated objects the detector looks for.",
        ),
        DeclareLaunchArgument(
            "mock_latency_s",
            default_value="0.0",
            description="How far behind the camera the mock detector's masks are. 1.5 is what "
            "the real one costs.",
        ),
        DeclareLaunchArgument(
            "mock_rate_hz",
            default_value="10.0",
            description="How often the mock detector answers. The real one manages 0.7.",
        ),
        DeclareLaunchArgument(
            "mock_margin_m",
            default_value="0.005",
            description="How far past an object's box the mock's mask may spill; positive "
            "simulates a sloppy segmenter.",
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
            "to the workbench first; 'tabletop' is five of different shapes, for perception.",
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
