"""Arguments must survive every include boundary bringup.launch.py introduces.

Included launch files inherit the parent's configurations, so a child's own
DeclareLaunchArgument default never fires for anything the parent declared. Nothing crashes
when that happens; the wrong value simply arrives. So this walks the includes rather than
launching them, resolving what each file actually forwards. It needs no simulator and no DDS.

Both callers of the same two pieces are checked: bringup.launch.py and nav_sim.launch.py each
stage a simulator and include nav_stack.launch.py, which is itself checked for staging no
simulator, since that is what makes including it next to one safe.

Lives in g1_navigation because it reads both packages' launch files and g1_bringup cannot
depend on g1_navigation.
"""

import importlib.util
import os

import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

BRINGUP_LAUNCH_DIR = os.environ["G1_BRINGUP_LAUNCH_DIR"]
NAVIGATION_LAUNCH_DIR = os.environ["G1_NAVIGATION_LAUNCH_DIR"]


def _load(launch_dir, name):
    path = os.path.join(launch_dir, name)
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_setup(module, **configurations):
    """Execute a launch file's OpaqueFunction body with the given arguments already set.

    Returns (actions, context). The context has to come back with them: forwarded values are
    substitutions, and resolving them against a fresh context fails with "launch
    configuration does not exist" rather than reporting what was actually threaded.
    """
    context = LaunchContext()
    for key, value in configurations.items():
        context.launch_configurations[key] = value
    # Defaults for anything the caller did not set, mirroring DeclareLaunchArgument.
    for action in module.generate_launch_description().entities:
        name = getattr(action, "name", None)
        if name is not None and name not in context.launch_configurations:
            default = getattr(action, "default_value", None)
            if default is not None:
                context.launch_configurations[name] = perform_substitutions(context, default)
    return module._setup(context), context


def _resolve(context, value):
    """launch normalises plain strings to substitutions, so nothing here is a str."""
    if isinstance(value, str):
        return value
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _included_path(context, action):
    """The path an include points at, without loading it.

    LaunchDescriptionSource.location repr()s its substitutions until the source is loaded, and
    loading it would execute the included launch file. The unexpanded substitutions are only
    reachable through the name-mangled attribute.
    """
    subs = action.launch_description_source._LaunchDescriptionSource__location
    return perform_substitutions(context, subs)


def _includes(setup_result):
    """Every IncludeLaunchDescription in a _run_setup result as (basename, {arg: value})."""
    actions, context = setup_result
    found = []
    for action in actions:
        if not isinstance(action, IncludeLaunchDescription):
            continue
        args = {
            _resolve(context, k): _resolve(context, v) for k, v in action.launch_arguments
        }
        found.append((os.path.basename(_included_path(context, action)), args))
    return found


@pytest.fixture(scope="module")
def bringup():
    return _load(BRINGUP_LAUNCH_DIR, "bringup.launch.py")


@pytest.fixture(scope="module")
def nav_sim():
    return _load(NAVIGATION_LAUNCH_DIR, "nav_sim.launch.py")


@pytest.fixture(scope="module")
def nav_stack():
    return _load(NAVIGATION_LAUNCH_DIR, "nav_stack.launch.py")


def _nodes(setup_result):
    """Every Node action, which is how the component container shows up."""
    actions, _ = setup_result
    return [a for a in actions if isinstance(a, Node)]


# --- nav_stack.launch.py: the stack itself, with no simulator under it ----------------


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_nav_stack_never_stages_a_simulator(nav_stack, mode):
    """Both callers stage their own simulator, so a second one here would put two writers on
    rt/lowcmd at once and the robot collapses."""
    nav = "false" if mode == "mapping" else "true"
    actions, context = _run_setup(nav_stack, mode=mode, nav=nav)
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            path = _included_path(context, action)
            assert "sim.launch.py" not in path, path
            assert "g1_bringup" not in path, path


def test_nav_stack_includes_the_pipeline_for_each_mode(nav_stack):
    mapping = [name for name, _ in _includes(_run_setup(nav_stack, mode="mapping"))]
    assert mapping == ["scan.launch.py", "slam.launch.py"]

    localization = [name for name, _ in _includes(_run_setup(nav_stack, mode="localization"))]
    assert localization == ["scan.launch.py", "localization.launch.py"]

    with_nav = [
        name for name, _ in _includes(_run_setup(nav_stack, mode="localization", nav="true"))
    ]
    assert with_nav == ["scan.launch.py", "localization.launch.py", "nav2.launch.py"]


def test_only_the_composable_pieces_are_told_about_the_container(nav_stack):
    # slam_toolbox is deliberately left out of the container (40 MB serialization stack), so it
    # must not be handed a container name.
    includes = dict(_includes(_run_setup(nav_stack, mode="mapping")))
    assert includes["scan.launch.py"]["container_name"] == "nav2_container"
    assert includes["slam.launch.py"] == {}

    localization = dict(_includes(_run_setup(nav_stack, mode="localization")))
    assert localization["localization.launch.py"]["container_name"] == "nav2_container"


def test_nav2_is_never_composed(nav_stack):
    """Composed, the costmaps silently fall back to Costmap2DROS's built-in defaults and
    controller_server hangs in Activating forever."""
    includes = dict(_includes(_run_setup(nav_stack, mode="localization", nav="true")))
    assert includes["nav2.launch.py"]["use_composition"] == "false"


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_exactly_one_container_is_created(nav_stack, mode):
    assert len(_nodes(_run_setup(nav_stack, mode=mode))) == 1


def test_nav_stack_refuses_bad_input(nav_stack):
    with pytest.raises(RuntimeError, match="needs mode:=localization"):
        _run_setup(nav_stack, mode="mapping", nav="true")
    with pytest.raises(RuntimeError, match="is not a mode"):
        _run_setup(nav_stack, mode="slam")


# --- boundary 1: bringup.launch.py -> the stack below it ------------------------------


def _sim_args(setup_result):
    """The arguments bringup hands its one simulator, looked up by name rather than by index:
    bringup emits several includes and their order is not what these tests are about."""
    sims = [args for name, args in _includes(setup_result) if name == "sim.launch.py"]
    assert len(sims) == 1, f"expected exactly one simulator, got {len(sims)}"
    return sims[0]


def test_bare_mode_stages_only_a_simulator_and_names_no_optional_package(bringup):
    result = _run_setup(bringup, mode="none")
    assert [name for name, _ in _includes(result)] == ["sim.launch.py"]
    # The undeclared reference exists so this path never touches the package.
    actions, context = result
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            assert "g1_navigation" not in _included_path(context, action)


def test_navigation_modes_stage_the_simulator_then_the_stack(bringup):
    for mode in ("mapping", "localization"):
        includes = _includes(_run_setup(bringup, mode=mode))
        assert [name for name, _ in includes] == ["sim.launch.py", "nav_stack.launch.py"], mode
        assert dict(includes)["nav_stack.launch.py"]["mode"] == mode


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_the_navigation_branch_forces_sensors_on(bringup, mode):
    """Navigation needs the LiDAR, the relay and the odom chain, so an operator asking for
    sensors:=false on a navigation mode gets them anyway rather than a stack that cannot see."""
    args = _sim_args(_run_setup(bringup, mode=mode, sensors="false"))
    assert args["sensors"] == "true"


def test_the_bare_branch_leaves_sensors_to_the_operator(bringup):
    assert _sim_args(_run_setup(bringup, mode="none", sensors="false"))["sensors"] == "false"
    assert _sim_args(_run_setup(bringup, mode="none", sensors="true"))["sensors"] == "true"


def test_the_nav_flag_reaches_the_stack(bringup):
    includes = dict(_includes(_run_setup(bringup, mode="localization", nav="true")))
    assert includes["nav_stack.launch.py"]["nav"] == "true"
    includes = dict(_includes(_run_setup(bringup, mode="localization")))
    assert includes["nav_stack.launch.py"]["nav"] == "false"


def test_the_container_arguments_are_left_to_the_stacks_own_defaults(bringup):
    # Not forwarded and not declared here, which is the safe case: inheritance only leaks
    # through a name the parent also declares.
    forwarded = dict(_includes(_run_setup(bringup, mode="localization")))["nav_stack.launch.py"]
    assert "use_composition" not in forwarded
    assert "container_name" not in forwarded


@pytest.mark.parametrize(
    "mode,expected", [("none", "2.0"), ("mapping", "4.0"), ("localization", "4.0")]
)
def test_each_branch_supplies_its_own_start_delay(bringup, mode, expected):
    # The child's own default can never fire, so the branch has to pick a concrete value.
    # Getting this wrong hands sim.launch.py an empty string and float() raises.
    assert _sim_args(_run_setup(bringup, mode=mode))["sim_start_delay_s"] == expected


@pytest.mark.parametrize("mode", ["none", "mapping", "localization"])
def test_an_explicit_start_delay_beats_the_branch_default(bringup, mode):
    args = _sim_args(_run_setup(bringup, mode=mode, sim_start_delay_s="9.5"))
    assert args["sim_start_delay_s"] == "9.5"


@pytest.mark.parametrize("mode", ["none", "mapping", "localization"])
def test_world_threads_through_unchanged(bringup, mode):
    assert _sim_args(_run_setup(bringup, mode=mode, world="perception"))["world"] == "perception"


@pytest.mark.parametrize("mode", ["none", "mapping", "localization"])
def test_rviz_is_always_suppressed_in_the_simulator(bringup, mode):
    # Without an explicit false the child inherits rviz:=true and opens a second window on the
    # wrong config.
    assert _sim_args(_run_setup(bringup, mode=mode, rviz="true"))["rviz"] == "false"


def test_pin_pelvis_threads_to_the_simulator(bringup):
    assert _sim_args(_run_setup(bringup, mode="none", pin_pelvis="true"))["pin_pelvis"] == "true"
    assert _sim_args(_run_setup(bringup, mode="none"))["pin_pelvis"] == "false"


def test_rviz_config_is_chosen_per_mode(bringup):
    bare = _includes(_run_setup(bringup, mode="none", rviz="true"))
    nav = _includes(_run_setup(bringup, mode="localization", rviz="true"))
    bare_rviz = [a for name, a in bare if name == "rviz.launch.py"]
    nav_rviz = [a for name, a in nav if name == "rviz.launch.py"]
    assert len(bare_rviz) == 1 and len(nav_rviz) == 1
    assert bare_rviz[0]["rviz_config"].endswith("g1_bringup/config/g1_sensors.rviz")
    assert nav_rviz[0]["rviz_config"].endswith("g1_navigation/config/g1_navigation.rviz")


def test_no_rviz_include_when_not_asked(bringup):
    for mode in ("none", "localization"):
        names = [name for name, _ in _includes(_run_setup(bringup, mode=mode, rviz="false"))]
        assert "rviz.launch.py" not in names, mode


# --- boundary 2: the value keeps its meaning all the way down to sim.launch.py --------


def test_nav_sim_composes_one_simulator_and_one_stack(nav_sim):
    """nav_sim is not in bringup's path, so its own contract is asserted directly."""
    names = [name for name, _ in _includes(_run_setup(nav_sim, mode="localization"))]
    assert names == ["sim.launch.py", "nav_stack.launch.py"]


def test_nav_sim_forces_sensors_and_suppresses_rviz(nav_sim):
    """The same fact bringup asserts above. If the two disagree, navigation silently loses its
    scan or gains a second RViz."""
    sim = dict(_includes(_run_setup(nav_sim, mode="mapping")))["sim.launch.py"]
    assert sim["sensors"] == "true"
    assert sim["rviz"] == "false"


def test_nav_sim_forwards_world_and_delay_to_the_simulator(nav_sim):
    sim = dict(
        _includes(_run_setup(nav_sim, mode="mapping", world="perception", sim_start_delay_s="9.5"))
    )["sim.launch.py"]
    assert sim["world"] == "perception"
    assert sim["sim_start_delay_s"] == "9.5"


def test_nav_sim_forwards_the_container_arguments_it_exposes(nav_sim):
    # Unlike bringup, nav_sim declares these, so it must forward them explicitly or its own
    # values would be the ones that never arrive.
    stack = dict(_includes(_run_setup(nav_sim, mode="localization", use_composition="false")))[
        "nav_stack.launch.py"
    ]
    assert stack["use_composition"] == "false"
    assert stack["container_name"] == "nav2_container"


def test_nav_sim_opens_rviz_only_when_asked(nav_sim):
    """Pins the resolve-before-include ordering: the sim include sets rviz=false in this scope,
    so reading rviz after it would find false and silently open no window at all."""
    assert len(_nodes(_run_setup(nav_sim, mode="mapping", rviz="true"))) == 1
    assert _nodes(_run_setup(nav_sim, mode="mapping", rviz="false")) == []


# --- the guards ----------------------------------------------------------------------


def test_nav_without_localization_is_refused(bringup):
    for mode in ("none", "mapping"):
        with pytest.raises(RuntimeError, match="needs mode:=localization"):
            _run_setup(bringup, mode=mode, nav="true")


def test_pin_pelvis_with_navigation_is_refused(bringup):
    with pytest.raises(RuntimeError, match="cannot drive anywhere"):
        _run_setup(bringup, mode="mapping", pin_pelvis="true")


def test_an_unknown_mode_is_refused(bringup):
    with pytest.raises(RuntimeError, match="is not a mode"):
        _run_setup(bringup, mode="slam")
