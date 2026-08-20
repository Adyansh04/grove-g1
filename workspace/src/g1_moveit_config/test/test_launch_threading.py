"""Arguments must survive the include boundary bringup.launch.py's moveit branch introduces.

The same failure mode g1_navigation's test_launch_threading exists for: an included launch
file inherits the parent's configurations, so a wrong value arrives quietly and the launch
looks successful. This file covers the half that one cannot.

The split is forced, not stylistic. bringup's moveit branch calls
get_package_share_directory("g1_moveit_config"), so those assertions cannot live in
g1_navigation, because a workspace built without this package would hit the actionable RuntimeError
instead of the assertion. The reverse holds for the navigation branch, which is why that
package keeps its own copy.

_run_setup / _includes / _included_path below are copied from
g1_navigation/test/test_launch_threading.py rather than shared. They are test-only, they reach
into launch internals that move only on a distro bump, and each file uses its own copy, so a
drift between them cannot mislead anyone. Extracting them would mean giving g1_bringup a
Python package it does not otherwise have, to save sixty lines once. If a third consumer
appears, that trade flips: move them into an installed module in g1_bringup then.
"""

import importlib.util
import os

import pytest
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

BRINGUP_LAUNCH_DIR = os.environ["G1_BRINGUP_LAUNCH_DIR"]
MOVEIT_LAUNCH_DIR = os.environ["G1_MOVEIT_LAUNCH_DIR"]


def _load(launch_dir, name):
    path = os.path.join(launch_dir, name)
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_setup(module, **configurations):
    """Execute a launch file's OpaqueFunction body with the given arguments already set."""
    context = LaunchContext()
    for key, value in configurations.items():
        context.launch_configurations[key] = value
    for action in module.generate_launch_description().entities:
        name = getattr(action, "name", None)
        if name is not None and name not in context.launch_configurations:
            default = getattr(action, "default_value", None)
            if default is not None:
                context.launch_configurations[name] = perform_substitutions(context, default)
    return module._setup(context), context


def _resolve(context, value):
    if isinstance(value, str):
        return value
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _included_path(context, action):
    """The path an include points at, without loading it."""
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
def moveit_sim():
    return _load(MOVEIT_LAUNCH_DIR, "moveit_sim.launch.py")


# --- bringup's moveit branch ----------------------------------------------------------


def test_the_moveit_branch_stages_a_simulator_and_move_group(bringup):
    names = [name for name, _ in _includes(_run_setup(bringup, mode="none", moveit="true"))]
    assert names == ["sim.launch.py", "move_group.launch.py"]


def test_moveit_gets_a_simulator_with_sensors(bringup):
    """move_group refuses to plan until every active joint has a state.

    joint_state_broadcaster covers all 29 body motors from the hardware component, so the
    only thing MoveIt still needs staged for it is the simulator itself.
    """
    includes = dict(_includes(_run_setup(bringup, mode="none", moveit="true")))
    assert "sim.launch.py" in includes
    assert "move_group.launch.py" in includes

    bare = dict(_includes(_run_setup(bringup, mode="none")))["sim.launch.py"]
    assert "non_arm_joint_states" not in bare


@pytest.mark.parametrize("mode", ["none", "localization"])
def test_moveit_gets_the_loaded_start_delay(bringup, mode):
    # move_group starts alongside the simulator, so even mode:=none is not a bare launch.
    sim = dict(_includes(_run_setup(bringup, mode=mode, moveit="true")))["sim.launch.py"]
    assert sim["sim_start_delay_s"] == "4.0"


def test_moveit_rviz_wins_wherever_both_are_asked_for(bringup):
    """Substituting g1_navigation.rviz into moveit_rviz.launch.py launches cleanly and leaves
    the MotionPlanning panel absent, because that panel comes from the config's display list and
    the nav config has none. So MoveIt's own config is what runs, in every mode.
    """
    for mode in ("none", "localization"):
        rviz = [
            (name, args)
            for name, args in _includes(
                _run_setup(bringup, mode=mode, moveit="true", nav="false", rviz="true")
            )
            if "rviz" in name
        ]
        assert len(rviz) == 1, mode
        assert rviz[0][0] == "moveit_rviz.launch.py", mode
        # No rviz_config: bringup never declares that name, so the child's own default is the
        # MoveIt config and there is nothing to inherit from.
        assert rviz[0][1] == {}, mode


def test_the_mode_config_still_wins_without_moveit(bringup):
    rviz = dict(_includes(_run_setup(bringup, mode="localization", rviz="true")))
    assert "rviz.launch.py" in rviz
    assert rviz["rviz.launch.py"]["rviz_config"].endswith("g1_navigation/config/g1_navigation.rviz")


def test_the_arm_development_combination_is_not_blocked(bringup):
    """pin_pelvis is refused on the navigation modes but must stay available with MoveIt: a
    balancing robot sways, and the whole arm chain hangs off the pelvis."""
    sim = dict(
        _includes(_run_setup(bringup, mode="none", moveit="true", pin_pelvis="true"))
    )["sim.launch.py"]
    assert sim["pin_pelvis"] == "true"


def test_no_moveit_means_g1_moveit_config_is_never_named(bringup):
    """Keeps a workspace without this package launchable, which is what the undeclared
    dependency buys and what _share() would otherwise refuse."""
    for mode in ("none", "localization"):
        actions, context = _run_setup(bringup, mode=mode, rviz="true")
        for action in actions:
            if isinstance(action, IncludeLaunchDescription):
                assert "g1_moveit_config" not in _included_path(context, action)


def test_a_missing_package_is_reported_actionably(bringup, monkeypatch):
    def absent(name):
        raise PackageNotFoundError(name)

    monkeypatch.setattr(bringup, "get_package_share_directory", absent)
    with pytest.raises(RuntimeError, match="colcon build --packages-select g1_moveit_config"):
        bringup._share("g1_moveit_config")


# --- the standalone wrapper, which this refactor must leave alone ---------------------


def test_moveit_sim_still_composes_the_same_two_pieces(moveit_sim):
    includes = _includes(_run_setup(moveit_sim))
    assert [name for name, _ in includes] == ["sim.launch.py", "move_group.launch.py"]

    sim = dict(includes)["sim.launch.py"]
    # The other copy of each fact bringup states; if these drift, one path plans and the other
    # silently does not.
    assert sim["sensors"] == "false"
    assert sim["rviz"] == "false"
