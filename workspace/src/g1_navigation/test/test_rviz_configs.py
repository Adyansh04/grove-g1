"""The sensor display group is duplicated across two packages and must not drift.

g1_bringup/config/g1_sensors.rviz and g1_navigation/config/g1_navigation.rviz are two static
files rather than one generated at launch: they differ in Fixed Frame as well as in which
groups exist. The price is drift, and neither package's own tests can see the pair.
"""

import os

import pytest
import yaml

BRINGUP_CONFIG_DIR = os.environ["G1_BRINGUP_CONFIG_DIR"]
NAVIGATION_CONFIG_DIR = os.environ["G1_NAVIGATION_CONFIG_DIR"]

# nav2_rviz_plugins is the only non-default plugin either config may use, and only the
# navigation one may use it: g1_bringup shipping it would mean a Nav2 dependency.
NAV2_PLUGIN_PREFIX = "nav2_rviz_plugins/"


def _displays(path):
    with open(path) as handle:
        return yaml.safe_load(handle)["Visualization Manager"]


def _group(displays, name):
    for item in displays["Displays"]:
        if item.get("Class") == "rviz_common/Group" and item.get("Name") == name:
            return item
    return None


@pytest.fixture(scope="module")
def sensors():
    return _displays(os.path.join(BRINGUP_CONFIG_DIR, "g1_sensors.rviz"))


@pytest.fixture(scope="module")
def navigation():
    return _displays(os.path.join(NAVIGATION_CONFIG_DIR, "g1_navigation.rviz"))


def test_both_configs_carry_a_sensor_group(sensors, navigation):
    assert _group(sensors, "Sensors") is not None
    assert _group(navigation, "Sensors") is not None


def test_the_sensor_group_contents_are_identical(sensors, navigation):
    """Everything except the group's own Enabled flag, which is the point of the split."""
    a = _group(sensors, "Sensors")["Displays"]
    b = _group(navigation, "Sensors")["Displays"]
    assert a == b, (
        "the Sensors group has drifted between g1_bringup and g1_navigation; "
        "edit both or neither"
    )


def test_the_two_configs_disagree_only_where_intended(sensors, navigation):
    # Enabled differs by design: navigation starts with sensors folded away.
    assert _group(sensors, "Sensors")["Enabled"] is True
    assert _group(navigation, "Sensors")["Enabled"] is False


def test_only_the_navigation_config_has_a_nav2_group(sensors, navigation):
    assert _group(navigation, "Nav2") is not None
    assert _group(sensors, "Nav2") is None


def test_fixed_frames_match_what_each_mode_actually_publishes(sensors, navigation):
    # There is no map frame without SLAM or AMCL, so the bare config cannot use one.
    assert sensors["Global Options"]["Fixed Frame"] == "odom"
    assert navigation["Global Options"]["Fixed Frame"] == "map"


def test_g1_bringup_ships_no_nav2_plugin_classes(sensors):
    """The constraint that put the merged config in g1_navigation in the first place."""

    def classes(node):
        if isinstance(node, dict):
            if "Class" in node and isinstance(node["Class"], str):
                yield node["Class"]
            for value in node.values():
                yield from classes(value)
        elif isinstance(node, list):
            for value in node:
                yield from classes(value)

    offenders = [c for c in classes(sensors) if c.startswith(NAV2_PLUGIN_PREFIX)]
    assert not offenders, (
        f"g1_bringup's RViz config uses {offenders}, which would give the bring-up package a "
        "runtime dependency on nav2_rviz_plugins"
    )


def test_the_navigation_config_actually_uses_the_amcl_swarm(navigation):
    # The display that forced the split; if it is gone, the split has no reason to exist.
    nav2 = _group(navigation, "Nav2")
    assert any(d.get("Class", "").startswith(NAV2_PLUGIN_PREFIX) for d in nav2["Displays"])
