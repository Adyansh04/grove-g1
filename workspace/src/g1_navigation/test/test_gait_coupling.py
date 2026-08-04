"""Nav2's rotational limits must clear the gait shaper's engage threshold.

This is a cross-package invariant with nothing but a comment holding it up, and it has already
been broken once: behavior_server shipped upstream's max_rotational_vel of 1.0 against a
yaw_engage of 1.20, so the shaper zeroed every Spin command and the only recovery left in the
trees was a guaranteed no-op. Nothing failed -- Spin reported success, having commanded a
rotation that never reached the robot -- and it took a review to notice.

The two numbers live in different packages, so neither package's own tests can see the pair.
This one reads both shipped files.
"""

import os
import pathlib

import pytest
import yaml

NAV_CONFIG = pathlib.Path(
    os.environ.get("G1_NAVIGATION_CONFIG_DIR", pathlib.Path(__file__).parent.parent / "config")
)
LOCO_CONFIG = pathlib.Path(
    os.environ.get(
        "G1_LOCOMOTION_CONFIG_DIR",
        pathlib.Path(__file__).parent.parent.parent / "g1_locomotion" / "config",
    )
)


def _params(path, node):
    with open(path) as f:
        return yaml.safe_load(f)[node]["ros__parameters"]


@pytest.fixture(scope="module")
def yaw_engage():
    path = LOCO_CONFIG / "g1_gait_shaper.yaml"
    assert path.is_file(), f"{path} is missing; this test would silently pass"
    return _params(path, "g1_gait_shaper")["yaw_engage"]


@pytest.fixture(scope="module")
def behavior_server():
    path = NAV_CONFIG / "nav2_params.yaml"
    assert path.is_file(), f"{path} is missing; this test would silently pass"
    return _params(path, "behavior_server")


def test_spin_can_clear_the_deadband(behavior_server, yaw_engage):
    assert behavior_server["max_rotational_vel"] >= yaw_engage, (
        f"behavior_server max_rotational_vel {behavior_server['max_rotational_vel']} is below "
        f"g1_gait_shaper's yaw_engage {yaw_engage}. Every Spin would be clamped under the "
        f"threshold and zeroed, and Spin would still report success."
    )


def test_spin_does_not_decelerate_into_the_deadband(behavior_server, yaw_engage):
    # The floor matters as much as the ceiling: Spin ramps down approaching spin_dist, and
    # upstream's min of 0.4 drops it back under the threshold before it arrives.
    assert behavior_server["min_rotational_vel"] >= yaw_engage, (
        f"behavior_server min_rotational_vel {behavior_server['min_rotational_vel']} is below "
        f"yaw_engage {yaw_engage}; Spin's deceleration tail would stop the robot short."
    )
