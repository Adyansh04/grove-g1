"""No shipped config may enable use_sim_time.

There is no /clock on this track: the simulator links no ROS at all, so a node with
use_sim_time gets a clock that never advances and the symptom surfaces as a TF lookup failing
somewhere unrelated. The upstream configs this package adapts all ship it true.
"""

import os
import pathlib
import re

import pytest

# Matches the parameter at any indentation, true in any casing. Deliberately not a YAML
# parse: a value under a node name we do not know still has to be caught.
SIM_TIME_TRUE = re.compile(r"^\s*use_sim_time\s*:\s*(true|True|TRUE)\s*(#.*)?$")

CONFIG_DIR = pathlib.Path(
    os.environ.get("G1_NAVIGATION_CONFIG_DIR", pathlib.Path(__file__).parent.parent / "config")
)
# Test-only overrides are copies of the shipped configs and inherit the same trap.
YAMLS = sorted(CONFIG_DIR.glob("*.yaml")) + sorted(pathlib.Path(__file__).parent.glob("*.yaml"))


def test_config_dir_exists():
    assert CONFIG_DIR.is_dir(), f"{CONFIG_DIR} is missing; this test would silently pass"


@pytest.mark.parametrize("path", YAMLS, ids=lambda p: p.name)
def test_use_sim_time_is_never_true(path):
    offenders = [
        f"{path.name}:{n}: {line.rstrip()}"
        for n, line in enumerate(path.read_text().splitlines(), start=1)
        if SIM_TIME_TRUE.match(line)
    ]
    assert not offenders, "use_sim_time must be false on this track:\n" + "\n".join(offenders)
