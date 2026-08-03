"""No shipped config may enable use_sim_time.

The converged track has no /clock -- the simulator links no ROS at all. Every upstream
config this package adapts (nav2_bringup's nav2_params.yaml, slam_toolbox's mapper_params_*)
ships use_sim_time: True, so this is a copy-paste hazard rather than a hypothetical one, and
one missed block gives that node a clock that never advances. The symptom is a TF lookup
failing in some unrelated node, which points nowhere near the cause.
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


def test_config_dir_exists():
    assert CONFIG_DIR.is_dir(), f"{CONFIG_DIR} is missing; this test would silently pass"


@pytest.mark.parametrize("path", sorted(CONFIG_DIR.glob("*.yaml")), ids=lambda p: p.name)
def test_use_sim_time_is_never_true(path):
    offenders = [
        f"{path.name}:{n}: {line.rstrip()}"
        for n, line in enumerate(path.read_text().splitlines(), start=1)
        if SIM_TIME_TRUE.match(line)
    ]
    assert not offenders, "use_sim_time must be false on this track:\n" + "\n".join(offenders)
