"""
The Inspire hands exist twice, in the URDF and in the simulator, and copies drift.

The simulator's `g1_29dof_inspire.xml` (submodule `workspace/vendor/unitree_mujoco`) is generated
from this package's URDF by g1_bringup's `tools/inspire_hand_mjcf.py`, and its
`simulate/src/inspire_handler.cc` maps each actuator's angle register to its joint with constants
that mirror `config/inspire_params.yaml`. Change one copy and not the other, and the simulated hand
moves to angles the driver did not ask for, or MoveIt plans for fingers that are not where it thinks.
"""

import pathlib
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

_HERE = pathlib.Path(__file__).resolve().parent.parent
_URDF = _HERE / "urdf" / "g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf"
_PARAMS = _HERE / "config" / "inspire_params.yaml"
_FORK = pathlib.Path(__file__).resolve().parents[3] / "vendor" / "unitree_mujoco"
_MJCF = _FORK / "unitree_robots" / "g1" / "g1_29dof_inspire.xml"
_HANDLER = _FORK / "simulate" / "src" / "inspire_handler.cc"


def _hand_joints():
    root = ET.parse(_URDF).getroot()
    return [j for j in root.findall("joint") if j.get("name")[:2] in ("L_", "R_")]


def _mjcf():
    return ET.parse(_MJCF).getroot()


def _floats(text):
    return [float(v) for v in text.split()]


@pytest.mark.parametrize("joint", _hand_joints(), ids=lambda j: j.get("name"))
def test_each_hand_body_and_joint_matches_the_urdf(joint):
    root = _mjcf()
    child = joint.find("child").get("link")
    body = root.find(f".//body[@name='{child}']")
    assert body is not None, f"{child} is not in {_MJCF.name}: rerun tools/inspire_hand_mjcf.py"
    assert _floats(body.get("pos")) == pytest.approx(
        _floats(joint.find("origin").get("xyz")), abs=1e-6
    )
    if joint.get("type") != "revolute":
        return
    sim = body.find(f"joint[@name='{joint.get('name')}']")
    assert sim is not None
    assert _floats(sim.get("axis")) == pytest.approx(_floats(joint.find("axis").get("xyz")))
    limit = joint.find("limit")
    assert _floats(sim.get("range")) == pytest.approx(
        [float(limit.get("lower")), float(limit.get("upper"))]
    )


def test_the_coupled_joints_follow_as_the_urdf_mimics_say():
    couplings = {e.get("joint1"): e for e in _mjcf().iter("joint") if e.get("polycoef") is not None}
    mimics = [j for j in _hand_joints() if j.find("mimic") is not None]
    assert len(mimics) == 12
    for joint in mimics:
        mimic = joint.find("mimic")
        coupling = couplings[joint.get("name")]
        assert coupling.get("joint2") == mimic.get("joint")
        offset, multiplier = _floats(coupling.get("polycoef"))[:2]
        assert offset == pytest.approx(float(mimic.get("offset", "0")))
        assert multiplier == pytest.approx(float(mimic.get("multiplier", "1")))


def _handler_array(name):
    """Parse the values out of `constexpr std::array<..., kActuators> <name> = { ... };`."""
    text = _HANDLER.read_text()
    match = re.search(rf"{name}\s*=\s*{{([^}}]*)}}", text)
    assert match, f"{name} not found in {_HANDLER.name}"
    return [token.strip().strip('"') for token in match.group(1).split(",") if token.strip()]


def test_the_simulated_firmware_maps_angles_as_the_driver_does():
    actuators = yaml.safe_load(_PARAMS.read_text())["actuators"]
    suffixes = _handler_array("kJointSuffixes")
    # Register order is the YAML's key order and the simulator's array order alike.
    assert suffixes == list(actuators)
    for key, field in (
        ("kOpenRad", "open_rad"),
        ("kClosedRad", "closed_rad"),
        ("kLeverArmM", "lever_arm_m"),
    ):
        assert [float(v) for v in _handler_array(key)] == pytest.approx(
            [actuators[s][field] for s in suffixes]
        ), key
