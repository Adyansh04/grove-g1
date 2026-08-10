"""
The sensor mounts exist twice, and copies drift.

`workspace/vendor/unitree_mujoco/sensor_publisher.cc` carries `kMountXyz`/`kMountRpy` and
`kCamXyz`/`kCamRpy` as compile-time constants, because the simulator computes the sweep and the
render inside its own process and links no ROS: it cannot ask TF where the sensors are. The URDF
owns the same four numbers. Move one and not the other and the cloud arrives in a frame that
does not describe where it was taken from, which reads downstream as an odometry or a
calibration fault rather than as what it is.
"""

import math
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest

_URDF = (
    pathlib.Path(__file__).resolve().parent.parent
    / "urdf"
    / "g1_29dof_with_hand_rev_1_0.urdf"
)
_SENSOR_PUBLISHER = (
    pathlib.Path(__file__).resolve().parents[3] / "vendor" / "unitree_mujoco" / "sensor_publisher.cc"
)

# Which simulator constant mirrors which URDF joint. Both are expressed in torso_link, which is
# the body the simulator resolves by name.
_MOUNTS = (("kMountXyz", "kMountRpy", "mid360_joint"), ("kCamXyz", "kCamRpy", "d435_joint"))


def _urdf_joint_origin(joint_name):
    root = ET.parse(_URDF).getroot()
    for joint in root.findall("joint"):
        if joint.get("name") != joint_name:
            continue
        assert joint.find("parent").get("link") == "torso_link", (
            f"{joint_name} no longer hangs off torso_link; the simulator resolves that body by "
            "name and composes these constants onto it"
        )
        origin = joint.find("origin")
        return (
            [float(v) for v in origin.get("xyz").split()],
            [float(v) for v in origin.get("rpy").split()],
        )
    raise AssertionError(f"{joint_name} not found in {_URDF.name}")


def _sim_constant(name):
    """Parse the doubles out of `constexpr double <name>[3] = {...};`."""
    text = _SENSOR_PUBLISHER.read_text()
    match = re.search(rf"constexpr double {name}\[3\]\s*=\s*{{([^}}]*)}}", text)
    assert match, f"{name} not found in {_SENSOR_PUBLISHER.name}"
    values = []
    for token in match.group(1).split(","):
        token = token.strip()
        # The rpy constants are written with M_PI rather than a decimal.
        values.append(math.pi if token == "M_PI" else float(token))
    return values


@pytest.mark.parametrize(("xyz_name", "rpy_name", "joint_name"), _MOUNTS)
def test_the_simulator_mounts_match_the_urdf(xyz_name, rpy_name, joint_name):
    xyz, rpy = _urdf_joint_origin(joint_name)
    assert _sim_constant(xyz_name) == pytest.approx(xyz, abs=1e-9), (
        f"{xyz_name} vs {joint_name}'s xyz"
    )
    assert _sim_constant(rpy_name) == pytest.approx(rpy, abs=1e-9), (
        f"{rpy_name} vs {joint_name}'s rpy"
    )


def test_the_two_copies_of_the_wire_header_are_the_same_file():
    # Same class of duplication, one directory over: g1_sensor_relay's test asserts this too,
    # and it is cheap to catch here as well since a mismatch breaks the sensor path entirely.
    vendor = _SENSOR_PUBLISHER.parent / "sensor_frame.h"
    package = (
        _SENSOR_PUBLISHER.parents[2]
        / "src"
        / "g1_sensor_relay"
        / "include"
        / "g1_sensor_relay"
        / "sensor_frame.h"
    )
    assert vendor.read_bytes() == package.read_bytes(), (
        "workspace/vendor/unitree_mujoco/sensor_frame.h and g1_sensor_relay's copy differ"
    )
