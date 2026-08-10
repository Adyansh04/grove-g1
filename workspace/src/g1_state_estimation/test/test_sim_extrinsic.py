"""In simulation there is no constant lidar-to-IMU extrinsic to write down.

FAST-LIO reads a flat parameter file and cannot look at TF, so the obvious thing is to copy the
mount out of the URDF as twelve literal numbers. That was done, and it was wrong: the sim IMU is
the pelvis, the sim lidar is on torso_link, and the walking policy drives the three waist joints
in between. g1_livox_bridge restates the sweep in the pelvis frame instead, which leaves the
extrinsic identity. These tests hold that arrangement in place.
"""

import math
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

_CONFIG = pathlib.Path(__file__).resolve().parent.parent / "config" / "fastlio_mid360_sim.yaml"


def _rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_mul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _joints_from_urdf():
    urdf = (
        pathlib.Path(get_package_share_directory("g1_description"))
        / "urdf"
        / "g1_29dof_with_hand_rev_1_0.urdf"
    )
    root = ET.parse(urdf).getroot()
    joints = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz_text = origin.get("xyz") if origin is not None else None
        rpy_text = origin.get("rpy") if origin is not None else None
        xyz = [float(v) for v in (xyz_text or "0 0 0").split()]
        rpy = [float(v) for v in (rpy_text or "0 0 0").split()]
        joints[joint.find("child").get("link")] = (
            joint.find("parent").get("link"),
            xyz,
            _rpy_to_matrix(*rpy),
            joint.get("type"),
        )
    return joints


def _chain_from_urdf(child_link, ancestor_link):
    """mid360-in-pelvis from the URDF, every joint on the way at zero."""
    joints = _joints_from_urdf()
    rotation = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    translation = [0.0, 0.0, 0.0]
    link = child_link
    while link != ancestor_link:
        assert link in joints, f"no joint leads to {link}"
        parent, xyz, joint_rotation, _ = joints[link]
        translation = [c + v for c, v in zip(xyz, _mat_vec(joint_rotation, translation))]
        rotation = _mat_mul(joint_rotation, rotation)
        link = parent
    return translation, rotation


def _config_extrinsic():
    parameters = yaml.safe_load(_CONFIG.read_text())["/**"]["ros__parameters"]["mapping"]
    return parameters["extrinsic_T"], parameters["extrinsic_R"]


def test_sim_extrinsic_is_identity():
    config_t, config_r = _config_extrinsic()
    assert config_t == pytest.approx([0.0, 0.0, 0.0], abs=1e-12), (
        "the sweep reaches FAST-LIO already in the IMU frame; a translation here double-counts "
        "the mount"
    )
    assert config_r == pytest.approx([1, 0, 0, 0, 1, 0, 0, 0, 1], abs=1e-12), (
        "same for the rotation -- g1_livox_bridge has already applied it, per scan"
    )


def test_the_sensor_is_not_rigidly_attached_to_the_imu():
    # The reason the extrinsic above cannot be a constant, asserted rather than remembered.
    joints = _joints_from_urdf()
    link = "mid360_link"
    movable = []
    while link != "pelvis":
        parent, _, _, kind = joints[link]
        if kind != "fixed":
            movable.append(link)
        link = parent
    assert movable, (
        "mid360_link now reaches pelvis through fixed joints only. If the waist really was "
        "frozen, a constant extrinsic is correct again and g1_livox_bridge's per-scan transform "
        "is dead weight -- but check who froze it before deleting anything."
    )


def test_the_mount_is_actually_upside_down():
    # Guards the URDF side of the comparison: if someone rights the sensor there, the config
    # comparison above would happily follow it, and this is the assertion that asks whether
    # that was meant. The physical Mid360 on a G1 points its +z at the floor.
    _, rotation = _chain_from_urdf("mid360_link", "pelvis")
    assert rotation[2][2] < -0.9, "mid360 +z no longer points down; was the mount changed?"


def test_the_imu_frame_inverts_the_livox_lever_arm():
    # mid360_imu is hand-written as the inverse of Livox's published lidar-in-IMU offset;
    # check the two cancel instead of trusting the sign flip was done right.
    xacro = (
        pathlib.Path(get_package_share_directory("g1_description"))
        / "urdf"
        / "g1_arm_sdk.urdf.xacro"
    )
    match = re.search(
        r'<child link="mid360_imu"/>\s*<origin xyz="([^"]+)"',
        xacro.read_text(),
    )
    assert match, "mid360_imu joint not found in g1_arm_sdk.urdf.xacro"
    offset = [float(v) for v in match.group(1).split()]
    livox_published = [-0.011, -0.02329, 0.04412]
    for axis in range(3):
        assert offset[axis] == pytest.approx(-livox_published[axis], abs=1e-9)
