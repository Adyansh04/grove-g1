"""The sim FAST-LIO extrinsic is a copy of the URDF mount, and copies drift.

fastlio_mid360_sim.yaml carries mid360_link expressed in the pelvis frame as twelve literal
numbers, because FAST-LIO reads a flat parameter file and cannot look at TF. The URDF is the
source of truth for the same transform. This recomputes the chain from the URDF, waist at
zero, and fails if someone moves the sensor in one place and not the other.
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


def _chain_from_urdf(child_link, ancestor_link):
    """mid360-in-pelvis from the URDF, every joint on the way at zero."""
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
        )

    rotation = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    translation = [0.0, 0.0, 0.0]
    link = child_link
    while link != ancestor_link:
        assert link in joints, f"no joint leads to {link}"
        parent, xyz, joint_rotation = joints[link]
        translation = [c + v for c, v in zip(xyz, _mat_vec(joint_rotation, translation))]
        rotation = _mat_mul(joint_rotation, rotation)
        link = parent
    return translation, rotation


def _config_extrinsic():
    parameters = yaml.safe_load(_CONFIG.read_text())["/**"]["ros__parameters"]["mapping"]
    return parameters["extrinsic_T"], parameters["extrinsic_R"]


def test_sim_extrinsic_matches_the_urdf_mount():
    translation, rotation = _chain_from_urdf("mid360_link", "pelvis")
    config_t, config_r = _config_extrinsic()

    for axis in range(3):
        assert config_t[axis] == pytest.approx(translation[axis], abs=1e-5), (
            f"extrinsic_T[{axis}]: config {config_t[axis]} vs URDF {translation[axis]}"
        )
    flat = [rotation[i][j] for i in range(3) for j in range(3)]
    for index in range(9):
        assert config_r[index] == pytest.approx(flat[index], abs=1e-6), (
            f"extrinsic_R[{index}]: config {config_r[index]} vs URDF {flat[index]}"
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
