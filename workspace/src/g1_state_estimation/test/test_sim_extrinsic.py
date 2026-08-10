"""The lidar-to-IMU extrinsic is the same constant in simulation and on the robot.

It is a constant only because the IMU is inside the sensor in both places: the simulator models
one there (patches/unitree_mujoco/006-add-mid360-imu.patch) rather than substituting the pelvis
IMU, which sits three actuated waist joints away and made every scan arrive rotated by a
different wrong amount. These tests hold both halves of that in place -- the numbers, and the
reason they cannot come from the URDF chain.
"""

import math
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

_CONFIG_DIR = pathlib.Path(__file__).resolve().parent.parent / "config"
# Livox's published lidar-in-IMU offset for the Mid360, the number both configs and the MJCF
# site are built from.
_LIVOX_LIDAR_IN_IMU = [-0.011, -0.02329, 0.04412]


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


def _config_extrinsic(name):
    text = (_CONFIG_DIR / name).read_text()
    parameters = yaml.safe_load(text)["/**"]["ros__parameters"]["mapping"]
    return parameters["extrinsic_T"], parameters["extrinsic_R"]


def test_both_configs_carry_the_livox_lever_arm():
    # The simulator's IMU site is placed from these same numbers, so a change here without a
    # matching change to the MJCF patch silently moves one and not the other.
    for name in ("fastlio_mid360_sim.yaml", "fastlio_mid360_hardware.yaml"):
        translation, rotation = _config_extrinsic(name)
        assert translation == pytest.approx(_LIVOX_LIDAR_IN_IMU, abs=1e-9), name
        assert rotation == pytest.approx([1, 0, 0, 0, 1, 0, 0, 0, 1], abs=1e-12), (
            f"{name}: both sensors sit in one housing, so the rotation between them is identity "
            "and the upside-down mount belongs in the URDF"
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
        "frozen, the pelvis IMU would do after all and the MJCF would not need a sensor in the "
        "Mid360 -- but check who froze it before simplifying anything."
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
    for axis in range(3):
        assert offset[axis] == pytest.approx(-_LIVOX_LIDAR_IN_IMU[axis], abs=1e-9)
