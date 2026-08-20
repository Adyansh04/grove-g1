"""The lidar-to-IMU extrinsic is the same constant in simulation and on the robot.

It is a constant only because the IMU is inside the sensor in both places, the simulator modelling
one there rather than substituting the pelvis IMU, which is three actuated waist joints away. See
g1_state_estimation's README. These tests hold the arrangement in place: where the simulator puts
its IMU, what both configs say about it, and the joints that make the substitution wrong.
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
_MJCF_PATCH = (
    pathlib.Path(__file__).resolve().parents[3]
    / "patches"
    / "unitree_mujoco"
    / "006-add-mid360-imu.patch"
)


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
        translation = [
            c + v for c, v in zip(xyz, _mat_vec(joint_rotation, translation), strict=True)
        ]
        rotation = _mat_mul(joint_rotation, rotation)
        link = parent
    return translation, rotation


def _config_extrinsic(name):
    text = (_CONFIG_DIR / name).read_text()
    parameters = yaml.safe_load(text)["/**"]["ros__parameters"]["mapping"]
    return parameters["extrinsic_T"], parameters["extrinsic_R"]


def _quat_to_matrix(w, x, y, z):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _mjcf_site():
    """pos and quat of the mid360_imu site, out of the MJCF patch."""
    # Added lines only, with the diff marker off, so the element reads as XML.
    text = "\n".join(
        line[1:] for line in _MJCF_PATCH.read_text().splitlines() if line.startswith("+")
    )
    match = re.search(r'<site name="mid360_imu"[^>]*?pos="([^"]+)"\s+quat="([^"]+)"', text)
    assert match, "mid360_imu site not found in the MJCF patch"
    return (
        [float(v) for v in match.group(1).split()],
        [float(v) for v in match.group(2).split()],
    )


def test_the_mjcf_site_is_where_the_urdf_says_the_sensor_imu_is():
    # This is the number the whole arrangement rests on: the simulator's IMU has to sit at the
    # pose the extrinsic below is measured from, and nothing else compares the two. Recomputed
    # from the URDF mount composed with Livox's offset rather than copied.
    translation, rotation = _chain_from_urdf("mid360_link", "torso_link")
    imu_in_lidar = [-v for v in _LIVOX_LIDAR_IN_IMU]
    expect = [
        t + v for t, v in zip(translation, _mat_vec(rotation, imu_in_lidar), strict=True)
    ]

    pos, quat = _mjcf_site()
    for axis in range(3):
        assert pos[axis] == pytest.approx(expect[axis], abs=1e-6), (
            f"site pos[{axis}]: MJCF {pos[axis]} vs URDF chain {expect[axis]}"
        )
    # Same orientation as mid360_link: both sensors are in one housing and flip together.
    site_rotation = _quat_to_matrix(*quat)
    for i in range(3):
        for j in range(3):
            assert site_rotation[i][j] == pytest.approx(rotation[i][j], abs=1e-6), (
                f"site quat row {i} col {j}: MJCF {site_rotation[i][j]} vs URDF {rotation[i][j]}"
            )


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
    # check the two cancel instead of trusting the sign flip was done right. Read from
    # g1_common.xacro, which is where the shared sensor frames live.
    xacro = (
        pathlib.Path(get_package_share_directory("g1_description")) / "urdf" / "g1_common.xacro"
    )
    match = re.search(
        r'<child link="mid360_imu"/>\s*<origin xyz="([^"]+)"',
        xacro.read_text(),
    )
    assert match, "mid360_imu joint not found in g1_common.xacro"
    offset = [float(v) for v in match.group(1).split()]
    for axis in range(3):
        assert offset[axis] == pytest.approx(-_LIVOX_LIDAR_IN_IMU[axis], abs=1e-9)
