"""The MJCF and the URDF carry the same sensor mount poses. Assert they agree.

MuJoCo needs the mounts in its own MJCF, and ros2_control/TF needs them in the URDF, so
the numbers necessarily exist twice. Generating one from the other at launch was rejected
(g1_bringup already found launch-time file staging fiddly, and it buys one number), so
this test is the guard instead: if either file drifts, it fails.

No sim, no ROS. Just parses both files.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
MJCF = PKG / "mjcf" / "g1_perception_base.xml"
MOUNTS = PKG / "config" / "sensor_mounts.yaml"

TOL = 1e-6


def rpy_to_quat(roll, pitch, yaw):
    """Extrinsic XYZ (URDF rpy) to (w, x, y, z), matching MuJoCo's quat ordering."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quats_equal(a, b, tol=TOL):
    """q and -q are the same rotation, so compare both signs."""
    same = all(abs(x - y) < tol for x, y in zip(a, b, strict=True))
    flipped = all(abs(x + y) < tol for x, y in zip(a, b, strict=True))
    return same or flipped


@pytest.fixture(scope="module")
def mounts():
    with open(MOUNTS) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def mjcf():
    return ET.parse(MJCF).getroot()


def _find(root, tag, name):
    for el in root.iter(tag):
        if el.get("name") == name:
            return el
    raise AssertionError(f"<{tag} name='{name}'> not found in {MJCF}")


def test_base_spawn_height_matches_mount_config(mounts, mjcf):
    """The one place base_link's height above the floor is written.

    g1_odometry_publisher publishes this as the z of odom -> base_link, which is what puts
    `odom` on the ground plane. If the MJCF spawn height and the yaml drift apart, every
    point transformed into odom is silently offset and Nav2's height bands stop meaning
    what they say.
    """
    body = _find(mjcf, "body", "base_link")
    spawn_z = float(body.get("pos").split()[2])
    assert abs(spawn_z - mounts["base_link"]["spawn_z"]) < TOL, (
        f"base_link spawn height: MJCF {spawn_z} != yaml {mounts['base_link']['spawn_z']}"
    )


def test_livox_site_matches_mount_config(mounts, mjcf):
    site = _find(mjcf, "site", "livox_frame")
    cfg = mounts["livox_frame"]

    pos = [float(v) for v in site.get("pos").split()]
    for got, want, axis in zip(pos, cfg["xyz"], "xyz", strict=True):
        assert abs(got - want) < TOL, f"livox_frame {axis}: MJCF {got} != yaml {want}"

    quat = [float(v) for v in site.get("quat").split()]
    expected = rpy_to_quat(*cfg["rpy"])
    assert quats_equal(quat, expected), (
        f"livox_frame orientation: MJCF quat {quat} != yaml rpy {cfg['rpy']} "
        f"(= quat {[round(v, 8) for v in expected]}). The pi roll is load-bearing -- "
        "the Mid360 is mounted upside down on the real robot."
    )


def test_camera_body_matches_mount_config(mounts, mjcf):
    body = _find(mjcf, "body", "camera_link")
    cfg = mounts["camera_link"]

    pos = [float(v) for v in body.get("pos").split()]
    for got, want, axis in zip(pos, cfg["xyz"], "xyz", strict=True):
        assert abs(got - want) < TOL, f"camera_link {axis}: MJCF {got} != yaml {want}"

    quat = [float(v) for v in body.get("quat").split()]
    expected = rpy_to_quat(*cfg["rpy"])
    assert quats_equal(quat, expected), (
        f"camera_link orientation: MJCF quat {quat} != yaml rpy {cfg['rpy']} "
        f"(= quat {[round(v, 8) for v in expected]})"
    )


def test_mount_poses_match_the_vendored_g1_urdf(mounts):
    """The whole point of these numbers is that they are the robot's, not invented.

    Re-derives them from g1_description's URDF so a well-meaning "cleanup" of
    sensor_mounts.yaml cannot quietly replace real geometry with round numbers.
    """
    urdf = PKG.parent / "g1_description" / "urdf" / "g1_29dof_with_hand_rev_1_0.urdf"
    if not urdf.exists():
        pytest.skip("g1_description URDF not present")
    root = ET.parse(urdf).getroot()

    def joint_origin(name):
        for j in root.iter("joint"):
            if j.get("name") == name:
                o = j.find("origin")
                return (
                    [float(v) for v in o.get("xyz").split()],
                    [float(v) for v in o.get("rpy").split()],
                )
        raise AssertionError(f"joint {name} not found in the vendored URDF")

    # torso_link's offset from pelvis at zero pose, via the waist chain.
    torso_xyz = [-0.0039635, 0.0, 0.044]

    for joint_name, mount_name in (("mid360_joint", "livox_frame"), ("d435_joint", "camera_link")):
        xyz, rpy = joint_origin(joint_name)
        want_xyz = [xyz[i] + torso_xyz[i] for i in range(3)]
        cfg = mounts[mount_name]
        for got, want, axis in zip(cfg["xyz"], want_xyz, "xyz", strict=True):
            assert abs(got - want) < 1e-5, (
                f"{mount_name} {axis}: yaml {got} != vendored URDF {joint_name} "
                f"+ torso offset = {want}"
            )
        for got, want, axis in zip(cfg["rpy"], rpy, ("roll", "pitch", "yaw"), strict=True):
            assert abs(got - want) < 1e-9, (
                f"{mount_name} {axis}: yaml {got} != vendored URDF {joint_name} {want}"
            )
