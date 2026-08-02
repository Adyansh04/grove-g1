"""The simulated Mid360 produces a point cloud that matches the room it is in.

Geometry, not plumbing: test_perception_sim_bringup already proves the topic exists. Here
the cloud is transformed into base_link and checked against facts of the MJCF, so a wrong
mount, a wrong roll, a wrong scale or a dead stream all fail rather than pass quietly.
"""

import os
import sys
import time
import unittest

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from perception_sim_fixture import (  # noqa: E402
    BASE_SPAWN_Z,
    LIVOX_RPY,
    LIVOX_XYZ,
    SENSOR_QOS,
    PerceptionSimTestNode,
    perception_sim_description,
    rpy_to_matrix,
)

TOPIC = "/livox/lidar"

# config/mujoco_plugins.yaml + the MJCF lidar instance.
EXPECTED_WIDTH = 360
EXPECTED_HEIGHT = 32
EXPECTED_POINTS = EXPECTED_WIDTH * EXPECTED_HEIGHT
CONFIGURED_RATE_HZ = 10.0
MIN_RANGE = 0.1
MAX_RANGE = 40.0

# Room geometry from the MJCF: inner wall faces at +/-4.0 m, floor at z=0.
WALL_PX_X = 4.0
ROOM_HALF = 4.0

R_LIVOX = np.array(rpy_to_matrix(LIVOX_RPY))
T_LIVOX = np.array(LIVOX_XYZ)


@pytest.mark.launch_test
def generate_test_description():
    return perception_sim_description()


class LidarStreamTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = PerceptionSimTestNode("test_lidar_stream")
        cls.clouds = []
        cls.stamps = []
        cls.node.create_subscription(PointCloud2, TOPIC, cls._on_cloud, SENSOR_QOS)
        cls.node.wait_until(lambda: len(cls.clouds) > 0)

    @classmethod
    def _on_cloud(cls, msg):
        cls.clouds.append(msg)
        cls.stamps.append(time.time())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def latest(self):
        self.assertTrue(self.clouds, f"nothing published on {TOPIC}")
        return self.clouds[-1]

    def points_sensor(self, msg):
        """The xyz block as an (N, 3) array, in the sensor frame."""
        n = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        return (
            np.frombuffer(raw[:, 0:12].tobytes(), dtype=np.float32)
            .reshape(n, 3)
            .astype(np.float64)
        )

    def points_world(self, msg):
        """Sensor frame to world, valid because the base sits at the origin at rest."""
        pts = self.points_sensor(msg)
        finite = np.isfinite(pts).all(axis=1)
        base = (R_LIVOX @ pts[finite].T).T + T_LIVOX
        return base + np.array([0.0, 0.0, BASE_SPAWN_Z])

    def wall_px_distance(self, msg):
        """Distance from base_link to the +x wall, from near-horizontal forward returns."""
        pts = self.points_sensor(msg)
        finite = np.isfinite(pts).all(axis=1)
        base = (R_LIVOX @ pts[finite].T).T + T_LIVOX
        forward = (
            (np.abs(base[:, 1]) < 0.3)
            & (base[:, 0] > 0.5)
            & (np.abs(base[:, 2] - T_LIVOX[2]) < 0.2)
        )
        self.assertGreater(
            forward.sum(), 5, "no near-horizontal forward returns to measure the wall with"
        )
        return float(np.median(base[forward][:, 0]))

    def test_01_publishes_at_the_configured_rate(self):
        self.node.spin(3.0)
        self.assertGreater(len(self.stamps), 2, f"{TOPIC} is not streaming")
        elapsed = self.stamps[-1] - self.stamps[0]
        rate = (len(self.stamps) - 1) / elapsed
        self.assertGreater(rate, 7.0, f"{rate:.2f} Hz, configured {CONFIGURED_RATE_HZ}")
        self.assertLess(rate, 13.0, f"{rate:.2f} Hz, configured {CONFIGURED_RATE_HZ}")

    def test_02_cloud_layout_matches_the_configured_grid(self):
        msg = self.latest()
        self.assertEqual(msg.header.frame_id, "livox_frame")
        self.assertEqual((msg.width, msg.height), (EXPECTED_WIDTH, EXPECTED_HEIGHT))
        self.assertEqual(
            msg.width * msg.height,
            EXPECTED_POINTS,
            "point count no longer matches the configured resolution",
        )
        self.assertEqual([f.name for f in msg.fields][:3], ["x", "y", "z"])
        self.assertGreaterEqual(msg.point_step, 12)

    def test_03_every_return_is_finite_and_in_range(self):
        """A closed room returns a hit for every ray, so NaNs mean something broke."""
        msg = self.latest()
        pts = self.points_sensor(msg)
        finite = np.isfinite(pts).all(axis=1)
        self.assertEqual(
            int(finite.sum()), EXPECTED_POINTS, f"{EXPECTED_POINTS - finite.sum()} non-finite points"
        )
        ranges = np.linalg.norm(pts, axis=1)
        self.assertGreaterEqual(
            ranges.min(), MIN_RANGE - 1e-3, f"return at {ranges.min():.4f} m, below min_range"
        )
        self.assertLessEqual(ranges.max(), MAX_RANGE, f"return at {ranges.max():.4f} m")

    def test_04_the_floor_is_visible_below_the_robot(self):
        """The pi roll points the Mid360 down; lose it and the floor lands overhead."""
        world = self.points_world(self.latest())
        floor = np.abs(world[:, 2]) < 0.05
        self.assertGreater(
            int(floor.sum()),
            200,
            f"only {floor.sum()} returns near the floor plane. The Mid360 is mounted "
            "upside down on the real G1; without that roll the downward rays go up.",
        )
        self.assertGreaterEqual(
            world[:, 2].min(), -0.05, "returns below the floor plane"
        )

    def test_05_the_forward_wall_is_where_the_mjcf_puts_it(self):
        distance = self.wall_px_distance(self.latest())
        self.assertAlmostEqual(
            distance,
            WALL_PX_X,
            delta=0.05,
            msg=f"+x wall measured at {distance:.4f} m, MJCF puts its inner face at "
            f"{WALL_PX_X} m. A scale or mount error shows up here first.",
        )

    def test_06_most_returns_land_inside_the_room(self):
        """Guards against the walls dropping out of the cloud entirely.

        Not all returns: the lidar plugin lets a band of downward rays (roughly 10 to 18
        degrees below horizontal) pass through the walls and land on the floor plane
        beyond the room, about 4% of the cloud. The camera renders those same walls as
        solid at the same heights, so this is the plugin's raycast, not the scene. See
        g1_sim/README.md.
        """
        world = self.points_world(self.latest())
        inside = (np.abs(world[:, 0]) <= ROOM_HALF + 0.01) & (
            np.abs(world[:, 1]) <= ROOM_HALF + 0.01
        )
        fraction = inside.mean()
        self.assertGreater(
            fraction,
            0.90,
            f"only {fraction * 100:.1f}% of returns are inside the room; the known "
            "plugin leak accounts for about 4%, so this is something larger",
        )

    def test_07_the_wall_closes_in_when_the_base_drives_at_it(self):
        """Proves the cloud tracks the world instead of being a frozen first frame."""
        before = self.wall_px_distance(self.latest())
        travelled = self.node.drive_to_x(1.0)
        self.node.spin(0.5)
        after = self.wall_px_distance(self.latest())

        self.assertAlmostEqual(
            before - after,
            travelled,
            delta=0.05,
            msg=f"wall distance went {before:.3f} -> {after:.3f} m ({before - after:+.3f}) "
            f"while the base moved {travelled:.3f} m in x",
        )
        self.node.drive_to_x(0.0)
