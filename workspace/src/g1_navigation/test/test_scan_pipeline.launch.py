"""The frame chain and the scan Nav2 and slam_toolbox will consume.

Two things nothing else covers. First, that odom -> base_footprint really is gravity-aligned
and base_footprint -> pelvis carries the height and tilt it drops: the math is unit tested, the
wiring between the odometry node, the launch and TF is not. Second, that the flatten produces a
scan of the shape and density the SLAM config was tuned against.

Pinned pelvis, same reason as test_lidar_geometry: a free-standing G1 drifts and its waist
leans, so the numbers would move under the test without anything being wrong.
"""

import os
import time
import unittest

import launch_testing
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# config/scan.yaml
EXPECTED_BEAMS = 360
EXPECTED_INCREMENT = np.radians(1.0)
RANGE_MAX = 25.0

# Measured on the converged track: the pelvis stands about 0.75 m up. Loose because it is a
# property of the stance, not a constant.
PELVIS_HEIGHT_M = 0.75
PELVIS_HEIGHT_TOL = 0.12

# A gravity-aligned frame is constructed, not estimated, so this tolerance only has to absorb
# quaternion round-tripping through the message.
FLAT_TOL = 1e-6

BRINGUP_TIMEOUT_S = 120.0


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "sensors": "true",
            # Ground truth, not the stack default: this asserts scan geometry through TF, so
            # the odometry under it should contribute nothing.
            "odometry": "ground_truth",
            "pin_pelvis": "true",
            "world": "perception",
        }.items(),
    )
    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_navigation"), "launch", "scan.launch.py")
        ),
        # No container is running under this test, so the node has to stand alone.
        launch_arguments={"use_composition": "false"}.items(),
    )
    return (
        LaunchDescription(
            [sim, scan, TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()])]
        ),
        {},
    )


def rpy(q):
    """geometry_msgs Quaternion to roll, pitch, yaw."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(sinr, cosr), np.arcsin(sp), np.arctan2(siny, cosy)


class ScanPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("scan_pipeline_test")
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.node)
        cls.scans = []
        cls.node.create_subscription(
            LaserScan, "/scan", lambda msg: cls.scans.append(msg), qos_profile_sensor_data
        )

        deadline = time.time() + BRINGUP_TIMEOUT_S
        while time.time() < deadline and len(cls.scans) < 20:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        cls.settled = len(cls.scans) >= 20

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def lookup(self, parent, child):
        deadline = time.time() + 20.0
        while time.time() < deadline:
            try:
                return self.buffer.lookup_transform(parent, child, rclpy.time.Time())
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail(f"{parent} -> {child} never appeared on /tf")

    def test_scan_arrived(self):
        self.assertTrue(self.settled, f"only {len(self.scans)} scans in {BRINGUP_TIMEOUT_S}s")

    def test_footprint_is_gravity_aligned(self):
        t = self.lookup("odom", "base_footprint").transform
        roll, pitch, _ = rpy(t.rotation)
        self.assertAlmostEqual(t.translation.z, 0.0, delta=FLAT_TOL, msg="footprint is on the floor")
        self.assertAlmostEqual(roll, 0.0, delta=FLAT_TOL)
        self.assertAlmostEqual(pitch, 0.0, delta=FLAT_TOL)

    def test_body_offset_is_purely_vertical(self):
        # The split puts x and y at exactly zero, because the footprint sits
        # directly beneath the pelvis, and the height the footprint dropped lands here.
        t = self.lookup("base_footprint", "pelvis").transform
        self.assertAlmostEqual(t.translation.x, 0.0, delta=FLAT_TOL)
        self.assertAlmostEqual(t.translation.y, 0.0, delta=FLAT_TOL)
        self.assertAlmostEqual(t.translation.z, PELVIS_HEIGHT_M, delta=PELVIS_HEIGHT_TOL)

    def test_sensor_frames_reachable_from_the_footprint(self):
        # A tree that resolves but is broken at the waist would still pass the two edges
        # above; this is the lookup slam_toolbox and the costmaps actually perform.
        self.lookup("base_footprint", "mid360_link")

    def test_scan_shape_matches_the_slam_config(self):
        scan = self.scans[-1]
        self.assertEqual(scan.header.frame_id, "base_footprint")
        self.assertEqual(len(scan.ranges), EXPECTED_BEAMS)
        self.assertAlmostEqual(scan.angle_increment, EXPECTED_INCREMENT, delta=1e-6)
        self.assertAlmostEqual(scan.range_max, RANGE_MAX, delta=1e-3)

    def test_scan_is_dense_and_misses_are_inf(self):
        # In a closed room every bearing sees a wall. Anything much below full coverage means
        # the height band has drifted off the walls.
        finite = [np.isfinite(np.array(s.ranges)).mean() for s in self.scans[-10:]]
        self.assertGreater(min(finite), 0.85, f"coverage per scan: {finite}")

        # slam_toolbox reads range_max as "something is there, at the edge" and would map a
        # phantom wall at the scan boundary; +inf is the only honest way to say nothing.
        for s in self.scans[-10:]:
            r = np.array(s.ranges)
            misses = r[~np.isfinite(r)]
            self.assertTrue(np.isinf(misses).all(), "a miss was reported as something finite")

    def test_scan_rate_tracks_the_sensor(self):
        stamps = [
            s.header.stamp.sec + s.header.stamp.nanosec * 1e-9 for s in self.scans[-20:]
        ]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        # The sweep runs at 10 Hz. Dropping below 8 means transforms are missing and scans
        # are being discarded, which starves the matcher without any error appearing.
        self.assertGreater(rate, 8.0, f"scan rate {rate:.2f} Hz")
