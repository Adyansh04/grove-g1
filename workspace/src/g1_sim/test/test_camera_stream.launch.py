"""The simulated D435i produces colour, depth and intrinsics that match the scene.

The depth assertions are the point. The camera is pitched 47.6 degrees down, so its
optical axis lands on the floor 1.16 m ahead and reads 1.7156 m, and once the base is
close enough it lands on the +x wall instead. Both are arithmetic from the MJCF, so a
wrong mount height, a wrong pitch, a non-metric depth buffer or a frozen stream all fail.
"""

import math
import os
import sys
import time
import unittest
from collections import deque

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from perception_sim_fixture import (  # noqa: E402
    CAMERA_PITCH,
    CAMERA_XYZ,
    SENSOR_QOS,
    PerceptionSimTestNode,
    perception_sim_description,
)

COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/color/camera_info"

# The MJCF camera element: resolution 848x480, fovy 58 degrees.
EXPECTED_WIDTH = 848
EXPECTED_HEIGHT = 480
FOVY_RAD = math.radians(58.0)

WALL_PX_X = 4.0
PATCH_HALF = 16

# Only the newest frame is read; the rest just satisfy the liveness count in test_01.
# These are 1.2 MB colour and 1.6 MB depth frames at 15 Hz, so the buffer stays small.
FRAME_HISTORY = 8


def expected_axis_range(base_x, camera_world_z):
    """Range along the optical axis: the floor while far from the wall, the wall once near.

    Both are exact for the centre pixel, where the optical-frame z the depth image stores
    and the euclidean range are the same number.
    """
    to_floor = camera_world_z / math.sin(CAMERA_PITCH)
    to_wall = (WALL_PX_X - (base_x + CAMERA_XYZ[0])) / math.cos(CAMERA_PITCH)
    return min(to_floor, to_wall)


def depth_metres(msg):
    """Depth as metres regardless of how the publisher encodes it.

    mujoco_ros2_control publishes 32FC1 metres at the pinned SHA. realsense2_camera on the
    real robot publishes 16UC1 millimetres, so the conversion is here rather than baked
    into the assertions.
    """
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    if msg.encoding == "16UC1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return raw.astype(np.float64) / 1000.0
    raise AssertionError(f"unhandled depth encoding {msg.encoding!r}")


@pytest.mark.launch_test
def generate_test_description():
    return perception_sim_description()


class CameraStreamTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = PerceptionSimTestNode("test_camera_stream")
        cls.color = deque(maxlen=FRAME_HISTORY)
        cls.depth = deque(maxlen=FRAME_HISTORY)
        cls.info = deque(maxlen=FRAME_HISTORY)
        cls.color_stamps = deque(maxlen=512)
        cls.node.create_subscription(Image, COLOR_TOPIC, cls._on_color, SENSOR_QOS)
        cls.node.create_subscription(Image, DEPTH_TOPIC, cls.depth.append, SENSOR_QOS)
        cls.node.create_subscription(CameraInfo, INFO_TOPIC, cls.info.append, SENSOR_QOS)
        cls.node.wait_until(lambda: cls.color and cls.depth and cls.info)

    @classmethod
    def _on_color(cls, msg):
        cls.color.append(msg)
        cls.color_stamps.append(time.time())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def camera_world_z(self):
        """Camera height above the floor, straight off TF."""
        return float(self.node.odom_from("camera_link")[1][2])

    def centre_median(self):
        """Median of the central patch, in metres. Median so a stray pixel cannot move it."""
        self.assertTrue(self.depth, f"nothing published on {DEPTH_TOPIC}")
        metres = depth_metres(self.depth[-1])
        cy, cx = metres.shape[0] // 2, metres.shape[1] // 2
        patch = metres[cy - PATCH_HALF : cy + PATCH_HALF, cx - PATCH_HALF : cx + PATCH_HALF]
        finite = patch[np.isfinite(patch)]
        self.assertGreater(finite.size, 0, "central patch is entirely non-finite")
        return float(np.median(finite))

    def test_01_all_three_streams_publish(self):
        # Drop the discovery backlog; it drains as a burst and inflates the rate.
        self.color_stamps.clear()
        self.node.spin(3.0)
        for name, buf in (("colour", self.color), ("depth", self.depth), ("info", self.info)):
            self.assertGreaterEqual(len(buf), 5, f"only {len(buf)} {name} messages in 3 s")
        elapsed = self.color_stamps[-1] - self.color_stamps[0]
        rate = (len(self.color_stamps) - 1) / elapsed
        self.assertGreater(rate, 10.0, f"colour at {rate:.2f} Hz, configured 15")
        self.assertLess(rate, 18.0, f"colour at {rate:.2f} Hz, configured 15")

    def test_02_camera_info_matches_the_mjcf(self):
        info = self.info[-1]
        self.assertEqual((info.width, info.height), (EXPECTED_WIDTH, EXPECTED_HEIGHT))
        self.assertEqual(info.header.frame_id, "camera_color_optical_frame")

        expected_f = EXPECTED_HEIGHT / (2.0 * math.tan(FOVY_RAD / 2.0))
        fx, fy = info.k[0], info.k[4]
        self.assertAlmostEqual(
            fy,
            expected_f,
            delta=expected_f * 0.01,
            msg=f"fy={fy:.3f}, but fovy=58 deg over {EXPECTED_HEIGHT} rows gives "
            f"{expected_f:.3f}. MuJoCo's fovy is vertical.",
        )
        self.assertAlmostEqual(fx, fy, delta=expected_f * 0.01, msg="square pixels expected")
        self.assertAlmostEqual(info.k[2], EXPECTED_WIDTH / 2.0, delta=1.0)
        self.assertAlmostEqual(info.k[5], EXPECTED_HEIGHT / 2.0, delta=1.0)

    def test_03_colour_image_is_rendered_not_blank(self):
        """A headless GL context that silently fails renders a uniform frame."""
        msg = self.color[-1]
        self.assertEqual(msg.encoding, "rgb8")
        self.assertEqual((msg.width, msg.height), (EXPECTED_WIDTH, EXPECTED_HEIGHT))
        distinct = len(np.unique(np.frombuffer(msg.data, dtype=np.uint8)))
        self.assertGreater(
            distinct, 1, "colour frame has one pixel value; rendering produced a blank image"
        )

    def test_04_depth_image_is_metric_and_mostly_valid(self):
        msg = self.depth[-1]
        self.assertEqual((msg.width, msg.height), (EXPECTED_WIDTH, EXPECTED_HEIGHT))
        self.assertEqual(msg.header.frame_id, "camera_color_optical_frame")
        metres = depth_metres(msg)
        finite = np.isfinite(metres) & (metres >= 0.1) & (metres <= 40.0)
        self.assertGreater(
            finite.mean(),
            0.60,
            f"only {finite.mean() * 100:.1f}% of depth pixels are finite and in range",
        )

    def test_05_depth_reads_the_floor_where_the_mount_geometry_says(self):
        """Height and pitch together: get either wrong and this number moves."""
        camera_z = self.camera_world_z()
        expected = expected_axis_range(0.0, camera_z)
        measured = self.centre_median()
        self.assertAlmostEqual(
            measured,
            expected,
            delta=0.05,
            msg=f"central depth {measured:.4f} m, but a camera {camera_z:.4f} m up "
            f"pitched {math.degrees(CAMERA_PITCH):.1f} deg down meets the floor at "
            f"{expected:.4f} m",
        )

    def test_06_depth_tracks_the_wall_as_the_base_approaches(self):
        """At x=3.0 the optical axis clears the floor and lands on the +x wall."""
        camera_z = self.camera_world_z()
        far = self.centre_median()
        stopped_at = self.node.drive_to_x(3.0)
        self.node.spin(0.5)
        near = self.centre_median()

        expected = expected_axis_range(stopped_at, camera_z)
        self.assertLess(
            expected,
            camera_z / math.sin(CAMERA_PITCH) - 0.1,
            f"base stopped at x={stopped_at:.3f}, not close enough for the axis to leave "
            "the floor; this test would not be measuring the wall",
        )
        self.assertAlmostEqual(
            near,
            expected,
            delta=0.05,
            msg=f"central depth {near:.4f} m at x={stopped_at:.3f}, expected {expected:.4f} m "
            f"off the +x wall (was {far:.4f} m off the floor)",
        )
        self.assertLess(near, far - 0.3, "depth did not change as the base drove forward")
        self.node.drive_to_x(0.0)
