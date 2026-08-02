"""odom -> base_link is live, exact, and usable for transforming sensor data.

The tolerances are tight on purpose. This is MuJoCo ground truth, not an estimate, so it
has no drift budget to hide in: if commanding 1 m of travel does not move the transform by
1 m, something is wrong rather than merely noisy.
"""

import os
import sys
import unittest

import pytest
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from perception_sim_fixture import (  # noqa: E402
    BASE_SPAWN_Z,
    LIVOX_XYZ,
    PerceptionSimTestNode,
    perception_sim_description,
)

import tf2_geometry_msgs  # noqa: E402,F401  registers the PointStamped transform


@pytest.mark.launch_test
def generate_test_description():
    return perception_sim_description()


class OdomGroundTruthTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = PerceptionSimTestNode("test_odom_ground_truth")
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.odoms = []
        cls.node.create_subscription(Odometry, "/g1_odometry_publisher/odom", cls.odoms.append, 10)
        cls.node.wait_until(
            lambda: cls.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time())
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def base_in_odom(self):
        self.assertTrue(
            self.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time()),
            "odom -> base_link never appeared. The publisher defaults to odometry_source="
            "'hardware', which refuses to configure; the launch must set sim_ground_truth.",
        )
        tf = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        return tf.transform.translation.x, tf.transform.translation.y

    def test_01_odom_to_base_link_is_published(self):
        x, y = self.base_in_odom()
        self.assertLess(abs(x), 0.05, f"base starts at the origin, transform says x={x:.4f}")
        self.assertLess(abs(y), 0.05, f"base starts at the origin, transform says y={y:.4f}")

    def test_02_odometry_message_agrees_with_the_transform(self):
        """Nav2 reads velocity from Odometry, not from TF, so both have to be right."""
        self.node.spin(1.0)
        self.assertTrue(self.odoms, "nothing published on ~/odom")
        odom = self.odoms[-1]
        self.assertEqual(odom.header.frame_id, "odom")
        self.assertEqual(odom.child_frame_id, "base_link")
        x, y = self.base_in_odom()
        self.assertAlmostEqual(odom.pose.pose.position.x, x, delta=0.02)
        self.assertAlmostEqual(odom.pose.pose.position.y, y, delta=0.02)
        self.assertGreater(
            odom.pose.covariance[0], 0.0, "all-zero covariance is a known Nav2 footgun"
        )

    def test_03_a_lidar_point_transforms_into_odom(self):
        """The whole chain in one assertion: odom -> base_link -> livox_frame.

        A point at the sensor origin must land at the sensor's known height above the
        floor. This is what every downstream consumer, costmap or SLAM, actually does.
        """
        point = PointStamped()
        point.header.frame_id = "livox_frame"
        point.header.stamp = rclpy.time.Time().to_msg()

        transformed = self.tf_buffer.transform(point, "odom", timeout=rclpy.duration.Duration(seconds=5.0))
        self.assertAlmostEqual(
            transformed.point.z,
            LIVOX_XYZ[2],
            delta=0.01,
            msg=f"livox origin lands at z={transformed.point.z:.4f} in odom, expected "
            f"{LIVOX_XYZ[2]:.4f} above base_link",
        )
        # odom is at the base's spawn pose, so this is height above base_link, not the floor.
        self.assertLess(
            transformed.point.z + BASE_SPAWN_Z, 2.0, "sensor height is implausible in odom"
        )

    def test_04_the_transform_tracks_commanded_motion_exactly(self):
        start_x, _ = self.base_in_odom()
        travelled = self.node.drive_to_x(1.0)
        self.node.spin(0.5)
        end_x, end_y = self.base_in_odom()

        self.assertAlmostEqual(
            end_x - start_x,
            travelled,
            delta=0.05,
            msg=f"odom moved {end_x - start_x:.4f} m while the base joint moved "
            f"{travelled:.4f} m. Ground truth has no drift budget.",
        )
        self.assertLess(abs(end_y), 0.05, f"y drifted to {end_y:.4f} under a pure +x command")
        self.node.drive_to_x(0.0)
