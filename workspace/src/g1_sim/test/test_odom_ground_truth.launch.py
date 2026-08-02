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
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tf2_geometry_msgs  # noqa: E402,F401  registers the PointStamped transform
from perception_sim_fixture import (  # noqa: E402
    LIVOX_XYZ,
    PerceptionSimTestNode,
    perception_sim_description,
)


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

    def base_height(self):
        """The canonical base height, from the file the MJCF is checked against.

        Reading it here rather than hardcoding is deliberate: test_sensor_mount_consistency
        pins this value to the MJCF spawn height, so there is exactly one number and one
        place that asserts it.
        """
        mounts_path = os.path.join(
            get_package_share_directory("g1_sim"), "config", "sensor_mounts.yaml"
        )
        with open(mounts_path) as f:
            return float(yaml.safe_load(f)["base_link"]["spawn_z"])

    def base_in_odom(self):
        self.assertTrue(
            self.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time()),
            "odom -> base_link never appeared. The publisher defaults to odometry_source="
            "'hardware', which refuses to configure; the launch must set sim_ground_truth.",
        )
        tf = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        )

    def test_01_odom_is_the_ground_plane(self):
        """odom sits on the floor, not at the base's spawn height.

        The z is the whole point: it is what lets a consumer transform a point cloud into
        odom and have the floor come out at 0, which is what Nav2's obstacle height bands
        assume. Publishing 0 here would put every floor return at minus the spawn height.
        """
        x, y, z = self.base_in_odom()
        self.assertLess(abs(x), 0.05, f"base starts at the origin, transform says x={x:.4f}")
        self.assertLess(abs(y), 0.05, f"base starts at the origin, transform says y={y:.4f}")
        self.assertAlmostEqual(
            z,
            self.base_height(),
            delta=1e-6,
            msg=f"odom -> base_link z is {z:.6f}, expected the canonical spawn height "
            f"{self.base_height():.6f} from g1_sim/config/sensor_mounts.yaml",
        )

    def test_02_odometry_message_agrees_with_the_transform(self):
        """Nav2 reads velocity from Odometry, not from TF, so both have to be right."""
        self.node.spin(1.0)
        self.assertTrue(self.odoms, "nothing published on ~/odom")
        odom = self.odoms[-1]
        self.assertEqual(odom.header.frame_id, "odom")
        self.assertEqual(odom.child_frame_id, "base_link")
        x, y, z = self.base_in_odom()
        self.assertAlmostEqual(odom.pose.pose.position.x, x, delta=0.02)
        self.assertAlmostEqual(odom.pose.pose.position.y, y, delta=0.02)
        self.assertAlmostEqual(odom.pose.pose.position.z, z, delta=1e-6)
        self.assertGreater(
            odom.pose.covariance[0], 0.0, "all-zero covariance is a known Nav2 footgun"
        )

    def test_03_a_lidar_point_lands_at_its_real_height_above_the_floor(self):
        """The whole chain in one assertion: odom -> base_link -> livox_frame.

        `odom` is the ground plane, so a point at the sensor origin must come out at the
        sensor's physical height above the floor: the base spawn height plus the mount.
        Drop the odom z and this reads 0.472 instead of 1.265, which is exactly the bug
        that made every other test subtract the spawn height by hand.
        """
        base_z = self.base_height()
        expected = base_z + LIVOX_XYZ[2]

        point = PointStamped()
        point.header.frame_id = "livox_frame"
        point.header.stamp = rclpy.time.Time().to_msg()
        transformed = self.tf_buffer.transform(
            point, "odom", timeout=rclpy.duration.Duration(seconds=5.0)
        )
        self.assertAlmostEqual(
            transformed.point.z,
            expected,
            delta=0.01,
            msg=f"livox origin lands at z={transformed.point.z:.4f} in odom; the sensor "
            f"sits {LIVOX_XYZ[2]:.4f} above a base_link that is {base_z:.4f} above the "
            f"floor, so it should be {expected:.4f}. If this reads {LIVOX_XYZ[2]:.4f}, "
            "odom -> base_link is missing its z and odom is not the ground plane.",
        )

    def test_04_the_transform_tracks_commanded_motion_exactly(self):
        start_x, _, _ = self.base_in_odom()
        travelled = self.node.drive_to_x(1.0)
        self.node.spin(0.5)
        end_x, end_y, _ = self.base_in_odom()

        self.assertAlmostEqual(
            end_x - start_x,
            travelled,
            delta=0.05,
            msg=f"odom moved {end_x - start_x:.4f} m while the base joint moved "
            f"{travelled:.4f} m. Ground truth has no drift budget.",
        )
        self.assertLess(abs(end_y), 0.05, f"y drifted to {end_y:.4f} under a pure +x command")
        self.node.drive_to_x(0.0)
