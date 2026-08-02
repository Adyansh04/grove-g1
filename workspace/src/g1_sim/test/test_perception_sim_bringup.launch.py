"""Headless integration test: the perception sim track comes up and is commandable.

Asserts the wiring, not the sensor data -- stream content is test_lidar_stream and
test_camera_stream's job. What is pinned here: both controllers reach `active`, the
robot description and static frames are published with the real mount poses, simulated
time is running, and commanding the base actually moves it.
"""

import os
import time
import unittest

import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

# The node needs to load MuJoCo, register the engine plugin, spawn controllers and start
# rendering. Generous so a loaded machine cannot flake it.
BRINGUP_TIMEOUT_S = 45.0

# Mount poses, from config/sensor_mounts.yaml (themselves derived from the vendored G1
# URDF). Duplicated here deliberately: a test that reads the same yaml it is checking
# would pass no matter what the value became.
LIVOX_XYZ = (-0.00368, 0.00003, 0.472434)
CAMERA_XYZ = (0.05366, 0.01753, 0.473870)
MOUNT_TOL = 1e-4


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_sim"), "launch", "perception_sim.launch.py")
        )
    )
    return (
        LaunchDescription(
            [sim, TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()])]
        ),
        {},
    )


class PerceptionSimBringupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_perception_sim_bringup")
        # This track runs on simulated time; a listener on the wrong clock sees every
        # transform as either stale or in the future.
        cls.node.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.joint_states = []
        cls.clock_msgs = []
        cls.node.create_subscription(JointState, "/base_joint_states", cls.joint_states.append, 10)
        cls.node.create_subscription(Clock, "/clock", cls.clock_msgs.append, 10)
        cls.cmd_pub = cls.node.create_publisher(
            Float64MultiArray, "/base_velocity_controller/commands", 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_until(self, predicate, timeout_s):
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _joint(self, name):
        for msg in reversed(self.joint_states):
            if name in msg.name:
                return msg.position[msg.name.index(name)]
        return None

    def test_01_simulated_clock_is_running(self):
        """mujoco_ros2_control drives /clock; everything in this track reads it."""
        self.assertTrue(
            self._wait_until(lambda: len(self.clock_msgs) > 20, BRINGUP_TIMEOUT_S),
            "no /clock -- the track is not running on simulated time, which will make "
            "every tf lookup either stale or in the future",
        )
        first, last = self.clock_msgs[0].clock, self.clock_msgs[-1].clock
        self.assertGreater(
            (last.sec - first.sec) + (last.nanosec - first.nanosec) * 1e-9,
            0.0,
            "/clock is published but not advancing",
        )

    def test_02_base_state_is_published(self):
        """The three planar joints reach us on /base_joint_states, not /joint_states."""
        self.assertTrue(
            self._wait_until(lambda: self._joint("base_x_joint") is not None, BRINGUP_TIMEOUT_S),
            "no base_x_joint on /base_joint_states",
        )
        for joint in ("base_x_joint", "base_y_joint", "base_yaw_joint"):
            self.assertIsNotNone(self._joint(joint), f"{joint} missing from /base_joint_states")

    def test_03_sensor_frames_are_published_at_the_real_mounts(self):
        """Static TF carries the mounts taken from Unitree's own vendored URDF."""
        self.assertTrue(
            self._wait_until(
                lambda: self.tf_buffer.can_transform(
                    "base_link", "livox_frame", rclpy.time.Time()
                ),
                BRINGUP_TIMEOUT_S,
            ),
            "base_link -> livox_frame never appeared on /tf_static",
        )
        for child, want in (("livox_frame", LIVOX_XYZ), ("camera_link", CAMERA_XYZ)):
            self.assertTrue(
                self.tf_buffer.can_transform("base_link", child, rclpy.time.Time()),
                f"base_link -> {child} missing",
            )
            tf = self.tf_buffer.lookup_transform("base_link", child, rclpy.time.Time())
            got = (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            )
            for g, w, axis in zip(got, want, "xyz", strict=True):
                self.assertAlmostEqual(
                    g, w, delta=MOUNT_TOL, msg=f"{child} {axis}: TF {g} != mount config {w}"
                )

    def test_04_lidar_is_mounted_upside_down(self):
        """The Mid360's pi roll is real robot geometry, and easy to lose silently.

        Missing it inverts every point cloud relative to hardware, which no amount of
        downstream code will notice until the robot is in front of you.
        """
        self.assertTrue(
            self.tf_buffer.can_transform("base_link", "livox_frame", rclpy.time.Time()),
            "base_link -> livox_frame missing",
        )
        tf = self.tf_buffer.lookup_transform("base_link", "livox_frame", rclpy.time.Time())
        q = tf.transform.rotation
        # A pi roll about x puts nearly all the magnitude in qx and leaves qw near zero.
        self.assertLess(
            abs(q.w),
            0.1,
            f"livox_frame quaternion w={q.w:.4f}; expected near 0 for the pi roll. The "
            "Mid360 is mounted upside down on the real G1.",
        )
        self.assertGreater(abs(q.x), 0.9, f"livox_frame quaternion x={q.x:.4f}; expected near 1")

    def test_05_base_is_commandable(self):
        """Driving +x moves the base in +x, and only in +x."""
        self.assertTrue(
            self._wait_until(lambda: self._joint("base_x_joint") is not None, BRINGUP_TIMEOUT_S),
            "no base state before commanding",
        )
        start_x = self._joint("base_x_joint")
        start_y = self._joint("base_y_joint")

        cmd = Float64MultiArray()
        cmd.data = [0.5, 0.0, 0.0]
        end = time.time() + 3.0
        while time.time() < end:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.02)

        cmd.data = [0.0, 0.0, 0.0]
        self.cmd_pub.publish(cmd)
        self._spin(0.5)

        moved_x = self._joint("base_x_joint") - start_x
        moved_y = self._joint("base_y_joint") - start_y
        self.assertGreater(
            moved_x, 0.5, f"base moved only {moved_x:.3f} m in x under a 0.5 m/s command"
        )
        self.assertLess(
            abs(moved_y),
            0.1,
            f"base drifted {moved_y:.3f} m in y under a pure +x command -- the planar "
            "joints should give world-frame translation with no coupling",
        )
