#!/usr/bin/env python3
"""Checks the poses and sizes on /objects against the simulator's ground truth.

Every other consumer takes /objects as the truth, so this is the only check on it. The mock
detector supplies masks (no GPU); everything after the mask runs for real: deprojection of
MuJoCo's depth, the support plane, the box fit, tracking, the odom frame chain and the pose source.
"""

import os
import unittest

import launch_testing
import pytest
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped with the tf2 buffer
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.duration import Duration
from rclpy.node import Node as RclpyNode
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from vision_msgs.msg import Detection3DArray

# The sim, control stack and MoveIt must be up first; matches the manipulation suites.
STACK_SETTLE_S = 55.0
READY_TIMEOUT_S = 85.0
OBJECTS_TIMEOUT_S = 90.0
SAMPLES = 10

# The tabletop props this test measures, with their true sizes in metres.
TRUE_SIZE = {
    "red_block": (0.045, 0.045, 0.09),
    "blue_block": (0.045, 0.045, 0.09),
    "green_cylinder": (0.06, 0.06, 0.09),
    "blue_sphere": (0.074, 0.074, 0.074),
    "yellow_box": (0.08, 0.05, 0.05),
    "white_cup": (0.07, 0.07, 0.09),
}

# Set by the green cylinder at about 9 mm: a round object seen from one side has only its near
# surface to fit, which biases the centre toward the camera.
POSITION_TOLERANCE_M = 0.012
SIZE_TOLERANCE_M = 0.008


@pytest.mark.launch_test
@launch_testing.ready_to_test_action_timeout(READY_TIMEOUT_S)
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "world": "tabletop",
            "pin_pelvis": "true",
            "odometry": "ground_truth",
            "moveit": "true",
            "manipulation": "true",
            "perception": "true",
            "detector": "mock",
            "headless": "true",
            "rviz": "false",
        }.items(),
    )
    return LaunchDescription(
        [bringup, TimerAction(period=STACK_SETTLE_S, actions=[launch_testing.actions.ReadyToTest()])]
    )


class TestPerceptionObjects(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("perception_probe")
        cls.buffer = tf2_ros.Buffer()
        cls.listener = tf2_ros.TransformListener(cls.buffer, cls.node)
        cls.measured = []
        cls.truth = {}
        cls.node.create_subscription(Detection3DArray, "/objects", cls._on_objects, 1)
        cls.node.create_subscription(
            Detection3DArray,
            "/g1_sensor_relay/object_poses",
            cls._on_truth,
            qos_profile_sensor_data,
        )
        cls._collect(OBJECTS_TIMEOUT_S)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_objects(cls, msg):
        named = {
            detection.results[0].hypothesis.class_id: detection
            for detection in msg.detections
            if detection.results
        }
        if named:
            cls.measured.append(named)

    @classmethod
    def _on_truth(cls, msg):
        # Truth arrives in the camera frame, like perception, so one transform serves both and a
        # TF error cannot flatter either.
        for detection in msg.detections:
            # Only the props in TRUE_SIZE; the scene reports others too, such as the drop box.
            if not detection.results or detection.results[0].hypothesis.class_id not in TRUE_SIZE:
                continue
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = detection.bbox.center
            # Latest transform, not the stamp: truth is stamped ~1 ms ahead of TF, and with the
            # robot pinned the two are the same.
            pose.header.stamp = Time().to_msg()
            try:
                in_odom = cls.buffer.transform(pose, "odom", timeout=Duration(seconds=0.3))
            except tf2_ros.TransformException:
                return
            cls.truth[detection.results[0].hypothesis.class_id] = in_odom.pose.position

    @classmethod
    def _collect(cls, timeout_s):
        deadline = cls.node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while cls.node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.2)
            if len(cls.measured) >= SAMPLES and len(cls.truth) == len(TRUE_SIZE):
                return

    def _average(self, object_id):
        """Mean position and size over the samples that carried this object."""
        seen = [frame[object_id] for frame in self.measured if object_id in frame]
        self.assertTrue(seen, f"{object_id} never appeared on /objects")
        count = len(seen)
        position = [
            sum(d.bbox.center.position.x for d in seen) / count,
            sum(d.bbox.center.position.y for d in seen) / count,
            sum(d.bbox.center.position.z for d in seen) / count,
        ]
        size = [
            sum(d.bbox.size.x for d in seen) / count,
            sum(d.bbox.size.y for d in seen) / count,
            sum(d.bbox.size.z for d in seen) / count,
        ]
        return position, size, count

    def test_01_objects_are_published_from_the_camera(self):
        self.assertGreaterEqual(
            len(self.measured), SAMPLES, "/objects did not keep publishing measured poses"
        )
        self.assertEqual(len(self.truth), len(TRUE_SIZE), "the simulator did not report every body")

    def test_02_every_object_is_found_and_named(self):
        for name in TRUE_SIZE:
            with self.subTest(object=name):
                # Both the tracked id and the bare-phrase alias, because trees address either.
                self.assertTrue(
                    any(f"{name}_0" in frame for frame in self.measured), f"{name}_0 is missing"
                )
                self.assertTrue(
                    any(name in frame for frame in self.measured),
                    f"{name} has no bare-phrase alias, so an existing tree could not name it",
                )

    def test_03_positions_match_the_simulator(self):
        for name in TRUE_SIZE:
            with self.subTest(object=name):
                position, _, _ = self._average(f"{name}_0")
                true = self.truth[name]
                for axis, measured, expected in zip(
                    "xyz", position, (true.x, true.y, true.z), strict=True
                ):
                    self.assertLess(
                        abs(measured - expected),
                        POSITION_TOLERANCE_M,
                        f"{name} {axis} is {measured:.3f} against {expected:.3f}",
                    )

    def test_04_sizes_match_the_objects(self):
        for name, expected in TRUE_SIZE.items():
            with self.subTest(object=name):
                _, size, _ = self._average(f"{name}_0")
                # Sorted: which axis is which depends on the fitted yaw.
                for measured, true in zip(sorted(size), sorted(expected), strict=True):
                    self.assertLess(
                        abs(measured - true),
                        SIZE_TOLERANCE_M,
                        f"{name} measured {[round(v, 3) for v in size]} against {expected}",
                    )

    def test_05_poses_are_stamped_when_they_were_measured(self):
        # The pose source keeps the capture stamp, so a skill can judge a measurement's age.
        frames = [frame for frame in self.measured if frame]
        self.assertTrue(frames)
        detection = next(iter(frames[-1].values()))
        self.assertEqual(detection.header.frame_id, "odom")
        age = (self.node.get_clock().now() - Time.from_msg(detection.header.stamp)).nanoseconds / 1e9
        self.assertLess(age, 5.0, "the newest object pose is older than any skill would accept")
        self.assertGreater(age, 0.0)
