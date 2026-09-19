#!/usr/bin/env python3
"""Runs the grasp adapter against a stub generator.

What fails quietly without this: a depth frame sent with NaNs the server rejects, an instance
mask built at the wrong scale, sweep params in the wrong order, or grasps returned in a frame
nobody can transform. All of those look like a generator that produces bad grasps.

No simulator and no GPU: the stub is the point, and CI runs this.
"""

import os
import sys
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from rclpy.node import Node as RclpyNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from g1_msgs.msg import InstanceMask, InstanceMaskArray
from g1_msgs.srv import GenerateGrasps

PORT = 5592
STUB = os.path.join(os.path.dirname(__file__), "graspgen_server_stub.py")
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/aligned_depth_to_color/camera_info"
MASK_TOPIC = "/g1_object_geometry/tracked_masks"
SERVICE = "/g1_graspgen_adapter/generate_grasps"
WIDTH, HEIGHT = 64, 48
OBJECT_ID = "red_block_0"


@pytest.mark.launch_test
def generate_test_description():
    adapter = Node(
        package="g1_perception",
        executable="g1_graspgen_adapter",
        name="g1_graspgen_adapter",
        output="screen",
        remappings=[
            ("tracked_masks", MASK_TOPIC),
            ("depth/image_raw", DEPTH_TOPIC),
            ("depth/camera_info", INFO_TOPIC),
        ],
        parameters=[
            {
                "server_address": f"tcp://127.0.0.1:{PORT}",
                "zmq_timeout_ms": 5000,
                "sweep_volume": [0.10, 0.06, 0.04, 0.0, 0.0, 0.07, 0.04, 0.06, 0.04, 0.007, 0.0, 0.06],
                "gripper_type": 2,
                "fingertip_depth_m": 0.07,
                "hand": "right",
            }
        ],
    )
    return LaunchDescription(
        [
            ExecuteProcess(cmd=[sys.executable, STUB, str(PORT)], output="screen"),
            # The adapter asks the server for its health in its constructor, so bind first.
            TimerAction(period=3.0, actions=[adapter]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGraspGenAdapter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("graspgen_probe")
        cls.depth = cls.node.create_publisher(Image, DEPTH_TOPIC, qos_profile_sensor_data)
        cls.info = cls.node.create_publisher(CameraInfo, INFO_TOPIC, qos_profile_sensor_data)
        cls.masks = cls.node.create_publisher(InstanceMaskArray, MASK_TOPIC, 2)
        cls.client = cls.node.create_client(GenerateGrasps, SERVICE)
        if not cls.client.wait_for_service(timeout_sec=60.0):
            raise AssertionError("the adapter never came up")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _await_subscribers(self, timeout_s=30.0):
        """Publishing before the adapter has matched is a message nobody receives."""
        deadline = self.node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.node.get_clock().now().nanoseconds < deadline:
            if all(
                publisher.get_subscription_count() > 0
                for publisher in (self.depth, self.info, self.masks)
            ):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def _publish_scene(self, label=OBJECT_ID, rounds=5):
        """A depth frame, its intrinsics and one mask, all sharing a stamp as the relay does.

        Sent several times: a subscription that has just matched can still drop the first
        best-effort message, and a test that fails on that is testing the middleware.
        """
        self.assertTrue(self._await_subscribers(), "the adapter never subscribed")
        for _ in range(rounds):
            depth = self._publish_once(label)
        return depth

    def _publish_once(self, label):
        stamp = self.node.get_clock().now().to_msg()

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = "camera_depth_optical_frame"
        depth.height, depth.width = HEIGHT, WIDTH
        depth.encoding = "32FC1"
        depth.step = depth.width * 4
        # Half a metre everywhere, which is where the tabletop sits.
        depth.data = bytes(bytearray(b"\x00\x00\x00\x3f" * (HEIGHT * WIDTH)))
        self.depth.publish(depth)

        info = CameraInfo()
        info.header = depth.header
        info.height, info.width = HEIGHT, WIDTH
        info.k = [432.98, 0.0, 32.0, 0.0, 432.98, 24.0, 0.0, 0.0, 1.0]
        self.info.publish(info)

        masks = InstanceMaskArray()
        masks.header = depth.header
        masks.image_width, masks.image_height = WIDTH, HEIGHT
        masks.model = "stub"
        instance = InstanceMask()
        instance.label = label
        instance.score = 0.9
        instance.roi.x_offset, instance.roi.y_offset = 20, 16
        instance.roi.width, instance.roi.height = 12, 10
        instance.data = [255] * (12 * 10)
        masks.instances.append(instance)
        self.masks.publish(masks)

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return depth

    def _call(self, object_id=OBJECT_ID, hand="right"):
        request = GenerateGrasps.Request()
        request.object_id = object_id
        request.hand = hand
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), "the adapter never answered")
        return future.result()

    def test_01_grasps_come_back_for_a_tracked_object(self):
        depth = self._publish_scene()

        result = self._call()

        self.assertTrue(result.ok, result.message)
        self.assertEqual(len(result.grasps), len(result.scores))
        self.assertEqual(len(result.grasps), 2)
        self.assertEqual(result.header.frame_id, depth.header.frame_id)
        self.assertEqual(result.header.stamp, depth.header.stamp)

    def test_02_the_best_grasp_comes_first(self):
        self._publish_scene()

        result = self._call()

        self.assertTrue(result.ok, result.message)
        self.assertEqual(list(result.scores), sorted(result.scores, reverse=True))
        self.assertAlmostEqual(result.scores[0], 0.91, places=5)
        # The stub reaches straight down, so the grasp's approach axis is the world's -z. A
        # client that dropped the rotation would hand back an identity orientation instead.
        self.assertAlmostEqual(abs(result.grasps[0].orientation.x), 1.0, places=5)

    def test_03_an_unknown_object_is_refused_and_says_what_is_there(self):
        self._publish_scene()

        result = self._call(object_id="green_cylinder_0")

        self.assertFalse(result.ok)
        self.assertIn("green_cylinder_0", result.message)
        self.assertIn(OBJECT_ID, result.message)

    def test_04_the_left_hand_is_refused_rather_than_mirrored(self):
        self._publish_scene()

        result = self._call(hand="left")

        self.assertFalse(result.ok)
        self.assertIn("right", result.message)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
