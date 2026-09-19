#!/usr/bin/env python3
"""Runs the instruction grounder against a stub vision server.

What fails quietly without this: phrases that come back but are never installed, so the detector
keeps looking for whatever it was looking for before; or a target the detector cannot be asked
for, which reads as an object that is simply never found.

No simulator, no GPU and no vision-language model: the stub is the point, and CI runs this.
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
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node as RclpyNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from g1_msgs.srv import GroundInstruction

PORT = 5593
STUB = os.path.join(os.path.dirname(__file__), "vision_server_stub.py")
CAMERA_TOPIC = "/camera/color/image_raw"
SERVICE = "/g1_instruction_grounder/ground"
WIDTH, HEIGHT = 64, 48


@pytest.mark.launch_test
def generate_test_description():
    # A detector against the same stub, because what the grounder does with its answer is write
    # it onto that node: without one there is nothing to check the install against.
    detector = Node(
        package="g1_perception",
        executable="g1_detector",
        name="g1_detector",
        output="screen",
        remappings=[("color/image_raw", CAMERA_TOPIC)],
        parameters=[
            {
                "server_address": f"tcp://127.0.0.1:{PORT}",
                "zmq_timeout_ms": 5000,
                "detect_rate_hz": 1.0,
                "phrases": ["something else"],
            }
        ],
    )
    grounder = Node(
        package="g1_perception",
        executable="g1_instruction_grounder",
        name="g1_instruction_grounder",
        output="screen",
        remappings=[("color/image_raw", CAMERA_TOPIC)],
        parameters=[
            {
                "server_address": f"tcp://127.0.0.1:{PORT}",
                "zmq_timeout_ms": 5000,
                "max_image_age_s": 30.0,
                "detector_node": "/g1_detector",
            }
        ],
    )
    return LaunchDescription(
        [
            ExecuteProcess(cmd=[sys.executable, STUB, str(PORT)], output="screen"),
            TimerAction(period=3.0, actions=[detector, grounder]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGrounder(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("grounder_probe")
        cls.camera = cls.node.create_publisher(Image, CAMERA_TOPIC, qos_profile_sensor_data)
        cls.client = cls.node.create_client(GroundInstruction, SERVICE)
        if not cls.client.wait_for_service(timeout_sec=60.0):
            raise AssertionError("the grounder never came up")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish_frames(self, rounds=5):
        for _ in range(rounds):
            image = Image()
            image.header.stamp = self.node.get_clock().now().to_msg()
            image.header.frame_id = "camera_color_optical_frame"
            image.height, image.width = HEIGHT, WIDTH
            image.encoding = "rgb8"
            image.step = image.width * 3
            image.data = bytes(image.height * image.step)
            self.camera.publish(image)
            for _ in range(5):
                rclpy.spin_once(self.node, timeout_sec=0.05)

    def _ground(self, instruction):
        request = GroundInstruction.Request()
        request.instruction = instruction
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), "the grounder never answered")
        return future.result()

    def _detector_phrases(self):
        client = self.node.create_client(GetParameters, "/g1_detector/get_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=20.0))
        request = GetParameters.Request()
        request.names = ["phrases"]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertIsNotNone(future.result(), "the detector never answered get_parameters")
        return list(future.result().values[0].string_array_value)

    def test_01_an_instruction_becomes_phrases(self):
        self._publish_frames()

        result = self._ground("pick up the blue sphere next to the red block")

        self.assertTrue(result.ok, result.message)
        self.assertEqual(list(result.phrases), ["red block", "blue sphere"])
        self.assertEqual(result.target_phrase, "blue sphere")
        self.assertIn(result.target_phrase, result.phrases, "the target must be askable for")

    def test_02_the_phrases_reach_the_detector(self):
        self._publish_frames()

        self._ground("pick up the blue sphere")

        self.assertEqual(self._detector_phrases(), ["red block", "blue sphere"])

    def test_03_exemplar_points_are_carried_through(self):
        self._publish_frames()

        result = self._ground("pick up the blue sphere")

        self.assertEqual(len(result.exemplars), 1)
        self.assertEqual(result.exemplars[0].phrase, "blue sphere")
        self.assertEqual((result.exemplars[0].x, result.exemplars[0].y), (312, 204))
        self.assertEqual(result.header.frame_id, "camera_color_optical_frame")

    def test_04_an_empty_instruction_is_refused(self):
        self._publish_frames()

        result = self._ground("   ")

        self.assertFalse(result.ok)
        self.assertIn("instruction", result.message)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
