#!/usr/bin/env python3
"""Runs the detector client against a stub vision server.

What fails quietly without this: an image encoded the wrong way round, a mask published against
the publish time instead of the frame's own stamp, or a phrase list that is read once at startup
and never again. None of those look broken from outside; they look like a detector that is bad
at its job.

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
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from g1_msgs.msg import InstanceMaskArray

PORT = 5591
STUB = os.path.join(os.path.dirname(__file__), "vision_server_stub.py")
CAMERA_TOPIC = "/camera/color/image_raw"
MASK_TOPIC = "/g1_detector/instance_masks"
WIDTH, HEIGHT = 640, 480


@pytest.mark.launch_test
def generate_test_description():
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
                "detect_rate_hz": 4.0,
                "phrases": ["red cube"],
                "max_image_age_s": 5.0,
            }
        ],
    )
    return LaunchDescription(
        [
            ExecuteProcess(cmd=[sys.executable, STUB, str(PORT)], output="screen"),
            # The client pings in its constructor, so the stub has to be bound first.
            TimerAction(period=3.0, actions=[detector]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDetector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("detector_probe")
        cls.camera = cls.node.create_publisher(Image, CAMERA_TOPIC, qos_profile_sensor_data)
        cls.received = []
        cls.node.create_subscription(
            InstanceMaskArray, MASK_TOPIC, lambda msg: cls.received.append(msg), 2
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish_frame(self, stamp=None):
        image = Image()
        image.header.stamp = stamp or self.node.get_clock().now().to_msg()
        image.header.frame_id = "camera_color_optical_frame"
        image.height, image.width = HEIGHT, WIDTH
        image.encoding = "rgb8"
        image.step = image.width * 3
        image.data = bytes(image.height * image.step)
        self.camera.publish(image)
        return image

    def _await_mask(self, timeout_s=20.0, republish=True):
        """Waits for the next mask array, publishing frames while it waits."""
        self.received.clear()
        deadline = self.node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.node.get_clock().now().nanoseconds < deadline:
            if republish:
                self._publish_frame()
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.received:
                return self.received[-1]
        return None

    def test_01_masks_arrive_for_a_prompted_phrase(self):
        masks = self._await_mask(timeout_s=60.0)

        self.assertIsNotNone(masks, "no masks; check the client's handshake against the stub")
        self.assertEqual(masks.model, "stub")
        self.assertEqual(masks.image_width, WIDTH)
        self.assertEqual(masks.image_height, HEIGHT)
        self.assertEqual(masks.header.frame_id, "camera_color_optical_frame")
        self.assertEqual(len(masks.instances), 1)

    def test_02_the_mask_describes_its_own_region(self):
        masks = self._await_mask()

        self.assertIsNotNone(masks)
        instance = masks.instances[0]
        self.assertEqual(instance.label, "red cube")
        self.assertAlmostEqual(instance.score, 0.77, places=5)
        self.assertEqual((instance.roi.x_offset, instance.roi.y_offset), (100, 60))
        self.assertEqual((instance.roi.width, instance.roi.height), (40, 30))
        self.assertEqual(len(instance.data), 40 * 30)
        self.assertEqual(set(instance.data), {255})

    def test_03_the_stamp_is_a_frame_that_was_looked_at(self):
        # The geometry node pairs masks with depth on this stamp, so a publish-time stamp would
        # quietly deproject an outline onto whatever has moved under it since. Which frame the
        # detector picked is its own business; that the stamp is one of theirs is not.
        self.received.clear()
        sent = set()
        deadline = self.node.get_clock().now().nanoseconds + int(30e9)
        while self.node.get_clock().now().nanoseconds < deadline and not self.received:
            frame = self._publish_frame()
            sent.add((frame.header.stamp.sec, frame.header.stamp.nanosec))
            rclpy.spin_once(self.node, timeout_sec=0.2)

        self.assertTrue(self.received, "no masks for the frames that were published")
        answered = (self.received[-1].header.stamp.sec, self.received[-1].header.stamp.nanosec)
        self.assertIn(answered, sent, "the masks are stamped with something other than a frame")

    def test_04_an_empty_phrase_list_stops_the_stream(self):
        client = self.node.create_client(SetParameters, "/g1_detector/set_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        self._set_phrases(client, [])
        self.received.clear()
        deadline = self.node.get_clock().now().nanoseconds + int(5e9)
        while self.node.get_clock().now().nanoseconds < deadline:
            self._publish_frame()
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertEqual(self.received, [], "an empty phrase list should stop the detector")

        # And it comes back, which proves the list is read per detection rather than at startup.
        self._set_phrases(client, ["blue sphere"])
        masks = self._await_mask(timeout_s=30.0)
        self.assertIsNotNone(masks, "the detector did not resume after the phrases were set")
        self.assertEqual(masks.instances[0].label, "blue sphere")

    def _set_phrases(self, client, phrases):
        request = SetParameters.Request()
        # The type is spelled out because rclpy infers BYTE_ARRAY from an empty list, and the
        # node's parameter is a string array; the mismatch is refused rather than coerced.
        request.parameters = [
            Parameter("phrases", Parameter.Type.STRING_ARRAY, phrases).to_parameter_msg()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), "set_parameters never answered")
        self.assertTrue(future.result().results[0].successful)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
