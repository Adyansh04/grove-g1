#!/usr/bin/env python3
"""Feeds the perception visualizer synthetic frames and checks what it draws.

No simulator: a static transform stands in for the robot, and the probe plays the camera, the
geometry node and the relay's ground truth, all on one stamp.
"""

import os
import time
import unittest

import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.node import Node as RclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from g1_msgs.msg import InstanceMask, InstanceMaskArray

CAMERA_FRAME = "camera_color_optical_frame"
TRUTH_TOPIC = "/g1_sensor_relay/object_poses"
WIDTH, HEIGHT = 160, 120
# Unrotated, so a camera point is an odom point shifted along x.
CAMERA_IN_ODOM_X = 1.0
# x, y, width, height
MEASURED_ROI = (60, 40, 40, 40)
REJECTED_ROI = (10, 10, 20, 20)


@pytest.mark.launch_test
def generate_test_description():
    visualizer = Node(
        package="g1_perception",
        executable="g1_perception_visualizer",
        name="g1_perception_visualizer",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("g1_perception"),
                "config",
                "g1_perception_visualizer.yaml",
            ),
            {"ground_truth_topic": TRUTH_TOPIC},
        ],
        remappings=[
            ("color/image_raw", "/camera/color/image_raw"),
            ("color/camera_info", "/camera/color/camera_info"),
            ("tracked_masks", "/g1_object_geometry/tracked_masks"),
            ("object_poses", "/g1_object_geometry/object_poses"),
        ],
    )
    camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            str(CAMERA_IN_ODOM_X),
            "--frame-id",
            "odom",
            "--child-frame-id",
            CAMERA_FRAME,
        ],
    )
    return LaunchDescription([visualizer, camera, launch_testing.actions.ReadyToTest()])


def _detection(name, x, y, z, size):
    detection = Detection3D()
    detection.id = name
    detection.bbox.center.position = Point(x=x, y=y, z=z)
    detection.bbox.center.orientation.w = 1.0
    detection.bbox.size.x = detection.bbox.size.y = detection.bbox.size.z = size
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = name
    detection.results.append(hypothesis)
    return detection


def _instance(label, roi):
    instance = InstanceMask()
    instance.label = label
    instance.score = 0.9
    x, y, width, height = roi
    instance.roi.x_offset, instance.roi.y_offset = x, y
    instance.roi.width, instance.roi.height = width, height
    instance.data = bytes([255]) * (width * height)
    return instance


def _pixel(image, x, y):
    start = (y * image.width + x) * 3
    return tuple(image.data[start : start + 3])


# One tracked cube 5 mm from its truth, its bare-phrase alias, and a sphere the geometry node
# rejected, which keeps its raw label. The cup is never seen.
SCENE_INSTANCES = [_instance("red_cube_0", MEASURED_ROI), _instance("blue sphere", REJECTED_ROI)]
SCENE_DETECTIONS = [
    _detection("red_cube_0", 0.003, 0.004, 0.6, 0.06),
    _detection("red_cube", 0.003, 0.004, 0.6, 0.06),
]
TRUTH = [_detection("red_cube", 0.0, 0.0, 0.6, 0.06), _detection("white_cup", 0.1, 0.0, 0.6, 0.08)]


class TestVisualizer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("visualizer_probe")
        reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor = qos_profile_sensor_data
        cls.image_pub = cls.node.create_publisher(Image, "/camera/color/image_raw", sensor)
        cls.info_pub = cls.node.create_publisher(CameraInfo, "/camera/color/camera_info", sensor)
        cls.truth_pub = cls.node.create_publisher(Detection3DArray, TRUTH_TOPIC, sensor)
        cls.masks_pub = cls.node.create_publisher(
            InstanceMaskArray, "/g1_object_geometry/tracked_masks", reliable
        )
        cls.objects_pub = cls.node.create_publisher(
            Detection3DArray, "/g1_object_geometry/object_poses", reliable
        )
        cls.images = []
        cls.truth_markers = []
        cls.node.create_subscription(
            Image, "/g1_perception_visualizer/annotated_image", cls.images.append, sensor
        )
        cls.node.create_subscription(
            MarkerArray, "/g1_perception_visualizer/ground_truth", cls.truth_markers.append, latched
        )

        # Retried with fresh stamps: until discovery and the static transform settle, a frame or
        # the ground truth can be lost without anything to wait on.
        cls.scene = None
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            cls.scene = cls._render(SCENE_INSTANCES, SCENE_DETECTIONS) or cls.scene
            if cls.scene is not None and cls._labelled_truth() is not None:
                break

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _spin(cls, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def _render(cls, instances, detections, frame_last=False):
        """Plays one frame through, and returns what the visualizer drew for it, if anything."""
        stamp = cls.node.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = CAMERA_FRAME
        image.height, image.width = HEIGHT, WIDTH
        image.encoding = "rgb8"
        image.step = WIDTH * 3
        image.data = bytes(HEIGHT * WIDTH * 3)
        info = CameraInfo(header=image.header, width=WIDTH, height=HEIGHT)
        info.k = [100.0, 0.0, WIDTH / 2, 0.0, 100.0, HEIGHT / 2, 0.0, 0.0, 1.0]

        def send_frame():
            cls.image_pub.publish(image)
            cls.info_pub.publish(info)
            cls.truth_pub.publish(Detection3DArray(header=image.header, detections=TRUTH))

        def send_detections():
            # Masks first, the opposite of the geometry node's order; neither is promised.
            cls.masks_pub.publish(
                InstanceMaskArray(
                    header=image.header, image_width=WIDTH, image_height=HEIGHT, instances=instances
                )
            )
            cls.objects_pub.publish(Detection3DArray(header=image.header, detections=detections))

        first, second = (
            (send_detections, send_frame) if frame_last else (send_frame, send_detections)
        )
        first()
        cls._spin(0.3)
        second()
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            for drawn in cls.images:
                if drawn.header.stamp == stamp:
                    return drawn
        return None

    @classmethod
    def _render_until_drawn(cls, instances, detections, frame_last=False):
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            drawn = cls._render(instances, detections, frame_last)
            if drawn is not None:
                return drawn
        return None

    @classmethod
    def _labelled_truth(cls):
        return next(
            (
                markers
                for markers in reversed(cls.truth_markers)
                if any("mm off" in marker.text for marker in markers.markers)
            ),
            None,
        )

    def test_01_the_frame_comes_back_with_its_stamp_and_shape(self):
        self.assertIsNotNone(self.scene, "no annotated image for any frame that was published")
        self.assertEqual(self.scene.encoding, "rgb8")
        self.assertEqual((self.scene.width, self.scene.height), (WIDTH, HEIGHT))
        self.assertEqual(self.scene.header.frame_id, CAMERA_FRAME)

    def test_02_both_instances_are_tinted_and_nothing_else_is(self):
        self.assertIsNotNone(self.scene)
        self.assertNotEqual(_pixel(self.scene, 65, 50), (0, 0, 0), "measured instance untinted")
        self.assertNotEqual(_pixel(self.scene, 20, 25), (0, 0, 0), "rejected instance untinted")
        self.assertEqual(_pixel(self.scene, 150, 110), (0, 0, 0))

    def test_03_ground_truth_is_drawn_where_the_objects_are(self):
        markers = self._labelled_truth()
        self.assertIsNotNone(markers, "no ground truth labelled with an error")
        self.assertEqual(markers.markers[0].action, Marker.DELETEALL)
        boxes = [marker for marker in markers.markers if marker.type == Marker.CUBE]
        self.assertEqual(len(boxes), len(TRUTH))
        for box, truth in zip(boxes, TRUTH, strict=True):
            self.assertEqual(box.header.frame_id, "odom")
            self.assertAlmostEqual(
                box.pose.position.x, truth.bbox.center.position.x + CAMERA_IN_ODOM_X, places=6
            )

    def test_04_each_truth_is_labelled_with_how_far_perception_is(self):
        markers = self._labelled_truth()
        self.assertIsNotNone(markers)
        labels = {
            marker.text for marker in markers.markers if marker.type == Marker.TEXT_VIEW_FACING
        }
        self.assertEqual(labels, {"red_cube 5 mm off", "white_cup not seen"})

    def test_05_a_frame_with_nothing_found_comes_back_untouched(self):
        drawn = self._render_until_drawn([], [])
        self.assertIsNotNone(drawn)
        self.assertEqual(set(drawn.data), {0})

    def test_06_masks_that_beat_their_frame_are_still_drawn(self):
        # The mock detector cuts masks from depth, whose colour twin can still be in flight.
        drawn = self._render_until_drawn(SCENE_INSTANCES, SCENE_DETECTIONS, frame_last=True)
        self.assertIsNotNone(drawn, "masks published before their frame were never drawn")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
