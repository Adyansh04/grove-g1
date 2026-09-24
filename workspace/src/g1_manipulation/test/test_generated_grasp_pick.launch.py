#!/usr/bin/env python3
"""A pick with a grasp generator behind it.

Covers: a grasp reaching up through the table is refused; an unreported object is refused with
the generator in the loop; a sensible grasp picks the object up; with RViz off nothing is drawn.
Tests run in name order, and test_03 takes the object off the table, so it comes last.
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
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter
from vision_msgs.msg import Detection3DArray

from g1_msgs.action import Pick

STACK_SETTLE_S = 55.0
READY_TIMEOUT_S = 85.0
PICK_TIMEOUT_S = 240.0
OBJECT_ID = "red_block_0"
DRAWN_TOPICS = [
    "/g1_perception_visualizer/annotated_image",
    "/g1_perception_visualizer/ground_truth",
    "/g1_manipulation_server/grasp_plan",
    "/object_markers",
]

# The stand-in generator's gripper frame to right_hand_grasp_frame. It reports a pose 10 cm above
# the object's top face, so the grasp frame lands just under that face. Against the real
# generator this vector is a measurement.
GRASP_OFFSET = "[0.0, 0.0, 0.102, 1.5707963, 0.0, 0.0]"


def _bringup(**arguments):
    launch_arguments = {
        "world": "tabletop",
        "pin_pelvis": "true",
        "odometry": "ground_truth",
        "moveit": "true",
        "manipulation": "true",
        "perception": "true",
        "detector": "mock",
        "headless": "true",
        "rviz": "false",
        "activate_arm": "true",
        "activate_arm_delay_s": "40.0",
        "grasp_source": "generated",
        "grasp_offset": GRASP_OFFSET,
    }
    launch_arguments.update(arguments)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "bringup.launch.py")
        ),
        launch_arguments=launch_arguments.items(),
    )


@pytest.mark.launch_test
@launch_testing.ready_to_test_action_timeout(READY_TIMEOUT_S)
def generate_test_description():
    return LaunchDescription(
        [
            _bringup(grasp_engine="mock"),
            TimerAction(period=STACK_SETTLE_S, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


class TestGeneratedGraspPick(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("generated_grasp_probe")
        cls.pick = ActionClient(cls.node, Pick, "/g1_manipulation_server/pick")
        cls.objects = None
        cls.node.create_subscription(
            Detection3DArray, "/objects", lambda msg: setattr(cls, "objects", msg), 1
        )
        if not cls.pick.wait_for_server(timeout_sec=90.0):
            raise AssertionError("the manipulation server never came up")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _pick(self, object_id=OBJECT_ID):
        goal = Pick.Goal()
        goal.object_id = object_id
        goal.arm = "right"
        handle_future = self.pick.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, handle_future, timeout_sec=30.0)
        handle = handle_future.result()
        self.assertIsNotNone(handle, "the pick goal was never handled")
        self.assertTrue(handle.accepted, "the pick goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=PICK_TIMEOUT_S)
        self.assertIsNotNone(result_future.result(), "the pick never finished")
        return result_future.result().result

    def _object_pose(self, timeout_s=20.0):
        """The object's ground-truth pose, or None. Fresh each call: it moves."""
        self.__class__.objects = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            msg = self.__class__.objects
            if msg is None:
                continue
            for detection in msg.detections:
                if detection.results and detection.results[0].hypothesis.class_id == OBJECT_ID:
                    return detection.results[0].pose.pose
        return None

    def _set_only_from_below(self, value):
        """Switches the stand-in generator to offering nothing but the grasp from underneath."""
        client = self.node.create_client(SetParameters, "/g1_mock_grasp_source/set_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        request = SetParameters.Request()
        request.parameters = [
            Parameter("only_from_below", Parameter.Type.BOOL, value).to_parameter_msg()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertIsNotNone(future.result(), "the generator never answered set_parameters")
        self.assertTrue(future.result().results[0].successful)

    def test_00_nothing_is_drawn_with_rviz_off(self):
        # Against topics that must exist, so unfinished discovery cannot pass this.
        for topic in ("/g1_object_geometry/object_poses", "/objects"):
            self.assertNotEqual(self.node.get_publishers_info_by_topic(topic), [], topic)
        self.assertNotIn("g1_perception_visualizer", self.node.get_node_names())
        for topic in DRAWN_TOPICS:
            self.assertEqual(self.node.get_publishers_info_by_topic(topic), [], topic)

    def test_03_a_generated_grasp_picks_the_object_up(self):
        """Asserts the result: /objects comes from perception here, which loses the object
        once the hand covers it. test_pick_place measures the object against ground truth."""
        before = self._object_pose()
        self.assertIsNotNone(before)

        result = self._pick()
        self.assertTrue(result.success, result.message)
        self.assertIn(OBJECT_ID, result.message)
        self.assertIn("pressing", result.message)

    def test_01_a_grasp_from_under_the_table_is_refused(self):
        # The only candidate on offer reaches up through the table.
        self._set_only_from_below(True)
        try:
            result = self._pick()
        finally:
            self._set_only_from_below(False)

        self.assertFalse(result.success)
        self.assertIn("locating", result.message)
        self.assertIn("candidates", result.message)

    def test_02_an_unknown_object_is_still_refused(self):
        result = self._pick(object_id="no_such_object")

        self.assertFalse(result.success)
        self.assertIn("locating", result.message)


# No post-shutdown exit-code check: move_group segfaults in its own destructor on this MoveIt, and
# ros2_control_node exits 130 on SIGINT.
