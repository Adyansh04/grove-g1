#!/usr/bin/env python3
"""Checks what a pick does with a grasp generator behind it.

Three claims, none of which any other test covers:

  1. Offered sensible grasps, the pick takes one and gets past locating, which is where every
     generator-side failure lands: no service, no candidates, nothing solvable.
  2. Offered only a grasp reaching up through the table, it refuses that one too, and says so.
     A filter that never rejects anything has not been tested.
  3. An object nobody is reporting is still refused, with the generator in the loop.

There is no fallback to the fixed top-down pose anywhere in those paths, which is the property
worth the most here: a pick told to use a generator and quietly not using it would look like it
worked.

What this test deliberately does not assert is that the object ends up in the hand. The arm's
rest pose in a scene with a table already fails MoveIt's start-state check before anything moves,
which is why g1_manipulation's own pick suite is unregistered; that is a separate open problem
and this pipeline does not fix it.
"""

import os
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

from g1_msgs.action import Pick

STACK_SETTLE_S = 55.0
READY_TIMEOUT_S = 85.0
PICK_TIMEOUT_S = 240.0
OBJECT_ID = "red_cube_0"

# The stand-in generator answers in its own gripper frame, where +z is the approach direction.
# This is what turns one of its poses into a goal for right_hand_grasp_frame, and it is exactly
# the measurement the real generator needs before it can drive anything.
GRASP_OFFSET = "[0.0, 0.0, 0.09, 1.5707963, 0.0, 0.0]"


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

    def _set_only_from_below(self, value):
        """Switches the stand-in generator to offering nothing but the grasp from underneath."""
        client = self.node.create_client(
            SetParameters, "/g1_mock_grasp_source/set_parameters"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        request = SetParameters.Request()
        request.parameters = [
            Parameter("only_from_below", Parameter.Type.BOOL, value).to_parameter_msg()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertIsNotNone(future.result(), "the generator never answered set_parameters")
        self.assertTrue(future.result().results[0].successful)

    def test_01_a_generated_grasp_is_chosen_and_attempted(self):
        result = self._pick()

        # Locating covers the whole generator side: no service, no candidates, nothing usable.
        # Getting past it means a candidate was transformed, filtered, solved and planned for.
        self.assertNotIn(
            "locating",
            result.message,
            f"the pick never got a usable grasp out of the generator: {result.message}",
        )

    def test_02_a_grasp_from_under_the_table_is_refused(self):
        # The only candidate on offer now reaches up through the surface the object stands on.
        # Taking it would drive the hand through the table, so the pick has to end here.
        self._set_only_from_below(True)
        try:
            result = self._pick()
        finally:
            self._set_only_from_below(False)

        self.assertFalse(result.success)
        self.assertIn("locating", result.message)
        self.assertIn("candidates", result.message)

    def test_03_an_unknown_object_is_still_refused(self):
        result = self._pick(object_id="no_such_object")

        self.assertFalse(result.success)
        self.assertIn("locating", result.message)


# No post-shutdown exit-code check, matching the other sim suites: move_group segfaults in its
# own destructor on this MoveIt and ros2_control_node leaves 130 on SIGINT. Asserting on that
# tests their teardown, not this pick.
