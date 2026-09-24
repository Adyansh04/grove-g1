"""Headless sim integration test: a pick and a place, end to end.

The acceptance gate for this package. It measures the object, not the action result: the fingers
grip by contact and friction in this scene, so the block's own pose is the evidence.

Run via `colcon test --packages-select g1_manipulation`.
"""

import math
import os
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from g1_msgs.action import Pick, Place, SetArmPosture

STACK_SETTLE_S = 55.0
# launch_testing waits 15 s for ReadyToTest by default, less than this stack takes to settle.
READY_TIMEOUT_S = STACK_SETTLE_S + 30.0

OBJECT_ID = "red_block"

PICK_TIMEOUT_S = 240.0
POSTURE_TIMEOUT_S = 90.0

# How long a lifted object has to stay in the hand: long enough for a slipping grip to show it.
HOLD_S = 3.0


@launch_testing.ready_to_test_action_timeout(READY_TIMEOUT_S)
def generate_test_description():
    bringup = get_package_share_directory("g1_bringup")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup.launch.py")),
        launch_arguments={
            "moveit": "true",
            "manipulation": "true",
            # FAST-LIO gets no returns in this scene, so it never publishes odom.
            "odometry": "ground_truth",
            # The object is at arm's length, so the gait cannot make the test flaky.
            "world": "manipulation",
            "pin_pelvis": "true",
            "activate_arm": "true",
            "activate_arm_delay_s": "40.0",
            "headless": "true",
            "rviz": "false",
        }.items(),
    )
    return LaunchDescription(
        [stack, TimerAction(period=STACK_SETTLE_S, actions=[launch_testing.actions.ReadyToTest()])]
    )


class TestPickPlace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_pick_place")
        cls.objects = None

        def _on_objects(msg):
            cls.objects = msg

        cls.node.create_subscription(Detection3DArray, "/objects", _on_objects, 1)
        cls.pick = ActionClient(cls.node, Pick, "/g1_manipulation_server/pick")
        cls.place = ActionClient(cls.node, Place, "/g1_manipulation_server/place")
        cls.posture = ActionClient(
            cls.node, SetArmPosture, "/g1_manipulation_server/set_arm_posture"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _object_pose(self, timeout_s=20.0):
        """The block's ground-truth pose, or None. Fresh each call: it moves."""
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

    def _get_server_parameter(self, name):
        client = self.node.create_client(GetParameters, "/g1_manipulation_server/get_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), "no parameter service")
        future = client.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertIsNotNone(future.result(), "the server never answered get_parameters")
        return future.result().values[0].double_value

    def _set_server_parameter(self, name, value):
        """Retunes the running skill server, so a negative case needs no second bring-up."""
        client = self.node.create_client(SetParameters, "/g1_manipulation_server/set_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), "no parameter service")
        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = name
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = value
        request.parameters = [parameter]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertIsNotNone(future.result(), "the server never answered set_parameters")
        return all(result.successful for result in future.result().results)

    def _send(self, client, goal, timeout_s):
        self.assertTrue(client.wait_for_server(timeout_sec=30.0), "no action server")
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        self.assertIsNotNone(handle, "goal was never accepted")
        self.assertTrue(handle.accepted, "goal was rejected")
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=timeout_s)
        self.assertIsNotNone(result.result(), "goal did not return")
        return result.result().result

    def test_01_ground_truth_reaches_objects(self):
        """The sim-side chain: sampler, socket, relay, pose source."""
        pose = self._object_pose()
        self.assertIsNotNone(pose, "/objects never carried the block; is the pose source active?")
        self.assertGreater(pose.position.z, 0.7, "the block is not on the table")

    def test_02_the_arms_tuck_clear_of_the_workbench(self):
        """Both arms, as the mission does: a hanging hand sits inside the bench's octomap, and
        MoveIt checks the whole robot's start state, so one hanging arm refuses every plan."""
        for group in ("right_arm", "left_arm"):
            result = self._send(
                self.posture,
                SetArmPosture.Goal(group=group, named_target="tucked"),
                POSTURE_TIMEOUT_S,
            )
            self.assertTrue(result.success, f"{group} would not tuck: {result.message}")

    def test_03_a_pick_lifts_the_object_for_real(self):
        """Measures the OBJECT. A pick that only succeeds in the planning scene fails here."""
        before = self._object_pose()
        self.assertIsNotNone(before)

        result = self._send(self.pick, Pick.Goal(object_id=OBJECT_ID, arm="right"), PICK_TIMEOUT_S)
        self.assertTrue(result.success, f"pick failed: {result.message}")

        lifted = self._object_pose()
        self.assertIsNotNone(lifted)
        # More than the block's own height, so it cannot be resting on anything. The arm settles
        # short of lift_height_m under the load.
        self.assertGreater(
            lifted.position.z - before.position.z,
            0.08,
            f"the block did not come up with the hand: {before.position.z} -> {lifted.position.z}",
        )

        # Held, not just lifted: a friction grip that is going to fail does so after the lift.
        self._spin(HOLD_S)
        held = self._object_pose()
        self.assertIsNotNone(held)
        self.assertGreater(
            held.position.z - before.position.z,
            0.08,
            f"the block was dropped while held: {held.position.z}",
        )
        slip = math.dist(
            (held.position.x, held.position.y, held.position.z),
            (lifted.position.x, lifted.position.y, lifted.position.z),
        )
        self.assertLess(slip, 0.02, f"the block slipped {slip * 1000:.0f} mm in the hand")

    def test_04_a_place_puts_it_back_down(self):
        """Runs after the pick, so the arm is holding the block.

        The target is a fixed spot on the pedestal: the close drags the block toward the robot, so
        placing it where it is carried would walk it off the near edge over repeated runs.
        """
        held = self._object_pose()
        self.assertIsNotNone(held)

        goal = Place.Goal(arm="right")
        goal.pose.header.frame_id = "odom"
        # The block's spawn x, and the height its centre reads at when resting on the table.
        goal.pose.pose.position.x = 0.32
        goal.pose.pose.position.y = held.position.y - 0.06
        goal.pose.pose.position.z = 0.845
        goal.pose.pose.orientation.w = 1.0

        result = self._send(self.place, goal, PICK_TIMEOUT_S)
        self.assertTrue(result.success, f"place failed: {result.message}")

    def test_05_a_grasp_that_misses_is_reported_as_a_miss(self):
        """Proves the grip check can say no: the grip point is put 30 cm up, where the fingers
        close on air, and the pick must fail in the grasp phase without moving the block."""
        configured = self._get_server_parameter("min_grip_height_m")
        self.assertTrue(self._set_server_parameter("min_grip_height_m", 0.30))
        try:
            before = self._object_pose()
            self.assertIsNotNone(before)
            result = self._send(
                self.pick, Pick.Goal(object_id=OBJECT_ID, arm="right"), PICK_TIMEOUT_S
            )
            self.assertFalse(result.success, "a grasp above the block was reported as a pick")
            self.assertIn("grasp", result.message)
            after = self._object_pose()
            self.assertIsNotNone(after)
            self.assertLess(
                abs(after.position.z - before.position.z),
                0.02,
                "the block moved, so this failed for the wrong reason",
            )
        finally:
            self.assertTrue(self._set_server_parameter("min_grip_height_m", configured))

    def test_06_an_unknown_object_is_refused_not_guessed_at(self):
        """An object nobody reports is refused, not guessed at."""
        result = self._send(self.pick, Pick.Goal(object_id="no_such_object", arm="right"), 60.0)
        self.assertFalse(result.success)
        self.assertIn("locating", result.message)

    def test_07_a_bad_arm_is_refused(self):
        result = self._send(self.pick, Pick.Goal(object_id=OBJECT_ID, arm="middle"), 60.0)
        self.assertFalse(result.success)
        self.assertIn("left", result.message)
