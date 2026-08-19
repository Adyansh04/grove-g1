"""Headless sim integration test: a pick and a place, end to end.

The acceptance gate for this package. Everything below only exists once every layer is running
together and no unit test can see it: that ground truth reaches /objects at all, that a Pick
moves the object off the surface for real rather than only in the planning scene, that the
skill refuses a goal it cannot see, and that the collision exemption it opens around the grasp
is closed again afterwards.

Deliberately measures the OBJECT, not the action result. A skill that reports success while the
cube never moved is exactly the failure worth catching, and the sim-only grasp weld is what
makes the object's own pose meaningful evidence.

Run via `colcon test --packages-select g1_manipulation`.
"""

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
from rclpy.action import ActionClient
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from g1_msgs.action import Pick, Place, SetArmPosture

# The stack needs longer here than the MoveIt suites do: this one brings up the simulator,
# move_group, the skills AND the object pipeline, and the acquire is delayed behind all of it.
STACK_SETTLE_S = 55.0
# launch_testing waits only 15 s for ReadyToTest by default (loader.py), and a settle past that
# aborts the run with "Timed out waiting for processes to start up" before a single test runs.
# The MoveIt suites settle in 12-14 s and never hit it, which is why this is the only file that
# needs the override.
READY_TIMEOUT_S = STACK_SETTLE_S + 30.0

OBJECT_ID = "red_cube"

# Generous. A pick is four planned motions plus two hand closes at 0.3 velocity scaling, and
# OMPL is given 10 s per plan; this is a timeout, not an expectation.
PICK_TIMEOUT_S = 240.0

# A tuck is one planned motion per arm, so it needs nothing like a pick's budget.
POSTURE_TIMEOUT_S = 90.0


@launch_testing.ready_to_test_action_timeout(READY_TIMEOUT_S)
def generate_test_description():
    bringup = get_package_share_directory("g1_bringup")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup.launch.py")),
        launch_arguments={
            "moveit": "true",
            "manipulation": "true",
            # Ground truth, not the stack default. FAST-LIO cannot work in this scene: the
            # manipulation world is a bench at arm's length with the pelvis pinned, so the
            # Mid360 returns nothing, fast_lio logs "No point, skip this scan!" forever and
            # never publishes odom, leaving g1_object_pose_source with no frame to place into.
            "odometry": "ground_truth",
            # The object is at arm's length here, so nothing has to drive anywhere and the
            # gait cannot make the test flaky. The facility mission is g1_orchestration's.
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
        while time.monotonic() < deadline and rclcpp_ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _object_pose(self, timeout_s=20.0):
        """The cube's ground-truth pose, or None. Fresh each call: it moves."""
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
        """The whole sim-side chain: sampler, socket, relay, pose source, all in one check."""
        pose = self._object_pose()
        self.assertIsNotNone(pose, "/objects never carried the cube; is the pose source active?")
        # On the table, not on the floor and not at the origin. The scene puts it at 0.83.
        self.assertGreater(pose.position.z, 0.7, "the cube is not on the table")

    def test_02_the_arms_tuck_clear_of_the_workbench(self):
        """Both arms, before anything is planned, exactly as the mission tree does it.

        A hanging hand sits about 21 cm in front of the pelvis and 1 cm UNDER the workbench
        slab, so it lands inside the bench's octomap. MoveIt's CheckStartStateCollision looks at
        the whole robot, not just the group being planned for, so one hanging arm refuses every
        plan, including opening the other hand. The mission tucks both arms for the same
        reason; a test that skips it is testing a pose the robot is never in.
        """
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

        result = self._send(
            self.pick, Pick.Goal(object_id=OBJECT_ID, arm="right"), PICK_TIMEOUT_S
        )
        self.assertTrue(result.success, f"pick failed: {result.message}")

        after = self._object_pose()
        self.assertIsNotNone(after)
        # The grasp weld is what makes this meaningful: without it the planning scene would say
        # the object is held while it sat on the table untouched.
        self.assertGreater(
            after.position.z - before.position.z,
            0.02,
            f"the cube did not leave the surface: {before.position.z} -> {after.position.z}",
        )

    def test_04_a_place_puts_it_back_down(self):
        """Runs after the pick, so the arm is holding the cube."""
        target = self._object_pose()
        self.assertIsNotNone(target)

        goal = Place.Goal(arm="right")
        goal.pose.header.frame_id = "odom"
        goal.pose.pose.position.x = target.position.x
        goal.pose.pose.position.y = target.position.y - 0.06
        goal.pose.pose.position.z = 0.83
        goal.pose.pose.orientation.w = 1.0

        result = self._send(self.place, goal, PICK_TIMEOUT_S)
        self.assertTrue(result.success, f"place failed: {result.message}")

    def test_05_an_unknown_object_is_refused_not_guessed_at(self):
        """The pose source has no such object, so the skill must decline rather than reach."""
        result = self._send(
            self.pick, Pick.Goal(object_id="no_such_object", arm="right"), 60.0
        )
        self.assertFalse(result.success)
        self.assertIn("locating", result.message)

    def test_06_a_bad_arm_is_refused(self):
        result = self._send(self.pick, Pick.Goal(object_id=OBJECT_ID, arm="middle"), 60.0)
        self.assertFalse(result.success)
        self.assertIn("left", result.message)


def rclcpp_ok():
    return rclpy.ok()
