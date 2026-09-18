"""Headless sim integration test: a pick and a place, end to end.

The acceptance gate for this package. Everything below only exists once every layer is running
together and no unit test can see it: that ground truth reaches /objects at all, that a Pick
moves the object off the surface for real rather than only in the planning scene, that the
skill refuses a goal it cannot see, and that the collision exemption it opens around the grasp
is closed again afterwards.

Deliberately measures the OBJECT, not the action result. A skill that reports success while the
cube never moved is exactly the failure worth catching. The fingers grip by contact and friction
here, with no weld behind them, so the cube's own pose is the only evidence that means anything.

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
from rcl_interfaces.srv import SetParameters
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

# How long a lifted object has to stay in the hand. 1500 physics steps at 2 ms, which is long
# enough for a grip that is going to slip to have done it.
HOLD_S = 3.0


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
        while time.monotonic() < deadline and rclpy.ok():
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

    def _set_server_parameter(self, name, value):
        """Retunes the running skill server, so a negative case needs no second bring-up."""
        client = self.node.create_client(
            SetParameters, "/g1_manipulation_server/set_parameters"
        )
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

        lifted = self._object_pose()
        self.assertIsNotNone(lifted)
        # Well clear of the table, not a nudge. lift_height_m asks for 0.20 and the arm delivers
        # about 0.14 of it, because a position-only arm settles short under the cube's weight;
        # what matters here is that the cube is more than its own height off the surface, so it
        # cannot be resting on anything.
        self.assertGreater(
            lifted.position.z - before.position.z,
            0.12,
            f"the cube did not come up with the hand: {before.position.z} -> {lifted.position.z}",
        )

        # Held, not just lifted. The fingers grip by friction, so a grasp that is going to fail
        # does it during the seconds after the lift rather than at the moment of it.
        self._spin(HOLD_S)
        held = self._object_pose()
        self.assertIsNotNone(held)
        self.assertGreater(
            held.position.z - before.position.z,
            0.12,
            f"the cube was dropped while held: {held.position.z}",
        )
        slip = math.dist(
            (held.position.x, held.position.y, held.position.z),
            (lifted.position.x, lifted.position.y, lifted.position.z),
        )
        self.assertLess(slip, 0.02, f"the cube slipped {slip * 1000:.0f} mm in the hand")

    def test_04_a_place_puts_it_back_down(self):
        """Runs after the pick, so the arm is holding the cube."""
        target = self._object_pose()
        self.assertIsNotNone(target)

        goal = Place.Goal(arm="right")
        goal.pose.header.frame_id = "odom"
        goal.pose.pose.position.x = target.position.x
        goal.pose.pose.position.y = target.position.y - 0.06
        # Where a 7 cm cube's centre reports when it is sitting on this table, so the place puts
        # it back down rather than pressing it through the top.
        goal.pose.pose.position.z = 0.825
        goal.pose.pose.orientation.w = 1.0

        result = self._send(self.place, goal, PICK_TIMEOUT_S)
        self.assertTrue(result.success, f"place failed: {result.message}")

    def test_05_a_grasp_that_misses_is_reported_as_a_miss(self):
        """The one that proves the grip check can say no.

        Every other failure mode here is the skill refusing before it moves. This one lets it
        run the whole pick with the grip point put 30 cm up, where the fingers close on air, and
        asks whether it notices. Without the check the trajectory controller reports the close
        as success and the pick claims a cube it never touched.
        """
        self.assertTrue(self._set_server_parameter("min_grip_height_m", 0.30))
        try:
            before = self._object_pose()
            self.assertIsNotNone(before)
            result = self._send(
                self.pick, Pick.Goal(object_id=OBJECT_ID, arm="right"), PICK_TIMEOUT_S
            )
            self.assertFalse(result.success, "a grasp above the cube was reported as a pick")
            self.assertIn("grasp", result.message)
            after = self._object_pose()
            self.assertIsNotNone(after)
            self.assertLess(
                abs(after.position.z - before.position.z),
                0.02,
                "the cube moved, so this failed for the wrong reason",
            )
        finally:
            self.assertTrue(self._set_server_parameter("min_grip_height_m", 0.068))

    def test_06_an_unknown_object_is_refused_not_guessed_at(self):
        """The pose source has no such object, so the skill must decline rather than reach."""
        result = self._send(
            self.pick, Pick.Goal(object_id="no_such_object", arm="right"), 60.0
        )
        self.assertFalse(result.success)
        self.assertIn("locating", result.message)

    def test_07_a_bad_arm_is_refused(self):
        result = self._send(self.pick, Pick.Goal(object_id=OBJECT_ID, arm="middle"), 60.0)
        self.assertFalse(result.success)
        self.assertIn("left", result.message)

