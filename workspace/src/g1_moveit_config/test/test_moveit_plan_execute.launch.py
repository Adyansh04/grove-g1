"""Headless sim integration test: planning and executing through move_group.

Covers the properties that only exist once every layer is running together, and that no config
test can see: that MoveIt refuses to execute before the arm is acquired, that a coordinated
14-joint both_arms plan reaches the controller as one trajectory, that planned motion respects
the bridge's speed clamp, and that the arm chain is placed correctly when the waist is not at
zero -- the case a robot standing square hides completely.

Run via `colcon test --packages-select g1_moveit_config`.
"""

import math
import os
import subprocess
import time
import unittest

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

# Same reasoning as g1_bringup's suites: under launch_testing's hardcoded 15 s process-startup
# deadline. move_group needs longer than the bridge alone, so the sim start is delayed further
# by moveit_sim.launch.py rather than by raising this.
SIM_SETTLE_S = 12.0

# Held by motion_service_sim's stiff-hold, not commanded by anything in this stack. Large enough
# that ignoring it would displace the hands by roughly 10 cm, far outside the tolerances below.
WAIST_HOLD_YAW_RAD = 0.35

LEFT_ARM = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]
RIGHT_ARM = [name.replace("left_", "right_") for name in LEFT_ARM]
BOTH_ARMS = LEFT_ARM + RIGHT_ARM

LEFT_HAND = [
    f"left_hand_{suffix}_joint"
    for suffix in ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
]
# The `closed` group state from g1.srdf, restated rather than read out of it: a test that took
# its expectation from the file under test would pass no matter what that file said.
LEFT_HAND_CLOSED = dict(
    zip(LEFT_HAND, [-0.30, -0.50, 1.20, -1.20, -1.40, -1.20, -1.40], strict=True)
)

# g1_description/config/arm_sdk_params.yaml. joint_limits.yaml plans under this; the test proves
# the cap actually binds rather than trusting the two files agree.
MAX_JOINT_VELOCITY_RAD_S = 1.0

# The full robot: 12 legs + 3 waist + 14 arms + 14 hand joints.
EXPECTED_JOINT_COUNT = 43


def generate_test_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("g1_moveit_config"),
                    "launch",
                    "moveit_sim.launch.py",
                )
            ),
            launch_arguments={
                # Pinned so the arm is exercised against a base that is not swaying.
                "pin_pelvis": "true",
                "waist_hold_rad": f"{WAIST_HOLD_YAW_RAD},0.0,0.0",
            }.items(),
        ),
        TimerAction(period=SIM_SETTLE_S, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestMoveItPlanExecute(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_moveit_plan_execute")
        cls.joint_state = {}
        cls.peak_velocity = 0.0
        cls.node.create_subscription(JointState, "/joint_states", cls._joint_cb, 20)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.move_client = ActionClient(cls.node, MoveGroup, "/move_action")
        cls.fk_client = cls.node.create_client(GetPositionFK, "/compute_fk")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _joint_cb(cls, msg):
        for name, position in zip(msg.name, msg.position, strict=False):
            cls.joint_state[name] = position
        # Velocity is shorter than name on some publishers, so this must not be strict.
        for name, velocity in zip(msg.name, msg.velocity, strict=False):
            if name in BOTH_ARMS:
                cls.peak_velocity = max(cls.peak_velocity, abs(velocity))

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _spin_until(self, predicate, timeout_s, message):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail(message)

    def _run_ros2_run(self, executable, timeout_s=40.0):
        proc = subprocess.Popen(
            ["ros2", "run", "g1_bringup", executable],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        deadline = time.monotonic() + timeout_s
        while proc.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        stdout, stderr = proc.communicate(timeout=10.0)
        self.assertEqual(proc.returncode, 0, f"{executable} failed:\n{stdout}\n{stderr}")

    def _joint_goal(self, group, joints, offset):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        # Under the joint_limits.yaml caps, which are themselves under the bridge clamp.
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.start_state = RobotState()
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        for name in joints:
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = self.joint_state[name] + offset
            constraint.tolerance_above = 0.02
            constraint.tolerance_below = 0.02
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints = [constraints]
        return goal

    def _absolute_joint_goal(self, group, targets):
        """Same goal, but to a named posture rather than a nudge away from where we are."""
        goal = self._joint_goal(group, list(targets), 0.0)
        for constraint in goal.request.goal_constraints[0].joint_constraints:
            constraint.position = targets[constraint.joint_name]
        return goal

    def _send_move_goal(self, goal, timeout_s=90.0):
        send = self.move_client.send_goal_async(goal)
        self._spin_until(send.done, 20.0, "move_group never answered the goal request")
        handle = send.result()
        if handle is None or not handle.accepted:
            return None
        result = handle.get_result_async()
        self._spin_until(result.done, timeout_s, "move_group never returned a result")
        return result.result().result

    # 1. The stack is up and the robot state is complete.
    def test_01_every_joint_has_a_state(self):
        self._spin_until(
            lambda: len(self.joint_state) >= EXPECTED_JOINT_COUNT,
            60.0,
            f"only {len(self.joint_state)} joints on /joint_states; MoveIt will not plan until "
            f"every active joint has a state (expected {EXPECTED_JOINT_COUNT})",
        )
        for name in BOTH_ARMS:
            self.assertIn(name, self.joint_state)
        self.assertIn("waist_yaw_joint", self.joint_state)
        # Finger state comes from the hand components via joint_state_broadcaster, which
        # publishes them while they are merely configured. MoveIt will not plan until every
        # active joint has a state, so this is a precondition, not a detail.
        self.assertIn("left_hand_index_0_joint", self.joint_state)

    # 2. The precondition for every waist assertion below. Without it the waist tests would
    #    still pass while proving nothing.
    def test_02_the_waist_is_actually_turned(self):
        self._spin(2.0)
        waist = self.joint_state["waist_yaw_joint"]
        self.assertGreater(
            abs(waist), 0.2,
            f"waist_yaw is {waist:.3f} rad; the torso is square to the pelvis and the frame "
            "assertions below would hold even if MoveIt ignored the waist entirely",
        )

    def test_03_move_group_is_serving(self):
        self.assertTrue(
            self.move_client.wait_for_server(timeout_sec=30.0), "no /move_action server"
        )
        self.assertTrue(
            self.fk_client.wait_for_service(timeout_sec=30.0), "no /compute_fk service"
        )

    # 3. The safety property: nothing executes until the arm is deliberately acquired.
    def test_04_execution_is_refused_before_the_arm_is_acquired(self):
        result = self._send_move_goal(self._joint_goal("left_arm", LEFT_ARM, 0.05))
        self.assertIsNotNone(result, "move_group rejected the goal outright")
        # 1 is SUCCESS. Anything else is the controller refusing, which is what should happen
        # while G1ArmSdkSystem is inactive and the JTC is not running.
        self.assertNotEqual(
            result.error_code.val, 1,
            "MoveIt executed a trajectory before activate_arm; the acquire step is the whole "
            "safety model for this arm",
        )

    # 4. Coordinated dual-arm motion: one plan, one trajectory, both arms.
    def test_05_both_arms_plan_and_execute_together(self):
        self._run_ros2_run("activate_arm")
        self._spin(3.0)
        self.peak_velocity = 0.0

        before = {name: self.joint_state[name] for name in BOTH_ARMS}
        result = self._send_move_goal(self._joint_goal("both_arms", BOTH_ARMS, 0.08))
        self.assertIsNotNone(result, "both_arms goal was rejected")
        self.assertEqual(
            result.error_code.val, 1,
            f"both_arms plan+execute failed with error_code {result.error_code.val}",
        )
        self._spin(2.0)

        # Both arms moved, which is what separates this from a single-arm plan that happened to
        # be accepted by a 14-joint controller.
        for side, joints in (("left", LEFT_ARM), ("right", RIGHT_ARM)):
            moved = max(abs(self.joint_state[n] - before[n]) for n in joints)
            self.assertGreater(moved, 0.02, f"{side} arm did not move during a both_arms plan")

    # 5. The cross-package coupling between MoveIt's planned speed and the hardware bridge's clamp.
    def test_06_planned_motion_respects_the_bridge_clamp(self):
        # Well above the jitter a stiff-held arm shows at rest, so this cannot pass on noise
        # while the plan above quietly failed to execute.
        self.assertGreater(
            self.peak_velocity, 0.05,
            "no real arm motion was observed; the velocity bound below would pass vacuously",
        )
        self.assertLess(
            self.peak_velocity, MAX_JOINT_VELOCITY_RAD_S,
            f"peak commanded arm velocity {self.peak_velocity:.3f} rad/s reached the "
            f"{MAX_JOINT_VELOCITY_RAD_S} rad/s slew clamp; joint_limits.yaml is not binding and "
            "the bridge is silently stretching the trajectory",
        )

    # 6. The waist test proper: MoveIt's own kinematics against TF's, with the torso turned.
    def test_07_moveit_places_the_hands_where_tf_does_with_the_waist_turned(self):
        """The pelvis-rooted model and the torso-rooted groups have to agree about the waist.

        MoveIt roots its model at pelvis and its arm groups at torso_link, with three waist
        joints in between that nothing here commands. If it did not fold the live waist state
        into that transform, its idea of where a hand is would be wrong by roughly the arm
        reach times sin(waist), about 10 cm at this angle. robot_state_publisher computes the
        same transform through an entirely separate KDL path, so agreement is the check.
        """
        self._spin(2.0)
        state = JointState()
        for name, position in self.joint_state.items():
            state.name.append(name)
            state.position.append(position)

        for link in ("left_hand_palm_link", "right_hand_palm_link"):
            request = GetPositionFK.Request()
            request.header.frame_id = "pelvis"
            request.fk_link_names = [link]
            request.robot_state.joint_state = state

            future = self.fk_client.call_async(request)
            self._spin_until(future.done, 20.0, f"/compute_fk never answered for {link}")
            response = future.result()
            self.assertEqual(
                response.error_code.val, 1, f"/compute_fk failed for {link}"
            )
            self.assertEqual(len(response.pose_stamped), 1)
            moveit_pose = response.pose_stamped[0].pose.position

            transform = self.tf_buffer.lookup_transform(
                "pelvis", link, rclpy.time.Time()
            ).transform.translation

            distance = math.dist(
                (moveit_pose.x, moveit_pose.y, moveit_pose.z),
                (transform.x, transform.y, transform.z),
            )
            self.assertLess(
                distance, 0.005,
                f"MoveIt puts {link} {distance * 100:.1f} cm from where TF does, with the waist "
                f"at {self.joint_state['waist_yaw_joint']:.3f} rad. The arm groups are rooted at "
                "torso_link and the model at pelvis; this is the waist transform disagreeing.",
            )

    # 7. The gripper, through the same plan-and-execute path the arm uses.
    def test_08_the_hand_closes_and_opens_through_moveit(self):
        """A Dex3 is seven joints, so MoveIt drives it as a group, not as a GripperCommand.

        The whole chain is under test here and only the last hop differs on hardware:
        move_group plans, left_hand_controller executes, G1Dex3System publishes HandCmd, and
        the fingers that come back on /joint_states are the ones the simulator actually moved.
        """
        before = {name: self.joint_state[name] for name in LEFT_HAND}

        result = self._send_move_goal(self._absolute_joint_goal("left_hand", LEFT_HAND_CLOSED))
        self.assertIsNotNone(result, "left_hand closed goal was rejected")
        self.assertEqual(
            result.error_code.val, 1,
            f"closing the left hand failed with error_code {result.error_code.val}",
        )
        self._spin(2.0)

        for name, target in LEFT_HAND_CLOSED.items():
            self.assertAlmostEqual(
                self.joint_state[name], target, delta=0.05,
                msg=f"{name} is at {self.joint_state[name]:.3f}, commanded {target:.3f}",
            )
        moved = max(abs(self.joint_state[n] - before[n]) for n in LEFT_HAND)
        self.assertGreater(moved, 0.1, "no finger moved; the grasp was a no-op")

        # Back to open, so the hand is left where the rest of the suite found it.
        result = self._send_move_goal(
            self._absolute_joint_goal("left_hand", dict.fromkeys(LEFT_HAND, 0.0))
        )
        self.assertIsNotNone(result, "left_hand open goal was rejected")
        self.assertEqual(result.error_code.val, 1, "opening the left hand failed")

    # 8. Release, so the suite leaves the channel as it found it.
    def test_09_the_arm_releases_cleanly(self):
        self._run_ros2_run("deactivate_arm")
        self._spin(3.0)
