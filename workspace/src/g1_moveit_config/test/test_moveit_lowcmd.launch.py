"""Headless sim gate: MoveIt on the rt/lowcmd stack, with the balance policy running.

The counterpart to test_moveit_plan_execute, and it exists for the property the walk gate cannot
see: two controllers writing the same component every tick, one balancing the robot on 14 joints
and one executing a MoveIt trajectory on 14 others.
The pelvis is NOT pinned, so if acquiring the arms or moving them disturbed the policy the robot
would simply fall, and every assertion after that point would fail.

Also covers the ownership invariant that makes the split safe: the component leaves any
unclaimed joint unpowered, so the arm freeze and the trajectory controller have to trade places
in a single switch, never both out at once.

The hands are asserted here too, on the same acquire: three components now share one process and
one ChannelFactory, so "the hand activated and its fingers moved" is the only proof that the
body component's SDK init did not shut the other two out.
"""

import os
import re
import subprocess
import time
import unittest

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

# Generous on purpose: the policy has to bring the robot to a settled stand before anything is
# asked of the arms, and move_group starts alongside.
SIM_SETTLE_S = 14.0

# Tilt, never the quaternion's w: yawing drives w down while the robot stands perfectly
# straight. This is the world z-component of the body z-axis, 1.0 upright and 0.0 on its side.
MIN_UPRIGHT_Z = 0.64

LEFT_ARM = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]
RIGHT_ARM = [name.replace("left_", "right_") for name in LEFT_ARM]
BOTH_ARMS = LEFT_ARM + RIGHT_ARM

# The nudge test_06 plans. Shoulder pitch and elbow only, and mirrored in sign, because both
# arms rest hanging straight down: the same offset on every joint would take one shoulder roll
# outward and the other straight into the torso. These two lift the arms slightly forward, which
# is unambiguously away from the body on both sides.
ARM_NUDGE = {f"{side}_shoulder_pitch_joint": -0.20 for side in ("left", "right")} | {
    f"{side}_elbow_joint": 0.20 for side in ("left", "right")
}

# 12 legs + 3 waist + 14 arms + 14 hand joints. move_group will not plan until every joint it
# models has a state.
EXPECTED_JOINT_COUNT = 43

LEFT_HAND = [
    f"left_hand_{suffix}_joint"
    for suffix in ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
]
# The `closed` group state from g1.srdf, restated rather than read out of it: a test that took
# its expectation from the file under test would pass no matter what that file said.
LEFT_HAND_CLOSED = dict(
    zip(LEFT_HAND, [-0.30, -0.50, 1.20, -1.20, -1.40, -1.20, -1.40], strict=True)
)


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
                # The policy is holding the robot up while the arms move.
                "pin_pelvis": "false",
                "headless": "true",
                "sensors": "false",
            }.items(),
        ),
        TimerAction(period=SIM_SETTLE_S, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestMoveItLowCmd(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_moveit_lowcmd")
        cls.joint_state = {}
        cls.imu = []
        cls.node.create_subscription(JointState, "/joint_states", cls._joint_cb, 20)
        cls.node.create_subscription(
            Imu, "/imu_sensor_broadcaster/imu", lambda msg: cls.imu.append(msg), 10
        )
        cls.move_client = ActionClient(cls.node, MoveGroup, "/move_action")
        cls.execute_client = ActionClient(cls.node, ExecuteTrajectory, "/execute_trajectory")
        cls.limits = {}
        cls.node.create_subscription(
            String,
            "/robot_description",
            cls._description_cb,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        cls.controllers = cls.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        cls.components = cls.node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _joint_cb(cls, msg):
        for name, position in zip(msg.name, msg.position, strict=False):
            cls.joint_state[name] = position

    @classmethod
    def _description_cb(cls, msg):
        for name, lower, upper in re.findall(
            r'<joint name="([^"]+)" type="revolute">.*?'
            r'<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)"',
            msg.data,
            re.S,
        ):
            cls.limits[name] = (float(lower), float(upper))

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

    def _controller_states(self):
        self.assertTrue(
            self.controllers.wait_for_service(timeout_sec=30.0), "no list_controllers service"
        )
        future = self.controllers.call_async(ListControllers.Request())
        self._spin_until(future.done, 20.0, "list_controllers did not answer")
        return {c.name: c.state for c in future.result().controller}

    def _component_states(self):
        self.assertTrue(
            self.components.wait_for_service(timeout_sec=30.0),
            "no list_hardware_components service",
        )
        future = self.components.call_async(ListHardwareComponents.Request())
        self._spin_until(future.done, 20.0, "list_hardware_components did not answer")
        return {c.name: c.state.label for c in future.result().component}

    def _uprightness(self):
        # The subscriptions are made when the first test runs, not while the stack settles, so
        # the very first read has to wait for a sample rather than assume one arrived.
        self._spin_until(lambda: self.imu, 20.0, "no IMU messages on /imu_sensor_broadcaster/imu")
        orientation = self.imu[-1].orientation
        return 1.0 - (2.0 * ((orientation.x * orientation.x) + (orientation.y * orientation.y)))

    def _assert_still_standing(self, when):
        upright = self._uprightness()
        self.assertGreater(
            upright, MIN_UPRIGHT_Z, f"pelvis uprightness {upright:.3f} {when}: the robot fell"
        )

    def _run_bringup_script(self, executable, timeout_s=60.0):
        proc = subprocess.Popen(
            ["ros2", "run", "g1_bringup", executable],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        deadline = time.monotonic() + timeout_s
        while proc.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        stdout, stderr = proc.communicate(timeout=10.0)
        self.assertEqual(proc.returncode, 0, f"{executable} failed:\n{stdout}\n{stderr}")

    def _bounded_start_state(self, targets):
        """Every joint we know a position for, clamped into its limits."""
        state = RobotState()
        state.is_diff = False
        for name, position in self.joint_state.items():
            lower, upper = self.limits.get(name, (position, position))
            state.joint_state.name.append(name)
            state.joint_state.position.append(min(max(position, lower), upper))
        return state

    def _joint_goal(self, group, targets, absolute=False):
        """`targets` maps joint name to a delta from where it is now, or to an absolute
        position when `absolute`.
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        # Seeded from the measured state with this group's joints clamped into their URDF
        # limits, the same thing g1_manipulation's setStartStateInBounds does for the arm.
        # A Dex3 finger rests at exactly 0, which IS its limit, so the simulator settling it a
        # microradian past is enough for CheckStartStateBounds to abort the plan, and Jazzy
        # ships no adapter that clamps one back.
        goal.request.start_state = self._bounded_start_state(targets)

        constraints = Constraints()
        for name, target in targets.items():
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = target if absolute else self.joint_state[name] + target
            constraint.tolerance_above = 0.02
            constraint.tolerance_below = 0.02
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints = [constraints]
        return goal

    def _await_goal(self, client, goal, timeout_s, what):
        send = client.send_goal_async(goal)
        self._spin_until(send.done, 20.0, f"{what} never answered the goal request")
        handle = send.result()
        if handle is None or not handle.accepted:
            return None
        result = handle.get_result_async()
        self._spin_until(result.done, timeout_s, f"{what} never returned a result")
        return result.result().result

    def _send_move_goal(self, goal, timeout_s=90.0):
        """Plans, then executes what was planned.

        Two steps rather than one combined request, because move_group DISCARDS the start
        state of a plan-and-execute goal ("Ignoring the state supplied as start state") and
        re-reads the current one. Only the plan-only form honours it, and this test needs it
        honoured: a Dex3 finger rests at exactly 0, which is its own limit, so the simulator
        settling it a microradian past aborts the plan on CheckStartStateBounds. Planning and
        executing separately is what MoveGroupInterface does, so this matches g1_manipulation.
        """
        goal.planning_options.plan_only = True
        planned = self._await_goal(self.move_client, goal, timeout_s, "move_group")
        if planned is None or planned.error_code.val != 1:
            return planned

        execute = ExecuteTrajectory.Goal()
        execute.trajectory = planned.planned_trajectory
        executed = self._await_goal(
            self.execute_client, execute, timeout_s, "execute_trajectory"
        )
        if executed is not None:
            planned.error_code = executed.error_code
        return planned

    def test_01_every_joint_has_a_state(self):
        self._spin_until(
            lambda: len(self.joint_state) >= EXPECTED_JOINT_COUNT,
            60.0,
            f"only {len(self.joint_state)} joints on /joint_states; MoveIt will not plan until "
            f"every active joint has a state (expected {EXPECTED_JOINT_COUNT})",
        )

    def test_02_the_robot_is_standing_on_the_policy(self):
        """Everything below is only meaningful while the policy is holding the robot up."""
        # The arm freeze ramps to its rest pose at 0.5 rad/s from wherever the model dropped
        # the arms, so give it time to arrive before anything asks MoveIt to plan.
        self._spin(6.0)
        self._assert_still_standing("before anything touched the arms")

    def test_03_every_body_motor_is_claimed_before_the_arm_is_acquired(self):
        """29 motors, and the component leaves any it sees unclaimed unpowered."""
        states = self._controller_states()
        for name in (
            "waist_freeze_controller",
            "arm_freeze_controller",
            "locomotion_safety_controller",
            "agile_controller",
        ):
            self.assertEqual(states.get(name), "active", f"{name} is {states.get(name)}")
        self.assertEqual(
            states.get("arm_trajectory_controller"),
            "inactive",
            "the trajectory controller is active before anything acquired the arm",
        )

    def test_04_execution_is_refused_before_the_arm_is_acquired(self):
        left_only = {k: v for k, v in ARM_NUDGE.items() if k.startswith("left_")}
        result = self._send_move_goal(self._joint_goal("left_arm", left_only))
        self.assertIsNotNone(result, "move_group rejected the goal outright")
        # 1 is SUCCESS. Anything else is the JTC refusing while it is still inactive, which is
        # the acquire step doing its job.
        self.assertNotEqual(
            result.error_code.val,
            1,
            "MoveIt executed a trajectory before the arm was acquired; the acquire step is the "
            "whole safety model for these joints",
        )

    def test_05_acquiring_trades_the_freeze_for_the_controller(self):
        self._run_bringup_script("activate_arm")
        self._spin(3.0)

        states = self._controller_states()
        self.assertEqual(states.get("arm_trajectory_controller"), "active")
        self.assertNotEqual(
            states.get("arm_freeze_controller"),
            "active",
            "both arm controllers are active; they claim the same joints, so one switch failed",
        )
        # waist_yaw is deliberately not part of the trade, so acquiring must not disturb it.
        self.assertEqual(states.get("waist_freeze_controller"), "active")

        # The hands come up in the same acquire, each on its own component and its own SDK
        # channel pair. A component stuck inactive here means it never saw HandState, which on
        # one shared ChannelFactory is how a domain or init-order fault presents.
        components = self._component_states()
        for name in ("G1Dex3SystemLeft", "G1Dex3SystemRight"):
            self.assertEqual(components.get(name), "active", f"{name} did not activate")
        for name in ("left_hand_controller", "right_hand_controller"):
            self.assertEqual(states.get(name), "active", f"{name} did not activate")

        self._assert_still_standing("after acquiring the arms")

    def test_06_both_arms_move_while_the_policy_balances(self):
        before = {name: self.joint_state[name] for name in BOTH_ARMS}
        result = self._send_move_goal(self._joint_goal("both_arms", ARM_NUDGE))
        self.assertIsNotNone(result, "both_arms goal was rejected")
        self.assertEqual(
            result.error_code.val,
            1,
            f"both_arms plan+execute failed with error_code {result.error_code.val}",
        )
        self._spin(2.0)

        for side, joints in (("left", LEFT_ARM), ("right", RIGHT_ARM)):
            moved = max(abs(self.joint_state[n] - before[n]) for n in joints)
            self.assertGreater(moved, 0.02, f"{side} arm did not move during a both_arms plan")

        # The assertion this whole file exists for: the arms moved and the robot stayed up.
        self._assert_still_standing("after moving both arms")

    def test_07_the_hand_closes_and_opens_through_moveit(self):
        """A Dex3 is seven joints, so MoveIt drives it as a group, not as a GripperCommand.

        The fingers that come back on /joint_states are the ones the simulator actually moved,
        so this is the end-to-end proof that G1Dex3System's channel reaches the hand.
        """
        before = {name: self.joint_state[name] for name in LEFT_HAND}

        result = self._send_move_goal(
            self._joint_goal("left_hand", LEFT_HAND_CLOSED, absolute=True)
        )
        self.assertIsNotNone(result, "left_hand closed goal was rejected")
        self.assertEqual(
            result.error_code.val,
            1,
            f"closing the left hand failed with error_code {result.error_code.val}",
        )
        self._spin(2.0)

        for name, target in LEFT_HAND_CLOSED.items():
            self.assertAlmostEqual(
                self.joint_state[name],
                target,
                delta=0.05,
                msg=f"{name} is at {self.joint_state[name]:.3f}, commanded {target:.3f}",
            )
        moved = max(abs(self.joint_state[n] - before[n]) for n in LEFT_HAND)
        self.assertGreater(moved, 0.1, "no finger moved; the grasp was a no-op")

        # Back to open, so the release below finds the hand where the suite did.
        result = self._send_move_goal(
            self._joint_goal("left_hand", dict.fromkeys(LEFT_HAND, 0.0), absolute=True)
        )
        self.assertIsNotNone(result, "left_hand open goal was rejected")
        self.assertEqual(result.error_code.val, 1, "opening the left hand failed")
        self._assert_still_standing("after closing and opening the hand")

    def test_08_releasing_hands_the_arms_back_to_the_freeze(self):
        self._run_bringup_script("deactivate_arm")
        self._spin(3.0)

        states = self._controller_states()
        self.assertEqual(
            states.get("arm_freeze_controller"),
            "active",
            "the arms were released with nothing holding them; on this component that means "
            "unpowered, and they would drop",
        )
        self.assertNotEqual(states.get("arm_trajectory_controller"), "active")
        self._assert_still_standing("after releasing the arms")

    def test_09_the_policy_never_diverged(self):
        states = self._controller_states()
        self.assertEqual(
            states.get("locomotion_safety_controller"),
            "active",
            "the safety controller latched its emergency at some point during the run",
        )
        self.assertEqual(
            states.get("locomotion_freeze_controller"),
            "inactive",
            "the emergency freeze took over, so moving the arms disturbed the balance policy",
        )
