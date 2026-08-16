"""Headless sim gate: MoveIt on the rt/lowcmd stack, with the balance policy running.

The lowcmd counterpart to test_moveit_plan_execute, and it exists for the property neither the
walk gate nor the arm_sdk arm gate can see: two controllers writing the same component every
tick, one balancing the robot on 14 joints and one executing a MoveIt trajectory on 14 others.
The pelvis is NOT pinned, so if acquiring the arms or moving them disturbed the policy the robot
would simply fall, and every assertion after that point would fail.

Also covers the ownership invariant that makes the split safe: the component leaves any
unclaimed joint unpowered, so the arm freeze and the trajectory controller have to trade places
in a single switch, never both out at once.

No hand assertions here, unlike the arm_sdk suite. G1Dex3System reaches the hands as ROS topics,
which only ever matched the simulator because ROS-on-CycloneDDS mangles /dex3/left/state to the
same DDS name the SDK publishes; this stack runs fastrtps, so nothing publishes it and the
component cannot activate. Moving that transport belongs to the middleware unification, not
here. Finger *state* still arrives, because joint_state_broadcaster reads configured components
and MoveIt refuses to plan without every modelled joint.
"""

import os
import subprocess
import time
import unittest

# Set before rclpy initialises: the lowcmd stack runs on fastrtps, because the component owns
# the robot wire through unitree_sdk2's own CycloneDDS and ROS must not load a second one.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

# Longer than the arm_sdk gate's: the policy has to bring the robot to a settled stand before
# anything is asked of the arms, and move_group starts alongside.
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

# 12 legs + 3 waist + 14 arms + 14 hand joints. The fingers count even though the hands cannot
# be driven here: move_group will not plan until every joint it models has a state.
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
                "control_stack": "lowcmd",
                # The whole point: the policy is holding the robot up while the arms move.
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
        cls.controllers = cls.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _joint_cb(cls, msg):
        for name, position in zip(msg.name, msg.position, strict=False):
            cls.joint_state[name] = position

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
            ["ros2", "run", "g1_bringup", executable, "--stack", "lowcmd"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        deadline = time.monotonic() + timeout_s
        while proc.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        stdout, stderr = proc.communicate(timeout=10.0)
        self.assertEqual(proc.returncode, 0, f"{executable} failed:\n{stdout}\n{stderr}")

    def _joint_goal(self, group, offsets):
        """`offsets` maps joint name to a delta from where that joint is now."""
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.start_state = RobotState()
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        for name, offset in offsets.items():
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = self.joint_state[name] + offset
            constraint.tolerance_above = 0.02
            constraint.tolerance_below = 0.02
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints = [constraints]
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

    def test_07_releasing_hands_the_arms_back_to_the_freeze(self):
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

    def test_08_the_policy_never_diverged(self):
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
