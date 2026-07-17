"""Headless sim integration test: the full arm-command loop through the bridge.

Ordered acquire (reusing activate_arm's own entry point), then: /arm_sdk has
no publisher before activation; the blend weight ramps 0 -> 1 monotonically
within blend_ramp_up_s of activation; once held, /arm_sdk's arm targets
track measured /joint_states; a small FollowJointTrajectory goal converges
through the bridge with the commanded slew respected; and a second /arm_sdk
publisher appearing mid-active triggers the component's advisory rogue-writer
guard (weight ramps back down). Run via `colcon test --packages-select
g1_bringup`.
"""

import os
import subprocess
import time
import unittest

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unitree_hg.msg import LowCmd

# See test_sim_bringup.launch.py's comment on this constant: kept below
# launch_testing's own hardcoded 15 s process-startup deadline.
SIM_SETTLE_S = 10.0

JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
TARGET_JOINT = "left_elbow_joint"
TARGET_JOINT_MOTOR_INDEX = 18  # see unitree_ros2's G1Arm7JointIndex; matches config/arm_sdk_params.yaml

# From g1_hardware_interface/config (via g1_description/config/arm_sdk_params.yaml):
# blend_ramp_up_s = 2.0 s, max_joint_velocity_rad_s = 1.0 rad/s.
BLEND_RAMP_UP_S = 2.0
MAX_JOINT_VELOCITY_RAD_S = 1.0

ARM_SDK_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        )
    )
    return LaunchDescription(
        [
            sim_launch,
            TimerAction(period=SIM_SETTLE_S, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


class TestArmCommand(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_arm_command")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _latest_joint_states(self, timeout_s=3.0):
        result = {}

        def _cb(msg):
            for name, position, velocity in zip(
                msg.name, msg.position, msg.velocity, strict=True
            ):
                result[name] = (position, velocity)

        sub = self.node.create_subscription(JointState, "/joint_states", _cb, 10)
        deadline = time.monotonic() + timeout_s
        while not result and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(sub)
        return result

    def test_full_arm_command_sequence(self):
        # 1. /arm_sdk carries no traffic before activation. G1ArmSdkSystem's
        # publisher *object* already exists at this point (created in
        # on_configure, which the hardware_components_initial_state:
        # inactive setup already ran at controller_manager startup) -- the
        # self-gated behaviour this asserts is that it publishes nothing
        # until active, not that the endpoint itself doesn't exist yet.
        weight_samples = []

        def _arm_sdk_cb(msg):
            weight_samples.append((time.monotonic(), msg.motor_cmd[29].q))

        arm_sdk_sub = self.node.create_subscription(
            LowCmd, "/arm_sdk", _arm_sdk_cb, ARM_SDK_QOS
        )
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(
            len(weight_samples), 0, "/arm_sdk already publishing before activation"
        )

        # 2. Subscription is already in place, so the ramp's very first tick
        # is captured; now trigger activation via the same entry point the
        # operating procedure documents.
        activate_result = subprocess.run(
            ["ros2", "run", "g1_bringup", "activate_arm"],
            timeout=30.0,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            activate_result.returncode,
            0,
            f"activate_arm failed:\nstdout={activate_result.stdout}\nstderr={activate_result.stderr}",
        )

        # Keep collecting through the ramp plus margin.
        deadline = time.monotonic() + BLEND_RAMP_UP_S + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(arm_sdk_sub)

        self.assertGreater(len(weight_samples), 0, "no /arm_sdk samples observed after activation")

        # 3. Weight ramps monotonically 0 -> 1, reaching 1 within
        # blend_ramp_up_s of the *first* sample (a small margin covers
        # activate_arm's own service-call overhead before the first tick).
        weights = [w for _, w in weight_samples]
        previous = weights[0]
        for w in weights[1:]:
            self.assertGreaterEqual(w, previous - 1e-6, "weight decreased during ramp-up")
            previous = w
        self.assertAlmostEqual(weights[-1], 1.0, delta=1e-3)

        t0 = weight_samples[0][0]
        reached_one_at = next(t for t, w in weight_samples if w >= 1.0 - 1e-3)
        self.assertLessEqual(
            reached_one_at - t0,
            BLEND_RAMP_UP_S + 1.0,
            "weight took longer than blend_ramp_up_s (+ margin) to reach 1.0",
        )

        # 4. Holding: /arm_sdk's commanded arm position tracks measured
        # /joint_states (the hardware component seeded its hold target from
        # the measured position on activation, and nothing has commanded a
        # trajectory yet).
        hold_sample = {}

        def _hold_cb(msg):
            hold_sample["q"] = msg.motor_cmd[TARGET_JOINT_MOTOR_INDEX].q

        hold_sub = self.node.create_subscription(LowCmd, "/arm_sdk", _hold_cb, ARM_SDK_QOS)
        deadline = time.monotonic() + 2.0
        while "q" not in hold_sample and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(hold_sub)
        self.assertIn("q", hold_sample)

        measured = self._latest_joint_states()
        self.assertIn(TARGET_JOINT, measured)
        measured_position = measured[TARGET_JOINT][0]
        self.assertAlmostEqual(hold_sample["q"], measured_position, delta=0.1)

        # 5. Small FollowJointTrajectory goal on one joint, full 14-joint
        # list (allow_partial_joints_goal is false) -- converges through the
        # bridge, and observed velocity respects the slew clamp.
        client = ActionClient(
            self.node, FollowJointTrajectory, "/arm_trajectory_controller/follow_joint_trajectory"
        )
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))

        start_positions = {name: measured[name][0] for name in JOINTS}
        target_positions = dict(start_positions)
        target_positions[TARGET_JOINT] = start_positions[TARGET_JOINT] + 0.2

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [target_positions[name] for name in JOINTS]
        point.time_from_start = Duration(sec=3)
        goal.trajectory.points = [point]

        max_observed_velocity = {"value": 0.0}

        def _velocity_watch_cb(msg):
            if TARGET_JOINT in msg.name:
                idx = msg.name.index(TARGET_JOINT)
                max_observed_velocity["value"] = max(
                    max_observed_velocity["value"], abs(msg.velocity[idx])
                )

        velocity_sub = self.node.create_subscription(
            JointState, "/joint_states", _velocity_watch_cb, 10
        )

        send_goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)
        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted, "trajectory goal was not accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
        self.assertIsNotNone(result_future.result())

        self.node.destroy_subscription(velocity_sub)

        final = self._latest_joint_states()
        self.assertIn(TARGET_JOINT, final)
        self.assertAlmostEqual(
            final[TARGET_JOINT][0],
            target_positions[TARGET_JOINT],
            delta=0.1,
            msg="trajectory did not converge to target through the bridge",
        )
        self.assertLessEqual(
            max_observed_velocity["value"],
            MAX_JOINT_VELOCITY_RAD_S + 0.3,
            "observed joint velocity exceeded the slew clamp margin",
        )

        # 6. Rogue-publisher guard: a second /arm_sdk publisher appearing
        # while active must trigger the component's advisory ramp-down.
        # Subscribe *before* the rogue publisher exists and keep the same
        # subscription through publishing and the settle window, so the
        # ramp-down (which can complete in as little as
        # emergency_ramp_down_s = 0.5 s -- comfortably inside even a short
        # rogue-publish window) is never missed between two separately
        # constructed subscriptions. Our own rogue messages always carry
        # weight 1.0, so they can only ever pull the observed *minimum*
        # weight up, never mask a real decrease -- interleaving with them
        # is harmless for a min() check.
        guard_weight_samples = []

        def _guard_cb(msg):
            guard_weight_samples.append(msg.motor_cmd[29].q)

        guard_sub = self.node.create_subscription(LowCmd, "/arm_sdk", _guard_cb, ARM_SDK_QOS)

        rogue_pub = self.node.create_publisher(LowCmd, "/arm_sdk", ARM_SDK_QOS)
        rogue_msg = LowCmd()
        rogue_msg.motor_cmd[29].q = 1.0

        # Publish only long enough for the ~1 Hz advisory check to notice a
        # second publisher, then stop -- ramp-down, once triggered, runs to
        # completion on its own thread regardless of the rogue publisher's
        # continued existence.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rogue_pub.publish(rogue_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_publisher(rogue_pub)

        # Keep watching through the settle window on the *same* subscription.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(guard_sub)

        self.assertGreater(len(guard_weight_samples), 0)
        self.assertLess(
            min(guard_weight_samples),
            0.9,
            "component did not ramp down after a second /arm_sdk publisher appeared",
        )
