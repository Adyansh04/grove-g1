"""Headless sim integration test: the full arm-command loop through the bridge.

Ordered acquire (reusing activate_arm's own entry point), then: /arm_sdk has
no publisher before activation; the blend weight ramps 0 -> 1 monotonically,
starting well below 1.0 and taking a meaningful fraction of blend_ramp_up_s
(never snapping) to reach it; once held, /arm_sdk's arm targets track
measured /joint_states; a step-like FollowJointTrajectory goal converges
through the bridge with the observed velocity saturating at (never above)
the slew clamp; deactivate_arm ramps the weight back down to ~0 within
blend_ramp_down_s, /arm_sdk then goes silent, and the component ends
inactive; reactivating brings it back up so the final check has something
to trip: a second /arm_sdk publisher appearing mid-active triggers the
component's advisory rogue-writer guard, and a post-guard quiet window
proves the resulting ramp-down actually finished, not just started. Run via
`colcon test --packages-select g1_bringup`.
"""

import math
import os
import subprocess
import threading
import time
import unittest

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListHardwareComponents
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unitree_go.msg import SportModeState
from unitree_hg.msg import LowCmd, LowState

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
# blend_ramp_up_s = blend_ramp_down_s = 2.0 s, max_joint_velocity_rad_s = 1.0 rad/s.
BLEND_RAMP_UP_S = 2.0
BLEND_RAMP_DOWN_S = 2.0
MAX_JOINT_VELOCITY_RAD_S = 1.0

# Displacement for the slew-clamp trajectory step below and its
# time_from_start: 0.2 rad / 0.1 s implies ~2 rad/s, comfortably past the
# 1.0 rad/s clamp, so the clamp -- not the trajectory shape -- is
# unambiguously what bounds the observed velocity. JTC's path/goal
# tolerances (controllers.yaml) leave position unchecked, so this
# aggressive a time_from_start can't abort the goal.
TRAJECTORY_STEP_DISPLACEMENT_RAD = 0.2
TRAJECTORY_STEP_TIME_FROM_START_S = 0.1

ARM_SDK_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# For the sim's own bare-DDS topics (/sportmodestate, /lowstate): best-effort,
# matching how unitree_mujoco publishes them.
SIM_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Pelvis-pin bounds (sim.launch.py's pin_pelvis welds the floating base upright
# at its spawn pose). Measured pinned behaviour: z steady at 0.7925, tilt < 1
# deg, xy drift ~1 cm. A collapse instead drops the pelvis to z ~= 0.48 with a
# ~60 deg tilt, so these bounds separate the two decisively while tolerating the
# weld's small constraint compliance.
PELVIS_SPAWN_Z = 0.793
PELVIS_MIN_Z = 0.70
PELVIS_MAX_TILT_DEG = 15.0
PELVIS_MAX_XY_M = 0.15

# Deeper subscriber-side history than the publisher's own KEEP_LAST(1) --
# history depth is a subscriber-local setting, independent of what any
# publisher declares. Used where a rogue publisher's own samples could
# otherwise race the component's terminal weight-0 publish out of a
# shallow local queue between spin_once() calls.
ARM_SDK_DEEP_HISTORY_QOS = QoSProfile(
    depth=50,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        # pin_pelvis:=true is explicit now that sim.launch.py defaults it false: this suite
        # predates the walking policy and asserts against a welded, stiff-held robot, so
        # pinning keeps it deterministic and independent of policy regressions. The
        # unwelded, policy-driven path has its own suites (test_walk_*).
        launch_arguments={"pin_pelvis": "true"}.items(),
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

        # Pelvis-pin monitor, on its OWN node spun by a background thread rather
        # than sharing cls.node. It subscribes the sim's ~900 Hz /sportmodestate
        # (position) and /lowstate (orientation); if those callbacks ran on
        # cls.node they would compete with the main test's rclpy.spin_once()
        # loop, which processes only one ready callback per call, and starve the
        # rogue-guard step's shallow-history /arm_sdk captures of the terminal
        # weight-0 sample (observed directly). A dedicated executor thread keeps
        # them fully decoupled while still accumulating the pelvis's worst-case
        # deviation across the whole bring-up -> ... -> guard sequence, which
        # test_pelvis_stayed_pinned_through_sequence (sorted after it) asserts.
        cls.pelvis = {"min_z": float("inf"), "max_tilt_deg": 0.0, "max_xy": 0.0, "pos_n": 0, "tilt_n": 0}
        cls._pelvis_node = Node("test_arm_command_pelvis_monitor")
        cls._pelvis_node.create_subscription(
            SportModeState, "/sportmodestate", cls._pelvis_pos_cb, SIM_STATE_QOS
        )
        cls._pelvis_node.create_subscription(
            LowState, "/lowstate", cls._pelvis_tilt_cb, SIM_STATE_QOS
        )
        cls._pelvis_exec = SingleThreadedExecutor()
        cls._pelvis_exec.add_node(cls._pelvis_node)
        cls._pelvis_stop = threading.Event()
        cls._pelvis_thread = threading.Thread(target=cls._spin_pelvis_monitor, daemon=True)
        cls._pelvis_thread.start()

    @classmethod
    def _spin_pelvis_monitor(cls):
        while not cls._pelvis_stop.is_set():
            cls._pelvis_exec.spin_once(timeout_sec=0.1)

    @classmethod
    def _stop_pelvis_monitor(cls):
        if cls._pelvis_stop.is_set():
            return
        cls._pelvis_stop.set()
        cls._pelvis_thread.join(timeout=5.0)
        cls._pelvis_exec.remove_node(cls._pelvis_node)
        cls._pelvis_node.destroy_node()

    @classmethod
    def tearDownClass(cls):
        cls._stop_pelvis_monitor()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _pelvis_pos_cb(cls, msg):
        x, y, z = msg.position[0], msg.position[1], msg.position[2]
        cls.pelvis["min_z"] = min(cls.pelvis["min_z"], z)
        cls.pelvis["max_xy"] = max(cls.pelvis["max_xy"], math.hypot(x, y))
        cls.pelvis["pos_n"] += 1

    @classmethod
    def _pelvis_tilt_cb(cls, msg):
        # Tilt of the base frame from upright, from the IMU quaternion's scalar
        # part: angle = 2 * acos(|w|).
        w = max(-1.0, min(1.0, abs(msg.imu_state.quaternion[0])))
        cls.pelvis["max_tilt_deg"] = max(cls.pelvis["max_tilt_deg"], 2.0 * math.degrees(math.acos(w)))
        cls.pelvis["tilt_n"] += 1

    def _spin_for(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

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

    def _run_ros2_run(self, executable, timeout_s=30.0):
        """Runs `ros2 run g1_bringup <executable>` to completion while
        continuing to spin this test's own node. deactivate_arm's service
        call in particular blocks for the full ramp-down duration -- a
        plain blocking subprocess.run() here would starve any
        concurrently-registered /arm_sdk subscription of exactly the
        samples published during that window (confirmed directly: the
        first version of this test captured nothing but the pre-deactivate
        steady state).
        """
        proc = subprocess.Popen(
            ["ros2", "run", "g1_bringup", executable],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + timeout_s
        while proc.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        try:
            stdout, stderr = proc.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            self.fail(f"{executable} timed out after {timeout_s} s")
        self.assertEqual(
            proc.returncode,
            0,
            f"{executable} failed:\nstdout={stdout}\nstderr={stderr}",
        )

    def _hardware_component_state_label(self):
        client = self.node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), "service not available")
        future = client.call_async(ListHardwareComponents.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result()
        self.assertIsNotNone(result)
        matches = [c for c in result.component if c.name == "G1ArmSdkSystem"]
        self.assertEqual(len(matches), 1, "G1ArmSdkSystem not listed exactly once")
        return matches[0].state.label

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
        self._spin_for(2.0)
        self.assertEqual(
            len(weight_samples), 0, "/arm_sdk already publishing before activation"
        )

        # 2. Subscription is already in place, so the ramp's very first tick
        # is captured; now trigger activation via the same entry point the
        # operating procedure documents.
        self._run_ros2_run("activate_arm")

        # Keep collecting through the ramp plus margin.
        self._spin_for(BLEND_RAMP_UP_S + 2.0)
        self.node.destroy_subscription(arm_sdk_sub)

        self.assertGreater(len(weight_samples), 0, "no /arm_sdk samples observed after activation")

        # 3. Weight ramps monotonically 0 -> 1, starting well below 1.0
        # (catches a snap-to-1.0-on-activate regression that a
        # completion-only deadline check would miss) and taking at least a
        # meaningful fraction of blend_ramp_up_s to get there.
        weights = [w for _, w in weight_samples]
        self.assertLess(
            weights[0], 0.9, "weight already near 1.0 on the very first sample -- may have snapped"
        )
        previous = weights[0]
        for w in weights[1:]:
            self.assertGreaterEqual(w, previous - 1e-6, "weight decreased during ramp-up")
            previous = w
        self.assertAlmostEqual(weights[-1], 1.0, delta=1e-3)

        t0 = weight_samples[0][0]
        reached_one_at = next(t for t, w in weight_samples if w >= 1.0 - 1e-3)
        self.assertGreaterEqual(
            reached_one_at - t0,
            1.0,
            "weight reached 1.0 too quickly for a 2 s ramp -- may have snapped",
        )
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

        # 5. Step-like FollowJointTrajectory goal on one joint, full
        # 14-joint list (allow_partial_joints_goal is false): the implied
        # velocity (displacement / time_from_start) vastly exceeds the
        # slew clamp, so the clamp -- not the trajectory's own shape -- is
        # what the observed velocity saturates against.
        client = ActionClient(
            self.node, FollowJointTrajectory, "/arm_trajectory_controller/follow_joint_trajectory"
        )
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))

        start_positions = {name: measured[name][0] for name in JOINTS}
        target_positions = dict(start_positions)
        target_positions[TARGET_JOINT] = (
            start_positions[TARGET_JOINT] + TRAJECTORY_STEP_DISPLACEMENT_RAD
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [target_positions[name] for name in JOINTS]
        point.time_from_start = Duration(
            sec=0, nanosec=int(TRAJECTORY_STEP_TIME_FROM_START_S * 1e9)
        )
        goal.trajectory.points = [point]

        watch_state = {"max_velocity": 0.0, "position": None}

        def _joint_states_watch_cb(msg):
            if TARGET_JOINT in msg.name:
                idx = msg.name.index(TARGET_JOINT)
                watch_state["max_velocity"] = max(watch_state["max_velocity"], abs(msg.velocity[idx]))
                watch_state["position"] = msg.position[idx]

        watch_sub = self.node.create_subscription(
            JointState, "/joint_states", _joint_states_watch_cb, 10
        )

        goal_sent_at = time.monotonic()
        send_goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)
        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted, "trajectory goal was not accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
        self.assertIsNotNone(result_future.result())

        # JTC's own action-success signal fires once the nominal trajectory
        # time elapses without violating a configured *position* tolerance
        # -- none is set (controllers.yaml leaves goal/path tolerances at
        # their defaults) -- so it does not itself wait for the physically
        # slew-limited arm to catch up (confirmed directly: the action
        # completed within ~0.1 s, the trajectory's time_from_start, well
        # before the ~0.2 s the slew clamp implies). Poll /joint_states
        # directly instead of trusting that signal's timing, bounded
        # comfortably above the slew-limited timescale.
        convergence_tolerance = 0.02
        convergence_deadline = (
            time.monotonic() + TRAJECTORY_STEP_DISPLACEMENT_RAD / MAX_JOINT_VELOCITY_RAD_S + 2.0
        )
        converged_at = None
        while time.monotonic() < convergence_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            position = watch_state["position"]
            if (
                position is not None
                and abs(position - target_positions[TARGET_JOINT]) <= convergence_tolerance
            ):
                converged_at = time.monotonic()
                break

        self.node.destroy_subscription(watch_sub)

        self.assertIsNotNone(
            converged_at,
            "trajectory did not converge to target through the bridge within the "
            "slew-limited timescale",
        )

        # The clamp saturates near, but never far above, max_joint_velocity_rad_s
        # -- both bounds matter: the upper one proves the clamp is respected
        # (margin covers the PD-controlled sim's own small transient
        # overshoot chasing a ramping target, confirmed directly to land
        # a few percent over the kinematic clamp -- not the ~30%+ slack a
        # vacuous, never-saturating test would need), the lower one proves
        # this step-like goal actually engaged it (rather than
        # coincidentally staying under it regardless).
        self.assertLessEqual(
            watch_state["max_velocity"],
            MAX_JOINT_VELOCITY_RAD_S + 0.2,
            "observed joint velocity exceeded the slew clamp margin",
        )
        self.assertGreater(
            watch_state["max_velocity"],
            MAX_JOINT_VELOCITY_RAD_S * 0.5,
            "observed joint velocity never approached the slew clamp -- the step-like goal "
            "should have saturated it",
        )
        # converged_at is only ever set inside the bounded poll above, so
        # its mere presence already proves convergence landed within
        # displacement / max_joint_velocity_rad_s + margin of goal_sent_at
        # -- the slew-limited timescale, not JTC's much longer goal_time
        # budget.
        self.assertGreater(converged_at, goal_sent_at)

        # 6. Clean-stop coverage: deactivate_arm (controller-then-component,
        # see its own docstring) ramps the weight monotonically to ~0
        # within blend_ramp_down_s + margin, /arm_sdk then goes silent, and
        # the component itself ends inactive -- the documented safe-stop
        # path this package's README promises. Placed before the
        # rogue-guard step below because the guard leaves mode_ kInactive,
        # which would make a later deactivate a no-op.
        deactivate_samples = []

        def _deactivate_cb(msg):
            deactivate_samples.append((time.monotonic(), msg.motor_cmd[29].q))

        deactivate_sub = self.node.create_subscription(
            LowCmd, "/arm_sdk", _deactivate_cb, ARM_SDK_QOS
        )
        # Brief discovery window: the component has been publishing steadily
        # at weight 1.0 for a while now, so this just covers subscription
        # matching, not anything about the ramp itself.
        self._spin_for(0.3)

        self._run_ros2_run("deactivate_arm")

        self.node.destroy_subscription(deactivate_sub)

        self.assertGreater(
            len(deactivate_samples), 0, "no /arm_sdk samples observed during deactivate"
        )
        deactivate_weights = [w for _, w in deactivate_samples]
        previous = deactivate_weights[0]
        for w in deactivate_weights[1:]:
            self.assertLessEqual(w, previous + 1e-6, "weight increased during ramp-down")
            previous = w
        self.assertAlmostEqual(deactivate_weights[-1], 0.0, delta=1e-3)

        d_t0 = deactivate_samples[0][0]
        reached_zero_at = next(t for t, w in deactivate_samples if w <= 1e-3)
        self.assertLessEqual(
            reached_zero_at - d_t0,
            BLEND_RAMP_DOWN_S + 1.0,
            "weight took longer than blend_ramp_down_s (+ margin) to reach 0.0 on deactivate",
        )

        # /arm_sdk goes silent afterward.
        silence_samples = []
        silence_sub = self.node.create_subscription(
            LowCmd, "/arm_sdk", lambda msg: silence_samples.append(msg), ARM_SDK_QOS
        )
        self._spin_for(1.0)
        self.node.destroy_subscription(silence_sub)
        self.assertEqual(
            len(silence_samples), 0, "/arm_sdk kept publishing after deactivate_arm"
        )

        self.assertEqual(
            self._hardware_component_state_label(),
            "inactive",
            "G1ArmSdkSystem did not end inactive after deactivate_arm",
        )

        # 7. Reactivate so the rogue-publisher guard below has an active
        # component to trip -- step 6 left it inactive. Let the weight ramp
        # fully back up first so a later reading of "weight dropped" in the
        # guard step unambiguously means the guard tripped, not that the
        # reactivation ramp was still in progress.
        self._run_ros2_run("activate_arm")
        self._spin_for(BLEND_RAMP_UP_S + 2.0)

        # 8. Rogue-publisher guard: a second /arm_sdk publisher appearing
        # while active must trigger the component's advisory ramp-down.
        # Subscribe *before* the rogue publisher exists and keep the same
        # subscription through publishing and both settle windows below, so
        # the ramp-down (which can complete in as little as
        # emergency_ramp_down_s = 0.5 s -- comfortably inside even a short
        # rogue-publish window) is never missed between two separately
        # constructed subscriptions. Our own rogue messages always carry
        # weight 1.0, so they can only ever pull the observed *minimum*
        # weight up, never mask a real decrease -- interleaving with them is
        # harmless for a min() check. Deeper subscriber history than the
        # rogue publisher's own KEEP_LAST(1) so its samples can't race the
        # component's terminal weight-0 publish out of the queue.
        guard_samples = []

        def _guard_cb(msg):
            guard_samples.append((time.monotonic(), msg.motor_cmd[29].q))

        guard_sub = self.node.create_subscription(
            LowCmd, "/arm_sdk", _guard_cb, ARM_SDK_DEEP_HISTORY_QOS
        )

        rogue_pub = self.node.create_publisher(LowCmd, "/arm_sdk", ARM_SDK_QOS)
        rogue_msg = LowCmd()
        rogue_msg.motor_cmd[29].q = 1.0

        # Publish only long enough for the ~1 Hz advisory check to notice a
        # second publisher, then stop -- ramp-down, once triggered, runs to
        # completion on its own regardless of the rogue publisher's
        # continued existence.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rogue_pub.publish(rogue_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_publisher(rogue_pub)

        # Settle window comfortably past emergency_ramp_down_s so the ramp
        # actually completes, not just starts -- note the ~1 Hz advisory
        # check can trip anywhere in the 1.5 s publish window above, so the
        # ramp (<= 0.5 s) may well finish *before* the rogue publisher is
        # even stopped, not after.
        self._spin_for(2.0)

        weights_only = [w for _, w in guard_samples]
        self.assertGreater(len(guard_samples), 0)
        self.assertLess(
            min(weights_only),
            0.9,
            "component did not ramp down after a second /arm_sdk publisher appeared",
        )

        # Terminal weight: the rogue publisher's samples are always exactly
        # weight 1.0, and DDS gives no cross-writer delivery-ordering
        # guarantee (a rogue sample still in flight when its publisher was
        # destroyed above could be delivered interleaved with later,
        # already-converged component samples) -- so anchoring this check
        # to receive order or wall-clock time around when the rogue
        # publisher stopped is unreliable either way (confirmed directly).
        # Filtering out exact weight-1.0 samples instead sidesteps both: it
        # discards every rogue sample (and the component's own pre-ramp
        # steady state, harmless since only the tail matters here) and
        # leaves the component's real in-ramp and post-ramp values, whose
        # last entry must be the settled terminal weight.
        settling_samples = [(t, w) for t, w in guard_samples if w < 1.0 - 1e-6]
        self.assertGreater(
            len(settling_samples),
            0,
            "no component-originated (non-1.0) /arm_sdk samples observed during/after the ramp",
        )
        self.assertAlmostEqual(
            settling_samples[-1][1],
            0.0,
            delta=1e-2,
            msg="component-originated /arm_sdk weight did not settle near 0 after the guard "
            "tripped",
        )

        # Post-guard quiet window, same subscription: proves the ramp
        # actually finished and the component went kInactive (a guard
        # stalling partway would still show fresh samples here), not merely
        # that it started ramping down.
        quiet_window_start_len = len(guard_samples)
        self._spin_for(1.0)
        self.node.destroy_subscription(guard_sub)
        self.assertEqual(
            len(guard_samples),
            quiet_window_start_len,
            "component kept publishing /arm_sdk after the rogue guard should have gone quiet",
        )

    def test_pelvis_stayed_pinned_through_sequence(self):
        # Sorts after test_full_arm_command_sequence (alphabetical method order),
        # so by now the background pelvis monitor has accumulated across the
        # entire sequence. Stop it first for a stable snapshot. This is the
        # regression guard for the sim-only pelvis weld (sim.launch.py's
        # pin_pelvis): without it the floating-base G1 topples on spawn in
        # unitree_mujoco (no balance controller), and the arm-side checks above
        # would still pass on a fully collapsed robot -- exactly the blind spot
        # this closes. Fails loudly if the pin is off or not holding.
        self._stop_pelvis_monitor()
        self.assertGreater(self.pelvis["pos_n"], 0, "no /sportmodestate pelvis samples collected")
        self.assertGreater(self.pelvis["tilt_n"], 0, "no /lowstate orientation samples collected")
        self.assertGreater(
            self.pelvis["min_z"],
            PELVIS_MIN_Z,
            f"pelvis fell to z={self.pelvis['min_z']:.3f} m (spawn {PELVIS_SPAWN_Z}) during the "
            "sequence -- pelvis weld pin is not holding height",
        )
        self.assertLess(
            self.pelvis["max_tilt_deg"],
            PELVIS_MAX_TILT_DEG,
            f"pelvis tilted {self.pelvis['max_tilt_deg']:.1f} deg from upright during the "
            "sequence -- pelvis weld pin is not holding orientation",
        )
        self.assertLess(
            self.pelvis["max_xy"],
            PELVIS_MAX_XY_M,
            f"pelvis translated {self.pelvis['max_xy']:.3f} m in xy during the sequence -- "
            "pelvis weld pin is not holding position",
        )
