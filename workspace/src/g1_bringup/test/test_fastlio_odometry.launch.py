"""FAST-LIO odometry against MuJoCo ground truth: precision standing, sanity walking.

The unit suites prove the adapter math; this proves the pipeline. The full stack comes up with
`odometry:=fast_lio` and the odom -> base_footprint the stack publishes is compared against
the exact pelvis position MuJoCo reports. A regression anywhere in the chain -- bridge
formats, QoS, FAST-LIO config, the latch, the frame composition -- shows up here.

Two phases with very different tolerances, deliberately. Standing is repeatable, so it gets a
tight bound: measured drift is ~2 cm, asserted at 0.35 m. Walking is looser, because the gait
itself is not repeatable -- the same command can produce a smooth walk or a stumbling one, and
a stumble degrades any LIO. scripts/lio_bench measures ~10 cm worst-case over 21 m on a clean
run; this asserts 1.0 m, which still catches the failure class seen during bring-up (kilometres,
when the extrinsic estimator was left on) with room for a bad gait.

`odom` is latched wherever FAST-LIO first produced a pose and the robot does not settle on a
repeatable heading, so the two frames are aligned by the headings measured at the first paired
sample, not by a best fit over the path: that reads lower where the estimate is bad and it is
easy to get the sign wrong.
"""

import math
import os
import time
import unittest

import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from g1_msgs.action import SetLocoMode
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from unitree_go.msg import SportModeState
from unitree_hg.msg import LowState

LATCH_TIMEOUT_S = 120.0
STAND_S = 12.0
WALK_S = 15.0
CMD_VX = 0.4

# Standing: measured ~2 cm of wander; anything near this bound means the estimator is sick.
MAX_STANDING_DRIFT_M = 0.35
# Walking: it has to have actually moved for the comparison to mean anything. A clean run is
# ~10 cm worst-case over 21 m (scripts/lio_bench); this leaves an order of magnitude for a bad
# gait and still catches divergence, which is metres and upwards.
MIN_TRUTH_PATH_M = 0.8
MAX_WALK_GAP_M = 1.0


def yaw_of(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "sensors": "true",
            "odometry": "fast_lio",
            "world": "navigation",
        }.items(),
    )
    return (
        LaunchDescription(
            [sim, TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()])]
        ),
        {},
    )


class FastLioOdometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("fastlio_sim_test")
        cls.truth = None
        cls.lio = None
        cls.truth_yaw = None
        cls.node.create_subscription(
            SportModeState, "/sportmodestate", cls._on_truth, qos_profile_sensor_data
        )
        # The true heading, which /sportmodestate does not carry: the simulator leaves its
        # imu_state zeroed.
        cls.node.create_subscription(
            LowState, "/lowstate", cls._on_low_state, qos_profile_sensor_data
        )
        cls.node.create_subscription(TFMessage, "/tf", cls._on_tf, 100)
        cls.cmd_pub = cls.node.create_publisher(Twist, "/g1_loco_bridge/cmd_vel", 1)

    @classmethod
    def _on_truth(cls, msg):
        cls.truth = (msg.position[0], msg.position[1])

    @classmethod
    def _on_low_state(cls, msg):
        q = msg.imu_state.quaternion  # Unitree order the quaternion w-first.
        cls.truth_yaw = yaw_of(q[1], q[2], q[3], q[0])

    @classmethod
    def _on_tf(cls, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == "odom" and tf.child_frame_id == "base_footprint":
                r = tf.transform.rotation
                cls.lio = (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    yaw_of(r.x, r.y, r.z, r.w),
                )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_s, why):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(why)

    def _set_mode(self, fsm_id):
        client = ActionClient(self.node, SetLocoMode, "/g1_loco_bridge/set_mode")
        self.assertTrue(client.wait_for_server(timeout_sec=30.0), "set_mode server missing")
        goal = SetLocoMode.Goal()
        goal.fsm_id = fsm_id
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        handle = future.result()
        self.assertIsNotNone(handle, f"SetLocoMode({fsm_id}) never reached the server")
        self.assertTrue(handle.accepted, f"SetLocoMode({fsm_id}) rejected")
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=30.0)
        client.destroy()

    def _collect(self, duration_s, command=None):
        samples = []
        end = time.time() + duration_s
        while time.time() < end:
            if command is not None:
                self.cmd_pub.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if None not in (self.truth, self.lio, self.truth_yaw):
                samples.append((self.truth, self.lio, self.truth_yaw))
        return samples

    @staticmethod
    def _paths(samples):
        """Both paths in the truth frame, aligned by the headings at the first sample."""
        (t0, l0, truth_yaw0) = samples[0]
        theta = truth_yaw0 - l0[2]
        c, s = math.cos(theta), math.sin(theta)
        truth = [(t[0] - t0[0], t[1] - t0[1]) for t, _, _ in samples]
        lio = []
        for _, p, _ in samples:
            dx, dy = p[0] - l0[0], p[1] - l0[1]
            lio.append((c * dx - s * dy, s * dx + c * dy))
        return truth, lio

    def test_fastlio_tracks_ground_truth(self):
        # The latch is the pipeline's own readiness signal: TF appears only after FAST-LIO has
        # initialised its IMU and produced a pose the publisher accepted.
        self._spin_until(
            lambda: None not in (self.lio, self.truth, self.truth_yaw),
            LATCH_TIMEOUT_S,
            "odom -> base_footprint never appeared: FAST-LIO or the bridge is not running",
        )

        # Phase 1: standing. The policy holds the robot; both paths should go nowhere.
        samples = self._collect(STAND_S)
        self.assertGreater(len(samples), 40, "too few paired samples while standing")
        truth_path, lio_path = self._paths(samples)
        truth_wander = max(math.hypot(*p) for p in truth_path)
        lio_wander = max(math.hypot(*p) for p in lio_path)
        self.assertLess(truth_wander, 0.2, "the robot was not actually standing still")
        self.assertLess(
            lio_wander,
            MAX_STANDING_DRIFT_M,
            f"fast_lio wandered {lio_wander:.2f} m while the robot stood still",
        )

        # Phase 2: walk. The gait is not repeatable (see module docstring), so this bound has
        # room for a bad one; scripts/lio_bench is where the precision number comes from.
        self._set_mode(4)  # StandUp
        time.sleep(3.0)
        self._set_mode(500)  # Start
        time.sleep(2.0)
        cmd = Twist()
        cmd.linear.x = CMD_VX
        samples = self._collect(WALK_S, command=cmd)
        self.cmd_pub.publish(Twist())  # stop

        self.assertGreater(len(samples), 50, "too few paired samples during the walk")
        truth_path, lio_path = self._paths(samples)
        truth_end = math.hypot(*truth_path[-1])
        self.assertGreater(truth_end, MIN_TRUTH_PATH_M, "the robot never actually walked")

        worst = max(
            math.hypot(tx - lx, ty - ly)
            for (tx, ty), (lx, ly) in zip(truth_path, lio_path, strict=True)
        )
        self.assertLess(worst, MAX_WALK_GAP_M, f"worst aligned gap {worst:.2f} m")
