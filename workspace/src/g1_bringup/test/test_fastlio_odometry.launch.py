"""FAST-LIO odometry against MuJoCo ground truth: precision standing, sanity walking.

The unit suites prove the adapter math; this proves the pipeline. The full stack comes up with
`odometry:=fast_lio` and the odom -> base_footprint it publishes is compared against the exact
pelvis pose MuJoCo reports over the relay's sensor socket, so a regression anywhere in the chain
shows up here.

Two phases with very different tolerances. Standing is repeatable and gets a tight bound:
measured drift is ~2 cm, asserted at 0.35 m. Walking is looser, because a stumble degrades any
LIO and the bound has to survive one. scripts/lio_bench measures ~10 cm worst case over 21 m on
a clean run; this asserts 1.0 m, which still catches a divergence of kilometres.

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
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

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
        # Position and heading in one message, straight off the simulator's sensor socket.
        cls.node.create_subscription(
            Odometry, "/g1_sensor_relay/base_state", cls._on_truth, qos_profile_sensor_data
        )
        cls.node.create_subscription(TFMessage, "/tf", cls._on_tf, 100)
        cls.cmd_pub = cls.node.create_publisher(Twist, "/cmd_vel", 1)

    @classmethod
    def _on_truth(cls, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        cls.truth = (p.x, p.y)
        cls.truth_yaw = yaw_of(q.x, q.y, q.z, q.w)

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

        # Phase 2: walk. Nothing puts the robot into a walking mode first, because the policy is
        # already balancing it and takes velocity directly. CMD_VX clears the gait's deadband,
        # below which the command produces no motion at all.
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
