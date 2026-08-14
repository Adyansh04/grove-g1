"""Headless sim integration test: the walking policy stands the robot up and holds it.

Everything here requires the robot not to have moved, so it is deliberately
separate from test_walk_teleop.launch.py (which drives it). Launches the default
unwelded stack -- pin_pelvis defaults false, so nothing but the policy keeps the
robot upright.

# LOAD SENSITIVITY: Policy wall-timer vs sim CPU-time clock drift can cause failures.
# Run this test isolated (`colcon test ... -R test_walk_stand`) if flaky.
"""

import os
import time
import unittest
from collections import deque

import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from unitree_go.msg import SportModeState
from unitree_hg.msg import LowState

# The policy needs /lowstate and /sportmodestate flowing before it can produce a target, on top of
# sim.launch.py's own 2 s sim-start delay. Generous so a slow container start cannot flake it.
SETTLE_TIMEOUT_S = 25.0

# Standing height band. Measured settle is 0.761 m from a 0.793 m spawn; the band is wide enough to
# absorb gait micro-motion but tight enough that a collapse (or a robot still welded at 0.793) fails.
STAND_HEIGHT_MIN = 0.70
STAND_HEIGHT_MAX = 0.79

# How long the robot must hold that height once settled.
HOLD_DURATION_S = 12.0

# Max lower-body |dq| during the spawn->crouch settle. Regression guard against
# the entry transient becoming violent (measured peak ~12.5 rad/s).
ENTRY_PEAK_DQ_MAX = 20.0

LOWER_MOTORS = 15


def _best_effort_qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


@pytest.mark.launch_test
def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        )
        # No launch_arguments: this suite exercises the default stack, unwelded, with the
        # walking policy holding the robot up.
    )
    return (
        LaunchDescription([sim_launch, TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()])]),
        {},
    )


class WalkStandTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_walk_stand")
        # Bounded: see the note in test_walk_teleop.launch.py -- unbounded accumulation of
        # ~900 Hz /lowstate starves the simulator over a long suite.
        cls.sport_states = deque(maxlen=400)
        cls.low_states = deque(maxlen=1500)
        cls.node.create_subscription(
            SportModeState, "/sportmodestate", cls.sport_states.append, _best_effort_qos()
        )
        # The entry transient is measured in the callback rather than by keeping the messages:
        # the bounded buffer would have evicted the first second long before test_04 runs.
        cls.entry_peak_dq = 0.0
        cls.entry_samples = 0

        def on_low_state(msg):
            cls.low_states.append(msg)
            if cls.entry_samples < 1000:
                cls.entry_samples += 1
                cls.entry_peak_dq = max(
                    cls.entry_peak_dq,
                    max(abs(msg.motor_state[i].dq) for i in range(LOWER_MOTORS)),
                )

        cls.node.create_subscription(LowState, "/lowstate", on_low_state, _best_effort_qos())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, duration_s):
        end = time.time() + duration_s
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_until(self, predicate, timeout_s):
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _height(self):
        return self.sport_states[-1].position[2] if self.sport_states else None

    def test_01_policy_stands_the_robot_up(self):
        """The policy settles the robot into a stable standing height, unwelded."""
        self.assertTrue(
            self._wait_until(
                lambda: self._height() is not None
                and STAND_HEIGHT_MIN < self._height() < STAND_HEIGHT_MAX,
                SETTLE_TIMEOUT_S,
            ),
            f"robot never reached a standing height in [{STAND_HEIGHT_MIN}, {STAND_HEIGHT_MAX}] m "
            f"(last seen: {self._height()}) -- with no pelvis weld this means the policy failed "
            "to stand it up",
        )

    def test_02_it_keeps_standing(self):
        """Height stays in band for a sustained window -- a slow collapse must fail too."""
        self.assertTrue(
            self._wait_until(
                lambda: self._height() is not None
                and STAND_HEIGHT_MIN < self._height() < STAND_HEIGHT_MAX,
                SETTLE_TIMEOUT_S,
            ),
            "robot was not standing before the hold window",
        )
        start = time.time()
        worst = None
        while time.time() - start < HOLD_DURATION_S:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            height = self._height()
            if height is not None and (worst is None or height < worst):
                worst = height
        self.assertGreater(
            worst,
            STAND_HEIGHT_MIN,
            f"robot sank to {worst:.3f} m during a {HOLD_DURATION_S:.0f} s hold -- the policy is "
            "not maintaining a stable stance",
        )

    def test_03_stays_upright(self):
        """Tilt stays small: a robot can hold height briefly while already toppling."""
        self._spin(2.0)
        self.assertTrue(self.low_states, "no /lowstate received")
        recent = list(self.low_states)[-200:]
        worst_tilt = max(max(abs(s.imu_state.rpy[0]), abs(s.imu_state.rpy[1])) for s in recent)
        self.assertLess(
            worst_tilt,
            0.35,
            f"peak roll/pitch {worst_tilt:.2f} rad while standing still -- the policy is not "
            "holding the base level",
        )

    def test_04_entry_transient_is_not_violent(self):
        """The spawn->crouch settle must stay a settle, not a snap."""
        self.assertGreater(self.entry_samples, 0, "no /lowstate received during entry")
        peak_dq = self.entry_peak_dq
        self.assertLess(
            peak_dq,
            ENTRY_PEAK_DQ_MAX,
            f"peak lower-body joint velocity {peak_dq:.1f} rad/s during entry -- the policy is "
            "snapping into its default posture rather than settling",
        )

    def test_05_single_lowcmd_writer(self):
        """The single-writer rule still holds with the policy in the loop."""
        self._spin(1.0)
        self.assertEqual(
            self.node.count_publishers("/lowcmd"),
            1,
            "more than one publisher on /lowcmd -- the walking policy must reach the wire through "
            "motion_service_sim, never as a second writer",
        )
