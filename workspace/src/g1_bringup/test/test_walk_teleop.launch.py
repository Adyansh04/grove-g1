"""Headless sim integration test: driving the robot through the real LocoClient authority path.
Commands here are deliberately above the policy's measured gait-initiation thresholds.
"""

import os
import random
import time
import unittest
from collections import deque

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
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState
from unitree_hg.msg import LowState

SETTLE_TIMEOUT_S = 25.0
STAND_HEIGHT_MIN = 0.60

# Above the measured 0.40 m/s forward gait-initiation threshold. See the module docstring.
DRIVE_VX = 0.7
API_ID_SET_VELOCITY = 7105
CODE_LOCO_STATE_NOT_AVAILABLE = 7301
# This policy creeps underfoot even at a zero command (documented in the README). One figure,
# used by every drift bound here, so the suite cannot contradict itself about the same physics.
ZERO_COMMAND_DRIFT_M_PER_S = 0.10


def _best_effort_qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _sport_qos():
    """Vendor-matched reliable/volatile, matching both sides of /api/sport/*."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


@pytest.mark.launch_test
def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        )
    )
    return (
        LaunchDescription(
            [sim_launch, TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()])]
        ),
        {},
    )


class WalkTeleopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_walk_teleop")
        # Bounded buffers to avoid memory growth and starving the simulator.
        cls.sport_states = deque(maxlen=400)
        cls.low_states = deque(maxlen=1500)
        cls.responses = deque(maxlen=200)
        cls.node.create_subscription(
            SportModeState, "/sportmodestate", cls.sport_states.append, _best_effort_qos()
        )
        cls.node.create_subscription(
            LowState, "/lowstate", cls.low_states.append, _best_effort_qos()
        )
        cls.node.create_subscription(
            Response, "/api/sport/response", cls.responses.append, _sport_qos()
        )
        # NO raw /api/sport/request publisher here to avoid dual-writer guard trips.
        cls.cmd_vel_pub = cls.node.create_publisher(
            Twist,
            "/g1_loco_bridge/cmd_vel",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        cls.mode_client = ActionClient(cls.node, SetLocoMode, "/g1_loco_bridge/set_mode")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # --- helpers ---------------------------------------------------------------------------

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

    def _position(self):
        return self.sport_states[-1].position if self.sport_states else None

    def _height(self):
        pos = self._position()
        return pos[2] if pos is not None else None

    def _tilt(self):
        if not self.low_states:
            return 0.0
        rpy = self.low_states[-1].imu_state.rpy
        return max(abs(rpy[0]), abs(rpy[1]))

    def _await_standing(self, settle_s=0.0):
        """Waits for the robot to be up; `settle_s` additionally waits out the spawn transient.

        The robot spawns straight-legged at 0.793 m and the policy settles it into its crouch, so
        height crosses STAND_HEIGHT_MIN while the base is still moving. Any test measuring
        displacement has to wait that out, or it charges the settle to whatever it is asserting on.
        """
        self.assertTrue(
            self._wait_until(lambda: (self._height() or 0.0) > STAND_HEIGHT_MIN, SETTLE_TIMEOUT_S),
            "robot never stood up",
        )
        if settle_s:
            self._spin(settle_s)

    def _set_mode(self, fsm_id, timeout_s=10.0):
        """Drives the real SetLocoMode action -- the same path an operator or Nav2 would use."""
        self.assertTrue(self.mode_client.wait_for_server(timeout_sec=timeout_s), "no action server")
        goal = SetLocoMode.Goal()
        goal.fsm_id = fsm_id
        send_future = self.mode_client.send_goal_async(goal)
        self.assertTrue(self._wait_until(lambda: send_future.done(), timeout_s), "goal not accepted")
        handle = send_future.result()
        self.assertTrue(handle.accepted, f"SetLocoMode({fsm_id}) was rejected")
        result_future = handle.get_result_async()
        self.assertTrue(self._wait_until(lambda: result_future.done(), timeout_s), "no result")
        return result_future.result().result

    def _drive(self, vx, duration_s, vyaw=0.0):
        """Publishes cmd_vel continuously -- the bridge re-issues SET_VELOCITY on its own timer."""
        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = vyaw
        end = time.time() + duration_s
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.02)

    def _planar_distance(self, a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    # --- tests -----------------------------------------------------------------------------

    def test_01_velocity_before_start_is_rejected_and_moves_nothing(self):
        """SET_VELOCITY outside Start must return 7301 and not move the robot.

        Published as a raw request because the bridge's VelocityGate would never emit
        one outside kHeld. Publisher is created/destroyed inside this test to avoid
        tripping the single-writer guard for the rest of the session.
        """
        # Settle first: the measurement below is about whether a rejected command moved the
        # robot, so the spawn transient must not be inside the window.
        self._await_standing(settle_s=4.0)
        before = list(self._position())

        raw_pub = self.node.create_publisher(Request, "/api/sport/request", _sport_qos())
        try:
            request = Request()
            request.header.identity.id = 987654321
            request.header.identity.api_id = API_ID_SET_VELOCITY
            request.parameter = '{"velocity":[0.7,0.0,0.0],"duration":1.0}'
            for _ in range(5):
                raw_pub.publish(request)
                self._spin(0.1)

            matched = [r for r in self.responses if r.header.identity.id == 987654321]
            self.assertTrue(matched, "no response to the raw SET_VELOCITY request")
            self.assertEqual(
                matched[-1].header.status.code,
                CODE_LOCO_STATE_NOT_AVAILABLE,
                "SET_VELOCITY outside Start must be rejected with 7301",
            )
            self._spin(3.0)
        finally:
            self.node.destroy_publisher(raw_pub)

        # Drift bound: reject command movement must be < accumulated idle-drift.
        drift = self._planar_distance(self._position(), before)
        self.assertLess(
            drift,
            ZERO_COMMAND_DRIFT_M_PER_S * 3.5,
            f"robot moved {drift:.3f} m on a REJECTED velocity command -- the FSM legality gate is "
            "not actually gating the policy",
        )

    def test_02_start_sequence_then_drive(self):
        """The real Damp -> StandUp -> Start sequence, then cmd_vel actually walks the robot."""
        self._await_standing()
        self.assertTrue(self._set_mode(SetLocoMode.Goal.STAND_UP).success, "StandUp rejected")
        self.assertTrue(self._set_mode(SetLocoMode.Goal.START).success, "Start rejected")

        before = list(self._position())
        self._drive(DRIVE_VX, 8.0)
        travelled = self._planar_distance(self._position(), before)

        # ~4 m total expected; 2 m bound is conservative against the ~0.8 m cumulative drift.
        self.assertGreater(
            travelled,
            2.0,
            f"robot travelled only {travelled:.2f} m at vx={DRIVE_VX} over 8 s -- velocity is not "
            "reaching the policy through the LocoClient path",
        )
        self.assertGreater(self._height(), STAND_HEIGHT_MIN, "robot fell while walking")

    def test_03_zero_twist_stops(self):
        """A zero command stops forward progress -- the robot must not coast on the last latch."""
        self._drive(0.0, 3.0)
        before = list(self._position())
        self._drive(0.0, 3.0)
        self.assertLess(
            self._planar_distance(self._position(), before),
            ZERO_COMMAND_DRIFT_M_PER_S * 3.0,
            "robot kept moving on a zero command",
        )

    def test_04_dead_man_stops_a_silent_bridge(self):
        """Stop publishing entirely: the request's own duration expires and the robot stops.

        This is the vendor `duration` field acting as the dead-man, not a parallel timeout -- the
        reason this stack never latches the 864000 s "continuous" value.
        """
        self._drive(DRIVE_VX, 4.0)
        self._spin(2.5)  # silence, longer than the 1 s duration the bridge sends
        before = list(self._position())
        self._spin(3.0)
        self.assertLess(
            self._planar_distance(self._position(), before),
            0.10,
            "robot kept walking after cmd_vel went silent -- the duration dead-man did not fire",
        )

    def test_05_damp_releases_authority_and_robot_keeps_standing(self):
        """Releasing locomotion authority stops motion but must not drop the robot.

        The policy runs continuously regardless of FSM state -- leg authority is gated on policy
        freshness, not on the FSM -- so Damp ends walking without ending balance.
        """
        self.assertTrue(self._set_mode(SetLocoMode.Goal.DAMP).success, "Damp rejected")
        self._spin(1.5)
        before = list(self._position())
        self._drive(DRIVE_VX, 3.0)
        self.assertLess(
            self._planar_distance(self._position(), before),
            ZERO_COMMAND_DRIFT_M_PER_S * 3.0,
            "cmd_vel still moved the robot after Damp released locomotion authority",
        )
        self._spin(3.0)
        self.assertGreater(
            self._height(),
            STAND_HEIGHT_MIN,
            "robot collapsed after Damp -- the policy must keep balancing regardless of FSM state",
        )
        self.assertLess(self._tilt(), 0.4, "robot is toppling after Damp")

    def test_06_random_command_sequence_never_falls(self):
        """Fixed-seed random commands spanning above and below threshold, re-drawn at 2 Hz."""
        self.assertTrue(self._set_mode(SetLocoMode.Goal.STAND_UP).success, "StandUp rejected")
        self.assertTrue(self._set_mode(SetLocoMode.Goal.START).success, "Start rejected")

        rng = random.Random(20260802)
        worst_height = 10.0
        worst_tilt = 0.0
        for _ in range(24):
            vx = rng.choice([-0.5, 0.0, 0.2, 0.6, 0.8])
            vyaw = rng.choice([-1.6, 0.0, 0.3, 1.6])
            self._drive(vx, 0.5, vyaw=vyaw)
            worst_height = min(worst_height, self._height() or 0.0)
            worst_tilt = max(worst_tilt, self._tilt())

        self._drive(0.0, 1.0)
        self.assertGreater(
            worst_height,
            STAND_HEIGHT_MIN,
            f"robot fell to {worst_height:.3f} m during a random command sequence",
        )
        self.assertLess(worst_tilt, 0.6, f"robot reached {worst_tilt:.2f} rad tilt")

    def test_07_command_whiplash_never_falls(self):
        """Slam between above- and below-threshold commands.

        The policy has no hysteresis: dropping below threshold stops the gait outright. Doing that
        at speed, repeatedly, is the transition this test exists to cover.
        """
        worst_height = 10.0
        for _ in range(3):
            self._drive(0.8, 2.0)
            worst_height = min(worst_height, self._height() or 0.0)
            self._drive(0.2, 1.5)  # below threshold -- gait stops with no ramp
            worst_height = min(worst_height, self._height() or 0.0)
        self._drive(0.0, 1.5)
        self.assertGreater(
            worst_height,
            STAND_HEIGHT_MIN,
            f"robot fell to {worst_height:.3f} m while slamming between walk and stand",
        )
