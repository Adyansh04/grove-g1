"""Headless sim integration test: the LocoClient loop end to end -- g1_locomotion's g1_loco_bridge
talking real DDS to motion_service_sim's protocol-only /api/sport/* responder.

sim.launch.py brings up g1_loco_bridge itself (via loco.launch.py, lifecycle-configured and
activated automatically), so this test drives it: a manual SET_VELOCITY published directly
(the bridge itself never sends one outside kHeld, so this is the only way to exercise the
responder's Start-only gate before any SetLocoMode goal has run -- see g1_bringup's README's FSM
table), the STAND_UP -> START sequence the responder's legality table requires, the velocity
re-issue loop's rate and fixed duration, the zero-Twist stop, and DAMP releasing authority. All in
one launch and one ordered test method -- sim startup is the expensive part, see
test_sim_bringup.launch.py's comment on SIM_SETTLE_S.

Run via `colcon test --packages-select g1_bringup`.
"""

import json
import os
import time
import unittest

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from g1_msgs.action import SetLocoMode
from g1_msgs.msg import LocoStatus
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from unitree_api.msg import Request, Response

# See test_sim_bringup.launch.py's comment on this constant: kept below launch_testing's own
# hardcoded 15 s process-startup deadline. sim.launch.py also configures+activates
# g1_loco_bridge, a near-instant lifecycle handshake (no ramp) well inside this same window.
SIM_SETTLE_S = 10.0

# SET_VELOCITY's own api id -- see g1_locomotion's loco_api_ids.hpp.
API_ID_SET_VELOCITY = 7105

# Vendor-matched: rclcpp::QoS(1) reliable, volatile -- both g1_locomotion's bridge and
# motion_service_sim's responder use this exactly for /api/sport/*.
SPORT_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# g1_loco_bridge's ~/status: reliable, transient-local, so a subscription created well after
# activation still immediately receives the latest sample instead of racing the next heartbeat.
STATUS_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

CMD_VEL_QOS = QoSProfile(
    depth=1,
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


class TestLoco(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_loco")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _latest_status(self, timeout_s=5.0):
        result = {}
        sub = self.node.create_subscription(
            LocoStatus, "/g1_loco_bridge/status", lambda msg: result.update(msg=msg), STATUS_QOS
        )
        deadline = time.monotonic() + timeout_s
        while "msg" not in result and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(sub)
        return result.get("msg")

    def _send_set_loco_mode(self, client, fsm_id, timeout_s=10.0):
        goal = SetLocoMode.Goal()
        goal.fsm_id = fsm_id
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=timeout_s)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle, f"SetLocoMode({fsm_id}) send_goal_async timed out")
        self.assertTrue(goal_handle.accepted, f"SetLocoMode({fsm_id}) goal was not accepted")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_s)
        result = result_future.result()
        self.assertIsNotNone(result, f"SetLocoMode({fsm_id}) result timed out")
        return result.result

    def test_loco_client_sequence(self):
        # 1. A manual SET_VELOCITY, published directly, is rejected 7301 before anything has
        # asked for STAND_UP/START. The bridge itself only ever sends SET_VELOCITY from kHeld
        # (see VelocityGate), so there is no way to provoke this precondition through it --
        # this exercises the responder's own Start-only gate genuinely over real DDS instead of
        # mocking it out.
        responses = {}
        response_sub = self.node.create_subscription(
            Response,
            "/api/sport/response",
            lambda msg: responses.update({msg.header.identity.id: msg}),
            SPORT_QOS,
        )
        request_pub = self.node.create_publisher(Request, "/api/sport/request", SPORT_QOS)

        probe = Request()
        probe.header.identity.id = 987654321
        probe.header.identity.api_id = API_ID_SET_VELOCITY
        probe.parameter = json.dumps({"velocity": [0.1, 0.0, 0.0], "duration": 1.0})

        deadline = time.monotonic() + 5.0
        while probe.header.identity.id not in responses and time.monotonic() < deadline:
            request_pub.publish(probe)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(response_sub)
        self.node.destroy_publisher(request_pub)

        self.assertIn(
            probe.header.identity.id,
            responses,
            "no /api/sport/response observed for the manual SET_VELOCITY probe",
        )
        self.assertEqual(
            responses[probe.header.identity.id].header.status.code,
            7301,
            "SET_VELOCITY before START must be rejected 7301 (LocoState not available)",
        )

        # 2. STAND_UP then START (StandUp is a required intermediate hop from the responder's
        # Damp boot state -- see g1_bringup's README's FSM table) both succeed, and ~/status
        # settles on fsm_id 500 with authority HELD.
        action_client = ActionClient(self.node, SetLocoMode, "/g1_loco_bridge/set_mode")
        self.assertTrue(action_client.wait_for_server(timeout_sec=10.0))

        stand_up_result = self._send_set_loco_mode(action_client, SetLocoMode.Goal.STAND_UP)
        self.assertTrue(stand_up_result.success, f"STAND_UP rejected: {stand_up_result.message}")

        start_result = self._send_set_loco_mode(action_client, SetLocoMode.Goal.START)
        self.assertTrue(start_result.success, f"START rejected: {start_result.message}")

        status = self._latest_status()
        self.assertIsNotNone(status, "no /g1_loco_bridge/status sample observed after START")
        self.assertEqual(status.fsm_id, 500)
        self.assertEqual(status.authority, LocoStatus.HELD)

        # 3. Held: a non-zero ~/cmd_vel produces SET_VELOCITY traffic faster than 1 Hz, and every
        # sampled request carries the fixed 1.0 s duration (never the vendor's continuous-latch
        # value) -- see g1_locomotion's loco_payloads.hpp for why that value is fixed at all.
        velocity_requests = []

        def _collect_velocity_requests(msg):
            if msg.header.identity.api_id == API_ID_SET_VELOCITY:
                velocity_requests.append(msg)

        request_sub = self.node.create_subscription(
            Request, "/api/sport/request", _collect_velocity_requests, SPORT_QOS
        )
        cmd_vel_pub = self.node.create_publisher(Twist, "/g1_loco_bridge/cmd_vel", CMD_VEL_QOS)
        moving_twist = Twist()
        moving_twist.linear.x = 0.1

        sample_window_s = 2.0
        deadline = time.monotonic() + sample_window_s
        while time.monotonic() < deadline:
            cmd_vel_pub.publish(moving_twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        rate = len(velocity_requests) / sample_window_s
        self.assertGreater(rate, 1.0, f"SET_VELOCITY request rate too low: {rate:.2f} Hz")
        for msg in velocity_requests:
            self.assertEqual(
                json.loads(msg.parameter).get("duration"),
                1.0,
                "a SET_VELOCITY request did not carry the fixed 1.0 s duration",
            )

        # 4. A zero Twist stops it. Keep publishing the zero Twist throughout both windows below
        # -- staying fresh-and-zero is what actually discriminates this from the stale-command
        # branch (VelocityGate.tick() takes the same single-stop-then-idle path for both, so a
        # window that lets the command go stale "passes" even with the zero check deleted
        # entirely). First confirm the stop itself carried a zero velocity, then confirm traffic
        # ceases even though a fresh zero command keeps arriving.
        velocity_requests.clear()
        zero_twist    = Twist()
        stop_deadline = time.monotonic() + 1.0
        while time.monotonic() < stop_deadline:
            cmd_vel_pub.publish(zero_twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # >=1, not ==1: a re-issue tick can land between clear() and the first zero Twist and
        # still be mid-flight when the loop above starts, adding one more non-stop sample first.
        self.assertGreaterEqual(
            len(velocity_requests), 1, "no SET_VELOCITY request observed after a zero Twist"
        )
        self.assertEqual(
            json.loads(velocity_requests[-1].parameter).get("velocity"),
            [0.0, 0.0, 0.0],
            "the request following a zero Twist did not carry a zero velocity",
        )

        velocity_requests.clear()
        silence_deadline = time.monotonic() + 1.0
        while time.monotonic() < silence_deadline:
            cmd_vel_pub.publish(zero_twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(request_sub)
        self.node.destroy_publisher(cmd_vel_pub)
        self.assertEqual(
            len(velocity_requests),
            0,
            "SET_VELOCITY traffic did not cease after the single stop-then-idle intent, despite "
            "cmd_vel staying fresh and zero the whole time",
        )

        # 5. DAMP releases authority.
        damp_result = self._send_set_loco_mode(action_client, SetLocoMode.Goal.DAMP)
        self.assertTrue(damp_result.success, f"DAMP rejected: {damp_result.message}")
        action_client.destroy()

        status = self._latest_status()
        self.assertIsNotNone(status, "no /g1_loco_bridge/status sample observed after DAMP")
        self.assertEqual(status.authority, LocoStatus.RELEASED)
