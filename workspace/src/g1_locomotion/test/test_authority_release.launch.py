"""A failed acquire must not leak locomotion authority.

Safety rule 4 says a skill acquires exclusive control authority before acting and releases it
cleanly on success *or failure*. Success is covered by g1_navigation's test_nav_authority against
a live sim. Failure is not reachable there: the real bridge succeeds, reports HELD, and the
lifecycle handlers put it straight back to active, so the interesting window never opens.

The window is narrow and specific. on_activate sets acquired_ only after START returns, and the
one failure after that point is waitForHeld() timing out -- the bridge believes it holds
authority while the node is about to return FAILURE and never see on_deactivate. Miss the release
there and the robot is left walk-capable with nothing supervising it.

setmode_stub.py in "no_held" mode reproduces exactly that. No simulator, so this is fast and
deterministic, which is why it is a unit-grade launch test rather than another sim suite.
"""

import os
import sys
import time
import unittest

import launch_testing
import pytest
import rclpy
from g1_msgs.action import SetLocoMode
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node as LaunchNode
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32MultiArray

STAND_UP = SetLocoMode.Goal.STAND_UP
START = SetLocoMode.Goal.START

# Short, because nothing here waits on hardware: the stub answers immediately and the only real
# delay is the deliberate one this test is measuring.
ACQUIRE_TIMEOUT_S = 3.0


@pytest.mark.launch_test
def generate_test_description():
    stub = ExecuteProcess(
        cmd=[sys.executable, os.path.join(os.path.dirname(__file__), "setmode_stub.py"),
             "--ros-args", "-r", "__node:=setmode_stub", "-p", "mode:=no_held"],
        output="both",
    )
    authority = LaunchNode(
        package="g1_locomotion",
        executable="g1_loco_authority",
        name="g1_loco_authority",
        output="both",
        parameters=[{
            "set_mode_action": "/setmode_stub/set_mode",
            "acquire_timeout_s": ACQUIRE_TIMEOUT_S,
            # Zero, not the shipped 2.5: this test never reaches the settle, and paying for it
            # on the paths that do would only make the suite slower.
            "settle_after_start_s": 0.0,
        }],
        remappings=[("status", "/setmode_stub/status")],
    )
    return (
        LaunchDescription([
            # An isolated domain, FIRST, before anything is launched. This test deliberately
            # launches a node called g1_loco_authority and drives its change_state, so on the
            # shared domain it would answer -- or be answered by -- the real one attached to a
            # running robot, and a test would acquire locomotion authority for real. Same
            # reason test_loco_bridge_node.cpp pins 67 and test_odometry_publisher_node.cpp
            # pins 77. launch's environment is os.environ, so this covers the stub, the node
            # and this pytest process's own rclpy.init().
            SetEnvironmentVariable("ROS_DOMAIN_ID", "68"),
            stub,
            authority,
            TimerAction(period=3.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]),
        {},
    )


class AuthorityReleaseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("authority_release_test")
        cls.goals = []
        cls.node.create_subscription(
            Int32MultiArray,
            "/setmode_stub/goals",
            cls.goals.append,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.change = cls.node.create_client(
            ChangeState, "/g1_loco_authority/change_state"
        )
        cls.change_ready = cls.change.wait_for_service(timeout_sec=30.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def transition(self, transition_id, timeout_s):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change.call_async(request)
        deadline = time.time() + timeout_s
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(future.done(), f"transition {transition_id} never answered")
        return future.result().success

    def test_failed_acquire_releases(self):
        self.assertTrue(self.change_ready, "g1_loco_authority's change_state never appeared")

        self.assertTrue(self.transition(Transition.TRANSITION_CONFIGURE, 10.0), "configure failed")

        # The activate is expected to FAIL: the stub never reports HELD, so waitForHeld() burns
        # acquire_timeout_s and on_activate returns FAILURE.
        activated = self.transition(
            Transition.TRANSITION_ACTIVATE, ACQUIRE_TIMEOUT_S + 20.0
        )
        self.assertFalse(
            activated, "activate reported success although the bridge never reported HELD"
        )

        self.spin(2.0)
        self.assertTrue(self.goals, "the stub never published its goal log")
        got = list(self.goals[-1].data)

        # STAND_UP, START, then STAND_UP again. The third is the release, and it is the whole
        # point: without it the bridge sits in START with the node inactive and unable to
        # release, because a failed on_activate never reaches on_deactivate.
        self.assertEqual(
            got,
            [STAND_UP, START, STAND_UP],
            f"expected acquire then release, got fsm_ids {got}. A trailing {START} or a missing "
            f"final {STAND_UP} means authority leaked on the failure path.",
        )
