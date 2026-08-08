"""Locomotion authority is acquired and released by a lifecycle transition, and nothing before.

Covers the half of the cmd_vel story that has no unit test: that the bridge really does discard
commands until authority is held, that the transition really does acquire it, and that
deactivating really does hand it back. VelocityGate's counter is unit tested; this is the wiring
around it.

Not pinned, unlike the other suites here. pin_pelvis disables the walking policy, which is what
the FSM sequence exists to start, so pinning would make the whole test meaningless.

The authority node is launched unmanaged and left unconfigured on purpose: the test owns the
transitions, because their boundaries are exactly what it is asserting on.
"""

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
from launch_ros.actions import LifecycleNode
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from g1_msgs.msg import LocoStatus

BRINGUP_TIMEOUT_S = 150.0
# The acquire sends two FSM goals and then sleeps settle_after_start_s, and it retries while the
# stack is still coming up. Generous on purpose: a tight budget here would fail on the startup
# race the retry exists to absorb.
TRANSITION_TIMEOUT_S = 60.0


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "sensors": "true",
            "world": "navigation",
            "headless": "true",
            "sim_start_delay_s": "4.0",
        }.items(),
    )
    authority = LifecycleNode(
        package="g1_locomotion",
        executable="g1_loco_authority",
        name="g1_loco_authority",
        namespace="",
        output="both",
        parameters=[
            os.path.join(
                get_package_share_directory("g1_locomotion"),
                "config",
                "g1_loco_authority.yaml",
            )
        ],
        remappings=[("status", "/g1_loco_bridge/status")],
    )
    return (
        LaunchDescription([
            sim,
            authority,
            TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]),
        {},
    )


class NavAuthorityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("nav_authority_test")
        cls.status = []
        cls.node.create_subscription(
            LocoStatus,
            "/g1_loco_bridge/status",
            cls.status.append,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.cmd = cls.node.create_publisher(
            Twist,
            "/g1_loco_bridge/cmd_vel",
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        cls.change = cls.node.create_client(ChangeState, "/g1_loco_authority/change_state")
        cls.get = cls.node.create_client(GetState, "/g1_loco_authority/get_state")

        deadline = time.time() + BRINGUP_TIMEOUT_S
        while time.time() < deadline and not cls.status:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        # Recorded, not asserted: setUpClass raising gives an unreadable error for every test
        # in the class. test_* asserts on it instead, and names what actually failed.
        cls.change_ready = cls.change.wait_for_service(timeout_sec=30.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, secs, publish=None):
        end = time.time() + secs
        while time.time() < end:
            if publish is not None:
                self.cmd.publish(publish)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.03)

    def latest(self):
        self.spin(1.2)
        self.assertTrue(self.status, "no LocoStatus received")
        return self.status[-1]

    def transition(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change.call_async(req)
        deadline = time.time() + TRANSITION_TIMEOUT_S
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(future.done(), f"transition {transition_id} never returned")
        return future.result().success

    def test_the_whole_bracket(self):
        self.assertTrue(
            self.change_ready, "g1_loco_authority's change_state service never appeared"
        )
        # One test, not four: these are steps in a sequence, and a per-step test would either
        # re-drive the FSM each time or depend on execution order.
        moving = Twist()
        moving.linear.x = 0.6

        # 1. Before any transition, commands are discarded and the bridge says so.
        before = self.latest().ignored_cmd_vel
        self.spin(3.0, publish=moving)
        during = self.latest().ignored_cmd_vel
        self.assertGreater(
            during, before, "cmd_vel was not counted as discarded while authority was RELEASED"
        )
        self.assertNotEqual(self.latest().authority, LocoStatus.HELD)

        # 2. The transition acquires it.
        self.assertTrue(self.transition(Transition.TRANSITION_CONFIGURE), "configure failed")
        self.assertTrue(self.transition(Transition.TRANSITION_ACTIVATE), "activate failed")
        held = self.latest()
        self.assertEqual(held.authority, LocoStatus.HELD, "activate did not acquire authority")
        self.assertEqual(held.fsm_id, 500, "the robot is not in Start")

        # 3. With authority held, the same commands stop being discarded.
        base = self.latest().ignored_cmd_vel
        self.spin(3.0, publish=moving)
        self.assertEqual(
            self.latest().ignored_cmd_vel,
            base,
            "commands were still counted as discarded while authority was HELD",
        )

        # 4. Deactivating hands it back, and leaves the robot standing rather than dropped.
        self.spin(1.0, publish=Twist())
        self.assertTrue(self.transition(Transition.TRANSITION_DEACTIVATE), "deactivate failed")
        released = self.latest()
        self.assertNotEqual(released.authority, LocoStatus.HELD, "authority was never released")
        self.assertEqual(
            released.fsm_id, 4, "released to something other than StandUp; Damp would drop the robot"
        )
