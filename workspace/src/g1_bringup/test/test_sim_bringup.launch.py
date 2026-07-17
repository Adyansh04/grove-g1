"""Headless sim integration test: sim.launch.py comes up cleanly.

Asserts environment preconditions held, /lowstate flowing fast, the
G1ArmSdkSystem hardware component listed with its 14 command interfaces
(inactive), joint_state_broadcaster active with /joint_states at ~200 Hz
(finite values), and arm_trajectory_controller present but inactive.

Run via `colcon test --packages-select g1_bringup` (headless by default).
sim.launch.py manages its own internal Xvfb display regardless of the
ambient DISPLAY/xvfb-run state, so running the whole `colcon test` under an
*outer* `xvfb-run -a` too does not double-wrap anything -- the two Xvfb
instances, if both present, use different display numbers.
"""

import math
import os
import time
import unittest

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState

# Xvfb + a 2 s sim-start delay + unitree_mujoco's own startup +
# control.launch.py's controller loading all happen within this window (see
# g1_bringup/README.md's operating procedure for the individual pieces).
# Kept comfortably below launch_testing's own hardcoded 15 s "processes
# launched" deadline (test_runner.py's `_processes_launched.wait(timeout=15)`)
# -- setting this equal to or above that value races the two timeouts and
# was observed to lose that race, aborting the whole launch before
# ReadyToTest ever fired.
SIM_SETTLE_S = 10.0


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


class TestSimBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_sim_bringup")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _collect_for(self, topic_type, topic, duration_s):
        samples = []
        sub = self.node.create_subscription(topic_type, topic, samples.append, 10)
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(sub)
        return samples

    def test_lowstate_flowing_fast(self):
        samples = self._collect_for(LowState, "/lowstate", 3.0)
        rate = len(samples) / 3.0
        self.assertGreaterEqual(rate, 400.0, f"/lowstate rate too low: {rate:.1f} Hz")

    def test_hardware_component_inactive_with_14_command_interfaces(self):
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
        component = matches[0]
        self.assertEqual(component.state.label, "inactive")
        self.assertEqual(len(component.command_interfaces), 14)

    def test_controllers_present_in_expected_states(self):
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), "service not available")
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result()
        self.assertIsNotNone(result)

        by_name = {c.name: c for c in result.controller}
        self.assertIn("joint_state_broadcaster", by_name)
        self.assertEqual(by_name["joint_state_broadcaster"].state, "active")
        self.assertIn("arm_trajectory_controller", by_name)
        self.assertEqual(by_name["arm_trajectory_controller"].state, "inactive")

    def test_joint_states_rate_and_values_finite(self):
        samples = self._collect_for(JointState, "/joint_states", 3.0)
        rate = len(samples) / 3.0
        # controller_manager's update_rate is 200 Hz; generous margin below that.
        self.assertGreaterEqual(rate, 150.0, f"/joint_states rate too low: {rate:.1f} Hz")

        self.assertGreater(len(samples), 0)
        last = samples[-1]
        self.assertEqual(len(last.name), 14)
        for value in list(last.position) + list(last.velocity):
            self.assertTrue(math.isfinite(value), f"non-finite joint_states value: {value}")
