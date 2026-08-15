"""Headless sim gate for the rt/lowcmd component: command one joint, read it back.

Proves the whole path end to end -- component activation, the SDK channel, the CRC the sim's
bridge feeds into its PD law, the freeze controller holding the other 28 joints, and the
kPositionOnly branch on the probe joint. No policy is involved; that is PR 4.

The pelvis is pinned because nothing is balancing: rt/lowcmd replaces the onboard controller
outright, and a free-standing robot with no policy falls over. See docs/CONTROL_MODES.md.
"""

import os
import time
import unittest

# Set before rclpy is imported and initialised: this test talks to a stack that runs on
# fastrtps, because the lowcmd component owns the robot wire through unitree_sdk2's own
# CycloneDDS. See docs/notes/lowcmd-dds-config.md.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Matches the other simulator tests: kept under launch_testing's 15 s startup deadline.
SIM_SETTLE_S = 10.0

PROBE_JOINT = "left_elbow_joint"
# Well inside the elbow's range, and far enough that noise cannot fake it.
PROBE_TARGET_RAD = 0.6
# position_only_kp is 10.0, so tracking is soft on purpose. Assert the joint clearly moved
# towards the command rather than landing exactly on it.
PROBE_MIN_TRAVEL_RAD = 0.15

FROZEN_JOINT = "right_elbow_joint"
# The freeze holds at whatever it captured; anything larger is a joint that is not being held.
FROZEN_MAX_DRIFT_RAD = 0.15


def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "control_stack": "lowcmd",
            # Nothing balances the robot under rt/lowcmd until PR 4's policy lands.
            "pin_pelvis": "true",
            "sensors": "false",
        }.items(),
    )

    return LaunchDescription([sim_launch, launch_testing.actions.ReadyToTest()])


class TestLowCmdJoint(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_lowcmd_joint")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

        cls.joint_states = []
        cls.node.create_subscription(
            JointState,
            "/joint_states",
            lambda msg: cls.joint_states.append(msg),
            10,
        )
        cls.command_pub = cls.node.create_publisher(
            Float64MultiArray, "/probe_position_controller/commands", 10
        )

        cls._spin_for(SIM_SETTLE_S)

    @classmethod
    def tearDownClass(cls):
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _spin_for(cls, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            cls.executor.spin_once(timeout_sec=0.1)

    def _latest_position(self, joint):
        for msg in reversed(self.joint_states):
            if joint in msg.name:
                return msg.position[msg.name.index(joint)]
        return None

    def _call(self, srv_type, name):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), f"{name} never appeared")
        future = client.call_async(srv_type.Request())
        deadline = time.monotonic() + 20.0
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
        self.assertTrue(future.done(), f"{name} did not answer")
        return future.result()

    def test_component_activates(self):
        """A broken on_init or a failed SDK channel shows up here and nowhere else.

        No unit test reaches on_init -- they only load the plugin through pluginlib -- so this
        is the only proof that HardwareInfo really populated and rt/lowstate really arrived.
        """
        result = self._call(ListHardwareComponents, "/controller_manager/list_hardware_components")
        names = {component.name: component.state.label for component in result.component}
        self.assertIn("G1LowCmdSystem", names, f"component never loaded, saw {names}")
        self.assertEqual(names["G1LowCmdSystem"], "active")

    def test_controllers_are_active(self):
        result = self._call(ListControllers, "/controller_manager/list_controllers")
        states = {controller.name: controller.state for controller in result.controller}
        for name in ("body_freeze_controller", "probe_position_controller"):
            self.assertEqual(states.get(name), "active", f"{name} is {states.get(name)}")

    def test_probe_joint_follows_a_command(self):
        start = self._latest_position(PROBE_JOINT)
        self.assertIsNotNone(start, "no /joint_states for the probe joint")

        self.command_pub.publish(Float64MultiArray(data=[PROBE_TARGET_RAD]))
        self._spin_for(5.0)

        end = self._latest_position(PROBE_JOINT)
        travel = abs(end - start)
        self.assertGreater(
            travel,
            PROBE_MIN_TRAVEL_RAD,
            f"{PROBE_JOINT} moved {travel:.3f} rad from {start:.3f}; the command never reached "
            "the motors",
        )
        # Direction matters as much as magnitude: a wrong sign or a wrong motor index would
        # still register as travel.
        self.assertLess(
            abs(end - PROBE_TARGET_RAD),
            abs(start - PROBE_TARGET_RAD),
            f"{PROBE_JOINT} moved away from the target ({start:.3f} -> {end:.3f})",
        )

    def test_frozen_joint_holds(self):
        """The freeze controller is what keeps the body up; if it were a no-op this drifts."""
        first = self._latest_position(FROZEN_JOINT)
        self.assertIsNotNone(first, "no /joint_states for the frozen joint")
        self._spin_for(3.0)
        second = self._latest_position(FROZEN_JOINT)
        self.assertLess(
            abs(second - first),
            FROZEN_MAX_DRIFT_RAD,
            f"{FROZEN_JOINT} drifted {abs(second - first):.3f} rad while frozen",
        )
