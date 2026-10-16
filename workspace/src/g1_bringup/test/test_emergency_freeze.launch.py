"""The safety controller's emergency switch hands the legs to the freeze, the policy with it.

The overrides tighten the divergence limit so the real detector fires as the policy blends in.
The switch it sends is the one a fall sends, through the real chain: ros2_control refuses to
deactivate the safety controller while the policy chained above it stays active.
"""

import os
import time
import unittest

import launch_testing.actions
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

OVERRIDES = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "emergency_freeze_overrides.yaml"
)
CONTROLLERS_YAML = os.path.join(
    get_package_share_directory("g1_controllers"), "config", "lowcmd_controllers.yaml"
)

# The simulator's start and the spawners; a refused switch, such as one sent before the freeze
# has loaded, is retried each second, so the handover itself lands soon after.
SWITCH_TIMEOUT_S = 90.0

COMMAND_INTERFACES = ("position", "velocity", "effort", "kp", "kd")
OTHER_OWNERS = ("waist_freeze_controller", "arm_freeze_controller")
SETTLED = ("locomotion_freeze_controller", *OTHER_OWNERS)


def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "pin_pelvis": "false",
            "sensors": "false",
            "controllers_overrides": OVERRIDES,
        }.items(),
    )
    return LaunchDescription([sim_launch, launch_testing.actions.ReadyToTest()])


class TestEmergencyFreeze(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_emergency_freeze")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        with open(CONTROLLERS_YAML) as handle:
            config = yaml.safe_load(handle)
        cls.legs = config["locomotion_freeze_controller"]["ros__parameters"]["joints"]

        # Waits for the handover and for the arm and waist spawners, which the emergency can
        # beat, rather than a fixed time, so every case sees the settled outcome.
        cls.states = {}
        deadline = time.monotonic() + SWITCH_TIMEOUT_S
        while time.monotonic() < deadline:
            cls.states = cls._controller_states()
            if all(cls.states.get(name) == "active" for name in SETTLED):
                break
            cls._spin_for(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _spin_for(cls, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            cls.executor.spin_once(timeout_sec=0.05)

    @classmethod
    def _call(cls, srv_type, name):
        """The response, or None if the service is missing or silent."""
        client = cls.node.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=20.0):
                return None
            future = client.call_async(srv_type.Request())
            deadline = time.monotonic() + 20.0
            while not future.done() and time.monotonic() < deadline:
                cls.executor.spin_once(timeout_sec=0.1)
            return future.result() if future.done() else None
        finally:
            cls.node.destroy_client(client)

    @classmethod
    def _controller_states(cls):
        result = cls._call(ListControllers, "/controller_manager/list_controllers")
        return {} if result is None else {c.name: c.state for c in result.controller}

    def test_the_freeze_takes_over_from_the_policy(self):
        self.assertEqual(
            self.states.get("locomotion_freeze_controller"),
            "active",
            f"the freeze never took over, so the safety controller holds the legs: {self.states}",
        )
        for name in ("locomotion_safety_controller", "agile_controller"):
            self.assertEqual(
                self.states.get(name),
                "inactive",
                f"{name} is {self.states.get(name)} after the emergency switch",
            )

    def test_every_leg_joint_stays_claimed(self):
        """The component leaves an unclaimed joint unpowered, so a half-done swap shows here."""
        result = self._call(ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces")
        self.assertIsNotNone(result, "list_hardware_interfaces did not answer")
        claimed = {
            interface.name for interface in result.command_interfaces if interface.is_claimed
        }
        self.assertEqual(len(self.legs), 14)
        missing = [
            f"{joint}/{kind}"
            for joint in self.legs
            for kind in COMMAND_INTERFACES
            if f"{joint}/{kind}" not in claimed
        ]
        self.assertFalse(missing, f"unclaimed, so unpowered: {missing}")

    def test_the_arms_and_waist_keep_their_owners(self):
        """The switch claims the legs and no more."""
        for name in OTHER_OWNERS:
            self.assertEqual(self.states.get(name), "active", f"{name} is {self.states.get(name)}")
