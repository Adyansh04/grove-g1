"""Headless sim gate for the AGILE policy on the rt/lowcmd stack: stand, then walk.

The pelvis is not pinned: the robot stands because the policy is balancing it. The simulator
holds it up only until the control stack drives every motor, then releases its weld and never
re-applies it.

Displacement is deliberately not asserted here, because this runs with sensors:=false and the
relay carrying the simulator's ground truth into ROS is not up. The gait envelope is measured
separately against MuJoCo; see the g1_controllers README.
"""

import os
import statistics
import time
import unittest

import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

# Kept under launch_testing's 15 s startup deadline, then the policy needs a moment to settle.
SIM_SETTLE_S = 12.0

# Upright is measured as tilt, never as the quaternion's w: yawing drives w down while the robot
# stands perfectly straight, so a w threshold fails the moment anything turns. This is the world
# z-component of the body's own z-axis, 1.0 straight up and 0.0 on its side. 0.64 is about 50
# degrees of lean, far more than walking needs and far less than a fall.
MIN_UPRIGHT_Z = 0.64

DRIVE_VX = 0.3
DRIVE_S = 10.0

# Yaw tracks near 1:1 with no deadband, so 8 s at 0.5 rad/s is an unmistakable turn.
TURN_WZ = 0.5
TURN_S = 8.0

# Knee travel while walking versus standing. Standing is near-static, a gait swings the knee
# through a good fraction of a radian, so an order of magnitude separates them.
GAIT_KNEE_STDDEV_RAD = 0.03
GAIT_JOINT = "left_knee_joint"


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
        }.items(),
    )

    return LaunchDescription([sim_launch, launch_testing.actions.ReadyToTest()])


class TestAgileWalk(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_agile_walk")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

        cls.imu = []
        cls.joint_states = []
        cls.inferring = []
        cls.node.create_subscription(
            Imu, "/imu_sensor_broadcaster/imu", lambda msg: cls.imu.append(msg), 10
        )
        cls.node.create_subscription(
            JointState, "/joint_states", lambda msg: cls.joint_states.append(msg), 50
        )
        cls.node.create_subscription(
            Bool,
            "/agile_controller/inferring",
            lambda msg: cls.inferring.append(msg),
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            ),
        )
        cls.cmd_vel = cls.node.create_publisher(Twist, "/cmd_vel", 10)

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
            cls.executor.spin_once(timeout_sec=0.05)

    def _drive(self, seconds, vx=0.0, wz=0.0):
        """Publishes at 20 Hz throughout, so the controller's cmd_vel timeout never trips."""
        command = Twist()
        command.linear.x = vx
        command.angular.z = wz
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.cmd_vel.publish(command)
            self.executor.spin_once(timeout_sec=0.05)

    @staticmethod
    def _uprightness(orientation):
        """World z-component of the body z-axis, from the rotation matrix's lower-right term."""
        return 1.0 - (2.0 * ((orientation.x * orientation.x) + (orientation.y * orientation.y)))

    def _joint_series(self, joint, since_index):
        values = []
        for msg in self.joint_states[since_index:]:
            if joint in msg.name:
                values.append(msg.position[msg.name.index(joint)])
        return values

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
        """A broken on_init or a failed SDK channel shows up here and nowhere else."""
        result = self._call(ListHardwareComponents, "/controller_manager/list_hardware_components")
        names = {component.name: component.state.label for component in result.component}
        self.assertIn("G1LowCmdSystem", names, f"component never loaded, saw {names}")
        self.assertEqual(names["G1LowCmdSystem"], "active")

    def test_controllers_are_active(self):
        result = self._call(ListControllers, "/controller_manager/list_controllers")
        states = {controller.name: controller.state for controller in result.controller}
        for name in (
            "joint_state_broadcaster",
            "imu_sensor_broadcaster",
            "waist_freeze_controller",
            "arm_freeze_controller",
            "locomotion_safety_controller",
            "agile_controller",
        ):
            self.assertEqual(states.get(name), "active", f"{name} is {states.get(name)}")

    def test_policy_is_commanding(self):
        """Lifecycle-active is not the same as producing commands; this is the latter."""
        self.assertTrue(self.inferring, "no ~/inferring message; the policy never ran")
        self.assertTrue(self.inferring[-1].data, "the policy activated but never inferred")

    def test_robot_stands_without_a_pinned_pelvis(self):
        """Nothing but the policy is holding the robot up by this point."""
        self.assertTrue(self.imu, "no IMU messages")
        upright = self._uprightness(self.imu[-1].orientation)
        self.assertGreater(
            upright,
            MIN_UPRIGHT_Z,
            f"pelvis uprightness {upright:.3f}: the robot is not upright, it has fallen",
        )

    def test_walks_on_command_and_stays_up(self):
        standing_from = len(self.joint_states)
        self._spin_for(3.0)
        standing = self._joint_series(GAIT_JOINT, standing_from)

        walking_from = len(self.joint_states)
        self._drive(DRIVE_S, vx=DRIVE_VX)
        walking = self._joint_series(GAIT_JOINT, walking_from)

        self.assertGreater(len(standing), 10, "too few standing samples")
        self.assertGreater(len(walking), 10, "too few walking samples")

        standing_spread = statistics.pstdev(standing)
        walking_spread = statistics.pstdev(walking)
        self.assertGreater(
            walking_spread,
            GAIT_KNEE_STDDEV_RAD,
            f"{GAIT_JOINT} spread {walking_spread:.4f} rad under a {DRIVE_VX} m/s command; "
            f"the robot is not stepping (standing was {standing_spread:.4f})",
        )
        self.assertGreater(
            walking_spread,
            standing_spread * 3.0,
            f"{GAIT_JOINT} moved no more walking ({walking_spread:.4f}) than standing "
            f"({standing_spread:.4f})",
        )

        # Still upright after walking, which is the assertion a fall breaks.
        self.assertGreater(
            self._uprightness(self.imu[-1].orientation),
            MIN_UPRIGHT_Z,
            "the robot fell over while walking",
        )

    def test_turns_on_command_and_stays_up(self):
        """Also the case that catches a yaw-blind uprightness check: turning drives the
        quaternion's w right down while the robot stands perfectly straight."""
        before = self.imu[-1].orientation
        self._drive(TURN_S, wz=TURN_WZ)
        after = self.imu[-1].orientation

        yawed = abs(after.z - before.z)
        self.assertGreater(
            yawed, 0.1, f"pelvis yaw barely changed ({yawed:.3f}); the robot did not turn"
        )
        self.assertGreater(
            self._uprightness(after),
            MIN_UPRIGHT_Z,
            "the robot fell over while turning",
        )

    def test_the_emergency_target_is_loaded_and_ready(self):
        """An emergency can only switch to a controller that is already loaded, and this one is
        never active in a healthy run, so nothing else would notice it missing."""
        result = self._call(ListControllers, "/controller_manager/list_controllers")
        states = {controller.name: controller.state for controller in result.controller}
        self.assertEqual(
            states.get("locomotion_freeze_controller"),
            "inactive",
            f"the safety controller's emergency target is {states.get('locomotion_freeze_controller')}, "
            "so a divergence would leave the legs with no controller at all",
        )

    def test_safety_controller_never_latched(self):
        """The out-of-domain detector switches to the freeze on divergence, so if it is still
        active the policy stayed inside the range it was trained in for the whole run."""
        result = self._call(ListControllers, "/controller_manager/list_controllers")
        states = {controller.name: controller.state for controller in result.controller}
        self.assertEqual(
            states.get("locomotion_safety_controller"),
            "active",
            "the safety controller latched its emergency and handed over to the freeze",
        )
        self.assertNotEqual(
            states.get("locomotion_freeze_controller"),
            "active",
            "the emergency freeze took over, so the policy diverged at some point",
        )
