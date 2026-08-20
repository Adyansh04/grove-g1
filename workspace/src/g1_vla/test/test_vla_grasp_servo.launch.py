"""Headless sim integration test: the same gate, executing through MoveIt Servo.

test_vla_grasp_mock covers the trajectory path. This covers what the servo path adds and what it
gives up.

What it does NOT assert is that the arm follows the validated waypoints. It does not: jog
commands are integrated by the servo, which tracks velocity and never position, and the arm keeps
moving for up to incoming_command_timeout after a chunk's stream ends. The validated positions
are a reference the arm is steered toward. Asserting otherwise here would be encoding a promise
this execution mode does not make.

Servo brings its own collision monitor on top, which decelerates during motion rather than
refusing beforehand. The last case pins that separately, because a servo whose monitor is
misconfigured looks identical to one that works right up until something moves into the arm.

Run via `colcon test --packages-select g1_vla`.
"""

import os
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointJog
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from g1_msgs.action import Grasp

STACK_SETTLE_S = 55.0
READY_TIMEOUT_S = STACK_SETTLE_S + 30.0
GOAL_TIMEOUT_S = 8.0
MAX_REJECTED = 5

WATCHED = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_joint"]
# Same measured self-collision the trajectory suite uses: the arm meets the torso from about
# roll 0.1, and that is unaffected by the hand's octomap exemption.
BLOCKED_TARGET = [0.06, 0.6, 0.09]


@launch_testing.ready_to_test_action_timeout(READY_TIMEOUT_S)
def generate_test_description():
    bringup = get_package_share_directory("g1_bringup")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup.launch.py")),
        launch_arguments={
            "moveit": "true",
            "manipulation": "true",
            "vla": "true",
            "vla_engine": "mock",
            "vla_execution_mode": "servo",
            "odometry": "ground_truth",
            "world": "manipulation",
            "pin_pelvis": "true",
            "activate_arm": "true",
            "activate_arm_delay_s": "40.0",
            "headless": "true",
            "rviz": "false",
        }.items(),
    )
    return LaunchDescription(
        [stack, TimerAction(period=STACK_SETTLE_S, actions=[launch_testing.actions.ReadyToTest()])]
    )


class TestVlaGraspServo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_vla_grasp_servo")
        cls.joints = {}
        cls.jogs = 0

        def _on_joints(msg):
            for name, position in zip(msg.name, msg.position, strict=False):
                cls.joints[name] = position

        def _on_jog(_msg):
            cls.jogs += 1

        cls.node.create_subscription(JointState, "/joint_states", _on_joints, 20)
        cls.node.create_subscription(JointJog, "/servo_node/delta_joint_cmds", _on_jog, 50)
        cls.grasp = ActionClient(cls.node, Grasp, "/g1_vla_server/grasp")
        cls.server_params = cls.node.create_client(SetParameters, "/g1_vla_server/set_parameters")
        cls.engine_params = cls.node.create_client(
            SetParameters, "/g1_vla_mock_engine/set_parameters"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _set_params(self, client, parameters):
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), client.srv_name)
        request = SetParameters.Request()
        request.parameters = [p.to_parameter_msg() for p in parameters]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), f"{client.srv_name} never answered")
        for result in future.result().results:
            self.assertTrue(result.successful, result.reason)

    def _watched(self):
        return [self.joints[name] for name in WATCHED]

    def _run_grasp(self, timeout_s):
        goal = Grasp.Goal()
        goal.instruction = "pick up the red cube"
        goal.object_id = "red_cube"
        goal.arm = "right"
        handle_future = self.grasp.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, handle_future, timeout_sec=30.0)
        handle = handle_future.result()
        self.assertIsNotNone(handle, "the grasp server never answered")
        self.assertTrue(handle.accepted, "the grasp server rejected the goal")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_s)
        self.assertIsNotNone(result_future.result(), "the grasp goal never finished")
        return result_future.result().result

    def test_01_the_stack_is_up_with_servo(self):
        self.assertTrue(
            self.grasp.wait_for_server(timeout_sec=90.0), "/g1_vla_server/grasp never appeared"
        )
        end = time.time() + 60.0
        while time.time() < end and len(self.joints) < 40:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertGreaterEqual(len(self.joints), 40, "joint states never arrived")
        self._set_params(
            self.server_params,
            [
                Parameter("timeout_s", Parameter.Type.DOUBLE, GOAL_TIMEOUT_S),
                Parameter("max_rejected_chunks", Parameter.Type.INTEGER, MAX_REJECTED),
            ],
        )

    def test_02_validated_chunks_are_streamed_as_jog_commands(self):
        before = self._watched()
        self.__class__.jogs = 0

        result = self._run_grasp(GOAL_TIMEOUT_S + 60.0)

        self.assertNotIn("[0 executed", result.message, f"nothing ran: {result.message}")
        # The backend actually in use, not just the parameter: nothing publishes here in
        # trajectory mode, so a zero count would mean the mode never took effect.
        self.assertGreater(self.jogs, 50, "no jog commands were streamed")

        self._spin(2.0)
        moved = max(abs(a - b) for a, b in zip(self._watched(), before, strict=True))
        self.assertGreater(moved, 0.05, "the arm did not move for chunks that passed the gate")

    def test_03_a_blocked_chunk_is_never_streamed(self):
        self._set_params(
            self.engine_params,
            [Parameter("target_positions", Parameter.Type.DOUBLE_ARRAY, BLOCKED_TARGET)],
        )
        # Long enough for the previous case's motion to stop entirely, servo included.
        self._spin(10.0)
        self.__class__.jogs = 0

        result = self._run_grasp(GOAL_TIMEOUT_S + 60.0)

        self.assertFalse(result.success, result.message)
        self.assertTrue(
            result.message.startswith("blocked:"),
            f"expected a blocked abort, got: {result.message}",
        )
        self.assertIn(f"{MAX_REJECTED} rejected]", result.message)
        # Direct evidence, and better than the trajectory suite can get: in servo mode nothing
        # reaches the arm except through this topic, so a zero count is proof that a refused
        # chunk was never streamed.
        self.assertEqual(self.jogs, 0, "a refused chunk was streamed anyway")


    def test_04_servo_halts_for_a_collision_while_moving(self):
        """The layer servo adds over the gate: reacting during motion, not before it.

        Driven directly rather than through a chunk. The gate would refuse this motion outright,
        which is the point: what is under test here is the second line of defence.
        """
        switch = self.node.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.assertTrue(switch.wait_for_service(timeout_sec=30.0), "servo is not running")
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.JOINT_JOG
        future = switch.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertTrue(future.result().success, "servo refused joint-jog mode")

        codes = set()
        self.node.create_subscription(
            ServoStatus, "/servo_node/status", lambda msg: codes.add(msg.code), 20
        )
        jog = self.node.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)

        self._spin(1.0)
        before = self.joints["right_shoulder_roll_joint"]
        end = time.time() + 14.0
        while time.time() < end:
            message = JointJog()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.joint_names = ["right_shoulder_roll_joint"]
            # Toward the torso, which the arm meets at about 0.1 rad.
            message.velocities = [0.25]
            message.duration = 0.02
            jog.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self._spin(2.0)

        self.assertIn(ServoStatus.HALT_FOR_COLLISION, codes, f"servo never halted; saw {codes}")
        # Stopped short of contact rather than at it: 14 s at 0.25 rad/s would be 3.5 rad if
        # nothing intervened, and the collision starts at about 0.1.
        travelled = self.joints["right_shoulder_roll_joint"] - before
        self.assertLess(travelled, 0.12, f"servo let the arm travel {travelled:.3f} rad")
