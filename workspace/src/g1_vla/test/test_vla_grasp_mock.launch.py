"""Headless sim integration test: the gate executes what is safe and refuses what is not.

The acceptance gate for this package, and the half no unit test can reach. `test_chunk_utils`
proves the arithmetic; only a running stack proves that the arithmetic is wired to a real
planning scene, to real controllers, and that a refusal happens BEFORE the arm moves rather than
after.

Both halves matter and the second one matters more. A gate that never rejects anything passes a
happy-path test perfectly while protecting nothing, so the second case aims the engine at a pose
measured to self-collide and asserts the arm is still where it started.

Run via `colcon test --packages-select g1_vla`.
"""

import os
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from g1_msgs.action import Grasp

# Same budget as the manipulation suite: simulator, move_group, the skills, the object pipeline
# and a delayed acquire behind all of it.
STACK_SETTLE_S = 55.0
READY_TIMEOUT_S = STACK_SETTLE_S + 30.0

# Short on purpose. Nothing here should ever grasp the cube, so both cases end on this or on the
# rejection limit, and the server's 90 s default would only make the suite slow.
GOAL_TIMEOUT_S = 20.0
MAX_REJECTED = 5

WATCHED = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_joint"]
# Measured against this scene: the arm self-collides with the torso from about roll 0.1, so a
# walk toward 0.6 is refused from the first chunk that reaches it.
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
            # FAST-LIO cannot work in this scene: the bench is at arm's length with the pelvis
            # pinned, so the Mid360 returns nothing and no odom is ever published.
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


class TestVlaGraspMock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_vla_grasp_mock")
        cls.joints = {}

        def _on_joints(msg):
            for name, position in zip(msg.name, msg.position, strict=False):
                cls.joints[name] = position

        cls.node.create_subscription(JointState, "/joint_states", _on_joints, 20)
        cls.grasp = ActionClient(cls.node, Grasp, "/g1_vla_server/grasp")
        cls.server_params = cls.node.create_client(
            SetParameters, "/g1_vla_server/set_parameters"
        )
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

    def test_01_the_stack_is_up(self):
        self.assertTrue(
            self.grasp.wait_for_server(timeout_sec=90.0), "/g1_vla_server/grasp never appeared"
        )
        end = time.time() + 60.0
        while time.time() < end and len(self.joints) < 40:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertGreaterEqual(len(self.joints), 40, "joint states never arrived")

        # Shorter than the server's own default so the free-space case ends on the timeout
        # rather than on the suite's.
        self._set_params(
            self.server_params,
            [
                Parameter("timeout_s", Parameter.Type.DOUBLE, GOAL_TIMEOUT_S),
                Parameter("max_rejected_chunks", Parameter.Type.INTEGER, MAX_REJECTED),
            ],
        )

    def test_02_valid_chunks_reach_the_controllers(self):
        before = self._watched()
        result = self._run_grasp(GOAL_TIMEOUT_S + 60.0)

        # Never a success: the mock walks the arm through free space and does not grasp
        # anything, so the object never lifts. What is being tested is the path in between.
        self.assertIn("executed", result.message)
        self.assertIn("0 rejected", result.message, f"a free-space walk was refused: {result.message}")
        self.assertNotIn("[0 executed", result.message, f"nothing ran: {result.message}")

        self._spin(2.0)
        moved = max(abs(a - b) for a, b in zip(self._watched(), before, strict=True))
        self.assertGreater(moved, 0.05, "the arm did not move for chunks that passed the gate")

    def test_03_a_blocked_chunk_is_refused_before_the_arm_moves(self):
        self._set_params(
            self.engine_params,
            [Parameter("target_positions", Parameter.Type.DOUBLE_ARRAY, BLOCKED_TARGET)],
        )
        self._spin(10.0)
        before = self._watched()

        result = self._run_grasp(GOAL_TIMEOUT_S + 60.0)

        self.assertFalse(result.success, result.message)
        self.assertTrue(
            result.message.startswith("blocked:"),
            f"expected a blocked abort, got: {result.message}",
        )
        self.assertIn("in collision", result.message)
        self.assertIn(f"[0 executed, {MAX_REJECTED} rejected]", result.message)

        # The claim this whole package exists to make: refused before moving, not after.
        # The bar is 0.1 rad rather than zero because the arms run position-only on a soft gain
        # and the measured pose keeps trailing the commanded one by a few hundredths of a radian
        # after motion stops -- the same drift moveit_controllers.yaml sets a 0.05 start
        # tolerance for. One executed chunk would be 0.24 rad, so the two are not close.
        self._spin(2.0)
        moved = max(abs(a - b) for a, b in zip(self._watched(), before, strict=True))
        self.assertLess(moved, 0.1, f"the arm moved {moved:.3f} rad on a refused chunk")


# No post-shutdown exit-code check, matching the other sim suites: move_group segfaults in its
# own destructor on this MoveIt and ros2_control_node leaves 130 on SIGINT. Asserting on that
# tests their teardown, not this gate.
