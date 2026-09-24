"""Headless sim integration test: the gate executes what is safe and refuses what is not.

Proves the checks are wired to a real planning scene and real controllers, and that a refusal
happens before the arm moves. The refusal case matters more: a gate that never rejects passes a
happy-path test while protecting nothing.

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
# and a delayed acquire.
STACK_SETTLE_S = 55.0
READY_TIMEOUT_S = STACK_SETTLE_S + 30.0

# Short on purpose: nothing here grasps the block, so both cases end on this or on the rejection
# limit.
GOAL_TIMEOUT_S = 20.0
MAX_REJECTED = 5

WATCHED = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_joint"]
# Shoulder roll in opposite directions, so each case turns on self-collision rather than on how
# the octomap filled in. The arm meets the torso from about roll 0.1.
BLOCKED_TARGET = [0.06, 0.6, 0.09]
FREE_TARGET = [0.0, -0.55, 0.3]


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
            # FAST-LIO publishes no odom here: with the pelvis pinned at arm's length from the
            # bench, the Mid360 returns nothing.
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
        goal.instruction = "pick up the red block"
        goal.object_id = "red_block"
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

        # So the free-space case ends on the goal's timeout, not the suite's.
        self._set_params(
            self.server_params,
            [
                Parameter("timeout_s", Parameter.Type.DOUBLE, GOAL_TIMEOUT_S),
                Parameter("max_rejected_chunks", Parameter.Type.INTEGER, MAX_REJECTED),
            ],
        )

    def test_03_valid_chunks_reach_the_controllers(self):
        self._set_params(
            self.engine_params,
            [Parameter("target_positions", Parameter.Type.DOUBLE_ARRAY, FREE_TARGET)],
        )
        self._spin(3.0)
        before = self._watched()
        result = self._run_grasp(GOAL_TIMEOUT_S + 60.0)

        # Never a success, since the mock grasps nothing; what is tested is the path in between.
        self.assertIn("executed", result.message)
        self.assertIn(
            "0 rejected", result.message, f"a free-space walk was refused: {result.message}"
        )
        self.assertNotIn("[0 executed", result.message, f"nothing ran: {result.message}")

        self._spin(2.0)
        moved = max(abs(a - b) for a, b in zip(self._watched(), before, strict=True))
        self.assertGreater(moved, 0.05, "the arm did not move for chunks that passed the gate")

    def test_02_a_blocked_chunk_is_refused_before_the_arm_moves(self):
        # Runs first, from the rest pose: the very first chunk already reaches the torso, so
        # nothing legitimate can execute before the refusal.
        self._set_params(
            self.engine_params,
            [Parameter("target_positions", Parameter.Type.DOUBLE_ARRAY, BLOCKED_TARGET)],
        )
        self._spin(3.0)
        before = self._watched()

        result = self._run_grasp(GOAL_TIMEOUT_S + 60.0)

        self.assertFalse(result.success, result.message)
        self.assertTrue(
            result.message.startswith("blocked:"),
            f"expected a blocked abort, got: {result.message}",
        )
        self.assertIn("in collision", result.message)
        self.assertIn(f"[0 executed, {MAX_REJECTED} rejected]", result.message)

        # Refused before moving. 0.1 rad rather than zero because the soft position gain leaves
        # the measured pose a few hundredths behind; one executed chunk would move 0.24 rad.
        self._spin(2.0)
        moved = max(abs(a - b) for a, b in zip(self._watched(), before, strict=True))
        self.assertLess(moved, 0.1, f"the arm moved {moved:.3f} rad on a refused chunk")


# No post-shutdown exit-code check: move_group segfaults in its own destructor on this MoveIt and
# ros2_control_node exits 130 on SIGINT.
