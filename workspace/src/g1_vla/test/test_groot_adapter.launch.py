"""The adapter against a stub policy server: wire protocol, key mapping and action integration.

Each fails quietly when wrong, so the stub answers the real wire format with numbers checkable by
hand. It declares the checkpoint's own key set and rejects an observation that does not match,
so the adapter only answers if it mapped every state and video key and ignored the action keys it
does not drive. No simulator and no GPU.
"""

import os
import sys
import unittest

import launch_testing
import pytest
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from rclpy.node import Node as RclpyNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from g1_msgs.srv import GetActionChunk

PORT = 5599
HORIZON = 6
DELTA = 0.02

ARMS = {
    side: [
        f"{side}_shoulder_pitch_joint",
        f"{side}_shoulder_roll_joint",
        f"{side}_shoulder_yaw_joint",
        f"{side}_elbow_joint",
        f"{side}_wrist_roll_joint",
        f"{side}_wrist_pitch_joint",
        f"{side}_wrist_yaw_joint",
    ]
    for side in ("left", "right")
}
HANDS = {
    side: [
        f"{side}_hand_thumb_0_joint",
        f"{side}_hand_thumb_1_joint",
        f"{side}_hand_thumb_2_joint",
        f"{side}_hand_index_0_joint",
        f"{side}_hand_index_1_joint",
        f"{side}_hand_middle_0_joint",
        f"{side}_hand_middle_1_joint",
    ]
    for side in ("left", "right")
}
WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]

COMMANDED = ARMS["right"] + HANDS["right"]
ALL_JOINTS = ARMS["left"] + ARMS["right"] + HANDS["left"] + HANDS["right"] + WAIST
# A different number on every joint, so a mapping that is transposed within a key reads as a
# wrong position rather than as a coincidence.
MEASURED = {name: 0.01 * (index + 1) for index, name in enumerate(ALL_JOINTS)}
CAMERA_TOPIC = "/camera/color/image_raw"

STUB = os.path.join(os.path.dirname(__file__), "policy_server_stub.py")


@pytest.mark.launch_test
def generate_test_description():
    adapter = Node(
        package="g1_vla",
        executable="g1_vla_groot_adapter",
        name="g1_vla_groot_adapter",
        output="screen",
        parameters=[
            {
                "server_address": f"tcp://127.0.0.1:{PORT}",
                "zmq_timeout_ms": 5000,
                "action_dt_s": 0.1,
                "max_horizon": HORIZON,
                "action_mode": "absolute",
                # 20 steps of history at this spacing is a second of frames, which the probe
                # publishes well before it asks for anything.
                "history_step_s": 0.05,
                "state_joints": {
                    "left_arm": ARMS["left"],
                    "right_arm": ARMS["right"],
                    "left_hand": HANDS["left"],
                    "right_hand": HANDS["right"],
                    "waist": WAIST,
                },
                "state_eef_frames": {
                    "left_wrist_eef_9d": ["pelvis", "left_wrist_yaw_link"],
                    "right_wrist_eef_9d": ["pelvis", "right_wrist_yaw_link"],
                },
                "action_joints": {"right_arm": ARMS["right"], "right_hand": HANDS["right"]},
                "video_topics": {"ego_view": CAMERA_TOPIC},
            }
        ],
    )
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[sys.executable, STUB, str(PORT), str(HORIZON), str(DELTA)],
                output="screen",
            ),
            # The adapter handshakes in its constructor, so the stub has to be bound first.
            TimerAction(period=3.0, actions=[adapter]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGrootAdapter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("groot_adapter_probe")
        cls.joints = cls.node.create_publisher(JointState, "/joint_states", 1)
        cls.camera = cls.node.create_publisher(Image, CAMERA_TOPIC, qos_profile_sensor_data)
        cls.client = cls.node.create_client(
            GetActionChunk, "/g1_vla_groot_adapter/get_action_chunk"
        )

        # Wrist poses for the model's 9-D end-effector state. Only that they resolve matters here.
        cls.tf = tf2_ros.StaticTransformBroadcaster(cls.node)
        wrists = []
        for side in ("left", "right"):
            wrist = TransformStamped()
            wrist.header.stamp = cls.node.get_clock().now().to_msg()
            wrist.header.frame_id = "pelvis"
            wrist.child_frame_id = f"{side}_wrist_yaw_link"
            wrist.transform.translation.x = 0.2
            wrist.transform.rotation.w = 1.0
            wrists.append(wrist)
        cls.tf.sendTransform(wrists)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish_observation(self):
        state = JointState()
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.name = list(MEASURED)
        state.position = [MEASURED[name] for name in state.name]
        self.joints.publish(state)

        image = Image()
        image.header.stamp = self.node.get_clock().now().to_msg()
        image.height, image.width = 4, 4
        image.encoding = "rgb8"
        image.step = image.width * 3
        image.data = bytes(image.height * image.step)
        self.camera.publish(image)

    def _call(self, instruction):
        request = GetActionChunk.Request()
        request.instruction = instruction
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), "the adapter never answered")
        return future.result()

    def test_01_the_adapter_serves_the_chunk_service(self):
        self.assertTrue(
            self.client.wait_for_service(timeout_sec=60.0),
            "the adapter did not come up; check its handshake against the stub",
        )

    def test_02_a_chunk_comes_back_shaped_correctly(self):
        # Republished on every attempt: the adapter keeps only the latest of each.
        for _ in range(30):
            self._publish_observation()
            rclpy.spin_once(self.node, timeout_sec=0.1)
        result = self._call("pick up the red block")

        self.assertTrue(result.ok, result.message)
        # Only the joints this skill drives, not the waist, base height or navigation keys.
        self.assertEqual(list(result.chunk.joint_names), COMMANDED)
        self.assertEqual(len(result.chunk.points), HORIZON)
        # Waypoint times advance, as the gate's shape check demands.
        times = [
            p.time_from_start.sec + p.time_from_start.nanosec / 1e9 for p in result.chunk.points
        ]
        self.assertEqual(times, sorted(set(times)))
        self.assertAlmostEqual(times[0], 0.1, places=6)

    def test_03_absolute_actions_reach_the_chunk_unchanged(self):
        for _ in range(30):
            self._publish_observation()
            rclpy.spin_once(self.node, timeout_sec=0.1)
        result = self._call("pick up the red block")

        self.assertTrue(result.ok, result.message)
        # The stub answers DELTA above the state it was sent, in absolute units; adding the
        # measured pose again would double the angle.
        for point in result.chunk.points:
            for index, joint in enumerate(COMMANDED):
                self.assertAlmostEqual(point.positions[index], MEASURED[joint] + DELTA, places=6)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_the_adapter_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
