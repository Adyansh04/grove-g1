"""The octomap has to actually stop a plan, not merely exist.

A non-empty octomap proves the sensor is wired up. It does not prove MoveIt is collision
checking against it, and those are separate failures: the updater can be filling a map that the
planning scene never consults, and everything looks healthy.

So this drives a hand at `reach_obstacle`, the box g1_bringup's perception scene puts 0.42 m in
front of the chest, and asserts the state is rejected as colliding, then asserts the same
state is fine with sensors off, which is the control. Without that second half the test would
pass just as well against a pose that self-collides.

pin_pelvis because the octomap lands in the planning frame, `pelvis`, and a pelvis that moves
drags the voxel grid with it (config/sensors_3d.yaml). Pinned, the map is stationary and the
result is deterministic.
"""

import os
import sys
import unittest

import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_msgs.msg import PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Matches test_moveit_plan_execute. Longer than this and launch_testing's own startup timeout
# fires first, killing the whole launch with "Timed out waiting for processes to start up"
# and every test reports "Launch stopped before the active tests finished", which looks like a
# stack failure and is not one. The octomap needs far longer than 12 s to fill; that wait
# belongs inside the tests, where the timeout is ours to choose.
SIM_SETTLE_S = 12.0
ARM_JOINTS = [
    f"{side}_{joint}"
    for side in ("left", "right")
    for joint in (
        "shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint", "elbow_joint",
        "wrist_roll_joint", "wrist_pitch_joint", "wrist_yaw_joint",
    )
]

# Reaches the right hand forward into where reach_obstacle sits. Chosen by measurement, not by
# eye: test_octomap_blocks_a_plan fails loudly if it turns out to be self-colliding with
# sensors off, which is exactly what the control assertion is for.
REACH_INTO_BOX = {
    "right_shoulder_pitch_joint": -1.45,
    "right_shoulder_roll_joint": -0.10,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.10,
}


def _stack(sensors):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("g1_moveit_config"), "launch", "moveit_sim.launch.py"
            )
        ),
        launch_arguments={
            "sensors": sensors,
            "pin_pelvis": "true",
            "world": "perception",
            "headless": "true",
        }.items(),
    )


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        _stack("true"),
        TimerAction(period=SIM_SETTLE_S, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestOctomapBlocksAPlan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("octomap_block_probe")
        cls.joints = {}
        cls.node.create_subscription(JointState, "/joint_states", cls._joints_cb, 20)
        cls.validity = cls.node.create_client(GetStateValidity, "/check_state_validity")
        cls.scene = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        cls.fk = cls.node.create_client(GetPositionFK, "/compute_fk")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _joints_cb(cls, msg):
        for name, position in zip(msg.name, msg.position, strict=False):
            cls.joints[name] = position

    def _spin_until(self, predicate, timeout_s, message):
        end = self.node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(message)

    def _validity(self, overrides):
        state = JointState()
        for name, position in self.joints.items():
            state.name.append(name)
            state.position.append(overrides.get(name, position))
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = state
        request.robot_state.is_diff = False
        request.group_name = "right_arm"
        future = self.validity.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), "/check_state_validity never answered")
        return future.result()

    def test_01_the_octomap_is_populated(self):
        self._spin_until(lambda: len(self.joints) >= 43, 90.0, "joint states never arrived")
        self.assertTrue(self.scene.wait_for_service(timeout_sec=60.0))
        self.assertTrue(self.validity.wait_for_service(timeout_sec=60.0))

        def has_octomap():
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.OCTOMAP
            future = self.scene.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
            result = future.result()
            if result is None:
                return False
            octomap = result.scene.world.octomap
            if not octomap.octomap.data:
                return False
            # The frame is the planning frame, never octomap_frame, pinned here because a
            # change would silently move every voxel relative to the robot.
            self.assertEqual(octomap.header.frame_id, "pelvis")
            return True

        self._spin_until(has_octomap, 120.0, "the octomap never filled from /livox/lidar")

    def _palm_in_pelvis(self, overrides):
        state = JointState()
        for name, position in self.joints.items():
            state.name.append(name)
            state.position.append(overrides.get(name, position))
        request = GetPositionFK.Request()
        request.header.frame_id = "pelvis"
        request.fk_link_names = ["right_hand_palm_link"]
        request.robot_state.joint_state = state
        future = self.fk.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        result = future.result()
        if result is None or not result.pose_stamped:
            return None
        return result.pose_stamped[0].pose.position

    def test_02_reaching_into_the_mapped_box_is_rejected(self):
        # Where the palm actually ends up, so a failure says whether the pose missed the slab
        # or the octomap is simply not being checked. reach_obstacle is x 0.28 +/- 0.10 and
        # z 1.25 +/- 0.25 in world; the pelvis is pinned near 0.755, so in this frame that is
        # x 0.18 to 0.38 and z 0.245 to 0.745.
        self.assertTrue(self.fk.wait_for_service(timeout_sec=30.0))
        palm = self._palm_in_pelvis(REACH_INTO_BOX)
        self.assertIsNotNone(palm, "/compute_fk gave no pose")
        inside = 0.18 <= palm.x <= 0.38 and abs(palm.y) <= 0.45 and 0.245 <= palm.z <= 0.745
        self.node.get_logger().info(
            f"PALM AT x={palm.x:.3f} y={palm.y:.3f} z={palm.z:.3f} inside_slab={inside}"
        )
        self.assertTrue(
            inside,
            f"the test pose puts the palm at ({palm.x:.3f}, {palm.y:.3f}, {palm.z:.3f}), which is "
            "outside reach_obstacle -- fix the pose or the scene, this says nothing about the "
            "octomap",
        )

        result = self._validity(REACH_INTO_BOX)
        self.assertFalse(
            result.valid,
            "reaching into reach_obstacle was accepted -- the octomap exists but the planning "
            "scene is not collision checking against it",
        )
        # An octomap collision names <octomap> as one body; a self-collision names two links.
        # Without this the test would pass on a pose that merely self-collides.
        bodies = {c.contact_body_1 for c in result.contacts} | {
            c.contact_body_2 for c in result.contacts
        }
        self.assertIn(
            "<octomap>", bodies, f"rejected, but not by the octomap; contacts were {bodies}"
        )


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_no_process_died_badly(self, proc_info):
        # 130 is SIGINT through the shell wrapper control.launch.py uses; -11 is move_group's
        # own teardown segfault in MoveItCpp's destructor, which happens after every run and is
        # not this test's business.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, 130, -2, -6, -9, -11, -15]
        )


if __name__ == "__main__":
    sys.exit(unittest.main())
