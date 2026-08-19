"""The acceptance gate: the robot reaches a navigation goal on its own.

Everything else in this package tests a part; this tests that a planner, a controller and a
balancing walking policy add up to a robot that gets somewhere.

It is also the only place the whole machine runs at once. `sensors:=true` puts the LiDAR sweep,
the relay and FAST-LIO on the same box as the 200 Hz control loop and the 50 Hz policy, which
is what the two safety assertions at the end are there to catch.
"""

import math
import os
import time
import unittest

import launch_testing
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

# Derived from maps/facility.pgm. The robot spawns at the origin of a 4x4 m crossroads, where
# every axis-aligned 4 m ray hits a partition at 2 m. This pose is 3.54 m out at -45 degrees,
# with a clear line from spawn and 1.80 m to the nearest obstacle.
GOAL_X = 2.5
GOAL_Y = -2.5

# Tilt, never the quaternion's w: yawing drives w down while the robot stands perfectly straight.
# This is the world z-component of the body z-axis, 1.0 upright and 0.0 on its side.
MIN_UPRIGHT_Z = 0.64

BRINGUP_TIMEOUT_S = 180.0
GOAL_TIMEOUT_S = 180.0


@pytest.mark.launch_test
def generate_test_description():
    return (
        LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("g1_navigation"),
                        "launch",
                        "nav_sim.launch.py",
                    )
                ),
                launch_arguments={
                    "mode": "localization",
                    "nav": "true",
                    "headless": "true",
                    "rviz": "false",
                }.items(),
            ),
            TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]),
        {},
    )


class NavigateToPoseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("navigate_to_pose_test")
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.node)
        cls.client = ActionClient(cls.node, NavigateToPose, "navigate_to_pose")
        cls.controllers = cls.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        cls.nav_active = cls.node.create_client(
            Trigger, "/lifecycle_manager_navigation/is_active"
        )

        cls.imu = []
        cls.node.create_subscription(
            Imu,
            "/imu_sensor_broadcaster/imu",
            lambda msg: cls.imu.append(msg),
            QoSProfile(depth=10),
        )
        cls.costmaps = []
        cls.node.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            lambda msg: cls.costmaps.append(msg),
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # The whole stack has to be up: the sim, the control stack, the scan pipeline and AMCL.
        cls.ready = cls.client.wait_for_server(timeout_sec=BRINGUP_TIMEOUT_S)
        cls.tf_ready = False
        deadline = time.time() + 60.0
        while time.time() < deadline and not cls.tf_ready:
            try:
                cls.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                cls.tf_ready = True
            except Exception:
                rclpy.spin_once(cls.node, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def pose_in_map(self):
        t = self.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y

    def controller_states(self):
        self.assertTrue(
            self.controllers.wait_for_service(timeout_sec=30.0), "no list_controllers service"
        )
        future = self.controllers.call_async(ListControllers.Request())
        deadline = time.time() + 20.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(future.done(), "list_controllers did not answer")
        return {c.name: c.state for c in future.result().controller}

    def uprightness(self):
        end = time.time() + 20.0
        while not self.imu and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.imu, "no IMU messages on /imu_sensor_broadcaster/imu")
        q = self.imu[-1].orientation
        return 1.0 - (2.0 * ((q.x * q.x) + (q.y * q.y)))

    def test_reaches_the_goal(self):
        self.assertTrue(self.ready, "navigate_to_pose action server never appeared")
        # Without this the run still proceeds and fails later at pose_in_map(), reported as a
        # navigation failure rather than as the localization problem it actually is.
        self.assertTrue(self.tf_ready, "map -> base_footprint never became available")

        # Goals are accepted well before the map is rasterised, and one planned against an empty
        # global costmap makes the BT loop without ever returning a result. Global, not local:
        # the local one is a 3 m rolling window and is legitimately empty at spawn.
        deadline = time.time() + 60.0
        populated = False
        while time.time() < deadline and not populated:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.costmaps:
                # OccupancyGrid is int8 on the 0-100 scale, NOT the costmap's internal 0-255.
                # Thresholding at 253 reports an empty costmap on a perfectly healthy one.
                populated = any(v > 0 for v in self.costmaps[-1].data)
        self.assertTrue(populated, "the global costmap never loaded the static map")

        upright_before = self.uprightness()
        self.assertGreater(upright_before, MIN_UPRIGHT_Z, "the robot was already down")

        # Active, not merely present: bt_navigator is near the end of the lifecycle manager's
        # ordered activation and rejects goals until it gets there.
        self.assertTrue(
            self.nav_active.wait_for_service(timeout_sec=60.0),
            "no lifecycle_manager_navigation/is_active service",
        )
        deadline = time.time() + 90.0
        active = False
        while time.time() < deadline and not active:
            future = self.nav_active.call_async(Trigger.Request())
            inner = time.time() + 10.0
            while not future.done() and time.time() < inner:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            active = future.done() and future.result() is not None and future.result().success
        self.assertTrue(active, "the navigation lifecycle never reported active")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = GOAL_X
        goal.pose.pose.position.y = GOAL_Y
        goal.pose.pose.orientation.w = 1.0

        send = self.client.send_goal_async(goal)
        deadline = time.time() + 30.0
        while not send.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(send.done(), "the goal was never acknowledged")
        handle = send.result()
        self.assertTrue(handle.accepted, "bt_navigator rejected the goal")

        result_future = handle.get_result_async()
        deadline = time.time() + GOAL_TIMEOUT_S
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(result_future.done(), f"no result within {GOAL_TIMEOUT_S:.0f}s")
        self.assertEqual(
            result_future.result().status,
            GoalStatus.STATUS_SUCCEEDED,
            "NavigateToPose did not succeed",
        )

        x, y = self.pose_in_map()
        error = math.hypot(x - GOAL_X, y - GOAL_Y)
        # config/nav2_params.yaml's xy_goal_tolerance, with margin for the distance the gait
        # coasts after the goal checker fires.
        self.assertLess(
            error, 0.8, f"succeeded but stopped {error:.2f} m from the goal, at ({x:.2f}, {y:.2f})"
        )

    def test_the_policy_carried_the_robot_the_whole_way(self):
        # Runs after the goal, and this ordering is the point: Nav2 can report success on a robot
        # the emergency freeze caught, because the freeze holds it upright and the pose still
        # arrives. Uprightness alone would not catch that either.
        upright = self.uprightness()
        self.assertGreater(
            upright, MIN_UPRIGHT_Z, f"pelvis uprightness {upright:.3f} after navigating: it fell"
        )

        states = self.controller_states()
        self.assertEqual(
            states.get("locomotion_safety_controller"),
            "active",
            "the safety controller is not the one writing the joints any more",
        )
        self.assertEqual(
            states.get("locomotion_freeze_controller"),
            "inactive",
            "the emergency freeze took over during navigation, which with the perception stack "
            "sharing this machine most likely means the 200 Hz loop overran",
        )
