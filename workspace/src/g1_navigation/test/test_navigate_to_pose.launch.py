"""The acceptance gate: the robot reaches a navigation goal on its own.

Everything else in this package tests a part. This tests the claim the milestone is actually
making -- that a planner, a controller, a gait with three usable motions and a locomotion
authority bracket add up to a robot that gets somewhere.

EXPECT THIS TO FAIL OCCASIONALLY, for reasons outside this package. Roughly 1 in 8 fresh
launches produces a robot that reports fully healthy -- authority HELD, no errors -- and simply
does not walk. Nav2 cannot fix that: Spin commands
motion the robot ignores, Wait waits, ClearCostmap clears a costmap that was never the problem.
Re-run the suite alone before treating a red run as a regression, per the package README. There
is deliberately no retry wrapper here: retrying would hide exactly that number.
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
from g1_msgs.msg import LocoStatus
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

# Derived from maps/facility.pgm, not guessed. The robot spawns at the origin, in the middle of
# the facility's 4x4 m crossroads. Every axis-aligned 4 m ray from there hits a partition at 2 m,
# so the plan's original "4 m ahead" is inside a wall. This pose is 3.54 m out at exactly -45
# degrees, with a clear straight line from spawn and 1.80 m to the nearest obstacle -- which also
# makes it the intended decomposition: rotate in place, then one straight run.
GOAL_X = 2.5
GOAL_Y = -2.5

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
        cls.status = []
        cls.node.create_subscription(
            LocoStatus,
            "/g1_loco_bridge/status",
            lambda msg: cls.status.append(msg),
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.client = ActionClient(cls.node, NavigateToPose, "navigate_to_pose")
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

        # The whole stack has to be up: the sim, the scan pipeline, AMCL, and the acquire, which
        # by itself is two FSM goals plus a settle.
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

    def test_reaches_the_goal(self):
        self.assertTrue(self.ready, "navigate_to_pose action server never appeared")
        # Without this the run still proceeds and fails later at pose_in_map(), reported as a
        # navigation failure rather than as the localization problem it actually is.
        self.assertTrue(self.tf_ready, "map -> base_footprint never became available")

        # Authority must already be held: the launch's lifecycle handlers acquire it, and if they
        # did not, Nav2 would publish into a bridge that discards everything and the goal would
        # fail for a reason that has nothing to do with navigation.
        self.spin(2.0)
        self.assertTrue(self.status, "no LocoStatus received")
        self.assertEqual(
            self.status[-1].authority,
            LocoStatus.HELD,
            "locomotion authority was not acquired before the goal",
        )
        ignored_before = self.status[-1].ignored_cmd_vel

        # Wait for the GLOBAL costmap to carry the static map before asking for a plan. The
        # action server accepts goals as soon as bt_navigator is active, which is well before
        # the map has been rasterised into the costmap, and a goal planned against an empty
        # global costmap makes the BT loop without ever returning a result.
        #
        # Global, not local: the local one is a 3 m rolling window and at spawn the nearest wall
        # is further away than that, so it is legitimately empty and asserting on it fails a
        # perfectly healthy stack.
        deadline = time.time() + 60.0
        populated = False
        while time.time() < deadline and not populated:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.costmaps:
                # OccupancyGrid is int8 on the 0-100 scale, NOT the costmap's internal 0-255.
                # Thresholding at 253 reports an empty costmap on a perfectly healthy one.
                populated = any(v > 0 for v in self.costmaps[-1].data)
        self.assertTrue(populated, "the global costmap never loaded the static map")

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
        self.assertTrue(
            result_future.done(),
            f"no result within {GOAL_TIMEOUT_S:.0f}s. If the robot never moved at all, see this "
            f"file's docstring before assuming a regression.",
        )
        self.assertEqual(
            result_future.result().status,
            GoalStatus.STATUS_SUCCEEDED,
            "NavigateToPose did not succeed",
        )

        x, y = self.pose_in_map()
        error = math.hypot(x - GOAL_X, y - GOAL_Y)
        # config/nav2_params.yaml's xy_goal_tolerance, with margin for the settling distance a
        # gait with no approach deceleration covers after the checker fires.
        self.assertLess(
            error, 0.8, f"succeeded but stopped {error:.2f} m from the goal, at ({x:.2f}, {y:.2f})"
        )

        # Nothing was discarded for lack of authority across the whole run. Nav2's own zero
        # Twists never count, so this staying flat is a real statement that the bracket held.
        # 3 s, not 1: the counter only moves when the shaper publishes, and after the goal
        # succeeds it publishes zeros, which the bridge does not count. One second is inside the
        # window where a late discard has not been reported yet.
        self.spin(3.0)
        self.assertEqual(
            self.status[-1].ignored_cmd_vel,
            ignored_before,
            "the bridge discarded commands mid-run; authority was lost while navigating",
        )
