"""Acceptance test: legs walking and arms moving in the same session.

The two own disjoint motor ranges (policy 0-14, /arm_sdk 15-28) through a single /lowcmd
writer, and this is the test that says so end to end: drive the robot forward through
the real LocoClient path while a FollowJointTrajectory goal runs on the arms.

Runs unwelded (pin_pelvis defaults false), so the policy is genuinely balancing the
robot while the arms move.
"""

import os
import subprocess
import time
import unittest
from collections import deque

import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from g1_msgs.action import SetLocoMode
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unitree_go.msg import SportModeState

JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
TARGET_JOINT = "left_elbow_joint"

SETTLE_TIMEOUT_S = 25.0
STAND_HEIGHT_MIN = 0.60
# Above the policy's measured 0.40 m/s gait-initiation threshold -- below it the robot correctly
# stands still and this test would assert nothing.
DRIVE_VX = 0.7
# Well inside the 1 rad/s slew clamp over the window we allow, so the arm genuinely arrives.
ARM_STEP_RAD = 0.4
ARM_SETTLE_S = 6.0
ARM_TOLERANCE_RAD = 0.15


def _best_effort_qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


@pytest.mark.launch_test
def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        )
    )
    return (
        LaunchDescription(
            [sim_launch, TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()])]
        ),
        {},
    )


class WalkAndArmTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_walk_and_arm")
        # Bounded, for the reason documented in test_walk_teleop.launch.py: unbounded
        # accumulation of high-rate state starves the simulator over a long suite.
        cls.sport_states = deque(maxlen=400)
        cls.joint_states = deque(maxlen=200)
        cls.node.create_subscription(
            SportModeState, "/sportmodestate", cls.sport_states.append, _best_effort_qos()
        )
        cls.node.create_subscription(JointState, "/joint_states", cls.joint_states.append, 10)
        cls.cmd_vel_pub = cls.node.create_publisher(
            Twist,
            "/g1_loco_bridge/cmd_vel",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        cls.mode_client = ActionClient(cls.node, SetLocoMode, "/g1_loco_bridge/set_mode")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, duration_s):
        end = time.time() + duration_s
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_until(self, predicate, timeout_s):
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _position(self):
        return self.sport_states[-1].position if self.sport_states else None

    def _height(self):
        pos = self._position()
        return pos[2] if pos is not None else None

    def _arm_position(self, joint):
        for msg in reversed(self.joint_states):
            if joint in msg.name:
                return msg.position[msg.name.index(joint)]
        return None

    def _run_ros2_run(self, executable, timeout_s=30.0):
        """Runs `ros2 run g1_bringup <executable>` while still spinning this node.

        Same reason as test_arm_command.launch.py: deactivate_arm's service call blocks for the
        whole ramp-down, and a plain blocking call would stop us observing that window.
        """
        proc = subprocess.Popen(
            ["ros2", "run", "g1_bringup", executable],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        end = time.time() + timeout_s
        while proc.poll() is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIsNotNone(proc.poll(), f"{executable} did not finish within {timeout_s} s")
        self.assertEqual(proc.returncode, 0, f"{executable} failed: {proc.communicate()[1]}")

    def _set_mode(self, fsm_id, timeout_s=10.0):
        self.assertTrue(self.mode_client.wait_for_server(timeout_sec=timeout_s), "no action server")
        goal = SetLocoMode.Goal()
        goal.fsm_id = fsm_id
        send_future = self.mode_client.send_goal_async(goal)
        self.assertTrue(self._wait_until(lambda: send_future.done(), timeout_s), "goal not accepted")
        handle = send_future.result()
        self.assertTrue(handle.accepted, f"SetLocoMode({fsm_id}) rejected")
        result_future = handle.get_result_async()
        self.assertTrue(self._wait_until(lambda: result_future.done(), timeout_s), "no result")
        return result_future.result().result

    def test_walk_and_arm_trajectory_in_one_session(self):
        """The acceptance bar: the robot walks under cmd_vel while the arms run a trajectory."""
        # 1. Policy stands the robot up, unwelded.
        self.assertTrue(
            self._wait_until(lambda: (self._height() or 0.0) > STAND_HEIGHT_MIN, SETTLE_TIMEOUT_S),
            "robot never stood up",
        )
        self._spin(3.0)

        # 2. Arms: acquire control authority through the documented entry point.
        self._run_ros2_run("activate_arm")
        self._spin(3.0)
        start = self._arm_position(TARGET_JOINT)
        self.assertIsNotNone(start, f"no /joint_states entry for {TARGET_JOINT}")

        # 3. Legs: the real Damp -> StandUp -> Start sequence.
        self.assertTrue(self._set_mode(SetLocoMode.Goal.STAND_UP).success, "StandUp rejected")
        self.assertTrue(self._set_mode(SetLocoMode.Goal.START).success, "Start rejected")

        # 4. Send the arm trajectory, then drive the legs while it executes: run the two
        #    sequentially instead and the test would pass even with the two paths fighting
        #    over /lowcmd.
        client = ActionClient(
            self.node, FollowJointTrajectory, "/arm_trajectory_controller/follow_joint_trajectory"
        )
        self.assertTrue(client.wait_for_server(timeout_sec=10.0), "no trajectory action server")

        targets = {name: self._arm_position(name) or 0.0 for name in JOINTS}
        targets[TARGET_JOINT] = start + ARM_STEP_RAD

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [targets[name] for name in JOINTS]
        point.time_from_start = Duration(sec=int(ARM_SETTLE_S), nanosec=0)
        goal.trajectory.points = [point]
        traj_future = client.send_goal_async(goal)

        before = list(self._position())
        twist = Twist()
        twist.linear.x = DRIVE_VX
        end = time.time() + ARM_SETTLE_S + 2.0
        worst_height = 10.0
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            worst_height = min(worst_height, self._height() or 10.0)
            time.sleep(0.02)

        # 5. Both halves must have succeeded, in the same session.
        travelled = (
            (self._position()[0] - before[0]) ** 2 + (self._position()[1] - before[1]) ** 2
        ) ** 0.5
        # See test_walk_teleop.launch.py: 0.5 m is inside the standing-drift budget and cannot
        # fail. Shorter drive window here than there, so 1.5 m rather than 2.0.
        self.assertGreater(
            travelled,
            1.5,
            f"robot travelled only {travelled:.2f} m while the arms were moving -- locomotion "
            "and the arm bridge are not coexisting",
        )
        self.assertGreater(
            worst_height,
            STAND_HEIGHT_MIN,
            f"robot fell to {worst_height:.3f} m while walking with the arms in motion",
        )

        self.assertTrue(self._wait_until(lambda: traj_future.done(), 10.0), "trajectory not accepted")
        reached = self._arm_position(TARGET_JOINT)
        self.assertAlmostEqual(
            reached,
            start + ARM_STEP_RAD,
            delta=ARM_TOLERANCE_RAD,
            msg=f"{TARGET_JOINT} reached {reached:.3f} rad, expected "
            f"{start + ARM_STEP_RAD:.3f} +/- {ARM_TOLERANCE_RAD} -- the arm trajectory did not "
            "converge while the robot was walking",
        )

        # 6. Release the arms cleanly; the robot must still be walking-capable afterwards.
        self.cmd_vel_pub.publish(Twist())
        self._spin(1.0)
        self._run_ros2_run("deactivate_arm")
        self._spin(2.0)
        self.assertGreater(
            self._height(),
            STAND_HEIGHT_MIN,
            "robot fell after the arms were released -- deactivation must not disturb the legs",
        )
