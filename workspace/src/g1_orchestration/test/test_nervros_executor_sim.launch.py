"""Headless sim suite for the NervROS mission executor: a stop, a pause and a lend against the real
controllers, Nav2 and the arm stack, which the gmock tests meet only as stand-in servers.

g1_bringup brings the stack up and is found at run time: it test-depends on this package, so it
cannot be declared here.

Run with `./scripts/manage.sh test --sim g1_orchestration`.
"""

import hashlib
import math
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
from nervros_interfaces.action import ExecuteMission
from nervros_interfaces.msg import Heartbeat
from nervros_interfaces.srv import LendResources, PauseMission, StopAll
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# Nav2 is up and the arm taken (at 40 s) by then; the executor starts after it, as the demos do.
STACK_SETTLE_S = 75.0
# launch_testing waits 15 s for ReadyToTest by default, less than this stack takes to settle.
READY_TIMEOUT_S = STACK_SETTLE_S + 30.0
EXECUTOR = "/nervros_executor"
# A station across the facility, so a walk there is still under way when it is stopped or paused.
FAR_STATION = "4.30;-5.60;1.5708"
# The base walks more than MOVING_M in the time given; it stands when it moves less than
# SETTLED_M in a second.
MOVING_M = 0.15
SETTLED_M = 0.05
# How long discovery is given, once the executor's services show, to match their replies too.
DISCOVERY_SETTLE_S = 5.0
# A lend ends 2 s after its last heartbeat (lend_heartbeat_timeout_s): beats come faster.
BEAT_S = 0.25
ARMS = ["left_arm", "right_arm"]


def mission(body: str) -> str:
    return (
        '<root BTCPP_format="4" main_tree_to_execute="Mission"><BehaviorTree ID="Mission">'
        + body
        + "</BehaviorTree></root>"
    )


WALK = mission(f'<SubTree ID="GoToPose" name="s1_GoToPose" station="{FAR_STATION}"/>')
TUCK = mission('<SubTree ID="TuckForTravel" name="s1_TuckForTravel"/>')


def goal_of(xml: str) -> ExecuteMission.Goal:
    goal = ExecuteMission.Goal()
    goal.mission_id = "sim-suite"
    goal.tree_xml = xml
    goal.tree_sha256 = hashlib.sha256(xml.encode()).hexdigest()
    goal.mode = ExecuteMission.Goal.MODE_EXECUTE
    return goal


@launch_testing.ready_to_test_action_timeout(READY_TIMEOUT_S)
def generate_test_description() -> LaunchDescription:
    bringup = get_package_share_directory("g1_bringup")
    orchestration = get_package_share_directory("g1_orchestration")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup.launch.py")),
        launch_arguments={
            "mode": "localization",
            "nav": "true",
            "moveit": "true",
            "manipulation": "true",
            "world": "navigation",
            "activate_arm": "true",
            "activate_arm_delay_s": "40.0",
            "headless": "true",
            "rviz": "false",
        }.items(),
    )
    # The stack takes the arms at start with empty hands, so the executor may trust them.
    executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orchestration, "launch", "nervros_executor.launch.py")
        ),
        launch_arguments={"hands_empty_on_attach": "true"}.items(),
    )
    return LaunchDescription(
        [
            stack,
            TimerAction(
                period=STACK_SETTLE_S,
                actions=[executor, launch_testing.actions.ReadyToTest()],
            ),
        ]
    )


class TestNervrosExecutorSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = Node("test_nervros_executor_sim")
        cls.tf = Buffer()
        cls.listener = TransformListener(cls.tf, cls.node)
        cls.execute = ActionClient(cls.node, ExecuteMission, f"{EXECUTOR}/execute_mission")
        cls.stop = cls.node.create_client(StopAll, f"{EXECUTOR}/stop_all")
        cls.pause = cls.node.create_client(PauseMission, f"{EXECUTOR}/pause_mission")
        cls.lend = cls.node.create_client(LendResources, f"{EXECUTOR}/lend_resources")
        beats = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        cls.heartbeat = cls.node.create_publisher(Heartbeat, f"{EXECUTOR}/heartbeat", beats)
        # The executor comes up as the cases start. A call made the moment its service shows can
        # lose its answer ("failed to send response"): discovery has not matched the way back yet.
        for client in (cls.stop, cls.pause, cls.lend):
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f"{client.srv_name} did not come up")
        if not cls.execute.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("execute_mission did not come up")
        deadline = time.monotonic() + DISCOVERY_SETTLE_S
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    def tearDown(self) -> None:
        # Whatever a case left running ends before the next starts.
        if self.stop.wait_for_service(timeout_sec=5.0):
            self._call(self.stop, StopAll.Request(reason="sim suite: between cases"))

    def _spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait(self, future: Future, timeout_s: float) -> object:
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        self.assertTrue(future.done(), f"no answer within {timeout_s} s")
        return future.result()

    def _call(self, client: Client, request: object, timeout_s: float = 30.0) -> object:
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), f"{client.srv_name} is not up")
        return self._wait(client.call_async(request), timeout_s)

    def _base(self) -> tuple[float, float]:
        """The base's position on the map, m."""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.tf.can_transform("map", "base_footprint", Time()):
                at = self.tf.lookup_transform("map", "base_footprint", Time()).transform
                return at.translation.x, at.translation.y
        self.fail("no map to base_footprint transform")

    def _travel_m(self, seconds: float) -> float:
        """How far the base moves over the next `seconds`, m."""
        x0, y0 = self._base()
        self._spin(seconds)
        x1, y1 = self._base()
        return math.hypot(x1 - x0, y1 - y0)

    def _start(self, xml: str) -> ClientGoalHandle:
        self.assertTrue(self.execute.wait_for_server(timeout_sec=30.0), "no execute_mission")
        handle = self._wait(self.execute.send_goal_async(goal_of(xml)), 30.0)
        self.assertTrue(handle.accepted, "the mission was not accepted")
        return handle

    def _start_walk(self) -> ClientGoalHandle:
        """A walk to the far station, once the base is seen to move."""
        start = self._base()
        handle = self._start(WALK)
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            self._spin(0.5)
            x, y = self._base()
            if math.hypot(x - start[0], y - start[1]) > MOVING_M:
                return handle
        self.fail("the base did not start walking within 60 s")

    def _result(self, handle: ClientGoalHandle, timeout_s: float) -> ExecuteMission.Result:
        return self._wait(handle.get_result_async(), timeout_s).result

    def test_a_lend_hands_the_arms_over_and_they_come_back(self) -> None:
        lent = self._call(
            self.lend,
            LendResources.Request(client="sim_suite", resources=ARMS, lend=True, max_s=30.0),
        )
        self.assertTrue(lent.granted, lent.reason)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.heartbeat.publish(
                Heartbeat(stamp=self.node.get_clock().now().to_msg(), client="sim_suite")
            )
            self._spin(BEAT_S)
        back = self._call(
            self.lend,
            LendResources.Request(client="sim_suite", resources=ARMS, lend=False),
        )
        self.assertTrue(back.granted, back.reason)
        # Given back, the arms are the executor's again: a mission of its own takes and moves them.
        tucked = self._result(self._start(TUCK), 120.0)
        self.assertEqual(
            tucked.outcome, ExecuteMission.Result.OUTCOME_SUCCESS, tucked.failure_reason
        )

    def test_a_pause_holds_a_walk_and_a_resume_goes_on(self) -> None:
        handle = self._start_walk()
        paused = self._call(self.pause, PauseMission.Request(pause=True, reason="sim suite"))
        self.assertTrue(paused.ok, paused.message)
        self._spin(3.0)
        self.assertLess(self._travel_m(1.0), SETTLED_M, "the base walks on while paused")
        resumed = self._call(self.pause, PauseMission.Request(pause=False, reason="sim suite"))
        self.assertTrue(resumed.ok, resumed.message)
        self.assertGreater(self._travel_m(8.0), MOVING_M, "the walk did not go on")
        stopped = self._call(self.stop, StopAll.Request(reason="sim suite"))
        self.assertTrue(stopped.ok, stopped.message)
        self._result(handle, 30.0)

    def test_a_stop_ends_a_walk_and_the_base_stands(self) -> None:
        handle = self._start_walk()
        stopped = self._call(self.stop, StopAll.Request(reason="sim suite"))
        self.assertTrue(stopped.ok, stopped.message)
        ended = self._result(handle, 30.0)
        self.assertEqual(ended.outcome, ExecuteMission.Result.OUTCOME_CANCELED)
        self._spin(2.0)
        self.assertLess(self._travel_m(1.0), SETTLED_M, "the base still walks after the stop")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info: launch_testing.ProcInfoHandler) -> None:
        # The simulator is killed rather than stopped cleanly; the executor must end on its own,
        # having halted and handed the arms back.
        for info in proc_info:
            if "nervros_executor" in info.process_name:
                self.assertIn(info.returncode, (0, -2, -15), info.process_name)
