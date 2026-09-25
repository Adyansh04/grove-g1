"""The world model's acceptance run: explore the apartment, then score what it learned.

Brings up the apartment world with localization and Nav2, the mock detector (masks cut from
simulator ground truth, so this scores the mapping, not a detector on flat renders), the world
model and the exploration tree. Once the tree finishes, the rooms, the camera coverage and the
objects are scored against worlds/apartment.truth.yaml, and the robot is sent to an object by
name with no coordinates anywhere.
"""

import math
import os
import signal
import subprocess
import tempfile
import time
import unittest

import launch_testing
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node as LaunchNode
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from g1_msgs.msg import RoomArray, WorldObjectArray
from g1_msgs.srv import GetApproachPose

TRUTH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "..",
    "g1_bringup",
    "worlds",
    "apartment.truth.yaml",
)
WORLD_DIR = tempfile.mkdtemp(prefix="g1_world_explore_")

BRINGUP_TIMEOUT_S = 240.0
EXPLORE_TIMEOUT_S = 50 * 60.0

# Scoring thresholds; the numbers each run measured are printed beside them.
MIN_FLOOR_COVERAGE = 0.90
MIN_FACE_COVERAGE = 0.80
MIN_OBJECT_RECALL = 0.85
MAX_DUPLICATE_SHARE = 0.10
MATCH_MARGIN_M = 0.4

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def load_truth():
    with open(TRUTH) as handle:
        return yaml.safe_load(handle)


@pytest.mark.launch_test
def generate_test_description():
    truth = load_truth()
    phrases = sorted({item["label"] for item in truth["objects"]})
    return (
        LaunchDescription(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("g1_bringup"), "launch", "bringup.launch.py"
                        )
                    ),
                    launch_arguments={
                        "mode": "localization",
                        "nav": "true",
                        "world": "apartment",
                        "headless": "true",
                        "rviz": "false",
                    }.items(),
                ),
                LaunchNode(
                    package="g1_perception",
                    executable="g1_mock_detector",
                    name="g1_detector",
                    output="log",
                    parameters=[
                        os.path.join(
                            get_package_share_directory("g1_perception"),
                            "config",
                            "g1_mock_detector.yaml",
                        ),
                        {"phrases": phrases, "mock_rate_hz": 2.0},
                    ],
                    remappings=[
                        ("object_poses", "/g1_sensor_relay/object_poses"),
                        ("depth/image_raw", "/camera/aligned_depth_to_color/image_raw"),
                        ("camera_info", "/camera/color/camera_info"),
                        ("~/instance_masks", "/g1_perception/instance_masks"),
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("g1_world_model"),
                            "launch",
                            "world_model.launch.py",
                        )
                    ),
                    launch_arguments={"world_dir": WORLD_DIR}.items(),
                ),
                TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        {},
    )


def inside(polygon, x, y):
    """Even-odd point in polygon."""
    hit = False
    count = len(polygon)
    for i in range(count):
        x0, y0 = polygon[i]
        x1, y1 = polygon[(i + 1) % count]
        if (y0 > y) != (y1 > y) and x < x0 + ((y - y0) * (x1 - x0) / (y1 - y0)):
            hit = not hit
    return hit


def on_footprint(item, mapped):
    """Whether a mapped object's centre is on a truth object's footprint, grown by the margin.

    A table seen from one side fuses around the part the camera saw, so its centre can sit a
    metre from the true one while still being the table."""
    yaw = math.radians(item["yaw"])
    dx = mapped.pose.position.x - item["centre"][0]
    dy = mapped.pose.position.y - item["centre"][1]
    along = (math.cos(yaw) * dx) + (math.sin(yaw) * dy)
    across = (-math.sin(yaw) * dx) + (math.cos(yaw) * dy)
    return (
        abs(along) <= (0.5 * item["size"][0]) + MATCH_MARGIN_M
        and abs(across) <= (0.5 * item["size"][1]) + MATCH_MARGIN_M
    )


def centroid(polygon):
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return sum(xs) / len(xs), sum(ys) / len(ys)


class ExploreApartmentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("world_model_explore_test")
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.node)
        cls.navigate = ActionClient(cls.node, NavigateToPose, "navigate_to_pose")
        cls.approach = cls.node.create_client(GetApproachPose, "/g1_world_model/get_approach_pose")
        cls.save = cls.node.create_client(Trigger, "/g1_world_model/save")
        cls.rooms = None
        cls.objects = None
        cls.node.create_subscription(RoomArray, "/g1_world_model/rooms", cls._on_rooms, LATCHED)
        cls.node.create_subscription(
            WorldObjectArray, "/g1_world_model/objects", cls._on_objects, LATCHED
        )
        cls.truth = load_truth()

        cls.ready = cls.navigate.wait_for_server(timeout_sec=BRINGUP_TIMEOUT_S)
        cls.tf_ready = False
        deadline = time.time() + 90.0
        while time.time() < deadline and not cls.tf_ready:
            try:
                cls.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                cls.tf_ready = True
            except Exception:
                rclpy.spin_once(cls.node, timeout_sec=0.1)

    @classmethod
    def _on_rooms(cls, msg):
        cls.rooms = msg

    @classmethod
    def _on_objects(cls, msg):
        cls.objects = msg

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call(self, client, request, timeout_s=30.0):
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), f"{client.srv_name} missing")
        future = client.call_async(request)
        deadline = time.time() + timeout_s
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(future.done(), f"{client.srv_name} did not answer")
        return future.result()

    def test_1_explores_the_apartment(self):
        self.assertTrue(self.ready, "navigate_to_pose never appeared")
        self.assertTrue(self.tf_ready, "map -> base_footprint never appeared")
        tree = os.path.join(get_package_share_directory("g1_orchestration"), "trees", "explore.xml")
        # Its own process group, so a timeout takes the whole executor down with it.
        executor = subprocess.Popen(
            [
                "ros2",
                "run",
                "g1_orchestration",
                "g1_bt_executor",
                "--ros-args",
                "-p",
                f"tree_file:={tree}",
                "-p",
                "groot2_port:=0",
            ],
            start_new_session=True,
        )
        started = time.time()
        while executor.poll() is None and time.time() - started < EXPLORE_TIMEOUT_S:
            self.spin(1.0)
        if executor.poll() is None:
            os.killpg(executor.pid, signal.SIGINT)
            executor.wait(timeout=30)
            self.fail(f"exploration did not finish in {EXPLORE_TIMEOUT_S / 60:.0f} min")
        minutes = (time.time() - started) / 60.0
        print(f"exploration finished in {minutes:.1f} min with exit code {executor.returncode}")
        self.assertEqual(executor.returncode, 0, "the exploration tree failed")

    def test_2_finds_the_rooms(self):
        self.spin(3.0)
        self.assertIsNotNone(self.rooms, "no rooms published")
        outlines = {room.id: [(p.x, p.y) for p in room.outline.points] for room in self.rooms.rooms}
        print(
            f"{len(outlines)} rooms: "
            + ", ".join(
                f"{room.id} {room.name} {room.type} {room.area:.1f} m2" for room in self.rooms.rooms
            )
        )
        claimed = {}
        for truth_room in self.truth["rooms"]:
            x, y = centroid(truth_room["polygon"])
            owners = [room_id for room_id, outline in outlines.items() if inside(outline, x, y)]
            self.assertEqual(len(owners), 1, f"{truth_room['id']}'s middle is in {owners}")
            claimed.setdefault(owners[0], []).append(truth_room["id"])
        merged = {room: names for room, names in claimed.items() if len(names) > 1}
        self.assertFalse(merged, f"rooms merged into one: {merged}")

    def test_3_the_camera_saw_every_room(self):
        self.assertIsNotNone(self.rooms, "no rooms published")
        for room in self.rooms.rooms:
            print(
                f"{room.id} {room.name}: floor {room.floor_coverage:.3f}, faces "
                f"{room.face_coverage:.3f}, surfaces {room.surface_coverage:.3f}, "
                f"{room.unobservable_cells} unobservable, {room.object_count} objects"
            )
        for room in self.rooms.rooms:
            self.assertGreaterEqual(room.floor_coverage, MIN_FLOOR_COVERAGE, room.id)
            self.assertGreaterEqual(room.face_coverage, MIN_FACE_COVERAGE, room.id)

    def test_4_maps_the_objects(self):
        self.spin(2.0)
        self.assertIsNotNone(self.objects, "no objects published")
        mapped = [o for o in self.objects.objects if o.state == 0]
        wanted = [o for o in self.truth["objects"] if o["observable_from_standing"]]
        found = 0
        duplicates = 0
        used = set()
        missing = []
        for item in wanted:
            names = {item["label"], *item.get("synonyms", [])}
            near = [
                o for o in mapped if (o.label in names or o.name in names) and on_footprint(item, o)
            ]
            if near:
                found += 1
                duplicates += len(near) - 1
                used.update(o.id for o in near)
            else:
                missing.append(item["body"])
        recall = found / max(len(wanted), 1)
        stray = [o.id + " " + o.label for o in mapped if o.id not in used]
        print(
            f"objects: {found}/{len(wanted)} observable found (recall {recall:.2f}), "
            f"{duplicates} duplicates, {len(stray)} unmatched: {stray[:10]}; missing {missing}"
        )
        self.assertGreaterEqual(recall, MIN_OBJECT_RECALL)
        self.assertLessEqual(duplicates, MAX_DUPLICATE_SHARE * len(wanted))

    def test_5_goes_to_an_object_by_name(self):
        request = GetApproachPose.Request()
        request.target = "dustbin"
        request.room = "office"
        response = self.call(self.approach, request)
        self.assertTrue(response.success, response.message)
        goal = NavigateToPose.Goal()
        goal.pose = response.pose
        sent = self.navigate.send_goal_async(goal)
        deadline = time.time() + 30.0
        while not sent.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        handle = sent.result()
        self.assertTrue(handle is not None and handle.accepted, "goal refused")
        result = handle.get_result_async()
        deadline = time.time() + 240.0
        while not result.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(result.done(), "never arrived")
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)

        bins = [
            o for o in self.truth["objects"] if o["label"] == "dustbin" and o["room"] == "office"
        ]
        here = self.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        x, y = here.transform.translation.x, here.transform.translation.y
        nearest = min(math.hypot(b["centre"][0] - x, b["centre"][1] - y) for b in bins)
        print(f"stopped {nearest:.2f} m from the office dustbin ({response.target_id})")
        self.assertLess(nearest, 2.0)

    def test_6_saves_the_world(self):
        response = self.call(self.save, Trigger.Request())
        self.assertTrue(response.success, response.message)
        for name in ("world.yaml", "objects.bin", "coverage.bin"):
            self.assertTrue(os.path.exists(os.path.join(WORLD_DIR, name)), name)


@launch_testing.post_shutdown_test()
class ExploreApartmentShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # The simulator is killed rather than stopped cleanly; only the world model must exit 0.
        for info in proc_info:
            if "g1_world_model" in info.process_name:
                self.assertIn(info.returncode, (0, -2, -15))
