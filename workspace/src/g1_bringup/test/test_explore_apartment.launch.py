"""canopy's acceptance run on the G1: explore the apartment, then score what it learned.

Brings up the apartment world with Nav2, the arms hung at the sides out of the cameras' view, the
mock detector (masks cut from simulator ground truth, so this scores the mapping, not a detector
on flat renders), canopy's world model and the exploration tree. Once the tree finishes, the
rooms, the camera coverage and the objects are scored against worlds/apartment.truth.yaml, and the
robot is sent to an object by name with no coordinates anywhere.

With G1_EXPLORE_TEST_MAPPING=1 there is no map to start from: slam_toolbox builds it while the tree
walks to frontiers, and everything is scored on the map the robot made. G1_EXPLORE_TEST_CAMERAS
picks the cameras rendered and read (default head,chest), each with a detector of its own.

G1_EXPLORE_TEST_DETECTOR=semantic swaps the mock for canopy_perception's detector asking canopy's
servers/semantic_server.py on the host (YOLOE over the indoor word list, SigLIP 2 embeddings), and
turns the describer on. Start the server, and servers/start-vlm.sh for its local describer, before
the test.
"""

import math
import os
import re
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
from canopy_msgs.msg import RoomArray, WorldObjectArray
from canopy_msgs.srv import GetApproachPose
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node as LaunchNode
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from g1_msgs.action import StepClear

TRUTH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "worlds", "apartment.truth.yaml"
)
WORLD_DIR = tempfile.mkdtemp(prefix="canopy_explore_")

BRINGUP_TIMEOUT_S = 240.0
MAPPING = os.environ.get("G1_EXPLORE_TEST_MAPPING", "") == "1"
CAMERAS = os.environ.get("G1_EXPLORE_TEST_CAMERAS", "head,chest")
DETECTOR = os.environ.get("G1_EXPLORE_TEST_DETECTOR", "mock")
# Mapping first walks every frontier, which the committed map has none of.
EXPLORE_TIMEOUT_S = (75 if MAPPING else 50) * 60.0

# Scoring thresholds; the numbers each run measured are printed beside them.
MIN_FLOOR_COVERAGE = 0.90
MIN_FACE_COVERAGE = 0.80
MIN_OBJECT_RECALL = 0.85
MAX_DUPLICATE_SHARE = 0.10
MATCH_MARGIN_M = 0.4
# Standing poses the map's frame is fitted to; a run records several hundred.
MIN_TRACK_POSES = 50
# Boxes of furniture on the floor against the true footprints: median intersection over union.
MIN_BOX_IOU = 0.5

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def load_truth():
    with open(TRUTH) as handle:
        return yaml.safe_load(handle)


def mock_detector(camera, phrases):
    """Masks for one camera, cut from the ground truth the relay puts in that camera's frame.

    Named as canopy names a camera's detector, so its masks land where the world model reads them.
    """
    head = camera == "head"
    namespace = "/camera" if head else f"/{camera}_camera"
    return LaunchNode(
        package="canopy_perception",
        executable="mock_detector",
        name=f"detector_{camera}",
        output="log",
        parameters=[
            os.path.join(
                get_package_share_directory("canopy_perception"), "config", "mock_detector.yaml"
            ),
            {"phrases": phrases, "mock_rate_hz": 2.0},
        ],
        remappings=[
            (
                "object_poses",
                "/g1_sensor_relay/object_poses"
                if head
                else f"/g1_sensor_relay/{camera}/object_poses",
            ),
            ("depth/image_raw", f"{namespace}/aligned_depth_to_color/image_raw"),
            ("camera_info", f"{namespace}/color/camera_info"),
        ],
    )


@pytest.mark.launch_test
def generate_test_description():
    truth = load_truth()
    phrases = sorted({item["label"] for item in truth["objects"]})
    semantic = DETECTOR == "semantic"
    mocks = [] if semantic else [mock_detector(name, phrases) for name in CAMERAS.split(",")]
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
                        "mode": "mapping" if MAPPING else "localization",
                        "nav": "true",
                        "world": "apartment",
                        "cameras": CAMERAS,
                        "arms_at_sides": "true",
                        "headless": "true",
                        "rviz": "false",
                    }.items(),
                ),
                *mocks,
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("g1_bringup"),
                            "launch",
                            "world_model.launch.py",
                        )
                    ),
                    launch_arguments={
                        "world_dir": WORLD_DIR,
                        "cameras": CAMERAS,
                        "detector": "true" if semantic else "false",
                        "describe": "true" if semantic else "false",
                    }.items(),
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


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_world(frame, x, y):
    tx, ty, theta = frame
    return (
        tx + math.cos(theta) * x - math.sin(theta) * y,
        ty + math.sin(theta) * x + math.cos(theta) * y,
    )


def to_map(frame, x, y):
    tx, ty, theta = frame
    dx, dy = x - tx, y - ty
    return math.cos(theta) * dx + math.sin(theta) * dy, -math.sin(theta) * dx + math.cos(theta) * dy


def fit_frame(pairs):
    """World from map as (x, y, yaw): the rigid motion that best lays the robot's map positions
    on its true ones, least squares."""
    n = len(pairs)
    mx = sum(m[0] for m, _ in pairs) / n
    my = sum(m[1] for m, _ in pairs) / n
    wx = sum(w[0] for _, w in pairs) / n
    wy = sum(w[1] for _, w in pairs) / n
    cross = dot = 0.0
    for (ax, ay), (bx, by) in pairs:
        ax, ay, bx, by = ax - mx, ay - my, bx - wx, by - wy
        cross += (ax * by) - (ay * bx)
        dot += (ax * bx) + (ay * by)
    theta = math.atan2(cross, dot)
    return (
        wx - (math.cos(theta) * mx - math.sin(theta) * my),
        wy - (math.sin(theta) * mx + math.cos(theta) * my),
        theta,
    )


def world_pose(mapped, frame):
    x, y = to_world(frame, mapped.pose.position.x, mapped.pose.position.y)
    return x, y, yaw_of(mapped.pose.orientation) + frame[2]


def names_it(mapped, names):
    """Whether a mapped object's label, or whole words of its describer's name, is one of names:
    "plastic crate" is a crate, as FindObjects answers it."""
    return mapped.label in names or any(
        re.search(rf"\b{re.escape(name)}\b", mapped.name.lower()) for name in names
    )


def on_footprint(item, mapped, frame):
    """Whether a mapped object's centre is on a truth object's footprint, grown by the margin.

    A table seen from one side fuses around the part the camera saw, so its centre can sit a
    metre from the true one while still being the table."""
    yaw = math.radians(item["yaw"])
    x, y, _ = world_pose(mapped, frame)
    dx = x - item["centre"][0]
    dy = y - item["centre"][1]
    along = (math.cos(yaw) * dx) + (math.sin(yaw) * dy)
    across = (-math.sin(yaw) * dx) + (math.cos(yaw) * dy)
    return (
        abs(along) <= (0.5 * item["size"][0]) + MATCH_MARGIN_M
        and abs(across) <= (0.5 * item["size"][1]) + MATCH_MARGIN_M
    )


def box_iou(item, mapped, frame):
    """Intersection over union of a truth footprint and a mapped box, sampled every 2 cm."""
    truth = (
        item["centre"][0],
        item["centre"][1],
        item["size"][0],
        item["size"][1],
        math.radians(item["yaw"]),
    )
    x, y, yaw = world_pose(mapped, frame)
    box = (x, y, mapped.size.x, mapped.size.y, yaw)

    def inside(rect, x, y):
        cx, cy, sx, sy, a = rect
        dx, dy = x - cx, y - cy
        return (
            abs(math.cos(a) * dx + math.sin(a) * dy) <= sx / 2
            and abs(-math.sin(a) * dx + math.cos(a) * dy) <= sy / 2
        )

    reach = max(math.hypot(r[2], r[3]) / 2 for r in (truth, box))
    x0 = min(truth[0], box[0]) - reach
    y0 = min(truth[1], box[1]) - reach
    x1 = max(truth[0], box[0]) + reach
    y1 = max(truth[1], box[1]) + reach
    both = either = 0
    steps_x = int((x1 - x0) / 0.02) + 1
    steps_y = int((y1 - y0) / 0.02) + 1
    for i in range(steps_x):
        for j in range(steps_y):
            x, y = x0 + i * 0.02, y0 + j * 0.02
            a, b = inside(truth, x, y), inside(box, x, y)
            both += a and b
            either += a or b
    return both / either if either else 0.0


def centroid(polygon):
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def stop(process):
    """SIGINT to the process group, then SIGKILL past 30 s: the tree's arm release alone can make
    several 15 s service calls, and a survivor would drive the next suite's robot."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


class ExploreApartmentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("world_model_explore_test")
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.node)
        cls.navigate = ActionClient(cls.node, NavigateToPose, "navigate_to_pose")
        cls.step_clear = ActionClient(cls.node, StepClear, "/g1_base_approach/step_clear")
        cls.approach = cls.node.create_client(GetApproachPose, "/canopy/get_approach_pose")
        cls.save = cls.node.create_client(Trigger, "/canopy/save")
        cls.rooms = None
        cls.objects = None
        cls.truth_pose = None
        cls.world_from_map = None
        cls.track = []
        cls.last_truth = None
        cls.node.create_subscription(
            Odometry, "/g1_sensor_relay/base_state", cls._on_truth_pose, 10
        )
        cls.node.create_subscription(RoomArray, "/canopy/rooms", cls._on_rooms, LATCHED)
        cls.node.create_subscription(WorldObjectArray, "/canopy/objects", cls._on_objects, LATCHED)
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
    def _on_truth_pose(cls, msg):
        cls.truth_pose = msg

    def record_pose(self):
        """The robot's map and true positions, while it stands still: the two arrive apart, and a
        walking robot moves between them."""
        if self.truth_pose is None:
            return
        try:
            here = self.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:  # noqa: BLE001 - no map yet
            return
        truth = self.truth_pose.pose.pose.position
        world = (truth.x, truth.y)
        last = self.last_truth
        type(self).last_truth = world
        if last is not None and math.hypot(world[0] - last[0], world[1] - last[1]) < 0.05:
            self.track.append(((here.transform.translation.x, here.transform.translation.y), world))

    def frame(self):
        """World from map as (x, y, yaw), fitted to the robot's positions over the whole run: SLAM's
        map starts wherever the robot stood at boot, and one pose at the end would carry the
        localization error there into every object (0.1 m and 1 degree in run 44). From the
        simulator's pose of the robot alone when too few were recorded."""
        if self.world_from_map is None and len(self.track) >= MIN_TRACK_POSES:
            type(self).world_from_map = fit_frame(self.track)
            tx, ty, theta = self.world_from_map
            print(
                f"map to world: ({tx:.2f}, {ty:.2f}) m, {math.degrees(theta):.1f} deg, "
                f"from {len(self.track)} poses"
            )
        if self.world_from_map is None:
            self.spin(1.0)
            self.assertIsNotNone(self.truth_pose, "no ground-truth pose from the simulator")
            here = self.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            truth = self.truth_pose.pose.pose
            mx, my = here.transform.translation.x, here.transform.translation.y
            theta = yaw_of(truth.orientation) - yaw_of(here.transform.rotation)
            tx = truth.position.x - (math.cos(theta) * mx - math.sin(theta) * my)
            ty = truth.position.y - (math.sin(theta) * mx + math.cos(theta) * my)
            type(self).world_from_map = (tx, ty, theta)
            print(f"map to world: ({tx:.2f}, {ty:.2f}) m, {math.degrees(theta):.1f} deg")
        return self.world_from_map

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
        try:
            while executor.poll() is None and time.time() - started < EXPLORE_TIMEOUT_S:
                self.spin(1.0)
                self.record_pose()
            timed_out = executor.poll() is None
        finally:
            stop(executor)
        if timed_out:
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
        frame = self.frame()
        for truth_room in self.truth["rooms"]:
            x, y = to_map(frame, *centroid(truth_room["polygon"]))
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
        frame = self.frame()
        found = 0
        duplicates = 0
        used = set()
        missing = []
        for item in wanted:
            names = {item["label"], *item.get("synonyms", [])}
            near = [o for o in mapped if names_it(o, names) and on_footprint(item, o, frame)]
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
        # How well the boxes sit on the furniture: what an approach pose is computed from.
        fits = []
        for item in wanted:
            if item.get("on") or min(item["size"][:2]) < 0.3:
                continue
            names = {item["label"], *item.get("synonyms", [])}
            near = [o for o in mapped if o.label in names and on_footprint(item, o, frame)]
            if near:
                fits.append(max(box_iou(item, o, frame) for o in near))
        fits.sort()
        median_fit = fits[len(fits) // 2] if fits else 0.0
        print(
            f"boxes: median IoU {median_fit:.2f} over {len(fits)} floor objects, "
            f"lowest {[round(f, 2) for f in fits[:5]]}"
        )
        self.assertGreaterEqual(recall, MIN_OBJECT_RECALL)
        self.assertLessEqual(duplicates, MAX_DUPLICATE_SHARE * len(wanted))
        self.assertGreaterEqual(median_fit, MIN_BOX_IOU)

    def run_goal(self, client, goal, timeout_s):
        """Sends an action goal and waits for its result; None when refused or out of time."""
        if not client.wait_for_server(timeout_sec=10.0):
            return None
        sent = client.send_goal_async(goal)
        deadline = time.time() + timeout_s
        while not sent.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        handle = sent.result() if sent.done() else None
        if handle is None or not handle.accepted:
            return None
        result = handle.get_result_async()
        while not result.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return result.result() if result.done() else None

    def test_5_goes_to_an_object_by_name(self):
        request = GetApproachPose.Request()
        request.target = "dustbin"
        request.room = "office"
        response = self.call(self.approach, request)
        self.assertTrue(response.success, response.message)
        # As the exploration tree does before every walk: the gait's drift can leave the robot
        # inside Nav2's footprint of furniture, and from there Nav2 will not start.
        self.run_goal(self.step_clear, StepClear.Goal(clearance_m=0.55), 60.0)
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
        x, y = to_world(self.frame(), here.transform.translation.x, here.transform.translation.y)
        nearest = min(math.hypot(b["centre"][0] - x, b["centre"][1] - y) for b in bins)
        print(f"stopped {nearest:.2f} m from the office dustbin ({response.target_id})")
        self.assertLess(nearest, 2.0)

    def test_6_saves_the_world(self):
        response = self.call(self.save, Trigger.Request())
        self.assertTrue(response.success, response.message)
        for name in (
            "world.yaml",
            "objects.bin",
            "coverage.bin",
            "map.pgm",
            "map.yaml",
            "semantic_map.png",
        ):
            self.assertTrue(os.path.exists(os.path.join(WORLD_DIR, name)), name)


@launch_testing.post_shutdown_test()
class ExploreApartmentShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # The simulator is killed rather than stopped cleanly; only the world model must exit 0.
        for info in proc_info:
            if "world_model" in info.process_name:
                self.assertIn(info.returncode, (0, -2, -15))
