"""The converged track's LiDAR measures the room it is actually in.

Geometry, not plumbing. The cloud is published in the sensor frame, and the simulator
also publishes the sensor's exact world pose, so the points can be put into world
coordinates and checked against facts of g1_perception_pinned_scene.xml: floor at z=0,
inner wall faces at +/-4.0 m.

Runs with the pelvis pinned: a free-standing G1 drifts and its waist leans (the torso
pitches tens of degrees), so the numbers would move under the test without anything
being wrong.
"""

import os
import time
import unittest
from collections import deque

import launch_testing
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

CLOUD_TOPIC = "/livox/lidar"
POSE_TOPIC = "/g1_sensor_relay/sensor_pose"

# config/sim_sensors.yaml
EXPECTED_POINTS = 360 * 32
CONFIGURED_RATE_HZ = 10.0
RANGE_MIN = 0.1
RANGE_MAX = 40.0

# mjcf/g1_perception_pinned_scene.xml: inner wall faces at +/-4.0, floor at z=0, walls 2.5 tall.
ROOM_HALF = 4.0
WALL_TOP = 2.5

BRINGUP_TIMEOUT_S = 90.0


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "sensors": "true",
            "pin_pelvis": "true",
            # The small bare room this test's numbers come from, not the facility.
            "world": "perception",
        }.items(),
    )
    return (
        LaunchDescription(
            [sim, TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()])]
        ),
        {},
    )


def quat_to_matrix(q):
    """geometry_msgs Quaternion to a 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LidarGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_lidar_geometry")
        # Bounded: 11520-point clouds at 10 Hz over a long bring-up would otherwise
        # accumulate to hundreds of megabytes.
        cls.clouds = deque(maxlen=4)
        cls.poses = deque(maxlen=4)
        cls.stamps = deque(maxlen=512)
        cls.node.create_subscription(
            PointCloud2, CLOUD_TOPIC, cls._on_cloud, qos_profile_sensor_data
        )
        cls.node.create_subscription(
            PoseStamped, POSE_TOPIC, cls.poses.append, qos_profile_sensor_data
        )
        cls._wait(lambda: len(cls.clouds) > 0 and len(cls.poses) > 0, BRINGUP_TIMEOUT_S)

    @classmethod
    def _on_cloud(cls, msg):
        cls.clouds.append(msg)
        cls.stamps.append(time.time())

    @classmethod
    def _wait(cls, predicate, timeout_s):
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def latest_cloud(self):
        self.assertTrue(self.clouds, f"nothing published on {CLOUD_TOPIC}")
        return self.clouds[-1]

    def sensor_points(self, msg):
        n = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        return (
            np.frombuffer(raw[:, 0:12].tobytes(), dtype=np.float32)
            .reshape(n, 3)
            .astype(np.float64)
        )

    def world_returns(self):
        """Actual returns, in world coordinates, using the simulator's own sensor pose."""
        self.assertTrue(self.poses, f"nothing published on {POSE_TOPIC}")
        cloud = self.latest_cloud()
        pose = self.poses[-1]

        pts = self.sensor_points(cloud)
        # A zero point is "no return", not a point at the sensor origin.
        hit = np.linalg.norm(pts, axis=1) > 1e-6
        rotation = quat_to_matrix(pose.pose.orientation)
        origin = np.array(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        )
        return (rotation @ pts[hit].T).T + origin, origin

    def test_01_streams_at_the_configured_rate(self):
        self.stamps.clear()
        self.spin(3.0)
        self.assertGreater(len(self.stamps), 2, f"{CLOUD_TOPIC} is not streaming")
        rate = (len(self.stamps) - 1) / (self.stamps[-1] - self.stamps[0])
        self.assertGreater(rate, 7.0, f"{rate:.2f} Hz, configured {CONFIGURED_RATE_HZ}")
        self.assertLess(rate, 13.0, f"{rate:.2f} Hz, configured {CONFIGURED_RATE_HZ}")

    def test_02_cloud_layout_matches_the_configured_sweep(self):
        msg = self.latest_cloud()
        self.assertEqual(msg.header.frame_id, "mid360_link")
        self.assertEqual(msg.width * msg.height, EXPECTED_POINTS)
        self.assertEqual([f.name for f in msg.fields][:3], ["x", "y", "z"])
        self.assertEqual(msg.point_step, 12)

    def test_03_returns_are_in_range_and_plentiful(self):
        pts = self.sensor_points(self.latest_cloud())
        ranges = np.linalg.norm(pts, axis=1)
        hit = ranges > 1e-6
        self.assertGreater(
            hit.mean(), 0.5, f"only {100 * hit.mean():.1f}% of rays returned inside a closed room"
        )
        self.assertGreaterEqual(ranges[hit].min(), RANGE_MIN - 1e-3)
        self.assertLessEqual(ranges[hit].max(), RANGE_MAX)

    def test_04_the_sensor_is_where_the_mount_puts_it(self):
        """Height above the floor, from the vendored URDF mount on a pinned pelvis.

        The waist chain plus mid360_joint's own offset, so a wrong mount or a wrong torso
        pose moves it.
        """
        _, origin = self.world_returns()
        self.assertAlmostEqual(
            origin[2],
            1.275,
            delta=0.08,
            msg=f"sensor at z={origin[2]:.4f}; the Mid360 sits ~1.275 m up on a pinned G1",
        )
        self.assertLess(abs(origin[0]), 0.5, "sensor drifted in x on a pinned pelvis")
        self.assertLess(abs(origin[1]), 0.5, "sensor drifted in y on a pinned pelvis")

    def test_05_every_return_lies_inside_the_room(self):
        """The decisive geometric check: the cloud describes this room and no other."""
        world, _ = self.world_returns()
        self.assertGreater(len(world), 1000, "too few returns to characterise the room")

        inside = (
            (np.abs(world[:, 0]) <= ROOM_HALF + 0.02)
            & (np.abs(world[:, 1]) <= ROOM_HALF + 0.02)
            & (world[:, 2] >= -0.02)
            & (world[:, 2] <= WALL_TOP + 0.02)
        )
        self.assertGreater(
            inside.mean(),
            0.99,
            f"only {100 * inside.mean():.2f}% of returns are inside the room; the walls sit "
            f"at +/-{ROOM_HALF} m and the floor at z=0",
        )

    def test_06_the_floor_and_the_walls_are_both_seen(self):
        """A cloud that saw only the floor would still pass test_05."""
        world, _ = self.world_returns()

        floor = np.abs(world[:, 2]) < 0.05
        self.assertGreater(
            int(floor.sum()), 200, f"only {floor.sum()} returns on the floor plane"
        )

        # Three walls, so a sweep that lost a sector fails rather than averaging out. The +x
        # wall is deliberately absent: reach_obstacle sits 0.18 m in front of the sensor at its
        # own height and occludes that whole direction, so it is the only obstacle
        # g1_moveit_config's octomap test sees in +x. Measured, the cloud stops at x = 1.60.
        for axis, sign, name in ((0, -1, "-x"), (1, 1, "+y"), (1, -1, "-y")):
            on_wall = np.abs(world[:, axis] - sign * ROOM_HALF) < 0.05
            self.assertGreater(
                int(on_wall.sum()), 20, f"only {on_wall.sum()} returns on the {name} wall"
            )
