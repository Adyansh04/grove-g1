"""slam_toolbox maps the room it is actually in, and owns map -> odom.

Deliberately does not drive the robot. The perception room is 8 x 8 m of bare wall and the
LiDAR reaches 25 m, so one stationary sweep sees all of it, and driving would only drag the
nondeterministic gait in. This checks the SLAM wiring and the resulting geometry.

Pinned pelvis for the same reason as test_lidar_geometry.
"""

import json
import os
import time
import unittest

import launch_testing
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

# mjcf/g1_perception_pinned_scene.xml: inner wall faces at +/-4.0 m, so 8 x 8 m of interior.
ROOM_SIDE_M = 8.0
# Generous: the grid is padded past the walls and slam_toolbox sizes it to the raytraced
# extent, so this checks the room is about this size, not where the walls are.
EXTENT_TOL_M = 4.0

# config/slam_mapping.yaml
RESOLUTION_M = 0.05

BRINGUP_TIMEOUT_S = 150.0


@pytest.mark.launch_test
def generate_test_description():
    nav_share = get_package_share_directory("g1_navigation")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("g1_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "sensors": "true",
            # Ground truth, not the stack default: this scores the map, and estimator drift
            # would show up as map error that is not the mapper's.
            "odometry": "ground_truth",
            "pin_pelvis": "true",
            "world": "perception",
        }.items(),
    )
    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "scan.launch.py")),
        launch_arguments={"use_composition": "false"}.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "slam.launch.py")),
        # The shipped config keyframes on travel and a pinned robot never travels, so
        # slam_toolbox would integrate one scan and stop, mapping the right extents with no
        # walls. Overridden rather than copied so every other value still comes from the config.
        launch_arguments={
            "params_overrides": json.dumps({
                "minimum_travel_distance": 0.0,
                "minimum_travel_heading": 0.0,
                "map_update_interval": 1.0,
            })
        }.items(),
    )
    return (
        LaunchDescription([
            sim,
            scan,
            slam,
            TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]),
        {},
    )


class SlamMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("slam_map_test")
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.node)
        cls.maps = []
        # slam_toolbox latches /map, so a late subscriber still gets it, but only with matching
        # durability.
        cls.node.create_subscription(
            OccupancyGrid,
            "/map",
            lambda msg: cls.maps.append(msg),
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        deadline = time.time() + BRINGUP_TIMEOUT_S
        while time.time() < deadline and len(cls.maps) < 2:
            rclpy.spin_once(cls.node, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_map_published(self):
        self.assertGreaterEqual(
            len(self.maps), 1, f"no /map within {BRINGUP_TIMEOUT_S}s -- slam_toolbox never ran"
        )

    def test_owns_map_to_odom(self):
        # Nothing else on this stack publishes it. Without it Nav2's global costmap has no
        # frame to live in, and the failure is a TF timeout far from the cause.
        deadline = time.time() + 30.0
        while time.time() < deadline:
            try:
                self.buffer.lookup_transform("map", "odom", rclpy.time.Time())
                return
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("map -> odom never appeared on /tf")

    def test_grid_matches_the_configured_resolution(self):
        self.assertAlmostEqual(self.maps[-1].info.resolution, RESOLUTION_M, delta=1e-6)

    def test_mapped_extent_is_about_the_room(self):
        grid = self.maps[-1].info
        width_m = grid.width * grid.resolution
        height_m = grid.height * grid.resolution
        self.assertAlmostEqual(width_m, ROOM_SIDE_M, delta=EXTENT_TOL_M, msg=f"{width_m:.1f} m")
        self.assertAlmostEqual(height_m, ROOM_SIDE_M, delta=EXTENT_TOL_M, msg=f"{height_m:.1f} m")

    def test_walls_were_actually_marked(self):
        # A map that is all unknown, or all free, still has the right extent. The room's
        # perimeter is roughly 4 * 8 m of wall at 5 cm, so hundreds of occupied cells is the
        # floor for "it saw something", not a tuned number.
        data = np.array(self.maps[-1].data)
        occupied = int((data > 50).sum())
        unknown = int((data < 0).sum())
        self.assertGreater(occupied, 200, f"only {occupied} occupied cells")
        self.assertLess(
            unknown / data.size, 0.9, "almost everything is unknown -- no raytracing happened"
        )
