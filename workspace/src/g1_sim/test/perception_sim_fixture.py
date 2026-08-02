"""Shared launch and driving helpers for the perception-track stream tests.

Imported by test_lidar_stream and test_camera_stream, which both need to bring the sim
up and move the base to a known place before measuring. Kept out of the test files so
the assertions stay readable.
"""

import math
import os
import time

import launch_testing
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Loading MuJoCo, registering the engine plugin, spawning controllers and starting the
# render loop. Generous so a loaded machine cannot flake it.
BRINGUP_TIMEOUT_S = 45.0

# base_link -> livox_frame and the camera pose, from config/sensor_mounts.yaml. Duplicated
# deliberately, same reason as test_perception_sim_bringup: a test that reads the file it
# is checking passes no matter what the value becomes. test_perception_sim_bringup is what
# proves TF actually carries these.
LIVOX_XYZ = (-0.00368, 0.00003, 0.472434)
LIVOX_RPY = (math.pi, 0.05112069, 0.0)
CAMERA_XYZ = (0.05366, 0.01753, 0.473870)
CAMERA_PITCH = 0.83077672

# base_link spawn height in the MJCF. Turns base_link coordinates into world ones, which
# is where the scene geometry (floor at 0, walls at +/-4) is stated.
BASE_SPAWN_Z = 0.793

# Sensor streams are best-effort: matching the publisher matters more than it looks, a
# reliable subscriber simply receives nothing here.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)


def perception_sim_description():
    """Launch the track under test, then hand over to the test node."""
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("g1_sim"), "launch", "perception_sim.launch.py"
            )
        )
    )
    return (
        LaunchDescription(
            [sim, TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()])]
        ),
        {},
    )


def rpy_to_matrix(rpy):
    """Extrinsic XYZ (URDF rpy) to a rotation matrix."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


class PerceptionSimTestNode(Node):
    """Test-side node with the base-driving loop the stream tests share."""

    def __init__(self, name):
        super().__init__(name)
        # This track runs on simulated time.
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )
        self._joint_state = None
        self.create_subscription(JointState, "/base_joint_states", self._on_joints, 10)
        self._cmd = self.create_publisher(
            Float64MultiArray, "/base_velocity_controller/commands", 10
        )

    def _on_joints(self, msg):
        self._joint_state = msg

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, predicate, timeout_s=BRINGUP_TIMEOUT_S):
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def joint(self, name):
        js = self._joint_state
        if js is None or name not in js.name:
            return None
        return js.position[js.name.index(name)]

    def drive_to_x(self, target, speed=0.6, timeout_s=45.0):
        """Drive the base to a world x and return where it actually stopped.

        Closed loop on purpose. Open-loop timing undershoots badly here (a commanded
        0.5 m/s for 2 s travels about 0.75 m), so a test that assumed the commanded
        distance would be measuring the velocity ramp rather than the sensor. Callers
        assert against the returned position, not the target.
        """
        cmd = Float64MultiArray()
        end = time.time() + timeout_s
        while time.time() < end:
            x = self.joint("base_x_joint")
            if x is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            error = target - x
            if abs(error) < 0.01:
                break
            # Taper near the goal so it settles instead of hunting.
            cmd.data = [math.copysign(min(speed, abs(error) * 2.0 + 0.05), error), 0.0, 0.0]
            self._cmd.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

        cmd.data = [0.0, 0.0, 0.0]
        for _ in range(5):
            self._cmd.publish(cmd)
            self.spin(0.05)
        # Let the base coast to rest and a fresh sensor frame arrive.
        self.spin(1.5)
        return self.joint("base_x_joint")
