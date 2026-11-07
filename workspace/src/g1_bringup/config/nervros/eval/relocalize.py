"""Makes AMCL lost, or mends it: the fault suite's fault for a robot on the map and its undo.

    docker exec ros_dev_jazzy bash -c 'source /opt/ros/jazzy/setup.bash && \
        python3 /root/workspace/src/g1_bringup/config/nervros/eval/relocalize.py [yaw_offset_deg]'

With a yaw offset, in degrees, it keeps AMCL's pose and sets it to where the simulator has the
robot, turned that far: lost. Without one, it gives AMCL back the pose it kept. A map starts where
the robot stood when it was made, up to half a metre from the simulator's origin, so the
simulator's pose is not the robot's pose on the map; it is the mend only when nothing was kept.
"""

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

# AMCL's pose before the suite made the robot lost, for the mend.
KEPT = Path("/root/data/eval/amcl_before_lost.json")


def amcl_pose(node: Node) -> tuple[float, float, float] | None:
    """Return AMCL's latched pose on the map as (x, y, yaw), in metres and radians, if it has one."""
    seen: list[PoseWithCovarianceStamped] = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", seen.append, qos)
    deadline = time.monotonic() + 3.0
    while not seen and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not seen:
        return None
    pose = seen[-1].pose.pose
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return pose.position.x, pose.position.y, yaw


def main() -> None:
    offset = math.radians(float(sys.argv[1])) if len(sys.argv) > 1 else 0.0
    rclpy.init()
    node = rclpy.create_node("relocalize_to_truth")
    if offset != 0.0 and (good := amcl_pose(node)) is not None:
        KEPT.parent.mkdir(parents=True, exist_ok=True)
        KEPT.write_text(json.dumps(good))
    elif offset == 0.0 and KEPT.exists():
        x, y, yaw = json.loads(KEPT.read_text())
        KEPT.unlink()
        publish(node, x, y, yaw)
        return
    seen: list[Odometry] = []
    node.create_subscription(Odometry, "/g1_sensor_relay/base_state", seen.append, 10)
    deadline = time.monotonic() + 10.0
    while not seen and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not seen:
        sys.exit("no /g1_sensor_relay/base_state within 10 s")
    pose = seen[-1].pose.pose
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) + offset
    publish(node, pose.position.x, pose.position.y, yaw)


def publish(node: Node, x: float, y: float, yaw: float) -> None:
    """Give AMCL the pose (x, y, yaw) on the map, in metres and radians, then shut the node down."""
    out = PoseWithCovarianceStamped()
    out.header.frame_id = "map"
    out.pose.pose.position.x = x
    out.pose.pose.position.y = y
    out.pose.pose.orientation.z = math.sin(yaw / 2.0)
    out.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # AMCL's own defaults for a pose it is given: a quarter metre and a little turn of doubt.
    out.pose.covariance[0] = out.pose.covariance[7] = 0.25
    out.pose.covariance[35] = 0.07
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    deadline = time.monotonic() + 10.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    publisher.publish(out)
    rclpy.spin_once(node, timeout_sec=1.0)
    print(f"initial pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
