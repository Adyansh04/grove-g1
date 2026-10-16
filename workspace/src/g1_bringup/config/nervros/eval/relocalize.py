"""Sets AMCL's pose to where the simulator has the robot: the fault suite's undo for a robot it
made lost. The map is the simulator's world within about a centimetre, so the relay's base_state
is the robot's pose on the map.

    docker exec ros_dev_jazzy bash -c 'source /opt/ros/jazzy/setup.bash && \
        python3 /root/workspace/src/g1_bringup/config/nervros/eval/relocalize.py [yaw_offset_deg]'

A yaw offset, in degrees, sets it that far turned instead: how the suite makes the robot lost.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def main() -> None:
    offset = math.radians(float(sys.argv[1])) if len(sys.argv) > 1 else 0.0
    rclpy.init()
    node = rclpy.create_node("relocalize_to_truth")
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
    out = PoseWithCovarianceStamped()
    out.header.frame_id = "map"
    out.pose.pose.position.x = pose.position.x
    out.pose.pose.position.y = pose.position.y
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
    print(
        f"initial pose ({pose.position.x:.2f}, {pose.position.y:.2f}, {math.degrees(yaw):.0f} deg)"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
