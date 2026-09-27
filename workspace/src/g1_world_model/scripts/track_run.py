#!/usr/bin/env python3
"""Records an exploration run for run_summary.py: the robot's pose on the map, each room's
coverage whenever the world model republishes its rooms, and the rooms' outlines at the end.

    ros2 run g1_world_model track_run.py /root/data/runs/run52

writes run52.csv (t, x, y, yaw), run52_coverage.csv (t, room, floor, faces) and, on Ctrl-C or
SIGTERM, run52_rooms.yaml. Start it before the exploration tree.
"""

import argparse
import math
import signal
import sys

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener

from g1_msgs.msg import RoomArray


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("prefix", help="output path without extension")
    parser.add_argument("--rate", type=float, default=5.0, help="pose samples per second")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--rooms-topic", default="/g1_world_model/rooms")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node("run_tracker")
    buffer = Buffer()
    TransformListener(buffer, node)
    poses = open(f"{args.prefix}.csv", "w", buffering=1)
    poses.write("t,x,y,yaw\n")
    coverage = open(f"{args.prefix}_coverage.csv", "w", buffering=1)
    coverage.write("t,room,floor,faces\n")
    rooms = {"rooms": []}

    def sample():
        try:
            tf = buffer.lookup_transform(args.map_frame, args.base_frame, Time())
        except TransformException:
            return  # No map yet.
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        p = tf.transform.translation
        poses.write(f"{t:.2f},{p.x:.3f},{p.y:.3f},{yaw:.3f}\n")

    def on_rooms(msg):
        rooms["rooms"] = [
            {"id": r.id, "name": r.name, "outline": [[p.x, p.y] for p in r.outline.points]}
            for r in msg.rooms
        ]
        now = node.get_clock().now().nanoseconds * 1e-9
        for r in msg.rooms:
            coverage.write(f"{now:.2f},{r.id},{r.floor_coverage:.3f},{r.face_coverage:.3f}\n")

    def stop(*_):
        with open(f"{args.prefix}_rooms.yaml", "w") as handle:
            yaml.safe_dump(rooms, handle)
        poses.close()
        coverage.close()
        raise SystemExit(0)

    node.create_timer(1.0 / args.rate, sample)
    latched = QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
    )
    node.create_subscription(RoomArray, args.rooms_topic, on_rooms, latched)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
