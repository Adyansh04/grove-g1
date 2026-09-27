#!/usr/bin/env python3
"""Saves the occupancy grid SLAM publishes as a picture: free white, occupied black, unknown grey,
and frontier cells (free beside unknown) red. Prints the cell counts. It shows what the frontier
pass plans on, and is best taken as it ends.

    ros2 run canopy snapshot_map.py /tmp/map.png
"""

import argparse
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("out", help="picture to write, north up")
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--scale", type=int, default=3, help="pixels per cell")
    parser.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for a map")
    # map_saver's thresholds, as percentages: what the saved floor plan calls free and occupied.
    parser.add_argument("--free-max", type=int, default=25)
    parser.add_argument("--occupied-min", type=int, default=65)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node("map_snapshot")
    got = []
    latched = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(OccupancyGrid, args.topic, got.append, latched)
    deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    while not got and node.get_clock().now().nanoseconds * 1e-9 < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        sys.exit(f"no map on {args.topic} within {args.timeout:.0f} s")

    grid = got[-1]
    cells = np.array(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
    free = (cells >= 0) & (cells <= args.free_max)
    occupied = cells >= args.occupied_min
    unknown = ~free & ~occupied
    frontier = np.zeros_like(free)
    frontier[1:-1, 1:-1] = free[1:-1, 1:-1] & (
        unknown[1:-1, 2:] | unknown[1:-1, :-2] | unknown[2:, 1:-1] | unknown[:-2, 1:-1]
    )
    image = np.full((*cells.shape, 3), 128, np.uint8)
    image[free] = 255
    image[occupied] = 0
    image[frontier] = (230, 30, 30)
    size = (grid.info.width * args.scale, grid.info.height * args.scale)
    Image.fromarray(image[::-1]).resize(size, Image.NEAREST).save(args.out)
    origin = grid.info.origin.position
    print(
        f"{args.out}: {grid.info.width} x {grid.info.height} cells of {grid.info.resolution:.2f} m "
        f"from ({origin.x:.2f}, {origin.y:.2f}); free {free.sum()}, occupied {occupied.sum()}, "
        f"unknown {unknown.sum()}, frontier {frontier.sum()}"
    )


if __name__ == "__main__":
    main()
