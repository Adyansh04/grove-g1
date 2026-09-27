#!/usr/bin/env python3
"""A saved world against the simulator's truth: each floor object's box scored as the acceptance
test scores it, and a picture of the scene's walls (blue) and furniture (orange) over the saved
floor plan.

    ros2 run g1_world_model compare_truth.py /tmp/g1_world_explore_x --log run52.log
    ros2 run g1_world_model compare_truth.py WORLD --frame 0.46 -0.02 -5.5 --fit \\
        --overlay truth.png --crop 4 2.5 10.5 7.2 --mark 6.9 4.6

The frame puts the map in the world, as the test's "map to world" line prints it: pass --log to
read it from the run's log, or --frame. --fit refits it to the matched objects' centres, which
takes the frame's own error out when judging the boxes.
"""

import argparse
import math
import os
import re
import statistics
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from PIL import Image, ImageDraw

FRAME = re.compile(r"map to world: \(([-\d.]+), ([-\d.]+)\) m, ([-\d.]+) deg")


def bringup_file(relative):
    """A file of g1_bringup's share directory, or None outside a sourced workspace."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("g1_bringup"), relative)
    except (ImportError, LookupError):
        return None


def to_world(frame, x, y):
    tx, ty, theta = frame
    return (
        tx + math.cos(theta) * x - math.sin(theta) * y,
        ty + math.sin(theta) * x + math.cos(theta) * y,
    )


def inside(rect, x, y):
    cx, cy, sx, sy, yaw = rect
    dx, dy = x - cx, y - cy
    return (
        abs(math.cos(yaw) * dx + math.sin(yaw) * dy) <= sx / 2
        and abs(-math.sin(yaw) * dx + math.cos(yaw) * dy) <= sy / 2
    )


def iou(truth, box):
    """Intersection over union of two footprints, sampled every 2 cm as the test does."""
    reach = max(math.hypot(r[2], r[3]) / 2 for r in (truth, box))
    x0, y0 = min(truth[0], box[0]) - reach, min(truth[1], box[1]) - reach
    x1, y1 = max(truth[0], box[0]) + reach, max(truth[1], box[1]) + reach
    both = either = 0
    for i in range(int((x1 - x0) / 0.02) + 1):
        for j in range(int((y1 - y0) / 0.02) + 1):
            x, y = x0 + i * 0.02, y0 + j * 0.02
            a, b = inside(truth, x, y), inside(box, x, y)
            both += a and b
            either += a or b
    return both / either if either else 0.0


def score(truth, objects, frame, margin, min_size):
    """Per floor object of the truth: (IoU, body, truth size, box size, centre offset, pair)."""
    rows = []
    for item in truth:
        if item.get("on") or min(item["size"][:2]) < min_size:
            continue
        names = {item["label"], *item.get("synonyms", [])}
        true = (*item["centre"][:2], *item["size"][:2], math.radians(item["yaw"]))
        best = None
        for mapped in objects:
            if mapped["label"] not in names:
                continue
            x, y = to_world(frame, *mapped["centre"][:2])
            along = math.cos(true[4]) * (x - true[0]) + math.sin(true[4]) * (y - true[1])
            across = -math.sin(true[4]) * (x - true[0]) + math.cos(true[4]) * (y - true[1])
            if abs(along) > true[2] / 2 + margin or abs(across) > true[3] / 2 + margin:
                continue
            box = (x, y, *mapped["size"][:2], mapped["yaw"] + frame[2])
            fit = iou(true, box)
            if best is None or fit > best[0]:
                offset = (x - true[0], y - true[1])
                pair = (tuple(mapped["centre"][:2]), tuple(item["centre"][:2]))
                best = (fit, item["body"], item["size"][:2], mapped["size"][:2], offset, pair)
        if best:
            rows.append(best)
    return sorted(rows)


def fit_frame(pairs):
    """World from map that best lays the map positions on the world ones, least squares."""
    n = len(pairs)
    mx = sum(m[0] for m, _ in pairs) / n
    my = sum(m[1] for m, _ in pairs) / n
    wx = sum(w[0] for _, w in pairs) / n
    wy = sum(w[1] for _, w in pairs) / n
    cross = dot = 0.0
    for (ax, ay), (bx, by) in pairs:
        ax, ay, bx, by = ax - mx, ay - my, bx - wx, by - wy
        cross += ax * by - ay * bx
        dot += ax * bx + ay * by
    theta = math.atan2(cross, dot)
    return (
        wx - (math.cos(theta) * mx - math.sin(theta) * my),
        wy - (math.sin(theta) * mx + math.cos(theta) * my),
        theta,
    )


def overlay(frame, args):
    meta = yaml.safe_load(open(os.path.join(args.world, "map.yaml")))
    plan = np.array(Image.open(os.path.join(args.world, meta["image"])).convert("L"))
    resolution, (ox, oy) = meta["resolution"], meta["origin"][:2]
    height, width = plan.shape
    s = args.scale
    image = Image.fromarray(plan).convert("RGB").resize((width * s, height * s), Image.NEAREST)
    draw = ImageDraw.Draw(image)
    tx, ty, theta = frame

    def pixel(x, y):  # Map metres.
        return ((x - ox) / resolution * s, (height - (y - oy) / resolution) * s)

    def world_pixel(x, y):
        dx, dy = x - tx, y - ty
        return pixel(
            math.cos(theta) * dx + math.sin(theta) * dy,
            -math.sin(theta) * dx + math.cos(theta) * dy,
        )

    def box(cx, cy, yaw, hx, hy, colour):
        c, n = math.cos(yaw), math.sin(yaw)
        corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        points = [world_pixel(cx + c * a - n * b, cy + n * a + c * b) for a, b in corners]
        draw.polygon(points, outline=colour, width=2)

    def floats(text, default):
        return [float(v) for v in text.split()] if text else default

    def yaw_of(quat):
        w, x, y, z = quat
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    root = ET.parse(args.scene).getroot()
    for geom in root.iter("geom"):
        if geom.get("class") == args.wall_class:
            px, py, _ = floats(geom.get("pos"), [0, 0, 0])
            sx, sy, _ = floats(geom.get("size"), [0, 0, 0])
            box(px, py, 0.0, sx, sy, (40, 90, 255))
    for body in root.iter("body"):
        for geom in body.findall("geom"):
            if geom.get("class") != args.prop_class:
                continue
            bx, by, _ = floats(body.get("pos"), [0, 0, 0])
            yaw = yaw_of(floats(body.get("quat"), [1, 0, 0, 0]))
            gx, gy, _ = floats(geom.get("pos"), [0, 0, 0])
            hx, hy, _ = floats(geom.get("size"), [0, 0, 0])
            cx = bx + math.cos(yaw) * gx - math.sin(yaw) * gy
            cy = by + math.sin(yaw) * gx + math.cos(yaw) * gy
            box(cx, cy, yaw + yaw_of(floats(geom.get("quat"), [1, 0, 0, 0])), hx, hy, (255, 140, 0))
            lx, ly = world_pixel(bx, by)
            draw.text((lx - 20, ly - 6), body.get("name", ""), fill=(200, 60, 0))
    for x, y in args.mark:
        u, v = pixel(x, y)
        draw.ellipse([u - 6, v - 6, u + 6, v + 6], outline=(0, 190, 0), width=3)
    if args.crop:
        x0, y0, x1, y1 = args.crop
        (left, top), (right, bottom) = pixel(x0, y1), pixel(x1, y0)
        image = image.crop((int(left), int(top), int(right), int(bottom)))
    image.save(args.overlay)
    print(f"{args.overlay}: {image.width} x {image.height}, {s} pixels per cell")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("world", help="a saved world directory (world.yaml, map.yaml)")
    frame = parser.add_mutually_exclusive_group(required=True)
    frame.add_argument("--frame", nargs=3, type=float, metavar=("TX", "TY", "DEG"))
    frame.add_argument("--log", help="a run log with the test's map to world line")
    parser.add_argument("--fit", action="store_true", help="refit the frame to the objects")
    parser.add_argument("--truth", default=bringup_file("worlds/apartment.truth.yaml"))
    parser.add_argument("--scene", default=bringup_file("mjcf/g1_apartment_scene.xml"))
    parser.add_argument("--margin", type=float, default=0.4, help="the test's match margin, m")
    parser.add_argument("--min-size", type=float, default=0.3, help="smaller objects unscored, m")
    parser.add_argument("--overlay", help="picture of the truth over the saved floor plan")
    parser.add_argument("--crop", nargs=4, type=float, metavar=("X0", "Y0", "X1", "Y1"))
    parser.add_argument("--mark", nargs=2, type=float, action="append", default=[])
    parser.add_argument("--scale", type=int, default=4, help="pixels per cell")
    parser.add_argument("--wall-class", default="apt_wall", help="the scene's wall geom class")
    parser.add_argument("--prop-class", default="apt_proxy", help="its furniture box class")
    args = parser.parse_args()
    if not args.truth:
        parser.error("no --truth, and g1_bringup is not in a sourced workspace")

    if args.log:
        match = FRAME.search(open(args.log, errors="replace").read())
        if not match:
            parser.error(f"no map to world line in {args.log}")
        args.frame = [float(v) for v in match.groups()]
    frame = (args.frame[0], args.frame[1], math.radians(args.frame[2]))
    truth = [
        o for o in yaml.safe_load(open(args.truth))["objects"] if o["observable_from_standing"]
    ]
    objects = yaml.safe_load(open(os.path.join(args.world, "world.yaml")))["objects"]

    rows = score(truth, objects, frame, args.margin, args.min_size)
    if not rows:
        parser.error("no mapped object lies on a floor object of the truth")
    if args.fit:
        before = statistics.mean(math.hypot(*row[4]) for row in rows)
        frame = fit_frame([row[5] for row in rows])
        rows = score(truth, objects, frame, args.margin, args.min_size)
        after = statistics.mean(math.hypot(*row[4]) for row in rows)
        print(
            f"fitted frame ({frame[0]:.2f}, {frame[1]:.2f}) m, {math.degrees(frame[2]):.2f} deg: "
            f"mean centre error {before:.3f} -> {after:.3f} m"
        )
    for fit, body, true_size, box_size, offset, _ in rows:
        print(
            f"{fit:.2f}  {body:18s} truth {[round(v, 2) for v in true_size]}  "
            f"box {[round(v, 2) for v in box_size]}  centre off ({offset[0]:.2f}, {offset[1]:.2f})"
        )
    print(f"median IoU {rows[len(rows) // 2][0]:.2f} over {len(rows)} floor objects")

    if args.overlay:
        if not args.scene:
            parser.error("no --scene, and g1_bringup is not in a sourced workspace")
        overlay(frame, args)


if __name__ == "__main__":
    main()
