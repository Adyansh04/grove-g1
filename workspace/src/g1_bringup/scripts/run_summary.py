#!/usr/bin/env python3
"""Where an exploration run's time went, from its launch log: the behavior tree's transitions
and the world model's viewpoint lines. With track_run.py's files it also reports where the
robot walked and which rooms it went back to, and it can draw the walk over the saved floor plan.

    ros2 run g1_bringup run_summary.py run52.log
    ros2 run g1_bringup run_summary.py run52.log --track runs/run52 \\
        --world /tmp/g1_world_explore_x --plot run52_trail.png

A return is a viewpoint in a room the robot had left. The corner looks after the camera pass
visit rooms again by design, and are counted too.
"""

import argparse
import math
import re
from collections import defaultdict

import numpy as np
import yaml
from PIL import Image, ImageDraw

# The exploration tree's node names (g1_orchestration trees/explore.xml), by what they spend.
PHASES = {
    "walking (Nav2)": ["walk_there"],
    "turning": ["face_heading"],
    "dwelling for the detector": ["dwell_for_the_detector"],
    "stepping clear": ["step_clear_to_walk", "step_clear_to_turn"],
}
TRANSITION = re.compile(
    r"^\[(\d+\.\d+)\]: (\w+)\s+(IDLE|RUNNING) -> (RUNNING|SUCCESS|FAILURE)", re.MULTILINE
)
VIEWPOINT = re.compile(
    r"\[(\d+\.\d+)\] \[(?:canopy|g1_world_model)\]: viewpoint (\d+) at \(([-\d.]+), ([-\d.]+)\), "
    r"\d+ headings, gain (\d+), cost ([\d.]+) s"
)
FRONTIER_DONE = re.compile(r"\[(\d+\.\d+)\] \[(?:canopy|g1_world_model)\]: frontier done")


def time_budget(text):
    """Seconds each phase took and how often, and the tree's first and last transition."""
    started, spent, count = {}, defaultdict(float), defaultdict(int)
    first = last = None
    for match in TRANSITION.finditer(text):
        t, node, before, after = float(match[1]), match[2], match[3], match[4]
        first = t if first is None else first
        last = t
        if before == "IDLE" and after == "RUNNING":
            started[node] = t
        elif before == "RUNNING" and node in started:
            spent[node] += t - started.pop(node)
            count[node] += 1
    return spent, count, first, last


def inside(x, y, polygon):
    hit = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


def room_returns(viewpoints, rooms_yaml):
    rooms = [
        (room["id"], [tuple(p) for p in room["outline"]])
        for room in yaml.safe_load(open(rooms_yaml))["rooms"]
    ]
    order = []
    for _, number, x, y, gain, cost in viewpoints:
        room = next((rid for rid, poly in rooms if len(poly) > 2 and inside(x, y, poly)), "?")
        order.append((number, room, x, y, gain, cost))
    print("rooms in order:", " ".join(room for _, room, *_ in order))
    left, back, walked = set(), 0, 0.0
    for previous, current in zip(order, order[1:], strict=False):
        if current[1] == previous[1]:
            continue
        left.add(previous[1])
        if current[1] in left:
            back += 1
            walked += current[5]
            print(
                f"  back into {current[1]} at viewpoint {current[0]}: gain {current[4]}, "
                f"predicted cost {current[5]:.0f} s, from "
                f"{math.hypot(current[2] - previous[2], current[3] - previous[3]):.1f} m away"
            )
    print(f"returns: {back}, their predicted walks {walked / 60:.1f} min")


def walking(track, viewpoints, frontier_end, cell, after):
    step = np.hypot(np.diff(track[:, 1]), np.diff(track[:, 2]))
    step[step > 1.0] = 0.0  # A gap in the tracking, not a walk.
    total = step.sum()
    minutes = (track[-1, 0] - track[0, 0]) / 60
    if frontier_end is None:
        print(f"walked {total:.0f} m in {minutes:.1f} min")
    else:
        frontier = step[track[1:, 0] <= frontier_end].sum()
        print(
            f"walked {total:.0f} m in {minutes:.1f} min: frontier pass {frontier:.0f} m, "
            f"camera pass {total - frontier:.0f} m"
        )
    last_seen, revisits = {}, 0
    for t, x, y, _ in track:
        key = (round(x / cell), round(y / cell))
        if key in last_seen and t - last_seen[key] > after:
            revisits += 1
        last_seen[key] = t
    print(
        f"{cell:.1f} m cells entered again after {after:.0f} s away: {revisits}, "
        f"of {len(last_seen)} cells"
    )
    # From one viewpoint's issue to the next, the robot walks to it.
    walks = []
    for previous, current, following in zip(
        viewpoints, viewpoints[1:], viewpoints[2:], strict=False
    ):
        segment = track[(track[:, 0] >= current[0]) & (track[:, 0] < following[0])]
        if len(segment) > 1:
            length = np.hypot(np.diff(segment[:, 1]), np.diff(segment[:, 2])).sum()
            straight = math.hypot(current[2] - previous[2], current[3] - previous[3])
            walks.append((length, straight, current[4], current[1]))
    print("longest walks (walked m, straight m, gain, viewpoint):")
    for length, straight, gain, number in sorted(walks, reverse=True)[:8]:
        print(f"  {length:5.1f} {straight:5.1f} gain {gain:5d} viewpoint {number}")


def plot(track, viewpoints, world, out, low_gain, scale):
    meta = yaml.safe_load(open(f"{world}/map.yaml"))
    plan = Image.open(f"{world}/{meta['image']}").convert("RGB")
    resolution, (ox, oy) = meta["resolution"], meta["origin"][:2]
    image = plan.resize((plan.width * scale, plan.height * scale), Image.NEAREST)
    draw = ImageDraw.Draw(image)

    def px(x, y):
        return ((x - ox) / resolution * scale, (plan.height - (y - oy) / resolution) * scale)

    t0, t1 = track[0, 0], track[-1, 0]
    for a, b in zip(track[:-1], track[1:], strict=True):
        late = (a[0] - t0) / max(t1 - t0, 1.0)  # Blue early, red late.
        colour = (int(255 * late), int(80 + 100 * (1 - late)), int(255 * (1 - late)))
        draw.line([px(a[1], a[2]), px(b[1], b[2])], fill=colour, width=3)
    for n, (_, _, x, y, gain, _) in enumerate(viewpoints, 1):
        cx, cy = px(x, y)
        r = 5 if gain >= low_gain else 3
        fill = (255, 220, 0) if gain >= low_gain else (160, 160, 160)
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(0, 0, 0), fill=fill)
        if n % 5 == 0 or n <= 3:
            draw.text((cx + 6, cy - 6), str(n), fill=(0, 0, 0))
    image.save(out)
    print(f"{out}: the walk, blue early to red late; viewpoints yellow, grey under {low_gain}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("log", help="the run's launch or launch_test log")
    parser.add_argument("--track", help="track_run.py's output prefix")
    parser.add_argument("--world", help="the saved world directory, for --plot")
    parser.add_argument("--plot", help="picture of the walk over the saved floor plan")
    parser.add_argument("--low-gain", type=int, default=50, help="a viewpoint's tail gain")
    parser.add_argument("--gain-share", type=float, default=0.95)
    parser.add_argument("--revisit-cell", type=float, default=0.5, help="m")
    parser.add_argument("--revisit-after", type=float, default=60.0, help="s")
    parser.add_argument("--scale", type=int, default=4, help="pixels per cell in --plot")
    args = parser.parse_args()
    if args.plot and not (args.world and args.track):
        parser.error("--plot needs --track and --world")

    text = re.sub(r"\x1b\[[0-9;]*m", "", open(args.log, errors="replace").read())
    spent, count, first, last = time_budget(text)
    if first is None:
        parser.error(f"{args.log} has no behavior tree transitions")
    total = last - first
    print(f"tree ran {total / 60:.1f} min")
    used = 0.0
    for name, nodes in PHASES.items():
        seconds = sum(spent[node] for node in nodes)
        used += seconds
        print(
            f"  {name:28s} {seconds / 60:5.1f} min  {100 * seconds / total:4.1f}%  "
            f"({sum(count[node] for node in nodes)} times)"
        )
    rest = total - used
    print(f"  {'other (planning, services)':28s} {rest / 60:5.1f} min  {100 * rest / total:4.1f}%")

    viewpoints = [
        (float(t), int(n), float(x), float(y), int(g), float(c))
        for t, n, x, y, g, c in VIEWPOINT.findall(text)
    ]
    times = [v[0] for v in viewpoints] + [last]
    tail = sum(times[i + 1] - times[i] for i, v in enumerate(viewpoints) if v[4] < args.low_gain)
    low = sum(v[4] < args.low_gain for v in viewpoints)
    print(
        f"viewpoints {len(viewpoints)}, gain under {args.low_gain}: {low}, "
        f"{tail / 60:.1f} min on them ({100 * tail / total:.0f}%)"
    )
    gains = sorted((v[4] for v in viewpoints), reverse=True)
    cumulative = np.cumsum(gains)
    if gains:
        best = int(np.searchsorted(cumulative, args.gain_share * cumulative[-1])) + 1
        print(
            f"{100 * args.gain_share:.0f}% of the predicted gain came from the best {best} "
            f"of {len(gains)} viewpoints"
        )

    done = FRONTIER_DONE.search(text)
    frontier_end = float(done[1]) if done else None
    camera = [v for v in viewpoints if frontier_end is None or v[0] > frontier_end]
    if args.track:
        room_returns(camera, f"{args.track}_rooms.yaml")
        track = np.loadtxt(f"{args.track}.csv", delimiter=",", skiprows=1)
        walking(track, camera, frontier_end, args.revisit_cell, args.revisit_after)
        if args.plot:
            plot(track, viewpoints, args.world, args.plot, args.low_gain, args.scale)


if __name__ == "__main__":
    main()
