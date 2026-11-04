#!/usr/bin/env python3
"""Makes a route graph for nav2_route from an occupancy map.

Python: an offline tool that runs once per map, far from any control loop.

    make_route_graph.py maps/apartment.yaml config/route_graph.geojson

Nodes sit along the middle of free space, as far from walls as they can be, and edges join nodes
that see each other with room for the robot on the way. Where that leaves a doorway unjoined, a
chain of nodes through it joins the two sides, so every part of the map the robot fits through is
on one graph.
"""

import argparse
import collections
import json
import math
from pathlib import Path

import numpy as np

SPACING_M = 0.8
REACH_M = 1.6
EIGHT = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def load_map(yaml_path):
    """Returns whether each cell is free, the resolution and the origin, with row 0 at the bottom."""
    meta = {}
    for line in yaml_path.read_text().splitlines():
        key, _, value = line.partition(":")
        meta[key.strip()] = value.strip()
    data = (yaml_path.parent / meta["image"]).read_bytes()
    fields, at = [], 0
    while len(fields) < 4:
        end = at
        while data[end] not in b" \t\r\n":
            end += 1
        fields.append(data[at:end])
        at = end + 1
    if fields[0] != b"P5":
        raise ValueError(f"{meta['image']} is not a binary PGM")
    width, height = int(fields[1]), int(fields[2])
    pixels = np.frombuffer(data, np.uint8, width * height, at).reshape(height, width)
    occupied_share = (255 - pixels.astype(float)) / 255
    free = np.flipud(occupied_share < float(meta["free_thresh"]))
    origin = [float(v) for v in meta["origin"].strip("[] ").split(",")][:2]
    return free, float(meta["resolution"]), origin


def clearance_of(free):
    """Returns each cell's distance to the nearest cell that is not free, in cells."""
    clearance = np.where(free, np.inf, 0.0)
    queue = collections.deque(zip(*np.nonzero(~free)))
    height, width = free.shape
    while queue:
        row, col = queue.popleft()
        for d_row, d_col in EIGHT:
            r, c = row + d_row, col + d_col
            step = math.hypot(d_row, d_col)
            if 0 <= r < height and 0 <= c < width and clearance[r, c] > clearance[row, col] + step:
                clearance[r, c] = clearance[row, col] + step
                queue.append((r, c))
    return clearance


def clear_between(room, a, b):
    """Returns whether every cell on the line between two cells has room for the robot."""
    steps = int(max(abs(a[0] - b[0]), abs(a[1] - b[1]))) + 1
    rows = np.linspace(a[0], b[0], steps).round().astype(int)
    cols = np.linspace(a[1], b[1], steps).round().astype(int)
    return bool(room[rows, cols].all())


def spread_nodes(clearance, room, spacing):
    """Returns cells at least `spacing` apart, those farthest from walls first."""
    rows, cols = np.nonzero(room)
    order = np.argsort(-clearance[rows, cols], kind="stable")
    nodes = []
    for at in order:
        cell = (int(rows[at]), int(cols[at]))
        if all(math.dist(cell, n) >= spacing for n in nodes):
            nodes.append(cell)
    return nodes


def join_by_sight(nodes, room, reach):
    """Returns the edges between nodes within `reach` of each other that see each other."""
    return {
        (i, j)
        for i, a in enumerate(nodes)
        for j, b in enumerate(nodes[i + 1 :], i + 1)
        if math.dist(a, b) <= reach and clear_between(room, a, b)
    }


def groups_of(count, edges):
    """Returns a group number per node, joined by the edges."""
    group = list(range(count))

    def find(i):
        while group[i] != i:
            group[i] = group[group[i]]
            i = group[i]
        return i

    for i, j in edges:
        group[find(i)] = find(j)
    return [find(i) for i in range(count)]


def bridge(nodes, edges, room, spacing):
    """Adds chains of nodes through doorways until no reachable part of the graph stands alone."""
    while True:
        groups = groups_of(len(nodes), edges)
        counts = collections.Counter(groups)
        if len(counts) == 1:
            return
        small = min(counts, key=counts.get)
        starts = [n for n, g in zip(nodes, groups) if g == small]
        others = {n: i for i, (n, g) in enumerate(zip(nodes, groups)) if g != small}
        came_from = {s: None for s in starts}
        queue = collections.deque(starts)
        end = None
        while queue and end is None:
            cell = queue.popleft()
            for d_row, d_col in EIGHT:
                next_cell = (cell[0] + d_row, cell[1] + d_col)
                if (
                    next_cell not in came_from
                    and 0 <= next_cell[0] < room.shape[0]
                    and 0 <= next_cell[1] < room.shape[1]
                    and room[next_cell]
                ):
                    came_from[next_cell] = cell
                    if next_cell in others:
                        end = next_cell
                        break
                    queue.append(next_cell)
        if end is None:
            # Not reachable from the rest by a robot of this size: a closet, or an island.
            drop = {i for i, g in enumerate(groups) if g == small}
            keep = [i for i in range(len(nodes)) if i not in drop]
            renumber = {old: new for new, old in enumerate(keep)}
            edges.intersection_update({e for e in edges if not drop.intersection(e)})
            moved = {(renumber[i], renumber[j]) for i, j in edges}
            edges.clear()
            edges.update(moved)
            nodes[:] = [nodes[i] for i in keep]
            continue
        path = [end]
        while came_from[path[-1]] is not None:
            path.append(came_from[path[-1]])
        chain = [i for i, n in enumerate(nodes) if n == end]
        last = chain[0]
        walked = 0
        for before, cell in zip(path, path[1:]):
            walked += math.dist(before, cell)
            if walked >= spacing or came_from[cell] is None:
                walked = 0
                index = nodes.index(cell) if cell in nodes else len(nodes)
                if index == len(nodes):
                    nodes.append(cell)
                edges.add((last, index))
                last = index


def geojson(nodes, edges, resolution, origin):
    """Returns the graph as nav2_route's GeoJSON parser reads it, each edge both ways."""

    def at(cell):
        return [
            round(origin[0] + (cell[1] + 0.5) * resolution, 3),
            round(origin[1] + (cell[0] + 0.5) * resolution, 3),
        ]

    features = [
        {
            "type": "Feature",
            "properties": {"id": i, "frame": "map"},
            "geometry": {"type": "Point", "coordinates": at(cell)},
        }
        for i, cell in enumerate(nodes)
    ]
    next_id = len(nodes)
    for i, j in sorted(edges):
        for start, end in ((i, j), (j, i)):
            features.append(
                {
                    "type": "Feature",
                    "properties": {"id": next_id, "startid": start, "endid": end},
                    "geometry": {
                        "type": "MultiLineString",
                        "coordinates": [[at(nodes[start]), at(nodes[end])]],
                    },
                }
            )
            next_id += 1
    return {
        "type": "FeatureCollection",
        "name": "graph",
        "crs": {"type": "name", "properties": {"name": "urn:ogc:def:crs:EPSG::3857"}},
        "features": features,
    }


def main():
    """Writes the graph and says how large it came out."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("map", type=Path, help="the map's YAML file")
    parser.add_argument("out", type=Path, help="the GeoJSON file to write")
    parser.add_argument("--robot-radius", type=float, default=0.45, help="metres, as the costmaps")
    args = parser.parse_args()
    free, resolution, origin = load_map(args.map)
    clearance = clearance_of(free)
    room = clearance * resolution >= args.robot_radius
    spacing = SPACING_M / resolution
    nodes = spread_nodes(clearance, room, spacing)
    edges = join_by_sight(nodes, room, REACH_M / resolution)
    bridge(nodes, edges, room, spacing)
    args.out.write_text(json.dumps(geojson(nodes, edges, resolution, origin)) + "\n")
    groups = len(set(groups_of(len(nodes), edges)))
    print(f"{len(nodes)} nodes, {len(edges)} edges both ways, {groups} connected part(s)")


if __name__ == "__main__":
    main()
