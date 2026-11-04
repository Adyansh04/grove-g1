#!/usr/bin/env python3
"""Makes a route graph for nav2_route from an occupancy map.

Python: an offline tool that runs once per map, far from any control loop.

    make_route_graph.py maps/apartment.yaml config/route_graph_apartment.geojson

Nodes sit along the middle of free space, as far from walls as they can be, and edges join nodes
that see each other with room to spare: a controller stops short of an edge that grazes the
costmap's inscribed cost. Where that leaves a doorway unjoined, a chain of nodes through it joins
the two sides, keeping as far from the door frame as the doorway allows, so every part of the map
the robot fits through is on one graph.
"""

import argparse
import collections
import itertools
import json
import math
from pathlib import Path

import numpy as np

SPACING_M = 0.8
REACH_M = 1.6
# Room to spare on an open lane, beyond the robot's radius, m.
MARGIN_M = 0.2
# How far a doorway's chain gives up room at a time, looking for the widest way through, m.
NARROWING_M = 0.02
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
    queue = collections.deque(zip(*np.nonzero(~free), strict=True))
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


def way_between(starts, targets, room):
    """Returns the cells from a target back to a start, through `room`; none if there is no way."""
    came_from = dict.fromkeys(starts)
    queue = collections.deque(starts)
    while queue:
        cell = queue.popleft()
        for d_row, d_col in EIGHT:
            step = (cell[0] + d_row, cell[1] + d_col)
            inside = 0 <= step[0] < room.shape[0] and 0 <= step[1] < room.shape[1]
            if step in came_from or not inside or not room[step]:
                continue
            came_from[step] = cell
            if step in targets:
                path = [step]
                while came_from[path[-1]] is not None:
                    path.append(came_from[path[-1]])
                return path
            queue.append(step)
    return None


def add_chain(nodes, edges, path, room, spacing):
    """Adds nodes along `path`, each seeing the one before, joining its two ends."""
    index_of = {cell: i for i, cell in enumerate(nodes)}

    def node(cell):
        if cell not in index_of:
            index_of[cell] = len(nodes)
            nodes.append(cell)
        return index_of[cell]

    last = path[0]
    for before, cell in itertools.pairwise(path):
        if math.dist(last, cell) >= spacing or not clear_between(room, last, cell):
            edges.add((node(last), node(before)))
            last = before
    edges.add((node(last), node(path[-1])))


def bridge(nodes, edges, clearance, floor, top, spacing):
    """Joins the graph's parts through the widest way each has, down to clearance `floor`.

    Returns the nodes left in parts no way reaches, a closet or an island, out of the graph.
    """
    while True:
        groups = groups_of(len(nodes), edges)
        counts = collections.Counter(groups)
        if len(counts) == 1:
            return
        small = min(counts, key=counts.get)
        starts = [n for n, g in zip(nodes, groups, strict=True) if g == small]
        targets = {n for n, g in zip(nodes, groups, strict=True) if g != small}
        path = None
        for width in np.arange(top, floor - 1e-9, -NARROWING_M):
            room = clearance >= width
            path = way_between(starts, targets, room)
            if path:
                break
        if path:
            add_chain(nodes, edges, path, room, spacing)
            continue
        keep = [i for i, g in enumerate(groups) if g != small]
        renumber = {old: new for new, old in enumerate(keep)}
        moved = {(renumber[i], renumber[j]) for i, j in edges if i in renumber and j in renumber}
        edges.clear()
        edges.update(moved)
        nodes[:] = [nodes[i] for i in keep]


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
    clearance = clearance_of(free) * resolution
    open_room = clearance >= args.robot_radius + MARGIN_M
    spacing = SPACING_M / resolution
    nodes = spread_nodes(clearance, open_room, spacing)
    edges = join_by_sight(nodes, open_room, REACH_M / resolution)
    bridge(nodes, edges, clearance, args.robot_radius, args.robot_radius + MARGIN_M, spacing)
    args.out.write_text(json.dumps(geojson(nodes, edges, resolution, origin)) + "\n")
    groups = len(set(groups_of(len(nodes), edges)))
    print(f"{len(nodes)} nodes, {len(edges)} edges both ways, {groups} connected part(s)")


if __name__ == "__main__":
    main()
