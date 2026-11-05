"""The committed route graph fits the map: every lane has room for the robot, and all are one."""

import importlib.util
import json
import os
from pathlib import Path

CONFIG = Path(os.environ["G1_NAVIGATION_CONFIG_DIR"])
PACKAGE = CONFIG.parent
ROBOT_RADIUS_M = 0.45


def _maker():
    spec = importlib.util.spec_from_file_location(
        "make_route_graph", PACKAGE / "scripts" / "make_route_graph.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_every_lane_fits_the_robot_and_all_are_one():
    maker = _maker()
    free, resolution, origin = maker.load_map(PACKAGE / "maps" / "apartment.yaml")
    room = maker.clearance_of(free) * resolution >= ROBOT_RADIUS_M
    graph = json.loads((CONFIG / "route_graph_apartment.geojson").read_text())

    def cell(xy):
        return (
            round((xy[1] - origin[1]) / resolution - 0.5),
            round((xy[0] - origin[0]) / resolution - 0.5),
        )

    nodes = [f for f in graph["features"] if f["geometry"]["type"] == "Point"]
    lanes = [f for f in graph["features"] if f["geometry"]["type"] != "Point"]
    assert len(nodes) > 50
    for lane in lanes:
        a, b = lane["geometry"]["coordinates"][0]
        assert maker.clear_between(room, cell(a), cell(b)), lane["properties"]
    pairs = {(f["properties"]["startid"], f["properties"]["endid"]) for f in lanes}
    assert len(set(maker.groups_of(len(nodes), pairs))) == 1
