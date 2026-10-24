"""Numbers that must agree across packages, where neither package's own tests can see the pair.

No simulator, no ROS graph.
"""

import os

import yaml


def _parameters(package, name, node):
    directory = os.path.join(os.environ["G1_SOURCE_DIR"], package, "config")
    with open(os.path.join(directory, name)) as handle:
        return yaml.safe_load(handle)[node]["ros__parameters"]


def test_one_width_splits_tables_from_objects():
    # Perception publishes anything wider as a surface; the approach closes on such a box's edge
    # and the place sets things down on it rather than into it. One disagreeing package
    # mistakes a tray for a table, or a table for something to walk up to and grasp.
    widths = {
        "g1_perception max_extent_m": _parameters(
            "g1_perception", "g1_object_geometry.yaml", "g1_object_geometry"
        )["max_extent_m"],
        "g1_locomotion surface_min_extent_m": _parameters(
            "g1_locomotion", "g1_base_approach.yaml", "g1_base_approach"
        )["surface_min_extent_m"],
        "g1_manipulation place_container_max_m": _parameters(
            "g1_manipulation", "g1_manipulation_server.yaml", "g1_manipulation_server"
        )["place_container_max_m"],
    }
    assert len(set(widths.values())) == 1, widths
