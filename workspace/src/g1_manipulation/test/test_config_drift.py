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


def _merged(package, base, overlay, node):
    parameters = dict(_parameters(package, base, node))
    if overlay:
        parameters.update(_parameters(package, overlay, node))
    return parameters


def test_every_hand_given_generated_grasps_is_one_the_adapter_answers():
    # The server asks the adapter for each arm in generated_grasp_arms; the adapter answers its
    # own hand, and the other only when it mirrors. A disagreement fails every pick on that side.
    for overlay in ("", "_inspire"):
        server = _merged(
            "g1_manipulation",
            "g1_manipulation_server.yaml",
            overlay and f"g1_manipulation_server{overlay}.yaml",
            "g1_manipulation_server",
        )
        adapter = _merged(
            "g1_perception",
            "g1_graspgen_adapter.yaml",
            overlay and f"g1_graspgen_adapter{overlay}.yaml",
            "g1_graspgen_adapter",
        )
        answered = {adapter["hand"]} | (
            {"left", "right"} if adapter.get("mirror_other_hand", False) else set()
        )
        assert set(server["generated_grasp_arms"]) <= answered, (overlay, server, adapter)
