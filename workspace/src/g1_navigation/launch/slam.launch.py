"""slam_toolbox building a map, owning map -> odom while it does.

Mapping only; localization is map_server + AMCL in localization.launch.py. Does not include the
scan pipeline: scan.launch.py does, and both modes need it.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMS_FILE = os.path.join(
    get_package_share_directory("g1_navigation"), "config", "slam_mapping.yaml"
)

# slam_toolbox is a LifecycleNode from Jazzy on and comes up unconfigured, so the manager has
# to arrive after it.
MANAGER_DELAY_S = 2.0


def _overrides(context):
    """Parsed here rather than passed through, so a malformed value fails the launch."""
    raw = LaunchConfiguration("params_overrides").perform(context)
    try:
        overrides = json.loads(raw)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"params_overrides is not valid JSON: {raw!r}") from error
    if not isinstance(overrides, dict):
        raise RuntimeError(
            f"params_overrides must be a JSON object, got {type(overrides).__name__}"
        )
    return overrides


def _setup(context, *args, **kwargs):
    overrides = _overrides(context)
    # Later entries win, so the shipped config stays the source of every value the caller did
    # not name. A whole replacement file would drift.
    parameters = [PARAMS_FILE] + ([overrides] if overrides else [])

    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="both",
            parameters=parameters,
        ),
        TimerAction(
            period=MANAGER_DELAY_S,
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_slam",
                    output="both",
                    parameters=[PARAMS_FILE],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_overrides",
            default_value="{}",
            description="JSON object merged over config/slam_mapping.yaml, e.g. "
            "'{\"minimum_travel_distance\": 0.0}'. Match the shipped type exactly: JSON has "
            "no int/float distinction, so 0 becomes an integer and slam_toolbox rejects it "
            "against a double.",
        ),
        OpaqueFunction(function=_setup),
    ])
