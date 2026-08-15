"""slam_toolbox building a map, owning map -> odom while it does.

Mapping only. Localization went to map_server + AMCL (localization.launch.py) because
slam_toolbox's own localization mode loads a serialized pose graph, which for this map is 33 MB
against 130 KB for the grid -- see maps/README.md.

Does not include the scan pipeline; scan.launch.py does, and both modes need it.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    raw = LaunchConfiguration("params_overrides").perform(context)
    try:
        overrides = json.loads(raw)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"params_overrides is not valid JSON: {raw!r}") from error
    if not isinstance(overrides, dict):
        raise RuntimeError(f"params_overrides must be a JSON object, got {type(overrides).__name__}")

    # Later entries win, so the shipped config stays the single source of every value the
    # caller did not name -- which is the point. A whole replacement file would drift.
    parameters = [
        os.path.join(get_package_share_directory("g1_navigation"), "config", "slam_mapping.yaml")
    ]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="both",
            parameters=parameters,
        ),
        # slam_toolbox is a LifecycleNode from Jazzy on and comes up unconfigured.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_slam",
                    output="both",
                    # bond_timeout 0 disables the heartbeat: bond is a Nav2 server convention
                    # and slam_toolbox creates none, so any non-zero value tears it back down
                    # ~4 s after activating it.
                    parameters=[{
                        "autostart": True,
                        "node_names": ["slam_toolbox"],
                        "bond_timeout": 0.0,
                    }],
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
            "'{\"minimum_travel_distance\": 0.0}'. For the handful of values a test or a "
            "tuning run needs to change without copying the whole file. Match the shipped "
            "type exactly -- JSON has no int/float distinction, so 0 becomes an integer "
            "parameter and slam_toolbox rejects it against a double at startup.",
        ),
        OpaqueFunction(function=_setup),
    ])
