"""Brings up g1_loco_bridge and drives it to `active`.

Lifecycle transitions are event-chained (not delayed). Included by sim.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("g1_locomotion"), "config", "g1_loco_bridge.yaml"
    )

    loco_bridge_node = LifecycleNode(
        package="g1_locomotion",
        executable="g1_loco_bridge",
        name="g1_loco_bridge",
        namespace="",
        output="screen",
        parameters=[params_file],
    )

    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=loco_bridge_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(loco_bridge_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    activate_once_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=loco_bridge_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(loco_bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # No shutdown handler — this bridge doesn't actuate /lowcmd directly.
    return LaunchDescription(
        [
            loco_bridge_node,
            configure_on_start,
            activate_once_configured,
        ]
    )
