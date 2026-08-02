"""Brings up g1_locomotion's g1_loco_bridge and drives it straight to `active`.

Configure and activate are chained off the node's own lifecycle events
(RegisterEventHandler/OnStateTransition) rather than a fixed delay -- a TimerAction guessing "the
executable has started by now" would race the process's actual startup time exactly like the
sim/bridge DDS-match race sim.launch.py's own SIM_START_DELAY_S comment describes. Included by
sim.launch.py so a normal sim launch brings up the whole LocoClient loop (this bridge talking to
motion_service_sim's protocol-only responder); see each package's README for the topic/parameter/
authority-model documentation.
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

    # No OnProcessExit->Shutdown handler here (unlike sim.launch.py's/control.launch.py's for the
    # sim process/ros2_control_node): this bridge never actuates /lowcmd itself, so its death just
    # means velocity requests stop being sent -- the same safe outcome its own on_shutdown relies
    # on (see g1_locomotion's README), not a dangling control authority worth tearing the whole
    # launch down over.
    return LaunchDescription(
        [
            loco_bridge_node,
            configure_on_start,
            activate_once_configured,
        ]
    )
