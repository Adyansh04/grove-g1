"""Composition-pure control stack: robot_state_publisher + controller_manager.

No simulator, so this is the file that carries to hardware bring-up unchanged. G1LowCmdSystem
owns all 29 body motors with no onboard balance underneath: the policy spawned here IS the
balance controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

XACRO_PATH = os.path.join(
    get_package_share_directory("g1_description"), "urdf", "g1_lowcmd.urdf.xacro"
)
CONTROLLERS_YAML = os.path.join(
    get_package_share_directory("g1_controllers"), "config", "lowcmd_controllers.yaml"
)

# Forwards SIGTERM/INT to the `ros2 run` subprocess, which would otherwise be orphaned, and
# re-waits so launch sees a clean exit.
_SIGNAL_FORWARDING_WRAPPER = (
    "set -m; {command} & child=$!; "
    "trap 'kill -TERM -$child 2>/dev/null; wait $child' TERM INT; "
    "wait $child"
)

# Every body joint must be claimed from the start: one the component sees unclaimed is one it
# leaves unpowered. The policy and the safety controller it writes through activate in one
# switch, because a chainable controller's reference interfaces only become claimable inside it.
# The four loaded --inactive are switched in later, by the arm bracket and the safety controller.
CONTROLLERS = (
    (["joint_state_broadcaster"], []),
    (["imu_sensor_broadcaster"], []),
    (["waist_freeze_controller"], []),
    (["arm_freeze_controller"], []),
    (["locomotion_safety_controller", "agile_controller"], ["--activate-as-group"]),
    (["locomotion_freeze_controller"], ["--inactive"]),
    (["arm_trajectory_controller"], ["--inactive"]),
    (["left_hand_controller"], ["--inactive"]),
    (["right_hand_controller"], ["--inactive"]),
)


def _spawners():
    return [
        ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", *names, *extra],
            name=f"{names[0]}_spawner",
            output="screen",
        )
        for names, extra in CONTROLLERS
    ]


def _robot_state_publisher():
    description = Command([FindExecutable(name="xacro"), " ", XACRO_PATH])
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
    )


def _control_node():
    """`ros2 run` rather than launch_ros Node: under Node, arm_trajectory_controller's
    parameters reliably declare empty."""
    command = (
        "ros2 run controller_manager ros2_control_node --ros-args "
        "-r '~/robot_description:=/robot_description' "
        f"--params-file {CONTROLLERS_YAML}"
    )
    return ExecuteProcess(
        cmd=["bash", "-c", _SIGNAL_FORWARDING_WRAPPER.format(command=command)],
        name="ros2_control_node",
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
    control_node = _control_node()
    return [
        _robot_state_publisher(),
        control_node,
        *_spawners(),
        # Tear down the whole launch if controller_manager dies.
        RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
