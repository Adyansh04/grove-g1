"""Starts the mission executor a NervROS agent sends behavior trees to.

Starts nothing else: the simulator, Nav2, MoveIt and the skill servers come from g1_bringup, and
the executor takes the arms itself when a mission needs them.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMS_FILE = os.path.join(
    get_package_share_directory("g1_orchestration"), "config", "nervros_executor.yaml"
)


def _defaults():
    """Argument defaults come from the config file, so it stays the one place to edit."""
    with open(PARAMS_FILE) as params:
        return yaml.safe_load(params)["nervros_executor"]["ros__parameters"]


def generate_launch_description():
    defaults = _defaults()

    arguments = [
        DeclareLaunchArgument(
            "groot2_port",
            default_value=str(defaults["groot2_port"]),
            description=(
                "ZeroMQ port for Groot2 monitoring, or 0 (the default) to disable. It binds every "
                "interface without authentication: never on a network anyone else can reach."
            ),
        ),
        DeclareLaunchArgument(
            "hands_empty_on_attach",
            default_value=str(defaults["hands_empty_on_attach"]).lower(),
            description=(
                "true where nothing can be in a hand when the arms are found already taken, such "
                "as a simulator stack that activates them at bringup."
            ),
        ),
        DeclareLaunchArgument(
            "event_level",
            default_value=str(defaults["event_level"]),
            description="steps for the mission's steps and skill leaves, all for every node.",
        ),
    ]

    # No `name=`: it would remap every node in the process, and the skills node and the leaves'
    # client nodes would all become nervros_executor.
    #
    # On SIGINT the executor halts the mission, waits for the skill servers to go quiet and, when
    # both hands are empty, hands the arms back: several controller_manager calls that together can
    # take longer than launch's 5 s before SIGTERM and 5 s more before SIGKILL, and a kill half way
    # leaves the arms taken and the hands holding nothing that anyone is watching.
    executor = Node(
        package="g1_orchestration",
        executable="nervros_executor",
        output="screen",
        sigterm_timeout="30",
        sigkill_timeout="30",
        parameters=[
            PARAMS_FILE,
            {
                "groot2_port": LaunchConfiguration("groot2_port"),
                "event_level": LaunchConfiguration("event_level"),
                "hands_empty_on_attach": LaunchConfiguration("hands_empty_on_attach"),
            },
        ],
    )

    return LaunchDescription(arguments + [executor])
