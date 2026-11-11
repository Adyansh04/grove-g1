"""robot_state_publisher, ros2_control_node and the controller spawners.

G1LowCmdSystem owns all 29 body motors with no onboard balance underneath: the policy spawned here
is the balance controller. sim.launch.py includes this for the simulator; on the physical G1,
tethered by Ethernet, run it directly:

    ros2 launch g1_bringup control.launch.py hardware:=true robot_address:=192.168.123.222 \
        hands:=false
"""

import os
import socket

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESCRIPTION_SHARE = get_package_share_directory("g1_description")
CONTROLLERS_SHARE = get_package_share_directory("g1_controllers")
XACRO_PATH = os.path.join(DESCRIPTION_SHARE, "urdf", "g1_lowcmd.urdf.xacro")
CONTROLLERS_YAML = os.path.join(CONTROLLERS_SHARE, "config", "lowcmd_controllers.yaml")
HARDWARE_OVERRIDES_YAML = os.path.join(
    CONTROLLERS_SHARE, "config", "lowcmd_controllers_hardware.yaml"
)
LOWCMD_PARAMS_YAML = os.path.join(DESCRIPTION_SHARE, "config", "lowcmd_params.yaml")

# Forwards SIGTERM/INT to the node, which would otherwise be orphaned, and waits until it has
# finished, motor release included, so launch sees a clean exit.
_SIGNAL_FORWARDING_WRAPPER = (
    "set -m; {command} & child=$!; "
    "trap 'kill -TERM -$child 2>/dev/null; wait $child' TERM INT; "
    "wait $child"
)


# Every body joint is claimed from the start: one the component sees unclaimed is left unpowered.
# The policy and the safety controller it chains into activate as one group, because chained
# reference interfaces only become claimable inside the switch that activates them.
def _controllers(legs_frozen, hands):
    """The spawn set, differing in what drives the legs (the freeze and safety controllers claim
    the same 14 joints) and whether there are hands. The --inactive ones stay loaded for a later
    switch, such as activate_arm's, the policy's start on hardware, or the safety controller's
    emergency."""
    legs = (
        [
            (["locomotion_freeze_controller"], []),
            (["locomotion_safety_controller", "agile_controller"], ["--inactive"]),
        ]
        if legs_frozen
        else [
            (["locomotion_safety_controller", "agile_controller"], ["--activate-as-group"]),
            (["locomotion_freeze_controller"], ["--inactive"]),
        ]
    )
    return [
        (["joint_state_broadcaster"], []),
        (["imu_sensor_broadcaster"], []),
        (["waist_freeze_controller"], []),
        (["arm_freeze_controller"], []),
        *legs,
        (["arm_trajectory_controller"], ["--inactive"]),
        *(
            [
                (["left_hand_controller"], ["--inactive"]),
                (["right_hand_controller"], ["--inactive"]),
            ]
            if hands
            else []
        ),
    ]


def _spawners(legs_frozen, hands):
    return [
        ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", *names, *extra],
            name=f"{names[0]}_spawner",
            output="screen",
        )
        for names, extra in _controllers(legs_frozen, hands)
    ]


def _robot_domain_id():
    """The robot's own DDS domain, from g1_description's hardware parameters."""
    with open(LOWCMD_PARAMS_YAML, encoding="utf-8") as handle:
        return int(yaml.safe_load(handle)["hardware"]["domain_id"])


def _hardware_problems(robot_address, environ):
    """Why the hardware stack must not start, before any process runs; empty when it may."""
    problems = []
    rmw = environ.get("RMW_IMPLEMENTATION")
    if rmw != "rmw_fastrtps_cpp":
        problems.append(
            f"RMW_IMPLEMENTATION={rmw!r}: ROS must not load a second CycloneDDS beside the SDK's."
        )
    # ROS on the robot's domain would turn any ROS topic named /lowcmd into a motor command.
    robot_domain = _robot_domain_id()
    if environ.get("ROS_DOMAIN_ID", "0") == str(robot_domain):
        problems.append(
            f"ROS_DOMAIN_ID is {robot_domain}, the robot's own DDS domain; keep ROS off it."
        )
    if not robot_address:
        problems.append("robot_address is empty: give this machine's address on the robot's LAN.")
    else:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.bind((robot_address, 0))
        except OSError as err:
            problems.append(
                f"robot_address {robot_address!r} is not an address of this machine: {err}."
            )
    return problems


def _sdk_dds_config(robot_address):
    """CycloneDDS for unitree_sdk2 inside ros2_control_node, on the robot's LAN only. Pinned by
    address: by interface name it takes the NIC's first address, which on a NIC that also carries
    another network is one the robot cannot answer."""
    return (
        '<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface '
        f'address="{robot_address}" priority="default" multicast="default"/></Interfaces>'
        "</General></Domain></CycloneDDS>"
    )


def _robot_state_publisher(hardware, hands):
    description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            XACRO_PATH,
            f" hardware:={str(hardware).lower()} hands:={str(hands).lower()}",
        ]
    )
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
    )


def _control_node(params_files, environment):
    """The binary under a bash wrapper rather than launch_ros Node: under Node,
    arm_trajectory_controller's parameters reliably declare empty. Not `ros2 run`, whose Python
    process dies on the first SIGTERM while the node is still ramping the motors down. A later
    params file wins, so overrides go last."""
    executable = os.path.join(
        get_package_prefix("controller_manager"), "lib", "controller_manager", "ros2_control_node"
    )
    command = f"{executable} --ros-args -r '~/robot_description:=/robot_description' " + " ".join(
        f"--params-file {path}" for path in params_files
    )
    return ExecuteProcess(
        cmd=["bash", "-c", _SIGNAL_FORWARDING_WRAPPER.format(command=command)],
        name="ros2_control_node",
        output="screen",
        additional_env=environment,
    )


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def _launch_setup(context, *args, **kwargs):
    hardware = _flag(context, "hardware")
    hands = _flag(context, "hands")
    robot_address = LaunchConfiguration("robot_address").perform(context)
    if hardware:
        problems = _hardware_problems(robot_address, os.environ)
        if problems:
            raise RuntimeError(
                "g1_bringup/control.launch.py hardware:=true: refusing to start:\n"
                + "\n".join(f"  - {problem}" for problem in problems)
            )
    # A welded pelvis gives the policy observations its own actions cannot move, so it diverges.
    # On hardware the policy is started by hand, once the robot hangs held and the joints read.
    legs_frozen = _flag(context, "pin_pelvis") or hardware
    overrides = LaunchConfiguration("controllers_overrides").perform(context)
    params_files = [
        CONTROLLERS_YAML,
        *([HARDWARE_OVERRIDES_YAML] if hardware else []),
        *([overrides] if overrides else []),
    ]
    environment = {"CYCLONEDDS_URI": _sdk_dds_config(robot_address)} if hardware else {}
    control_node = _control_node(params_files, environment)
    return [
        _robot_state_publisher(hardware, hands),
        control_node,
        *_spawners(legs_frozen, hands),
        # Tear down the whole launch if controller_manager dies.
        RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pin_pelvis",
                default_value="false",
                description="SIM-ONLY, set by sim.launch.py: freeze the legs instead of running "
                "the balance policy.",
            ),
            DeclareLaunchArgument(
                "controllers_overrides",
                default_value="",
                description="A params file layered over g1_controllers' lowcmd_controllers.yaml, "
                "for tests such as test_emergency_freeze. Empty adds none.",
            ),
            DeclareLaunchArgument(
                "hardware",
                default_value="false",
                choices=["true", "false"],
                description="true drives the physical G1 over Ethernet: the SDK on the robot's "
                "DDS domain, Unitree's onboard controller released (a reboot gives it back), the "
                "legs frozen until the policy is switched in, and lowcmd_controllers_hardware.yaml "
                "layered on. false (default) is the simulator.",
            ),
            DeclareLaunchArgument(
                "robot_address",
                default_value="",
                description="With hardware:=true, required: this machine's IPv4 address on the "
                "robot's LAN (192.168.123.0/24), which the SDK's DDS is pinned to.",
            ),
            DeclareLaunchArgument(
                "hands",
                default_value="true",
                choices=["true", "false"],
                description="false leaves out the Dex3 hand components and their controllers, "
                "for a robot without Dex3 hands.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
