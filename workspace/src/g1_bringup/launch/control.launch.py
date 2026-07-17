"""Composition-pure control stack: robot_state_publisher + controller_manager.

No sim, no arm_sdk_sim_bridge here -- this is the launch file that carries
over unchanged to hardware bring-up (see README.md's domain/DDS story).
Included by sim.launch.py for the simulation milestone.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# controller_manager (and its dynamically pluginlib-loaded controllers) is
# launched via `ros2 run` inside an ExecuteProcess, not launch_ros's own
# Node action. This isn't stylistic: `Node` execs the resolved binary path
# directly, and under that invocation arm_trajectory_controller's on_init
# was observed to reliably declare its own parameters (joints,
# command_interfaces, state_interfaces) as empty -- every time, regardless
# of retries, unload/reload cycles, or a completely separate `ros2 run`
# spawner targeting the very same running controller_manager -- while
# joint_state_broadcaster (which has no required parameters) never showed
# any problem. Going through the `ros2` CLI's own entry point instead
# (confirmed directly, repeatedly) makes the exact same controllers.yaml
# configure correctly every time. Root cause not fully chased into
# launch_ros/rclcpp internals; this is the verified fix.
#
# `ros2 run` is itself a Python wrapper that starts the real binary as its
# own subprocess (not an exec-replace) and only relies on the OS delivering
# signals to the whole foreground process group, as a terminal would --
# it has no SIGTERM handler of its own to forward a directed kill. Launched
# bare under `ros2 launch`, a stop signal reaches only that wrapper (which
# has no handler for it and just dies), orphaning the actual
# ros2_control_node binary underneath -- confirmed directly. The
# `set -m; ... & child=$!; trap ...; wait $child` shell wrapper below puts
# the wrapper-and-binary pair in their own process group and explicitly
# forwards TERM/INT to it, so stopping this action (including sim.launch.py's
# whole-launch teardown) actually stops the real process.
_SIGNAL_FORWARDING_WRAPPER = "set -m; {command} & child=$!; trap 'kill -TERM -$child 2>/dev/null' TERM INT; wait $child"


def generate_launch_description():
    g1_description_share = get_package_share_directory("g1_description")
    g1_bringup_share = get_package_share_directory("g1_bringup")

    xacro_path = os.path.join(g1_description_share, "urdf", "g1_arm_sdk.urdf.xacro")
    controllers_yaml = os.path.join(g1_bringup_share, "config", "controllers.yaml")

    robot_description_content = Command([FindExecutable(name="xacro"), " ", xacro_path])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # robot_description is delivered via the (non-deprecated) '~/robot_description'
    # topic robot_state_publisher publishes, remapped here to the global
    # /robot_description name.
    control_node = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            _SIGNAL_FORWARDING_WRAPPER.format(
                command=(
                    "ros2 run controller_manager ros2_control_node --ros-args "
                    "-r '~/robot_description:=/robot_description' "
                    f"--params-file {controllers_yaml}"
                )
            ),
        ],
        name="ros2_control_node",
        output="screen",
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"],
        name="joint_state_broadcaster_spawner",
        output="screen",
    )

    # Loaded and configured, but left inactive: G1ArmSdkSystem itself starts
    # inactive (controllers.yaml's hardware_components_initial_state), and
    # the mandatory acquire order is component-then-controller (see
    # README.md) -- activate_arm.launch.py is the explicit entry point for
    # both.
    arm_trajectory_controller_spawner = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "arm_trajectory_controller",
            "--inactive",
        ],
        name="arm_trajectory_controller_spawner",
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner,
            arm_trajectory_controller_spawner,
        ]
    )
