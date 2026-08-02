"""Perception sim track: MuJoCo + mujoco_ros2_control with a 3D LiDAR and RGB-D camera.

SIM-ONLY, and the SECOND simulation track. `g1_bringup`'s sim.launch.py drives
unitree_mujoco for arm and locomotion fidelity; this one drives a simplified mobility
body for sensors. They share ROS_DOMAIN_ID=1 and must never run at the same time, which
_check_environment below enforces rather than trusts.

See g1_sim/README.md for what this body is and is not.
"""

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from lifecycle_msgs.msg import Transition

# g1_bringup owns :133. A separate display so a stray process from either track cannot
# collide with the other's.
XVFB_DISPLAY = ":134"

# The camera renders, so MUJOCO_GL=disable is not an option here and a real GL context is
# required. The container has the NVIDIA runtime, so this is hardware-accelerated.
SIM_START_DELAY_S = 2.0


def _check_environment(context, *args, **kwargs):
    """Refuse to start on a misconfigured environment, or alongside the other sim track.

    Same fail-fast principle as g1_bringup's sim.launch.py: an empty or crosstalking ROS
    graph is far harder to debug than an explicit error here.
    """
    problems = []

    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if rmw != "rmw_cyclonedds_cpp":
        problems.append(
            f"RMW_IMPLEMENTATION={rmw!r}, expected 'rmw_cyclonedds_cpp'."
        )
    if not os.environ.get("CYCLONEDDS_URI"):
        problems.append(
            "CYCLONEDDS_URI is unset -- expected the container-baked cyclonedds.xml."
        )
    domain_id = os.environ.get("ROS_DOMAIN_ID")
    if domain_id != "1":
        problems.append(
            f"ROS_DOMAIN_ID={domain_id!r}, expected '1'. Both sim tracks share domain 1 "
            "deliberately; see g1_sim/README.md."
        )

    if problems:
        raise RuntimeError(
            "g1_sim/perception_sim.launch.py: refusing to start, environment "
            "precondition(s) failed:\n"
            + "\n".join(f"  - {p}" for p in problems)
            + "\nThese are set container-wide by .devcontainer/Dockerfile; a fresh "
            "'manage.sh recreate' is the usual fix."
        )

    # The two tracks share a domain, so a running unitree_mujoco would inject a second
    # /robot_description, a second controller_manager and a competing /tf writer into
    # this graph. Cheaper to detect than to debug: /lowstate is unique to that track.
    import rclpy
    from rclpy.node import Node as RclpyNode

    already_running = False
    rclpy_was_ours = not rclpy.ok()
    try:
        if rclpy_was_ours:
            rclpy.init()
        probe = RclpyNode("g1_sim_track_conflict_probe")
        # One short spin: DDS discovery is not instant, and a false negative here only
        # costs the confusing failure this check exists to prevent.
        end = time.time() + 2.0
        while time.time() < end and not already_running:
            rclpy.spin_once(probe, timeout_sec=0.1)
            already_running = probe.count_publishers("/lowstate") > 0
        probe.destroy_node()
    finally:
        if rclpy_was_ours and rclpy.ok():
            rclpy.shutdown()

    if already_running:
        raise RuntimeError(
            "g1_sim/perception_sim.launch.py: refusing to start -- a publisher on "
            "/lowstate means g1_bringup's unitree_mujoco track is already running on "
            "this domain. The two tracks share ROS_DOMAIN_ID=1 and must not overlap: "
            "they would both publish /robot_description and /tf and both run a node "
            "named controller_manager.\n"
            "Stop it first (Ctrl-C its launch), then confirm nothing survived:\n"
            "  pkill -x unitree_mujoco; pkill -x motion_service_; pkill -x Xvfb\n"
            "(never pkill -f: the container shares the host PID namespace)"
        )
    return []


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("g1_sim")
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"

    robot_description_str = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            os.path.join(pkg_share, "urdf", "g1_perception_base.urdf.xacro"),
        ]
    ).perform(context)
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    controllers_file = os.path.join(pkg_share, "config", "controllers.yaml")
    plugins_file = os.path.join(pkg_share, "config", "mujoco_plugins.yaml")

    # mujoco_ros2_control drives simulated time, so every node in this track reads it.
    # A TF publisher on a different clock than its data is the classic cause of
    # "extrapolation into the future" from tf2.
    use_sim_time = {"use_sim_time": True}

    actions = []
    node_env = dict(os.environ)
    if headless:
        actions.append(
            ExecuteProcess(
                cmd=["Xvfb", XVFB_DISPLAY, "-screen", "0", "1280x1024x24", "-nolisten", "tcp"],
                name="xvfb",
                output="screen",
            )
        )
        node_env["DISPLAY"] = XVFB_DISPLAY

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, use_sim_time],
        )
    )

    # Package-qualified deliberately: controller_manager ships an executable of the same
    # name, and picking the wrong one silently gives a sim-less controller_manager.
    #
    # /joint_states is remapped away so the three planar pseudo-joints never reach
    # robot_state_publisher (they are not in the kinematic tree, by design). The
    # odometry publisher consumes /base_joint_states instead.
    actions.append(
        TimerAction(
            period=SIM_START_DELAY_S if headless else 0.0,
            actions=[
                Node(
                    package="mujoco_ros2_control",
                    executable="ros2_control_node",
                    # No name= override. Setting it puts __node:= on the process, which
                    # renames EVERY node created in it -- including each controller -- so
                    # their node-keyed parameter sections never match and they come up
                    # with empty params. The node already calls itself controller_manager.
                    emulate_tty=True,
                    output="both",
                    env=node_env,
                    parameters=[
                        use_sim_time,
                        ParameterFile(controllers_file),
                        ParameterFile(plugins_file),
                    ],
                    remappings=[
                        ("~/robot_description", "/robot_description"),
                        ("/joint_states", "/base_joint_states"),
                    ],
                    on_exit=Shutdown(reason="mujoco_ros2_control node exited"),
                )
            ],
        )
    )

    for controller in ("joint_state_broadcaster", "base_velocity_controller"):
        actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                # No --param-file here: controllers read their parameters from the file
                # given to the controller_manager above. Passing it again on the spawner
                # sets a competing `params_file` override and the controller comes up with
                # an empty `joints` list.
                arguments=[controller],
                parameters=[use_sim_time],
                output="both",
            )
        )

    # odom -> base_link. sim_ground_truth is set here rather than left to the package
    # default, which is `hardware` and refuses to configure on purpose.
    odometry_params = os.path.join(
        get_package_share_directory("g1_state_estimation"), "config", "g1_odometry_publisher.yaml"
    )
    # base_height_m comes from the same file the MJCF spawn height is checked against, so
    # `odom` is the ground plane and nothing downstream has to subtract a magic number.
    with open(os.path.join(pkg_share, "config", "sensor_mounts.yaml")) as mounts_file:
        base_height = yaml.safe_load(mounts_file)["base_link"]["spawn_z"]

    odometry_node = LifecycleNode(
        package="g1_state_estimation",
        executable="g1_odometry_publisher",
        name="g1_odometry_publisher",
        namespace="",
        output="both",
        parameters=[odometry_params, use_sim_time, {"base_height_m": base_height}],
        remappings=[("~/base_state", "/base_joint_states")],
    )
    actions.append(odometry_node)

    # Event-chained rather than delayed, same shape as g1_bringup's loco.launch.py.
    actions.append(
        RegisterEventHandler(
            OnProcessStart(
                target_action=odometry_node,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(odometry_node),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ],
            )
        )
    )
    actions.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=odometry_node,
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(odometry_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=_check_environment),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Render into our own managed Xvfb instead of a visible window. "
                "The camera requires a real GL context either way, so there is no "
                "render-nothing mode here.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
