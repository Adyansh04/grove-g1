"""LiDAR-inertial odometry: the Mid360 front end, FAST-LIO, and odom -> base_footprint.

The only odometry that runs on the real robot. `sim:=true` swaps the front end; FAST-LIO and the
odometry publisher see the same two topics either way. Needs the URDF already up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

SHARE = get_package_share_directory("g1_state_estimation")
RELAY_SHARE = get_package_share_directory("g1_sensor_relay")


def _config(share, name):
    return os.path.join(share, "config", name)


def _sim_front_end():
    """The relay's PointCloud2 restated as the driver's CustomMsg. /livox/imu comes straight
    off g1_sensor_relay: the simulator models an IMU inside the Mid360."""
    return [
        Node(
            package="g1_sensor_relay",
            executable="g1_livox_bridge",
            name="g1_livox_bridge",
            output="both",
            parameters=[_config(RELAY_SHARE, "g1_livox_bridge.yaml")],
        )
    ]


def _hardware_front_end():
    """The Mid360 driver plus the CustomMsg -> PointCloud2 republisher."""
    return [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="both",
            parameters=[
                _config(SHARE, "livox_driver.yaml"),
                # Only the launch file can resolve this package's share directory.
                {"user_config_path": _config(SHARE, "mid360_hardware.json")},
            ],
            # xfer_format picks the message TYPE, never the topic NAME: with multi_topic 0 the
            # driver always publishes on livox/lidar, where g1_livox_pointcloud puts
            # PointCloud2. Two types on one name without this.
            remappings=[("livox/lidar", "/livox/custom_msg")],
        ),
        Node(
            package="g1_state_estimation",
            executable="g1_livox_pointcloud",
            name="g1_livox_pointcloud",
            output="both",
            parameters=[_config(SHARE, "g1_livox_pointcloud.yaml")],
        ),
    ]


def _fastlio(sim):
    return Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="both",
        parameters=[
            _config(SHARE, "fastlio_mid360_sim.yaml" if sim else "fastlio_mid360_hardware.yaml"),
            {"use_sim_time": False},
        ],
        # Its camera_init -> body pair is a second, disconnected root and nothing consumes it.
        # Left on /tf every TF client sees two unconnected trees.
        remappings=[("/tf", "/fastlio/tf")],
    )


def _odometry_publisher():
    """The one owner of odom -> base_footprint. Lifecycle, so a refused source is a failed
    transition rather than a silent absence of transforms."""
    return LifecycleNode(
        package="g1_state_estimation",
        executable="g1_odometry_publisher",
        name="g1_odometry_publisher",
        namespace="",
        output="both",
        parameters=[_config(SHARE, "g1_odometry_publisher_fastlio.yaml")],
        remappings=[
            ("~/lidar_odometry", "/Odometry_loc"),
            # Gravity reference only; heading comes from the scan match. The broadcaster's copy
            # of the pelvis IMU, so one topic serves sim and hardware.
            ("~/imu", "/imu_sensor_broadcaster/imu"),
        ],
    )


def _bring_up(node):
    """Configure on start, activate once inactive."""

    def _transition(transition_id):
        return EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node), transition_id=transition_id
            )
        )

    return [
        RegisterEventHandler(
            OnProcessStart(
                target_action=node,
                on_start=[_transition(Transition.TRANSITION_CONFIGURE)],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                goal_state="inactive",
                entities=[_transition(Transition.TRANSITION_ACTIVATE)],
            )
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    sim = LaunchConfiguration("sim").perform(context).lower() == "true"

    odometry = _odometry_publisher()
    front_end = _sim_front_end() if sim else _hardware_front_end()
    return front_end + [_fastlio(sim), odometry] + _bring_up(odometry)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Take the LiDAR and IMU from the simulator via g1_livox_bridge "
            "instead of from livox_ros_driver2. Defaults to the robot.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
