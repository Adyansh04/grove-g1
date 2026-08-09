"""LiDAR-inertial odometry: the Mid360 front end, FAST-LIO, and odom -> base_footprint.

The only odometry that runs on the real robot. `sim:=true` swaps the front end for the
simulator's, and nothing behind it changes -- FAST-LIO and the odometry publisher see the same
two topics either way, which is the point of the split.

    front end                                  back end
    hardware:  livox_ros_driver2 ---------->  /livox/custom_msg  --> fastlio_mapping
               g1_livox_pointcloud ------->  /livox/lidar           |  /Odometry_loc
    sim:       g1_livox_bridge ------------>  both of the above      v
                                                              g1_odometry_publisher

The odometry publisher is a lifecycle node and is configured and activated here. It is the ONE
owner of odom -> base_footprint, so bringing this up alongside the converged track's publisher
would put two writers on the same transform; g1_bringup's `odometry:=` argument picks one.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("g1_state_estimation")
    sim = LaunchConfiguration("sim").perform(context).lower() == "true"

    fastlio_config = os.path.join(
        share, "config", "fastlio_mid360_sim.yaml" if sim else "fastlio_mid360_hardware.yaml"
    )

    actions = []
    if sim:
        # Restates the relay's PointCloud2 and LowState as the driver's two topics.
        actions.append(
            Node(
                package="g1_sensor_relay",
                executable="g1_livox_bridge",
                name="g1_livox_bridge",
                output="both",
            )
        )
    else:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("livox_ros_driver2"),
                        "launch",
                        "msg_MID360_launch.py",
                    )
                )
            )
        )
        # The driver is in CustomMsg mode for FAST-LIO's sake, so /livox/lidar has to come
        # from somewhere. Everything except FAST-LIO reads it.
        actions.append(
            Node(
                package="g1_state_estimation",
                executable="g1_livox_pointcloud",
                name="g1_livox_pointcloud",
                output="both",
            )
        )

    # Publishes camera_init -> body on /tf as well as /Odometry_loc. That pair is an orphan
    # tree, disconnected from odom: harmless, and worth having when a scan match goes wrong.
    actions.append(
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="both",
            parameters=[fastlio_config, {"use_sim_time": False}],
        )
    )

    # In sim FAST-LIO runs off the pelvis IMU, so its `body` already is the pelvis and the
    # shipped mid360_imu offset must not be applied on top.
    overrides = [{"lidar_body_frame_id": ""}] if sim else []
    odometry_node = LifecycleNode(
        package="g1_state_estimation",
        executable="g1_odometry_publisher",
        name="g1_odometry_publisher",
        namespace="",
        output="both",
        parameters=[os.path.join(share, "config", "g1_odometry_publisher_fastlio.yaml")]
        + overrides,
        remappings=[
            ("~/lidar_odometry", "/Odometry_loc"),
            # Only to level the odom frame at the origin latch; the attitude published
            # afterwards is FAST-LIO's, whose heading does not drift.
            ("~/imu_state", "/lowstate"),
        ],
    )
    actions.append(odometry_node)
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
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="Take the LiDAR and IMU from the simulator via g1_livox_bridge "
                "instead of from livox_ros_driver2. Defaults to the robot.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
