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

Neither branch stages robot_state_publisher: this needs the URDF already up, because the pose
it publishes is offset from the sensor to the pelvis through TF.
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


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("g1_state_estimation")
    sim = LaunchConfiguration("sim").perform(context).lower() == "true"

    fastlio_config = os.path.join(
        share, "config", "fastlio_mid360_sim.yaml" if sim else "fastlio_mid360_hardware.yaml"
    )

    actions = []
    if sim:
        # Restates the relay's PointCloud2 as the driver's CustomMsg. /livox/imu comes
        # straight off g1_sensor_relay: the simulator models an IMU inside the Mid360.
        actions.append(
            Node(
                package="g1_sensor_relay",
                executable="g1_livox_bridge",
                name="g1_livox_bridge",
                output="both",
            )
        )
    else:
        # The driver node directly, not the vendored msg_MID360_launch.py. That launch file
        # hardcodes frame_id 'livox_frame', which is not a frame this robot's URDF has, and
        # points user_config_path at a config inside the vcs checkout that
        # import-externals.sh overwrites.
        actions.append(
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="both",
                parameters=[
                    {
                        # CustomMsg. FAST-LIO needs the per-point timestamps, and
                        # g1_livox_pointcloud below covers everyone who wants PointCloud2.
                        "xfer_format": 1,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": 10.0,
                        "output_data_type": 0,
                        # What the URDF calls the sensor, and what the sim publishes.
                        "frame_id": "mid360_link",
                        "user_config_path": os.path.join(
                            share, "config", "mid360_hardware.json"
                        ),
                    }
                ],
                # xfer_format picks the message TYPE but never the topic NAME: with
                # multi_topic 0 the driver always publishes on livox/lidar (lddc.cpp,
                # GetCurrentPublisher). Left alone it would put CustomMsg on the topic
                # g1_livox_pointcloud publishes PointCloud2 to -- two types, one name.
                remappings=[("livox/lidar", "/livox/custom_msg")],
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
        # Without this there is no odometry at all: the waist joints reach /joint_states from
        # nowhere else on the robot, and mid360_imu -> pelvis crosses them. See
        # g1_hardware_interface's README.
        actions.append(
            Node(
                package="g1_hardware_interface",
                executable="g1_lowstate_joint_states",
                name="g1_lowstate_joint_states",
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

    odometry_node = LifecycleNode(
        package="g1_state_estimation",
        executable="g1_odometry_publisher",
        name="g1_odometry_publisher",
        namespace="",
        output="both",
        parameters=[os.path.join(share, "config", "g1_odometry_publisher_fastlio.yaml")],
        remappings=[
            ("~/lidar_odometry", "/Odometry_loc"),
            # Levels the odom frame at the origin latch, then stays on as the gravity
            # reference that keeps FAST-LIO's estimated gravity from wandering. Heading is
            # never taken from here -- that comes from the scan match, which does not drift.
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
