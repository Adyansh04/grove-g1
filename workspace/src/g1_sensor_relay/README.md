# g1_sensor_relay

Publishes the sensor data sampled inside the patched `unitree_mujoco`. The simulator computes the
LiDAR sweep and the camera render against its own `mjData` and hands finished frames over a local
socket; this node turns them into ROS 2 messages.

`ament_cmake`, C++20. Simulation only.

```mermaid
flowchart LR
    MJ["unitree_mujoco<br/>sweep and render<br/>against mjData"] -- "unix socket<br/>length-prefixed frames" --> R
    R["g1_sensor_relay"] --> PC["/livox/lidar"]
    R --> D["/camera/aligned_depth_to_color/image_raw"]
    R --> C["/camera/color/image_raw"]
    R --> CI["camera_info"]
    PC --> B["g1_livox_bridge"]
    LS["/lowstate"] --> B
    B --> CM["/livox/custom_msg"]
    B --> IM["/livox/imu"]
```

## Why the split exists

`unitree_sdk2` and `rmw_cyclonedds` both call `dds_create_domain` unconditionally, and CycloneDDS
allows exactly one explicit domain creation per domain id per process. They cannot coexist in
either order. So the simulator links no ROS at all, and this node owns the ROS side.

The sweep itself has to happen inside the simulator because it needs the scene: geometry, meshes
and current pose all live in `mjData` in that process, and no DDS topic carries them. The finished
frame is what crosses the boundary.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Sensor data QoS, so a reliable subscriber sees nothing. |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | `32FC1`, metres. Misses are NaN, not 0. |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `rgb8` |
| `/camera/aligned_depth_to_color/camera_info`, `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Same intrinsics. |
| `~/sensor_pose` | `geometry_msgs/msg/PoseStamped` | Where the simulator says the sensor is. Diagnostic. |
| `/livox/custom_msg` | `livox_ros_driver2/msg/CustomMsg` | `g1_livox_bridge` only. Reliable, depth 20, matching the real driver. |
| `/livox/imu` | `sensor_msgs/msg/Imu` | `g1_livox_bridge` only. Reliable, depth 10, matching the real driver. |

Depth and colour come from one render, so they share a pose, a timestamp and intrinsics by
construction. A real D435i only gets that alignment from its own align-depth-to-colour step, which
is why the depth topic is named as if it had run.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `socket_path` | `/tmp/g1_sensors.sock` | Where the relay listens. The simulator connects to it. |
| `frame_id` | `mid360_link` | Frame for the point cloud. |
| `depth_frame_id` | `camera_depth_optical_frame` | REP-145 optical frame. |
| `color_frame_id` | `camera_color_optical_frame` | REP-145 optical frame. |
| `world_frame_id` | `world` | Frame for the diagnostic sensor pose. |

`g1_livox_bridge` takes its own: `cloud_topic`, `low_state_topic`, `custom_msg_topic`,
`imu_topic`, `imu_frame_id` (`pelvis`) and `cloud_target_frame`, which defaults to
`imu_frame_id`. Setting `cloud_target_frame` empty passes the sweep through in `mid360_link`,
which is right only once something models an IMU inside the sensor.

Start order does not matter. The relay listens whenever it comes up and the simulator retries every
cycle, so either process can start, die or restart independently.

The optical frames are not `d435_link`. That is a body frame with x forward, and every depth
consumer assumes the optical convention with z forward, so publishing in the link frame rotates the
cloud 90 degrees.

## Running

The node starts with `sensors:=true`:

```bash
ros2 launch g1_bringup bringup.launch.py sensors:=true rviz:=true
ros2 topic hz /livox/lidar
```

## g1_livox_bridge

A second executable, run only when FAST-LIO is asked for (`odometry:=fast_lio`, via
`g1_state_estimation`'s `fastlio_odometry.launch.py`). It restates the relay's PointCloud2 as
the Livox `CustomMsg` FAST-LIO consumes, and the `/lowstate` IMU as `sensor_msgs/Imu` on
`/livox/imu` -- the two topics `livox_ros_driver2` publishes on the robot, so the odometry
pipeline downstream is identical in both places.

Every point goes out with `offset_time` zero, which is truthful rather than a shortcut: the
simulator raycasts the whole sweep against a frozen snapshot, so there is no motion inside a
frame for FAST-LIO's undistortion to undo. A real Mid360 sweeps continuously, which is exactly
what this bridge cannot reproduce and why undistortion stays unvalidated until hardware.

The sweep is also rotated out of `mid360_link` and into the IMU's frame (`cloud_target_frame`,
the pelvis) on the way through, using TF at the sweep's own timestamp. On the robot this would
be pointless -- the Mid360's IMU is bolted beside its laser -- but here the stand-in IMU is the
pelvis, and the three waist joints between the two are driven by the walking policy through tens
of degrees. Handing FAST-LIO a fixed extrinsic across them is what made it lose the scan match
whenever the robot turned. Doing it here rather than in FAST-LIO's config is the only option:
FAST-LIO reads a flat parameter file and cannot consult TF.

## Layout

| File | Contents |
|---|---|
| `frame_reader.{hpp,cpp}` | Framing and validation, free of ROS and sockets so the wire format tests without a simulator. |
| `g1_sensor_relay_node.cpp` | The socket, the poll loop and the publishers. |
| `g1_livox_bridge_node.cpp` | The FAST-LIO front end above. |
| `sensor_frame.h` | The wire struct, duplicated on the simulator side. |

The wire format is untrusted input: every length is validated before it is trusted, including the
one multiply that could overflow.

## Tests

```bash
colcon test --packages-select g1_sensor_relay
```

`test_frame_reader` covers the framing, the bounds checks and a drift check that reads both copies
of `sensor_frame.h`. No simulator needed. `g1_bringup`'s `test_lidar_geometry` asserts the
published cloud measures the room it is in.
