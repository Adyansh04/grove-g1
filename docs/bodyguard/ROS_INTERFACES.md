# Grove-G1 ROS 2 interfaces for Humanoid Bodyguard

## Evidence and verification status

**All interfaces below are source/configuration verified only; none are runtime verified.**
No runtime diagnostic output was supplied for this document, and no ROS graph queries were run.
The earlier verification-script tests used mocks and are not runtime evidence.
Names are resolved defaults with no extra ROS namespace; launch arguments, world selection,
installed overlays, and parameter overrides can change them. Publishers/subscribers below are
configured or implemented endpoints, not a claim that those endpoints are currently active.
“Not established” means the inspected files do not establish an endpoint or installed API.

Environment: Ubuntu 24.04 host, ROS 2 Jazzy in the existing development container,
MuJoCo Unitree G1, and Nav2. Preserve the architecture and restrictions in [AGENTS.md](../../AGENTS.md).

## Simulator and relay entry points

| Entry point | Interface/type and endpoints | Frames and conditions | Evidence |
| --- | --- | --- | --- |
| `g1_bringup/bringup.launch.py` | ROS launch entry point, not a topic/message; composes existing subsystems | Selects simulation and optional navigation/perception/manipulation | [Bringup launch][bringup] |
| `g1_bringup/sim.launch.py` | Launches `/opt/unitree_robotics/unitree_mujoco/simulate/build/unitree_mujoco -r g1 -s g1_grove_scene.staged.xml` and includes control launch | `sensors:=true` gates relay and odometry; `odometry:=fast_lio` or `ground_truth` | [Simulation launch][sim] |
| Simulator → relay socket | Local binary sensor protocol, **not a ROS topic/type**; simulator sends, `g1_sensor_relay` reads `/tmp/g1_sensors.sock` by default | `GROVE_G1_SENSOR_CONFIG` chooses sensor/world config; `GROVE_G1_CAMERA_MOUNTS` supplies mounts | [Simulation launch][sim], [sensor config][sensors], [relay implementation][relay] |
| `g1_sensor_relay/g1_sensor_relay` | ROS node `/g1_sensor_relay`; publishes the sensor topics below | Socket path overridden from the same config the simulator reads | [Simulation launch][sim], [relay parameters][relay-config] |
| `g1_navigation/nav_sim.launch.py` / `nav_stack.launch.py` | First includes a simulator; second stages navigation without a simulator | `mode:=localization` for repeatable fixed goals; navigation servers gated by `nav:=true` | [Navigation wrapper][nav-sim], [navigation stack][nav-stack] |

These are references, not instructions to launch another instance. Never start, stop, reset,
or reconfigure running simulation/controllers automatically. The simulation launch enforces
DDS/RMW assumptions; inspect its environment checks rather than substituting DDS settings.
The simulator sensor process uses a socket rather than ROS; low-level motor traffic uses
Unitree DDS separately. The shipped navigation track uses wall time (`use_sim_time: false`),
not an assumed `/clock`; the relay maps capture times to wall-clock stamps.

## Cameras

Every row has publisher `/g1_sensor_relay`, is **not runtime verified**, and is supported by
[relay parameters][relay-config] and [publisher construction/header/encoding code][relay].
Images use sensor-data QoS; avoid assuming a reliable-only subscriber will receive them.
RGB is `rgb8`; depth is `32FC1` in metres. Matching topic names do not prove synchronization,
valid calibration, fresh frames, or useful image content.

| Topic | ROS type | `header.frame_id` | Source-verified subscribers when enabled |
| --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `camera_color_optical_frame` | Vision `/g1_detector`, instruction grounder, visualizer in [perception launch][perception] |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_color_optical_frame` | Mock detector and visualizer in [perception launch][perception] |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | `camera_depth_optical_frame` | `/g1_object_geometry`, mock detector, optional grasp/mask components in [perception launch][perception] |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_depth_optical_frame` | `/g1_object_geometry`, optional grasp/mask components in [perception launch][perception] |
| `/chest_camera/color/image_raw` | `sensor_msgs/msg/Image` | `chest_camera_color_optical_frame` | Not established in inspected head-camera perception launch |
| `/chest_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `chest_camera_color_optical_frame` | Not established in inspected head-camera perception launch |
| `/chest_camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | `chest_camera_depth_optical_frame` | Not established in inspected head-camera perception launch |
| `/chest_camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | `chest_camera_depth_optical_frame` | Not established in inspected head-camera perception launch |

The relay declares both cameras, but the standard [sensor config][sensors] disables chest
rendering by default. `cameras` launch selection and world-specific sensor configs can differ.
Publisher existence therefore does not establish image delivery. Head optics attach to
`d435_link`; chest optics attach to `chest_camera_link`, mounted on `torso_link`.
See [camera mounts][mounts] and [description additions][description]. Both optical frames
use x right, y down, z forward; the color/depth optical origins are colocated in this simulation.
Do not assume these simulated extrinsics apply to real camera calibration.

## LiDAR, IMUs, and odometry

Every row is **not runtime verified**. Frame names are source/configured defaults.

| Interface | ROS type | Publisher → subscriber(s) established by source/config | Coordinate frame | Evidence |
| --- | --- | --- | --- | --- |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Simulation relay → `g1_livox_bridge`, `pointcloud_to_laserscan`, Nav2 costmaps/collision monitor; optional MoveIt octomap updater. Hardware `g1_livox_pointcloud` supplies the same topic | `mid360_link` in simulation | [Relay][relay], [FAST-LIO launch][lio-launch], [scan launch][scan-launch], [Nav2 config][nav-config], [MoveIt sensors][moveit-sensors] |
| `/livox/custom_msg` | `livox_ros_driver2/msg/CustomMsg` | Simulation `g1_livox_bridge` → `fastlio_mapping`; hardware `livox_lidar_publisher` → FAST-LIO and `g1_livox_pointcloud` | Bridge preserves input header; hardware header must be checked | [Bridge source][bridge], [bridge config][bridge-config], [FAST-LIO launch][lio-launch], [FAST-LIO parameters][lio-config] |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Simulation relay or hardware Livox driver → `fastlio_mapping` | Simulation `mid360_imu`; hardware frame not verified | [Relay][relay], [relay config][relay-config], [FAST-LIO parameters][lio-config] |
| `/imu_sensor_broadcaster/imu` | `sensor_msgs/msg/Imu` | `imu_sensor_broadcaster` from ros2_control state → `g1_odometry_publisher` in FAST-LIO mode | `pelvis` | [Controller config][controllers], [FAST-LIO launch][lio-launch], [odometry implementation][odom-source] |
| `/g1_sensor_relay/sensor_pose` | `geometry_msgs/msg/PoseStamped` | Relay → no production subscriber established here; geometry diagnostic input | Header `odom`, pose is exact simulator LiDAR pose; **not a TF broadcaster** | [Relay][relay], [relay config][relay-config] |
| `/g1_sensor_relay/base_state` | `nav_msgs/msg/Odometry` | Relay → `g1_odometry_publisher` only in ground-truth mode | Header `odom`, child `pelvis`; twist body-relative | [Relay][relay], [simulation launch][sim], [ground-truth config][gt-config] |
| `/Odometry_loc` | `nav_msgs/msg/Odometry` | `fastlio_mapping` → `g1_odometry_publisher` via `~/lidar_odometry` remap | Header `camera_init`, child `body`; interpreted with configured `lidar_body_frame_id: mid360_imu` extrinsic | [FAST-LIO implementation][lio-source], [FAST-LIO launch][lio-launch], [odometry config][odom-config] |
| `/g1_odometry_publisher/odom` | `nav_msgs/msg/Odometry` | `g1_odometry_publisher` → Nav2 `controller_server` (remap), `bt_navigator` (parameter) | Header `odom`, child `base_footprint`; body-relative twist | [Odometry implementation][odom-source], [Nav2 launch][nav-launch], [Nav2 config][nav-config] |
| `/scan` | `sensor_msgs/msg/LaserScan` | `pointcloud_to_laserscan` → AMCL or slam_toolbox | `base_footprint` target; cloud transformed before height filtering | [Scan launch][scan-launch], [scan config][scan-config], [localization config][localization-config], [SLAM config][slam-config] |

`ground_truth` uses exact MuJoCo pelvis state. It is a simulation diagnostic baseline, not
perception or a hardware state estimate. `fast_lio` estimates motion from LiDAR and its IMU;
it drifts. The odometry publisher also uses the pelvis IMU to level attitude, transforms the
LiDAR-body extrinsic, and supplies the shared navigation odometry/TF interface.
Configured output rate is 50 Hz in both odometry configs; configured sweep/IMU rates are
10/200 Hz in the standard sensor config. These are settings, **not measured rates**.
Select exactly one odometry source. Do not feed `/Odometry_loc` directly to Nav2 as if it
were already in the navigation frame chain.

## TF hierarchy

All links below are **not runtime verified**. `/tf` and `/tf_static` carry
`tf2_msgs/msg/TFMessage`; each transform names its own parent and child.
Consumers include Nav2, scan conversion, perception geometry, the relay's ground-truth object
projection, and other TF clients. Installed endpoints and timestamp validity need runtime checks.

```text
map
└── odom                         AMCL OR slam_toolbox
    └── base_footprint            g1_odometry_publisher
        └── pelvis               g1_odometry_publisher
            └── waist_yaw_link   robot_state_publisher + joint states
                └── waist_roll_link
                    └── torso_link
                        ├── d435_link
                        │   ├── camera_color_optical_frame
                        │   └── camera_depth_optical_frame
                        ├── chest_camera_link
                        │   ├── chest_camera_color_optical_frame
                        │   └── chest_camera_depth_optical_frame
                        └── mid360_link
                            └── mid360_imu
```

Evidence: [odometry implementation][odom-source], [odometry configs][odom-config],
[base URDF][urdf], [description additions][description], [localization config][localization-config],
[SLAM config][slam-config]. `base_footprint` is gravity aligned; `pelvis` retains body attitude.
The URDF root is `pelvis`; its example `world` joint is commented out.
Do not introduce a second `world → pelvis` or `odom → base_footprint` owner.
Fixed joints are published on `/tf_static`, movable joints on `/tf` by
`robot_state_publisher`. `/joint_states` is `sensor_msgs/msg/JointState` from
`joint_state_broadcaster` to `robot_state_publisher` (standard broadcaster API/configuration,
installed endpoint unverified); it is essential for waist-dependent sensor extrinsics.
See [control launch][control] and [controller config][controllers].

FAST-LIO's separate `camera_init → body` broadcast is remapped to `/fastlio/tf`
(`tf2_msgs/msg/TFMessage`), so it does not create a disconnected root on normal `/tf`.
Publisher is FAST-LIO; no production subscriber to this remapped TF topic was established.
See [FAST-LIO launch][lio-launch] and [FAST-LIO implementation][lio-source].

## Nav2 action and lifecycle interfaces

| Interface | Type | Endpoint ownership and coordinate frame | Evidence / status |
| --- | --- | --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Server `/bt_navigator`; existing demo/test clients submit goals. Goal contains `geometry_msgs/PoseStamped`, normally in `map` | [Demo][demo], [navigation test][nav-test], [Nav2 launch][nav-launch]. Not runtime verified; no goal sent |
| `/<node>/get_state` | `lifecycle_msgs/srv/GetState` | Standard lifecycle read service on the managed nodes below; request has no coordinate frame | Standard ROS lifecycle API; lifecycle node construction/config in [odometry source][odom-source] and launch files below. Installed service types/endpoints not runtime verified |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map_server` in localization, or slam_toolbox in mapping → AMCL (localization), global costmap; header expected `map` | [Localization config][localization-config], [SLAM config][slam-config], [Nav2 config][nav-config]. Standard package output type; installed endpoint not runtime verified |

`lifecycle_manager_navigation` manages `controller_server`, `planner_server`,
`behavior_server`, `bt_navigator`, and `collision_monitor` with autostart enabled.
`lifecycle_manager_localization` manages `map_server` and `amcl`.
`g1_odometry_publisher` is lifecycle-enabled and self-configures/activates with its
`autostart` setting. Optional keepout mode adds `costmap_filter_info_server` and
`lifecycle_manager_filters`. Managers orchestrate lifecycle nodes; do not assume that the
managers themselves expose the same lifecycle state service.
Evidence: [Nav2 launch][nav-launch], [Nav2 config][nav-config],
[localization launch][localization-launch], [ground-truth][gt-config] and [FAST-LIO odometry][odom-config] configs.

Fixed-goal readiness requires active navigation/localization nodes, an action server,
fresh sensor/odometry inputs, valid map and TF, and suitable costmaps. Advertisement alone
proves none of goal execution, collision avoidance, or stable locomotion. The launch explicitly
rejects composed Nav2 servers because nested costmap parameters fail in that configuration.
Do not change composition or lifecycle state as a diagnostic shortcut.

## Velocity-command path and collision monitor

```text
controller_server / behavior_server
  → /cmd_vel_raw → collision_monitor → /cmd_vel → agile_controller
                                                   ↓ chained references
                                      locomotion_safety_controller
                                                   ↓ ros2_control
                                             G1LowCmdSystem
                                                   ↓ Unitree DDS
                                               rt/lowcmd
```

| Interface | Type | Publisher → subscriber | Frame / evidence / status |
| --- | --- | --- | --- |
| `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | Nav2 `controller_server` or `behavior_server` → `collision_monitor` | Unstamped body velocity; intended `base_footprint` convention, no header to enforce it. [Nav2 remaps][nav-launch], [unstamped monitor config][nav-config]. Not runtime verified |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `collision_monitor`, **also `g1_base_approach` directly** → `agile_controller` | Body velocity; gait consumes planar x/y and yaw. [Monitor config][nav-config], [approach source][approach], [AGILE source][agile]. Not runtime verified |
| Safety chained references | ros2_control command/reference interfaces, **not a ROS topic/message** | `agile_controller` → `locomotion_safety_controller` → hardware command interfaces | Joint space, no TF frame. [Controller config][controllers], [AGILE source][agile], [safety source][safety]. Not runtime verified |

The collision monitor is the final cloud-based check on **Nav2's** velocity path. It uses
`/livox/lidar`, base `base_footprint`, odometry frame `odom`, a 0.35 m circular approach
region, and a 1 s time-before-collision setting. Point-cloud bands are 0.05–0.60 m and
1.05–1.40 m; source timeout is 1 s. It slows/limits velocity based on observed returns.
These configured limits do not establish complete obstacle coverage or a hardware safety guarantee.

`g1_base_approach` intentionally publishes directly to `/cmd_vel` for close approaches;
it does **not** traverse `/cmd_vel_raw` or the collision monitor. The existing launch describes
sequencing it separately from Nav2. There is no verified general-purpose velocity arbiter here.
Future packages must not add competing publishers or assume all `/cmd_vel` traffic is filtered.
AGILE's configured stale-command timeout is 0.5 s in its implementation defaults; a zero
velocity request is not permission to stop the balance controller.

## Controller ownership and safety boundaries

The control launch runs `controller_manager/ros2_control_node`, broadcasters,
`robot_state_publisher`, and controller spawners. The configured update rate is 200 Hz;
policy inference is 50 Hz through decimation. These are configuration values, not timing evidence.

| Owner/interface | Type | Ownership, endpoint/frame, and evidence | Status |
| --- | --- | --- | --- |
| `G1LowCmdSystem` | `g1_hardware_interface/G1LowCmdSystem` ros2_control plugin | Sole body command writer, all 29 motors; joint-space interfaces, no single TF frame. [Hardware source][hardware], [control launch][control] | Not runtime verified |
| `rt/lowcmd` | `unitree_hg::msg::dds_::LowCmd_` | `G1LowCmdSystem` → robot or simulator; raw Unitree DDS, **not an ordinary ROS topic**; motor-space data | [Hardware source][hardware]; not runtime verified |
| `rt/lowstate` | `unitree_hg::msg::dds_::LowState_` | Robot/simulator → `G1LowCmdSystem`; raw DDS motor/IMU state, not a framed ROS navigation estimate | [Hardware source][hardware]; not runtime verified |
| `locomotion_safety_controller` | `g1_controllers/G1SafetyController` | Claims 12 leg joints plus waist roll/pitch; receives AGILE chained references; emergency fallback configured to `locomotion_freeze_controller` | [Controller config][controllers], [safety source][safety]; not runtime verified |
| `waist_freeze_controller` | `g1_controllers/G1FreezeController` | Owns waist yaw | [Controller config][controllers]; not runtime verified |
| Arm controller pair | `g1_controllers/G1FreezeController` or `joint_trajectory_controller/JointTrajectoryController` | `arm_freeze_controller` or `arm_trajectory_controller` owns 14 arm joints; existing bracket switches ownership atomically | [Controller config][controllers], [control launch][control]; not runtime verified |
| Hand components/controllers | Separate `G1Dex3SystemLeft/Right` components and trajectory controllers | Separate from the 29 body motors; initially inactive under existing arm acquisition flow | [Description][description], [controller config][controllers]; not runtime verified |

The balance policy is the balance controller: no onboard balance runs underneath this whole-body
lowcmd stack. Unclaimed body joints are left unpowered. Pinned-pelvis mode uses an existing
alternative spawn set; diagnostics must not switch it. Preserve safety fallback and controller claims.
Never modify low-level motor control, `rt/lowcmd`, or the balance policy without explicit approval.
Never automatically start/stop/reset/reconfigure simulation or controllers. Docker, `sudo`,
destructive operations, resets/cleans, commits, and pushes require explicit approval under AGENTS.md.

## Suitable boundaries for future bodyguard packages

Prefer separate bodyguard ROS packages and consumers of existing images/CameraInfo, LiDAR,
odometry, TF, and read-only state diagnostics. For future high-level fixed-goal navigation,
use the existing NavigateToPose action after verifying its installed API and readiness; do not
write motor commands or add a parallel gait/velocity-control stack.

Existing optional object interfaces, **not runtime verified**:

| Interface | Type | Publisher → subscriber and frame | Evidence |
| --- | --- | --- | --- |
| `/g1_sensor_relay/object_poses`, `/g1_sensor_relay/chest/object_poses` | `vision_msgs/msg/Detection3DArray` | Relay → head mock detector/object-source consumers; chest consumer not established here. Per-camera color optical frame. **Exact simulation ground truth** | [Relay][relay], [perception launch][perception] |
| `/g1_perception/instance_masks` | `canopy_msgs/msg/InstanceMaskArray` | Configured detector → `g1_object_geometry`; camera image header frame | [Perception launch][perception], [geometry source][geometry] |
| `/g1_object_geometry/object_poses` | `vision_msgs/msg/Detection3DArray` | `g1_object_geometry` → configured perception object-source consumers; image optical frame (tracking uses `odom` internally) | [Geometry source][geometry], [geometry config][geometry-config], [object-source launch][manipulation] |
| `/objects` | `vision_msgs/msg/Detection3DArray` | `g1_object_pose_source` → `g1_base_approach` and manipulation consumers; configured output `odom` | [Object source][object-source], [object-source launch][manipulation], [approach source][approach] |

Object provenance depends on selected source and detector. A mock detector derives masks from
simulator truth; passing those masks through geometry does not make the result an independent
perception validation. `/objects` alone does not identify its provenance. These interfaces are
not a verified person-tracking API. Week 1 remains simulator validation, fixed-goal navigation,
interface inspection, and diagnostics; person tracking and the bodyguard Behavior Tree are out of scope.

## Manual runtime verification

Run only inside the existing sourced Jazzy development container. From the repository root:

```bash
bash scripts/bodyguard/verify_week1.sh
```

See [verification usage and limits](week1-verification.md). The script is read-only and bounded;
it sends no navigation goal. Its default chest-camera checks may fail when rendering is disabled,
and mapping-only sessions will not satisfy fixed-goal localization checks. Do not restart anything
to make a diagnostic pass. To inspect an installed interface definition separately:

```bash
timeout --kill-after=2s 5s ros2 interface show nav2_msgs/action/NavigateToPose
timeout --kill-after=2s 5s ros2 topic info /cmd_vel --verbose
timeout --kill-after=2s 5s ros2 param get /g1_odometry_publisher odometry_source
timeout --kill-after=2s 5s ros2 lifecycle get /collision_monitor
```

Record the world, launch selections, domain/RMW, overlay, command, output, and observation time
before upgrading any entry to runtime verified. Check data receipt, QoS, timestamps, header frames,
TF at sensor time, and actual publishers/subscribers. Runtime connectivity still does not prove
calibration, localization accuracy, collision coverage, or safe goal completion.

[bringup]: ../../workspace/src/g1_bringup/launch/bringup.launch.py
[sim]: ../../workspace/src/g1_bringup/launch/sim.launch.py
[sensors]: ../../workspace/src/g1_bringup/config/sim_sensors.yaml
[relay]: ../../workspace/src/g1_sensor_relay/src/g1_sensor_relay_node.cpp
[relay-config]: ../../workspace/src/g1_sensor_relay/config/g1_sensor_relay.yaml
[nav-sim]: ../../workspace/src/g1_navigation/launch/nav_sim.launch.py
[nav-stack]: ../../workspace/src/g1_navigation/launch/nav_stack.launch.py
[mounts]: ../../workspace/src/g1_description/config/cameras.yaml
[description]: ../../workspace/src/g1_description/urdf/g1_common.xacro
[urdf]: ../../workspace/src/g1_description/urdf/g1_29dof_with_hand_rev_1_0.urdf
[perception]: ../../workspace/src/g1_perception/launch/perception.launch.py
[lio-launch]: ../../workspace/src/g1_state_estimation/launch/fastlio_odometry.launch.py
[lio-config]: ../../workspace/src/g1_state_estimation/config/fastlio_mid360_sim.yaml
[lio-source]: ../../workspace/src/fast_lio_humanoid/FAST_LIO/src/laserMapping.cpp
[bridge]: ../../workspace/src/g1_sensor_relay/src/g1_livox_bridge_node.cpp
[bridge-config]: ../../workspace/src/g1_sensor_relay/config/g1_livox_bridge.yaml
[scan-launch]: ../../workspace/src/g1_navigation/launch/scan.launch.py
[scan-config]: ../../workspace/src/g1_navigation/config/scan.yaml
[nav-launch]: ../../workspace/src/g1_navigation/launch/nav2.launch.py
[nav-config]: ../../workspace/src/g1_navigation/config/nav2_params.yaml
[localization-launch]: ../../workspace/src/g1_navigation/launch/localization.launch.py
[localization-config]: ../../workspace/src/g1_navigation/config/localization.yaml
[slam-config]: ../../workspace/src/g1_navigation/config/slam_mapping.yaml
[moveit-sensors]: ../../workspace/src/g1_moveit_config/config/sensors_3d.yaml
[controllers]: ../../workspace/src/g1_controllers/config/lowcmd_controllers.yaml
[odom-source]: ../../workspace/src/g1_state_estimation/src/g1_odometry_publisher_node.cpp
[gt-config]: ../../workspace/src/g1_state_estimation/config/g1_odometry_publisher_converged.yaml
[odom-config]: ../../workspace/src/g1_state_estimation/config/g1_odometry_publisher_fastlio.yaml
[control]: ../../workspace/src/g1_bringup/launch/control.launch.py
[demo]: ../../scripts/demos/navigation-and-moveit.sh
[nav-test]: ../../workspace/src/g1_navigation/test/test_navigate_to_pose.launch.py
[approach]: ../../workspace/src/g1_locomotion/src/g1_base_approach_node.cpp
[agile]: ../../workspace/src/g1_controllers/src/g1_agile_controller.cpp
[safety]: ../../workspace/src/g1_controllers/src/g1_safety_controller.cpp
[hardware]: ../../workspace/src/g1_hardware_interface/src/g1_lowcmd_system.cpp
[geometry]: ../../workspace/src/g1_perception/src/g1_object_geometry_node.cpp
[geometry-config]: ../../workspace/src/g1_perception/config/g1_object_geometry.yaml
[object-source]: ../../workspace/src/g1_manipulation/src/g1_object_pose_source_node.cpp
[manipulation]: ../../workspace/src/g1_manipulation/launch/manipulation.launch.py
