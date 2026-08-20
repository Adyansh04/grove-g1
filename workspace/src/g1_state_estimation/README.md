# g1_state_estimation

Publishes `odom` to the robot's base frame, and the TF chain Nav2 and slam_toolbox need.

`ament_cmake`, C++20. One lifecycle node, `g1_odometry_publisher`, plus the hardware-side
`g1_livox_pointcloud` converter.

```mermaid
flowchart LR
    subgraph sim only
        SM["/g1_sensor_relay/base_state<br/>pelvis pose and twist"] --> N
    end
    subgraph either
        LO["/Odometry_loc<br/>FAST-LIO pose"] --> N
        LS["/imu_sensor_broadcaster/imu<br/>IMU orientation"] --> N
    end
    N["g1_odometry_publisher"] --> TF["/tf<br/>odom to base_footprint to pelvis"]
    N --> OD["~/odom"]
```

## Odometry source

The real G1 publishes no odometry. On hardware `/sportmodestate` carries only `fsm_id`,
`fsm_mode`, `task_id` and `task_time`: no pose, no velocity. What the robot does have is a Mid360
and an IMU, so the hardware source is LiDAR-inertial odometry, FAST-LIO2 from the vendored
`fast_lio` package.

| `odometry_source` | Behaviour |
|---|---|
| `ground_truth` | The `unitree_mujoco` track. Pose and twist together from the simulator's own pelvis sample, relayed off the sensor socket. Exact MuJoCo state: no drift, no noise, no latency. |
| `fast_lio` | Reads FAST-LIO's `nav_msgs/Odometry`, re-references it into `odom`, and differences the twist FAST-LIO leaves empty. An estimate, so it drifts, and `map -> odom` exists to correct it. Runs in sim and on the robot. |
| `hardware` (default) | Refuses to configure, pointing at `fast_lio`. |

`hardware` is the default so that a misconfigured bring-up cannot silently emit fabricated
odometry. Every working source has to be opted into. The launch files select `fast_lio`;
`ground_truth` stays available to isolate a fault to "not the odometry".

No `map` to `odom` transform is published here. That belongs to localization, in `g1_navigation`.

### How the fast_lio source works

FAST-LIO reports the pose of its IMU (`body`) in the frame that IMU occupied at startup
(`camera_init`). Neither is gravity-aligned on this robot, because the Mid360 is mounted upside
down. At the first sample the node latches a correction so `base_footprint` starts at (0, 0, yaw 0)
with the floor at z 0, using the pelvis IMU attitude to find which way is up and `start_height_m`
to find how far up. Everything after is FAST-LIO's motion re-expressed in that frame, offset from
the reporting IMU to the pelvis through TF (`lidar_body_frame_id`).

The pelvis IMU keeps working after the latch, as the gravity reference for roll and pitch. FAST-LIO
carries gravity as an estimated state and lets it wander, and nothing downstream would notice:
AMCL corrects x, y and yaw only, so a slowly tilting `odom` lands on the costmap instead, as floor
lifted over the obstacle cut. The node low-passes the difference between the two attitudes and
applies that. Only the slow part, because substituting the IMU's tilt outright would pin a fresh
IMU sample onto a scan one FAST-LIO period older, and the pelvis swings several degrees inside a
gait cycle. Heading is never touched: it comes from the scan match, which is what does not drift.

Nothing is published until both a LiDAR pose and a usable IMU attitude have arrived.

The TF chain from the sensor down to the pelvis crosses the three waist joints, and is looked up
per sample rather than cached because the waist moves. On the robot that means `/joint_states` has
to carry `waist_yaw_joint`, `waist_roll_joint` and `waist_pitch_joint`, or `mid360_imu -> pelvis`
never resolves and this source publishes nothing at all. The hardware component exports all 29
motors, so `joint_state_broadcaster` covers it on both tracks.

### The two front ends

```
hardware:  livox_ros_driver2 (CustomMsg mode) --> /livox/custom_msg --> fastlio_mapping
           g1_livox_pointcloud: /livox/custom_msg --> /livox/lidar (PointCloud2)
sim:       g1_sensor_relay --> /livox/lidar --> g1_livox_bridge --> /livox/custom_msg
           g1_sensor_relay --> /livox/imu (the Mid360's own, modelled in the MJCF)
```

The driver publishes one format per run. FAST-LIO needs the CustomMsg for its per-point timestamps,
while Nav2's costmaps, MoveIt's octomap and `pointcloud_to_laserscan` all read the PointCloud2. So
on hardware the driver runs in CustomMsg mode and `g1_livox_pointcloud` republishes it, which also
keeps `/livox/lidar` off FAST-LIO's critical path so perception cannot be taken down by it. In
simulation the conversion runs the other way, in `g1_sensor_relay`.

The IMU is the Mid360's own on both tracks: rigid with the laser inside one housing, so the
extrinsic is the few-centimetre offset Livox publish, `lidar_body_frame_id` is `mid360_imu`, and
the two configs carry the same numbers. It cannot be the pelvis IMU, because the waist joints
between them move through tens of degrees and FAST-LIO accepts one constant extrinsic.
`test_sim_extrinsic` asserts both.

## Frames

```
odom -> base_footprint -> pelvis
```

Nav2 and slam_toolbox both want a gravity-aligned base frame, and a walking G1 does not have one:
the pelvis rolls and pitches several degrees with the gait, and every sensor frame hangs off it.
`base_footprint` is the REP-105 ground projection, carrying x, y and heading with z pinned to zero,
and `base_footprint -> pelvis` carries the height and tilt the projection drops.

The pelvis edge is inserted into the chain rather than published as a second edge off `odom`. Two
sibling edges would make every sensor lookup compose two independently published dynamic
transforms, which agree only if they always carry an identical stamp. One chain cannot be
inconsistent even in principle, and both edges go out in a single `sendTransform` call for the same
reason.

Past `max_tilt_deg` the heading extraction is ill-conditioned, so the last well-conditioned heading
is held. Attitude keeps being published unchanged, because a fallen robot really is tilted.

FAST-LIO also broadcasts its own `camera_init -> body` edge. That pair is an orphan tree,
disconnected from `odom`: harmless, and useful when a scan match goes wrong.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `odometry_source` | `hardware` | See the table above. |
| `odom_frame_id` | `odom` | |
| `base_frame_id` | `base_footprint` | |
| `pelvis_frame_id` | `""` | Empty publishes one edge; naming a link splits it in two. |
| `lidar_body_frame_id` | `""` | `fast_lio` only: the frame FAST-LIO reports the pose of. Empty means it already is the base frame. |
| `start_height_m` | `0.0` | `fast_lio` only: body height above the floor at the origin latch. |
| `max_tilt_deg` | `80.0` | Beyond this the heading is held. |
| `publish_rate_hz` | `50.0` | |
| `publish_odom_msg` | `true` | |
| `source_timeout_ms` | `200.0` | Stops publishing once the sample stamp has not changed for this long. |
| `wall_timeout_ms` | `2000.0` | The same bound, and the tighter of the two decides. Both are measured on a steady clock, which a wedged simulator cannot freeze. |
| `tilt_correction_gain` | `0.05` | `fast_lio` only: slerp fraction per LiDAR sample toward the IMU's tilt. At ~10 Hz that is a ~2 s time constant. `0.0` disables the correction. |
| `pose_covariance`, `twist_covariance` | `1.0e-6` | Diagonal value. Placeholders, not characterisations. |

Shipped configs: `g1_odometry_publisher_converged.yaml` (sim ground truth),
`g1_odometry_publisher_fastlio.yaml` (LiDAR-inertial), and `fastlio_mid360_hardware.yaml` /
`fastlio_mid360_sim.yaml` for FAST-LIO itself.

`start_height_m` is the one to re-measure on hardware. It is this simulator's standing pelvis, a
property of the robot's stance rather than a constant, and a value a few centimetres high makes
floor returns compute above Nav2's `min_obstacle_height` cut and paint the floor as obstacle.

## Running

```bash
ros2 launch g1_bringup bringup.launch.py sensors:=true odometry:=fast_lio
ros2 run tf2_ros tf2_echo odom base_footprint
```

Expect a few seconds of nothing while FAST-LIO initialises its IMU and the origin latches.

On the robot, `launch/fastlio_odometry.launch.py` (the default `sim:=false`) starts the Livox
driver, the converter, the waist joint-state publisher, FAST-LIO and this publisher in one go. It
needs `robot_state_publisher` and the URDF already running; it stages neither.

The driver reads `config/mid360_hardware.json` from this package, not the one inside the
`livox_ros_driver2` checkout. That checkout is gitignored and regenerated by
`scripts/import-externals.sh`, so nothing in it is version-controlled and a robot's IP cannot live
there. Check the addresses in ours against the robot before the first run.

Configuration is where the source decision is enforced, so a refusal shows up as a failed lifecycle
transition rather than a silent absence of transforms.

`scripts/lio_bench` scores the result against MuJoCo's own pose with nothing else in the loop, and
writes the paired trace to `/tmp/lio_bench_trace.csv`. Over about 21 m of walking it measures a
final gap of 1 cm and a worst case of 10 cm, roughly 0.05% of path, repeatably across runs. It
waits for the robot to stand still before taking its baseline, and aborts rather than scoring a run
where the robot went over.

## What simulation does not validate

The sim track runs the real FAST-LIO binary on the real code path, so the plumbing, the frame math,
the latch and the guards are all exercised against MuJoCo's exact ground truth. These are what it
cannot reach:

| Deferred to hardware | Why sim cannot settle it |
|---|---|
| Motion undistortion | The simulated sweep is an instantaneous raycast against a frozen `mjData`, so every `offset_time` is legitimately zero. A real Mid360 sweeps continuously. |
| The non-repetitive scan pattern | The simulator ray-casts a uniform 360x32 grid. Coverage, density and their effect on the scan match all differ. |
| IMU noise and bias | The modelled IMU carries no noise model, so drift numbers stay attributable to the estimator. |
| Livox SDK networking | Nothing opens a socket to a sensor in sim, so `config/mid360_hardware.json` is unexercised. |
| The inverted mount | Modelled in the URDF, never verified against the real unit. |
| `start_height_m` | A property of the robot's stance. Re-measure it. |

Any tuning done against sim FAST-LIO is unvalidated on hardware.

## Tests

```bash
colcon test --packages-select g1_state_estimation
```

| Test | Covers |
|---|---|
| `test_odom_math` | Ground projection and its recomposition, heading extraction, the tilt guard, pose composition and inversion. |
| `test_odometry_publisher_node` | The node itself: source selection, the hardware refusal, the fast_lio latch and twist, frame chains, timeouts. |
| `test_livox_cloud` | The hardware CustomMsg to PointCloud2 conversion, which no sim path reaches. |
| `test_sim_extrinsic` | That both FAST-LIO configs carry Livox's published lidar-in-IMU offset, that the sensor is still not rigid with the pelvis, and that the mount is inverted. |

None need a simulator. `g1_navigation`'s `test_scan_pipeline` exercises the frame chain against a
live one, and `g1_bringup`'s `test_fastlio_odometry` scores the estimate against ground truth.
