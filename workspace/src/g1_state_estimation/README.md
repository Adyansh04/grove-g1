# g1_state_estimation

`odom -> base` state estimation for the G1.

## Odometry source

The real G1 **publishes no odometry**. On hardware `/sportmodestate` carries
`unitree_hg::SportModeState_`, which holds only `fsm_id`, `fsm_mode`, `task_id` and `task_time`. There
is no pose and no velocity, and `rt/odommodestate` does not exist. So this package has three sources
and only two of them are implemented:

| `odometry_source` | Behaviour |
|---|---|
| `sim_sportmodestate` | The converged `unitree_mujoco` track. Pelvis position from `/sportmodestate` (which the simulator fills from `framepos`/`framelinvel` on the pelvis imu site) and full orientation from `/lowstate`'s IMU. |
| `sim_ground_truth` | The `g1_sim` planar sandbox. MuJoCo generalized coordinates read from that track's planar joints. |
| `hardware` (default) | **Refuses to run.** A real source (leg odometry + IMU EKF, or LiDAR-inertial odometry) is a future milestone. |

`hardware` is the default deliberately. A misconfigured hardware bring-up must never silently emit
fabricated odometry, so simulation is the case that opts in.

**Both sim sources are ground truth, not estimates.** Zero drift, zero noise, zero latency. Nothing
here validates how a real estimator behaves under drift, foot slip, or the suspend/freeze handling
that goes with it. Treat any tuning done against this as unvalidated on hardware.

No `map -> odom` is published. That is SLAM's, and it comes from `g1_navigation`.

## Frames

Which chain gets published depends on `pelvis_frame_id`:

```
pelvis_frame_id: "pelvis"   ->   odom -> base_footprint -> pelvis     (converged track)
pelvis_frame_id: ""         ->   odom -> base_link                    (planar sandbox)
```

The split exists because Nav2 and slam_toolbox both want a gravity-aligned base frame, and a walking
G1 does not have one: the pelvis rolls and pitches several degrees with the gait, and every sensor
frame hangs off it. `base_footprint` is the REP-105 ground projection — x, y and heading only, z
pinned to 0 — and `base_footprint -> pelvis` carries the height and the tilt the projection drops.

Inserted rather than published as a second edge off `odom`. Two sibling edges would make every
`mid360_link -> base_footprint` lookup compose two independently-published dynamic transforms, which
agree only if they always carry the identical stamp; one chain cannot be inconsistent even in
principle. Both edges go out in a single `sendTransform` call for the same reason.

Past `max_tilt_deg` the heading extraction is ill-conditioned, so the last well-conditioned heading
is held. The attitude keeps being published unchanged — a fallen robot really is tilted.

## `g1_odometry_publisher`

Lifecycle node. Configuration is where the source decision is enforced, so a refusal is externally
observable: with `odometry_source=hardware` it returns FAILURE from `on_configure` and stays in
`unconfigured` having created **no publisher and no broadcaster**. Advertising `/tf` and then going
quiet would be indistinguishable from a healthy node with a stalled source.

| Interface | Dir | Type | Source |
|---|---|---|---|
| `~/sport_state` (remap to `/sportmodestate`) | in | `unitree_go/SportModeState` | `sim_sportmodestate` |
| `~/imu_state` (remap to `/lowstate`) | in | `unitree_hg/LowState` | `sim_sportmodestate` |
| `~/base_state` (remap to `/base_joint_states`) | in | `sensor_msgs/JointState` | `sim_ground_truth` |
| `/tf` | out | `tf2_msgs/TFMessage` | both |
| `~/odom` | out | `nav_msgs/Odometry` | both |

Orientation comes from `/lowstate` rather than `/sportmodestate` because `unitree_mujoco` leaves the
latter's `imu_state` at all zeros, and tf2 normalises a zero quaternion straight to NaN and then
silently drops the transform.

`~/odom` describes `base_frame_id`, taken from the transform that was just published so the two
cannot disagree. On a split chain that means the footprint: z and tilt are absent because
`child_frame_id` says `base_footprint`, and `toBodyTwist()` is yaw-only, which is exactly that frame.

Each track has its own file: `config/g1_odometry_publisher.yaml` for the **planar sandbox**, and
`config/g1_odometry_publisher_converged.yaml` for the **converged track**, which
`g1_bringup/launch/sim.launch.py` loads. Joints are looked up **by name**, since
`joint_state_broadcaster` makes no promise about ordering.

`base_height_m` applies to the planar track only: its base has no z DoF, so the spawn height comes
from `g1_sim/config/sensor_mounts.yaml` via the launch. The converged track measures z and leaves
this at 0.

Staleness has two budgets. `source_timeout_ms` bounds the data's age on the source clock;
`wall_timeout_ms` bounds it on `steady_clock`, measured from the last stamp **change**. The second
is not redundant: a wedged simulator freezes its own clock too, so a source-clock-only check would
never fire. Either budget tripping stops the transform rather than re-stamping the last pose. On the
converged track the orientation gets its own wall budget, because position and attitude arrive on
different topics and one can die while the other keeps flowing.

## `odom_math`

Frame and staleness math, kept free of ROS so it tests without a node, DDS or a running sim (same
split as `g1_bringup`'s `blend_math`). Covers the ground projection and its recomposition, the
world-to-body twist rotation, yaw/quaternion conversion, tilt from vertical, angle wrapping, the
staleness boundary, and covariance fill.

Two parts worth knowing about:

- `nav_msgs/Odometry` defines `twist` in the **child** frame, while both sources report velocity in
  the world frame. They agree only at yaw zero.
- `quaternionToYaw()` is the full ZYX yaw, not `2*atan2(z, w)`. The short form is exact only for a
  pure +z rotation, which the planar base always is and a walking G1 never is.

## Testing

```bash
colcon test --packages-select g1_state_estimation
```
