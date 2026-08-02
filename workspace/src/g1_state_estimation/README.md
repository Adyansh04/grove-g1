# g1_state_estimation

`odom -> base_link` state estimation for the G1.

## Odometry source

The real G1 **publishes no odometry**. `/sportmodestate` on hardware carries
`unitree_hg::SportModeState_`, which holds only `fsm_id`, `fsm_mode`, `task_id` and `task_time`. There
is no pose and no velocity, and `rt/odommodestate` does not exist. So this package has two sources
and only one of them is implemented:

| `odometry_source` | Behaviour |
|---|---|
| `sim_ground_truth` | MuJoCo generalized coordinates, read from the perception track's planar joints. Exact by construction: no sensor, no lever arm, no filter. |
| `hardware` (default) | **Refuses to run.** A real source (leg odometry + IMU EKF, or LiDAR-inertial odometry) is a future milestone. |

`hardware` is the default deliberately. A misconfigured hardware bring-up must never silently emit
fabricated odometry, so simulation is the case that opts in.

**Sim odometry is ground truth, not an estimate.** It has zero drift, zero noise and zero latency.
Nothing here validates how a real estimator behaves under drift, foot slip, or the suspend/freeze
handling that goes with it. Treat any tuning done against this as unvalidated on hardware.

No `map -> odom` is published. That is SLAM/AMCL and belongs to the Nav2 milestone.

## `g1_odometry_publisher`

Lifecycle node. Configuration is where the source decision is enforced, so a refusal is externally
observable: with `odometry_source=hardware` it returns FAILURE from `on_configure` and stays in
`unconfigured` having created **no publisher and no broadcaster**. Advertising `/tf` and then going
quiet would be indistinguishable from a healthy node with a stalled source.

| Interface | Dir | Type |
|---|---|---|
| `~/base_state` (remap to `/base_joint_states`) | in | `sensor_msgs/JointState` |
| `/tf` | out | `tf2_msgs/TFMessage`, `odom -> base_link` |
| `~/odom` | out | `nav_msgs/Odometry` |

Parameters are in `config/g1_odometry_publisher.yaml`. Joints are looked up **by name**, since
`joint_state_broadcaster` makes no promise about ordering.

### `odom` is the ground plane

`base_height_m` is published as the z of `odom -> base_link`, so a point cloud transformed into
`odom` puts the floor at z = 0 and Nav2's obstacle height bands mean what they say. It defaults to
**0**, which is right for a real floating-base estimator that measures its own height; the
perception sim's base is a planar body with no z DoF, so `perception_sim.launch.py` supplies the
value from `g1_sim/config/sensor_mounts.yaml`, which is where the number canonically lives and which
`test_sensor_mount_consistency` pins to the MJCF spawn height.

### Staleness has two budgets, and the wall one is the load-bearing half

Past `source_timeout_ms` it stops publishing rather than re-stamping the last pose: a frozen
transform with a fresh timestamp looks exactly like a stationary robot.

Measuring that on the source clock alone is not enough here. `/clock` is published by the **same
process** as the base state, so when the simulator wedges, sim time freezes with it, the measured
age stays pinned near zero, and a sim-time-only check never fires. The second budget runs on
`steady_clock` and measures time since the sample stamp last **advanced** rather than since a message
last arrived, which also catches a simulator still republishing a frozen sample.

## `odom_math`

Frame and staleness math, kept free of ROS so it tests without a node, DDS or a running sim (same
split as `g1_bringup`'s `blend_math`). Covers the world-to-body twist rotation, yaw/quaternion
conversion, angle wrapping, the staleness boundary, and covariance fill.

The twist rotation is the part worth knowing about: `nav_msgs/Odometry` defines `twist` in the
**child** frame, while the planar joints report velocity in the world frame. They agree only at yaw
zero.

## Testing

```bash
colcon test --packages-select g1_state_estimation
```
