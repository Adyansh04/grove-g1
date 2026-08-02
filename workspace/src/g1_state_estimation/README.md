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
