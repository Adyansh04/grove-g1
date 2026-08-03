# g1_navigation

SLAM Toolbox mapping and Nav2 navigation for the G1, on the converged `unitree_mujoco` track.

Configuration, launch and maps only. Every node here is upstream — `pointcloud_to_laserscan`,
`slam_toolbox`, Nav2 — and the two G1-specific nodes the stack needs (`g1_loco_authority` and
`g1_gait_shaper`) live in `g1_locomotion`, so nothing Nav2-shaped leaks into the package that has
to survive to hardware.

## Running

Everything here needs `sensors:=true` on `g1_bringup`, which gates the LiDAR, the relay, the
`odom -> base_footprint -> pelvis` chain and the waist joint states. The top-level launch passes it.

```bash
ros2 launch g1_navigation nav_sim.launch.py mode:=mapping rviz:=true
```

## Sensor inputs

| Consumer | Input | Why |
|---|---|---|
| slam_toolbox | `/scan` (`LaserScan`, `base_footprint`) | It is a 2D scan matcher and takes nothing else. |
| Nav2 costmaps | `/livox/lidar` (`PointCloud2`, `mid360_link`) raw | The layer accepts it natively and it keeps the low obstacles `/scan` discards. |

The flatten is `pointcloud_to_laserscan` against a gravity-aligned target frame, banded to
[0.30, 1.50] m above the floor. See `config/scan.yaml` — every value there carries the measurement
that chose it.

**The depth camera is not a navigation input.** The D435i is pitched 47.6 degrees down and looks at
the floor about 1.2 m ahead. It is a manipulation and near-field sensor.

## What sim validates, and what it does not

Odometry on this track is **exact MuJoCo ground truth** — zero drift, zero noise, zero latency. That
makes SLAM trivially easy here.

Validated: frame topology, QoS wiring, scan geometry and flatten quality, that the plugins load and
are configured coherently, and that the robot's achievable motion is enough to reach a goal.

**Not** validated: loop closure under drift, scan-matching robustness, relocalization from a wrong
initial pose, or any real odometry error model. Anything tuned against this is unvalidated on
hardware.

## Known limitations

**There is no `/clock`** — the simulator links no ROS. `use_sim_time` is false everywhere, and a
ctest fails the build if any shipped config sets it true.

**The ramp is mapped as an obstacle.** `slope_ramp`'s surface sits between 0.08 and 0.24 m, above the
costmap's floor cut. That is the right answer for this gait, but it looks like a bug in RViz.

**Mapped free space shows radial spokes at long range.** At 15 m adjacent 1-degree beams are 26 cm
apart, so raytraced clearing fans out. Cosmetic; shortening the scan's `range_max` would trade real
far-wall coverage for it.
