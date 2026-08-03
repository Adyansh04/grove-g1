# g1_navigation

SLAM Toolbox mapping and Nav2 navigation for the G1, on the converged `unitree_mujoco` track.

Configuration, launch and maps only — every node here is upstream.

**Mapping and localization only so far.** Nav2's planner and controller are not configured yet, and
nothing here drives the robot: both launch tests pin the pelvis. Driving a navigation goal needs a
node to acquire locomotion authority and one to shape Nav2's output onto the gait's usable velocity
set, and both belong in `g1_locomotion` so nothing Nav2-shaped leaks into the package that has to
survive to hardware.

## Running

Everything here needs `sensors:=true` on `g1_bringup`, which gates the LiDAR, the relay, the
`odom -> base_footprint -> pelvis` chain and the waist joint states. The top-level launch passes it.

```bash
ros2 launch g1_navigation nav_sim.launch.py mode:=mapping rviz:=true
```

`mode:=mapping` builds a new map with slam_toolbox. `mode:=localization` runs `map_server` + AMCL
against `maps/facility` — use that when a goal pose has to mean the same thing twice.

## Composition

The navigation nodes load into one `component_container_isolated` named `nav2_container`.
`nav_sim.launch.py` creates the container; the leaf launches load into it. Set
`use_composition:=false` for one process per node, which is what you want when a single node is
crashing and you need to see which.

This shares a process and gives each component its own executor. It is **not** zero-copy — nothing
sets `use_intra_process_comms`, and neither does `nav2_bringup`, because `/map` is transient-local
and Humble's intra-process path does not support that durability. With `mode:=mapping` the
container currently hosts a single component; it earns its keep in PR B, when the costmaps,
planner and controller join it.

Structure follows `nav2_bringup`'s own launch files, including which nodes stay out:
**slam_toolbox runs as a separate process** even though it ships a component. That is what
`nav2_bringup/launch/slam_launch.py` does, and its 40 MB stack requirement for map serialization is
not something to hand a shared process.

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

Validated: frame topology, QoS wiring, scan geometry and flatten quality, that slam_toolbox and
AMCL come up and own `map -> odom`, and that the map matches the room. Whether the robot's
achievable motion is enough to reach a navigation goal is **not** tested here — nothing in this
package drives it yet.

**Not** validated: loop closure under drift, scan-matching robustness, relocalization from a wrong
initial pose, or any real odometry error model. Anything tuned against this is unvalidated on
hardware.

## Known limitations

**There is no `/clock`** — the simulator links no ROS. `use_sim_time` is false everywhere, and a
ctest fails the build if any shipped config sets it true.

**The ramp will map as an obstacle** once the costmaps exist: `slope_ramp`'s surface sits between
0.08 and 0.24 m, above the floor cut a costmap needs to avoid painting the whole floor. That is the
right answer for this gait, but it looks like a bug in RViz.

**Mapped free space shows radial spokes at long range.** At 15 m adjacent 1-degree beams are 26 cm
apart, so raytraced clearing fans out. Cosmetic; shortening the scan's `range_max` would trade real
far-wall coverage for it.
