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
container currently hosts a single component — one process for one node. It exists for the
navigation servers that will join it, not for what is in it today.

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

## What is covered, and how

| | Coverage |
|---|---|
| The `odom -> base_footprint -> pelvis` chain and the scan | `test_scan_pipeline`, against a live headless sim |
| slam_toolbox owning `map -> odom`, and the map's geometry | `test_slam_map`, against a live headless sim |
| The frame split, the tilt guard, parameter validation | `g1_state_estimation`'s node and math suites |
| No shipped config enables `use_sim_time` | `test_no_sim_time` |
| **`localization.launch.py` and `config/localization.yaml`** | **None. One manual run only.** |

**Localization is not automatically tested.** `map_server` + AMCL, the composed and non-composed
branches, and every AMCL parameter in `config/localization.yaml` have been exercised exactly once,
by hand: both nodes reached `active` and AMCL published `map -> odom` against the committed map.
That is the whole of the evidence. Automated coverage arrives with the navigation acceptance test,
which needs a planner and a controller to send a goal to and therefore belongs with them, not here.
Treat any change to that launch file or config as unverified until then.

## What sim validates, and what it does not

Odometry on this track is **exact MuJoCo ground truth** — zero drift, zero noise, zero latency. That
makes SLAM trivially easy here.

Whether the robot's achievable motion is enough to reach a navigation goal is **not** tested —
nothing in this package drives the robot at all; both launch tests pin the pelvis.

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
