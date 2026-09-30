# g1_navigation

SLAM Toolbox mapping, AMCL localization and Nav2 for the G1 on the `unitree_mujoco` track.
Configuration, launch files and glue; the nodes themselves are upstream. `ament_cmake`. The Python
is launch files, tests and `nav_diag.py`, a 20 Hz diagnostic recorder with no part in control.

```mermaid
flowchart TB
    RELAY["g1_sensor_relay"] -- "/livox/lidar" --> SCAN["pointcloud_to_laserscan"]
    SCAN -- "/scan" --> SLAM["slam_toolbox"]
    SCAN -- "/scan" --> AMCL["map_server + AMCL"]
    SLAM -- "map to odom" --> NAV2
    AMCL -- "map to odom" --> NAV2
    ODOM["g1_odometry_publisher"] -- "odom to base_footprint" --> NAV2
    NAV2["Nav2"] -- "/cmd_vel" --> POL["the walking policy"]
    APPR["g1_base_approach"] -- "/cmd_vel" --> POL
```

## Launch files

| File | Purpose |
|---|---|
| `nav_stack.launch.py` | The stack without a simulator: the shared container, the scan pipeline, SLAM or localization, and Nav2 with `nav:=true`. `g1_bringup` includes this. |
| `nav_sim.launch.py` | A simulator, `nav_stack.launch.py` and optional RViz. `test_navigate_to_pose` launches this. |
| `scan.launch.py` | `pointcloud_to_laserscan`, flattening the LiDAR into the 2D scan SLAM and AMCL consume. |
| `slam.launch.py` | `slam_toolbox` in online async mapping mode. |
| `localization.launch.py` | `map_server` and AMCL against the committed map of `world`, or the one `map:=` names. |
| `nav2.launch.py` | Planner, controller, behaviors, BT navigator, lifecycle manager and `g1_base_approach`. Included only with `nav:=true`. |

`g1_bringup`'s `bringup.launch.py` is the operator entry point. `nav_stack.launch.py` and
`nav_sim.launch.py` default to `mode:=mapping` and also take `use_composition` and `container_name`,
which the bring-up entry point does not pass.

`nav_stack.launch.py` stages no simulator. Both callers stage exactly one, and a second would put
two writers on `rt/lowcmd`; `test_launch_threading` asserts it.

## Running

```bash
ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true
```

Drive with teleop and watch the map fill in. `maps/facility` is already committed;
[maps/README.md](maps/README.md) says how to regenerate a map. To navigate:

```bash
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}"
```

`mode:=mapping nav:=true` runs Nav2 on the map slam_toolbox is still building, which is what the
world model's exploration does (canopy); a goal can shift a little under a loop closure.

The navigation modes force `sensors:=true`, which brings up the LiDAR, the relay and the `odom` to
`base_footprint` chain. The [navigation guide](../../../docs/guides/navigation-and-moveit.md) walks
through the same steps.

## Gait constraints

The walking policy tracks a proportional fraction of the commanded velocity, with a deadband below
about 0.15 m/s on both linear axes: commanded 0.10 m/s, the robot does not move. Yaw has no
deadband and tracks near 1:1. `g1_controllers` clamps commands to 0.6 m/s and 0.9 rad/s.

Any controller or recovery must therefore command speeds above the deadband. That is why reverse
recovery is removed from both behavior trees: upstream's backup speed (0.05 m/s) sits inside it.
`behavior_plugins` has `drive_on_heading` for `nervros_executor`'s `WalkStraight`, which asks for
0.3 m/s either way; the gait delivers about 0.21 m/s forward and 0.24 m/s back.

Nav2 is not the only `/cmd_vel` writer. `nav2.launch.py` also starts `g1_locomotion`'s
`g1_base_approach`, which walks the last stretch because Nav2 stops within 0.5 m of a goal
(`xy_goal_tolerance`), wider than the band the arm reaches from. Nothing arbitrates between
the two: the mission tree runs `NavigateToPose` and `ApproachObject` in sequence, never together.

## Settings worth knowing before changing them

- Nav2 runs uncomposed while the scan and localization nodes compose. Composed, the costmaps come
  up on `Costmap2DROS` defaults instead of the params file and `controller_server` hangs
  activating, so `nav2.launch.py` rejects `use_composition:=true` and `nav_stack.launch.py` always
  passes `false`.
- `z_voxels` is 16, the voxel grid's maximum, so the column height comes from `z_resolution`:
  16 x 0.125 m = 2.0 m. The column must contain the sensor at 1.22 m, or `VoxelLayer` rejects every
  cloud.
- `obstacle_max_range` is 3.0 m, well inside the sensor's reach. Attitude error times range is
  height error, so distant floor returns cross `min_obstacle_height` and mark as obstacles. The
  value is the simulator's; re-measure it on the robot.
- `obstacle_min_range` is 0.6 m. It keeps the carried object and the robot's own arms off the
  costmap; anything the base can hit shows up beyond it.
- AMCL uses `OmniMotionModel` because the robot strafes: `g1_base_approach` commands lateral
  velocity and the gait drifts sideways. The alphas stay at Nav2's defaults, since neither
  simulated odometry source says anything about the real robot's noise.

## Configuration

| Path | Contents |
|---|---|
| `config/nav2_params.yaml` | Planner, controller, costmaps, behaviors, BT navigator. |
| `config/localization.yaml` | `map_server` and AMCL. |
| `config/scan.yaml` | The point cloud to laser scan flatten, including the height band. |
| `config/slam_mapping.yaml` | `slam_toolbox` online async. |
| `config/g1_navigation.rviz` | RViz for the navigation modes. Fixed frame `map`. |
| `config/navigate_to_pose.xml`, `config/navigate_through_poses.xml` | Behavior trees, with reverse recovery removed. |
| `maps/facility.{yaml,pgm}`, `maps/apartment.{yaml,pgm}` | The committed maps of the `navigation` and `apartment` worlds. |

`g1_navigation.rviz` has a Nav2 group and a folded Sensors group. `g1_bringup`'s
`g1_sensors.rviz` carries the same Sensors group with fixed frame `odom` for `mode:=none`, where no
`map` frame exists; `test_rviz_configs` keeps the two in step. The config lives here because its
AMCL particle display comes from `nav2_rviz_plugins`, a dependency `g1_bringup` should not take.

## Tests

```bash
./scripts/manage.sh test g1_navigation
./scripts/manage.sh test --sim g1_navigation   # the suites that need a simulator
```

| Test | Needs a simulator | Covers |
|---|---|---|
| `test_navigate_to_pose` | yes | The acceptance gate: the robot reaches a goal on its own, stays upright, and never trips the emergency freeze with the full sensor pipeline running. |
| `test_scan_pipeline` | yes | The frame chain and the scan. |
| `test_slam_map` | yes | slam_toolbox owning `map` to `odom`, and the map's geometry. |
| `test_launch_threading` | no | Arguments surviving every include boundary, for both callers: the include set, the single container, the uncomposed Nav2 pin, and that no simulator is staged. |
| `test_rviz_configs` | no | The Sensors display group not drifting between the two configs. |
| `test_no_sim_time` | no | No shipped config enables `use_sim_time`. There is no `/clock` on this track. |

`test_navigate_to_pose` drives the real gait and is timing-sensitive. It has no retry wrapper, so a
flaky rate stays visible. Re-run it alone before believing a red run:

```bash
ctest --test-dir build/g1_navigation -R '^test_navigate_to_pose$'
```

## Soak test

`nav_soak` brings the stack up, drives a list of goals across the facility one at a time, records
health, and tears everything down.

```bash
ros2 run g1_navigation nav_soak                        # default goal list
ros2 run g1_navigation nav_soak --rviz
ros2 run g1_navigation nav_soak --goals "3.0 -3.0,-2.5 2.0"
ros2 run g1_navigation nav_soak --odometry ground_truth
```

It reports distance driven, the share of Nav2 commands inside the gait deadband, `map` to `odom`
correction sizes, pelvis pitch and local costmap lethal-cell counts, plus the falls, progress
failures and recoveries counted in the launch log. `nav_diag.py` is the recorder. It also runs on
its own against a stack that is already up, for the given number of seconds (default 200):

```bash
ros2 run g1_navigation nav_diag.py 200
```

`nav_soak` works around three things:

- One goal at a time, each allowed to finish. A goal preempted mid-run replans abruptly and the
  robot can fall.
- Teardown by process group plus a `/proc/PID/cmdline` sweep. Processes running as `python3` or
  the `ros2` CLI otherwise survive as orphans into the next run.
- Readiness read from the launch log. Discovery can outlast any timeout, and AMCL publishes
  `/amcl_pose` only after the robot moves.

It checks for orphans before and after a run. After an interrupted run, check by hand: any output
from `ros2 node list --no-daemon | sort | uniq -d` means an orphan is still running.
