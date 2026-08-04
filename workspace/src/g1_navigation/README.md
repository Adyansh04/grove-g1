# g1_navigation

SLAM Toolbox mapping and Nav2 navigation for the G1, on the converged `unitree_mujoco` track.

Configuration, launch and maps only — every node here is upstream.

Nav2's planner, controller, behaviors and BT navigator are configured here; the two G1-specific
nodes they need — `g1_loco_authority` to acquire locomotion authority and `g1_gait_shaper` to
reduce Nav2's output onto the gait's three usable motions — live in `g1_locomotion`, so nothing
Nav2-shaped leaks into the package that has to survive to hardware.

## Running

Everything here needs `sensors:=true` on `g1_bringup`, which gates the LiDAR, the relay, the
`odom -> base_footprint -> pelvis` chain and the waist joint states. The top-level launch passes it.

```bash
ros2 launch g1_navigation nav_sim.launch.py mode:=mapping rviz:=true
```

`mode:=mapping` builds a new map with slam_toolbox. `mode:=localization` runs `map_server` + AMCL
against `maps/facility` — use that when a goal pose has to mean the same thing twice.

To actually navigate, add `nav:=true` (which requires `mode:=localization`):

```bash
ros2 launch g1_navigation nav_sim.launch.py mode:=localization nav:=true rviz:=true
```

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}"
```

## What the robot can actually do

Measured, and it shapes every controller decision here. The gait has a hard initiation deadband
with no hysteresis: nothing at or below 0.35 m/s forward, steps from 0.40; nothing at or below
0.60 rad/s yaw, steps from 1.50. Combined commands come out badly — a commanded (0.50, 0, 0.50)
measured (0.337, **0.299**, 0.390), a third of a metre per second of lateral nobody asked for.

So the usable action set is three elements: **stop, drive straight at ~0.6 m/s, turn in place at
~1.57 rad/s.** `g1_gait_shaper` collapses Nav2's continuous output onto exactly those, subtractive
only — it clamps and zeroes, never amplifies.

The consequence worth knowing before tuning: the shaper zeroes any yaw below 1.20, and RPP's
curvature steering is usually well under that, so it rarely reaches the robot. Effective behaviour
is bang-bang — drive straight until the heading error is large, then rotate in place. That makes
**`angular_dist_threshold` the dominant knob, not `lookahead_dist`.**

This deadband is a property of the **sim walking policy**, not of the real G1's onboard MPC, which
has no such dead zone. That is why it lives in `g1_locomotion`'s config and not in Nav2 tuning:
hardware bring-up simply does not launch the shaper.

## Composition

The navigation nodes load into one `component_container_isolated` named `nav2_container`.
`nav_sim.launch.py` creates the container; the leaf launches load into it. Set
`use_composition:=false` for one process per node, which is what you want when a single node is
crashing and you need to see which.

This shares a process and gives each component its own executor. It is **not** zero-copy — nothing
sets `use_intra_process_comms`, and neither does `nav2_bringup`, because `/map` is transient-local
and Humble's intra-process path does not support that durability.

**Nav2 itself runs uncomposed, and that is measured rather than preference.** Composed, the
costmaps silently come up on `Costmap2DROS`'s built-in defaults —
`/local_costmap/local_costmap` reported `base_link`, `map` and `robot_radius 0.1` while
`config/nav2_params.yaml` plainly says `base_footprint`, `odom` and `0.45` — and
`controller_server` then hangs forever activating against a frame that does not exist. A bare
path, `RewrittenYaml` and `ParameterFile` all behaved the same. The scan and localization nodes
still compose and do get their parameters; their sections are flat and named for the node itself,
which is consistent with the nested sections being the problem.

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
| Authority acquired on activate, released on deactivate, and `cmd_vel` discarded before it | `test_nav_authority`, live sim |
| **The robot reaching a navigation goal on its own** | `test_navigate_to_pose` — the acceptance gate |
| `localization.launch.py`, `config/localization.yaml`, `map_server` + AMCL | now covered: `test_navigate_to_pose` runs the whole stack in localization mode |
| `g1_gait_shaper`'s contract, including never amplifying | `g1_locomotion`'s `test_gait_shaper` |
| Authority released when the acquire *fails* | `g1_locomotion`'s `test_authority_release`, against a stub, no sim |

PR A shipped with localization deliberately untested; `test_navigate_to_pose` closes that, because
it runs `map_server` + AMCL exactly as shipped.

**`test_navigate_to_pose` will fail occasionally, for reasons outside this package.** See the two
failure modes below. Re-run it alone before treating a red run as a regression — that is the same
policy `g1_bringup`'s `test_walk_stand` carries, and there is deliberately no retry wrapper,
because a retry would hide the very number that matters.

## What sim validates, and what it does not

Odometry on this track is **exact MuJoCo ground truth** — zero drift, zero noise, zero latency. That
makes SLAM trivially easy here.

Whether the robot's achievable motion is enough to reach a navigation goal **is** now tested, by
`test_navigate_to_pose`. `test_scan_pipeline` and `test_slam_map` still pin the pelvis, because
they are measuring geometry and a wandering robot would move the numbers.

**Not** validated: loop closure under drift, scan-matching robustness, relocalization from a wrong
initial pose, or any real odometry error model. Anything tuned against this is unvalidated on
hardware.

## Two failure modes, two separate mitigations

These are unrelated, and only one of them has a mitigation that fires. Do not read the
`OnProcessExit` handler as covering both — it covers exactly one.

### A — `g1_loco_authority` dies while holding authority

Nav2 and the shaper keep publishing, the bridge keeps re-issuing `SET_VELOCITY`, its 1 s dead-man
never expires, and **the robot walks on to its goal with nobody supervising authority.**

*Mitigation:* `nav2.launch.py` registers an `OnProcessExit` handler on the authority node that
stops `g1_gait_shaper`. `cmd_vel` ceases, the bridge's staleness path emits one
`SetVelocity(0,0,0)`, and the robot stands balanced.

### B — the robot is fully healthy and will not walk

`authority` HELD, `fsm_id` 500, no errors anywhere, and the gait simply does not respond.
**Measured at 1 of 8 fresh launches.**

**`OnProcessExit` does nothing here.** `g1_loco_authority` is alive and correct; there is no
process exiting for anything to notice.

*Mitigation:* Nav2's progress checker aborts the goal — and this is confirmed against the measured
case, not assumed. `SimpleProgressChecker::check()` returns false when the robot has not moved
more than `required_movement_radius` from its baseline within `movement_time_allowance`, and the
baseline resets **only** when it does move enough. The recorded frozen case travelled 0.09 m in
30 s, never within 0.5 m of resetting the baseline, so at `movement_time_allowance: 20.0` the
checker trips and `controller_server` fails the goal. Observed live as
`[controller_server]: Failed to make progress`.

What that mitigation does **not** do is recover. It aborts. The robot stays frozen and only
relaunching the simulator clears it. Nav2's recoveries cannot help either: `Spin` commands motion
the robot ignores, `Wait` waits, and `ClearCostmap` clears a costmap that was never the problem.

### A caveat on the 1-in-8 figure

That number was measured with **`sensors:=true world:=navigation`**, which is what navigation
runs. It is 8 samples, so treat it as order-of-magnitude.

It may not transfer to other launch configurations. Three consecutive `sensors:=false` launches
were frozen while the next `sensors:=true` launch walked immediately — 3 against 1, which at a
12% base rate is unlikely (~0.2%) but not absurd, and no mechanism is known: `sensors` gates the
LiDAR, the relay, the odometry publisher and the waist joint states, none of which touch the
walking policy. Quote the figure for the navigation configuration only, and record the `sensors`
value if you ever reproduce a frozen robot.

Separately, the robot sometimes **falls over** rather than freezing — seen once during acceptance
testing, at 117 degrees of tilt. The odometry publisher's tilt guard holds the last heading and
says so, and the progress checker aborts the goal the same way. Three consecutive acceptance runs
passed afterwards.

## Known limitations

**There is no `/clock`** — the simulator links no ROS. `use_sim_time` is false everywhere, and a
ctest fails the build if any shipped config sets it true.

**The ramp will map as an obstacle** once the costmaps exist: `slope_ramp`'s surface sits between
0.08 and 0.24 m, above the floor cut a costmap needs to avoid painting the whole floor. That is the
right answer for this gait, but it looks like a bug in RViz.

**Mapped free space shows radial spokes at long range.** At 15 m adjacent 1-degree beams are 26 cm
apart, so raytraced clearing fans out. Cosmetic; shortening the scan's `range_max` would trade real
far-wall coverage for it.

**The robot's fingers do not render in RViz, and cannot.** The palms do — they hang off the wrists
by fixed joints — but the finger links need joint states, and nothing publishes them: the
simulated model has 30 joints and **zero** finger DoF, so there is no hand state to publish. The
URDF describes a robot with hands; the sim simulates one without. Publishing zeroed finger joints
would put fabricated state on `/tf`, which is the thing `g1_state_estimation` refuses to do with
odometry, so it is not done here either.

**Reverse recovery behaviours are removed from both behavior trees.** `BackUp` and
`DriveOnHeading` command reverse, and this gait reverses at −0.247 m/s for a commanded −0.60 and
at exactly 0.000 for −0.40. Upstream's `backup_speed` is **0.05 m/s** — squarely inside that dead
zone — so `BackUp` would command motion the robot cannot produce and burn its full
`time_allowance` before failing, consuming the round-robin slot `Spin` could have used. Both are
also dropped from `behavior_plugins`, so no action server exists to invoke. And independently of
all that, `g1_gait_shaper` compares forward velocity **signed**, so any negative command becomes
zero at any magnitude: even a hand-edited tree cannot produce a reverse lurch. Raising
`backup_speed` to 0.6 is the only setting that would make them move the robot; that is an
uncontrolled reverse lurch as a *recovery*, and it is rejected rather than overlooked.
