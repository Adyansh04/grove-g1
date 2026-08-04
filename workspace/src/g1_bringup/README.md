# g1_bringup

Sim bring-up for the G1 stack. Launches `unitree_mujoco` alongside the `ros2_control` stack
(`g1_description` + `g1_hardware_interface`), `g1_locomotion`'s bridge, and
`g1_motion_service_sim`'s sim-only stand-in for the robot's onboard motion service.

Launch files, config, scenes and scripts only -- no compiled code of its own. `ament_cmake`,
with Python launch files and integration tests.

## Launch files and nodes

| File | Purpose |
|---|---|
| `launch/bringup.launch.py` | **The operator entry point.** Routes to bare sim or, via `mode:`, to the navigation stack. Args: `mode` (`none`/`mapping`/`localization`), `nav`, `rviz`, `sensors`, `world`, `headless`, `pin_pelvis`, `sim_start_delay_s`. |
| `launch/sim.launch.py` | Checks the DDS environment, then starts `unitree_mujoco`, `motion_service_sim`, `control.launch.py` and `loco.launch.py`. Still works standalone. Args: `headless` (default `true`), `sensors` (default `false`), `pin_pelvis` (default `false`), `sim_start_delay_s` (default `2.0`). |
| `launch/rviz.launch.py` | Starts RViz on a caller-supplied `rviz_config` path. Knows nothing about navigation. |
| `launch/control.launch.py` | `robot_state_publisher`, `ros2_control_node` and spawners. No sim, no bridge, so it carries over to hardware unchanged. |
| `launch/loco.launch.py` | Starts `g1_loco_bridge` and drives it configure to active off its own lifecycle events. |
| `launch/activate_arm.launch.py` | Runs `scripts/activate_arm`, the ordered acquire step. |
| `launch/deactivate_arm.launch.py` | Runs `scripts/deactivate_arm`, the ordered release step. |

This package starts nodes it does not own. `motion_service_sim` is
`g1_motion_service_sim`'s, `g1_loco_bridge` is `g1_locomotion`'s, and the sensor and odometry
nodes belong to `g1_sensor_relay` and `g1_state_estimation`.

### Topics

`g1_hardware_interface`'s README covers `/lowstate` and `/arm_sdk`;
`g1_motion_service_sim`'s covers `/lowcmd`, `/api/sport/*` and the lower-body `/joint_states`.
What this package's own launch graph adds on top:

| Topic | Direction | Type | QoS |
|---|---|---|---|
| `/robot_description` | out | `std_msgs/msg/String` | transient-local |
| `/livox/lidar` | out | `sensor_msgs/msg/PointCloud2` | sensor data, `sensors:=true` only |
| `/g1_sensor_relay/sensor_pose` | out | `geometry_msgs/msg/PoseStamped` | sensor data, diagnostics |
| `/tf` (`odom` -> `pelvis`) | out | `tf2_msgs/msg/TFMessage` | `sensors:=true` only |

## `g1_navigation` is referenced but deliberately not depended on

`bringup.launch.py` includes launch files and an RViz config from `g1_navigation`, and
`package.xml` says nothing about it. That is not an oversight.

`g1_navigation` already declares `<exec_depend>g1_bringup</exec_depend>`, because navigation
composes bring-up and not the reverse. Adding the reciprocal dependency here does not merely
look untidy, it **does not build** — colcon refuses to order the workspace at all:

```
ERROR:colcon:colcon list: Unable to order packages topologically:
g1_bringup: ['g1_navigation']
g1_navigation: ['g1_bringup']
```

So the reference is a launch-time path lookup and nothing more. What follows from that:

- This package builds, installs and runs with `g1_navigation` absent. Only `mode:=mapping`
  and `mode:=localization` name it, and their absence is reported as an actionable message
  rather than a raw ament search-path dump.
- **colcon and rosdep cannot see this edge.** Nothing warns you if `g1_navigation` renames a
  launch file; `mode:=mapping` fails at runtime instead. `test_launch_threading` in
  `g1_navigation` is the compensating check — it lives there because a `test_depend` pointing
  the other way would re-create the same cycle.
- Do not "fix" this by adding the dependency. Its absence is load-bearing.

The composition direction is unchanged: on the navigation branch this package includes
`g1_navigation`'s `nav_sim.launch.py`, which includes `sim.launch.py` itself.
`bringup.launch.py` routes; it does not orchestrate navigation.

## Running

```bash
# Bare simulator.
ros2 launch g1_bringup bringup.launch.py

# Build a map, with RViz.
ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true

# Localize against the committed map and navigate.
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true
```

`mode:=none` is the default and never touches `g1_navigation`. The arm procedure below uses
`sim.launch.py` directly, which is equivalent to `bringup.launch.py` with the default mode:

```bash
# 1. Bring up sim + bridge + control stack (headless by default).
ros2 launch g1_bringup sim.launch.py

# 2. Once settled, acquire arm control authority (component, then controller).
ros2 launch g1_bringup activate_arm.launch.py

# 3. Command the arms, e.g. a FollowJointTrajectory goal to arm_trajectory_controller.

# 4. Release in reverse order before tearing down.
ros2 launch g1_bringup deactivate_arm.launch.py
```

To walk the robot, reach FSM `Start` first, then publish velocity:

```bash
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 4}"
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 500}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/g1_loco_bridge/cmd_vel -p speed:=0.6 -p turn:=1.57
```

Both teleop overrides matter. The package defaults are `speed=0.5`, which only just clears the
0.40 m/s threshold, and `turn=1.0`, which does not clear the 1.50 rad/s in-place yaw threshold at
all, so the robot will not turn on the spot with it. `1.57` is the bridge's yaw ceiling. Note that
`q` and `z` scale both speeds by 10%, so a few `z` presses drop you back under threshold and the
gait stops.

### Ordering rules

Acquire and release order is mandatory. Humble ties command-interface availability to hardware
component state, so activating the controller before the component (or deactivating the component
first) can fail the switch or strand a controller claiming interfaces. `activate_arm` and
`deactivate_arm` encode the correct order.

Deactivating before killing the launch is the documented clean stop. `deactivate_arm.launch.py`
blocks for about `blend_ramp_down_s` (2.0 s) while the blend weight ramps to zero synchronously.
That is by design, not a hang. Ctrl-C is also safe: `controller_manager` runs the component's
`on_deactivate` before `on_shutdown`, and a dead sim tears down the whole launch rather than
leaving controllers commanding nothing.

## Domain and DDS

The container runs on `ROS_DOMAIN_ID=1` with CycloneDDS pinned to `lo`, a dedicated local domain so
a real robot on the network is never at risk of crosstalk. `unitree_mujoco`'s own `config.yaml`
defaults to the same values, so its bare-DDS layer and the ROS graph see each other. Moving to
hardware is a domain and interface change, not a code change. `sim.launch.py` asserts
`RMW_IMPLEMENTATION`, `CYCLONEDDS_URI` and `ROS_DOMAIN_ID` up front and fails with an actionable
message rather than leaving you to debug an empty graph.

**Footgun:** `unitree_mujoco` is a native `unitree_sdk2` DDS app that links its own CycloneDDS from
`/opt/unitree_robotics/lib`. Sourcing a ROS environment puts `/opt/ros/humble/lib` ahead of that on
`LD_LIBRARY_PATH`, and because the binary uses `RUNPATH` (resolved after `LD_LIBRARY_PATH`), ROS's
ABI-incompatible `libddsc.so.0` wins and the sim aborts on its first DDS write. `sim.launch.py`
prepends the correct directory back for the sim process only.

## The sim-only motion service

`motion_service_sim` fills the three gaps `unitree_mujoco` leaves: it serves the weight-blended
`/arm_sdk` interface, answers the `/api/sport/*` LocoClient protocol, and runs the walking policy
that balances the robot. **Never launch it near real hardware.** `sim.launch.py` is the only thing
that starts it.

It lives in **`g1_motion_service_sim`**, whose README carries the arm blend and staleness policy,
the FSM legality table and API dispatch, the walking policy's contract and provenance, and the
measured gait deadband to read before commanding any velocity.

## Sensors on the converged track: SIM-ONLY

`sensors:=true` stages a room with known geometry, runs a LiDAR sweep **inside** the patched
`unitree_mujoco`, and starts `g1_sensor_relay` to publish it as `PointCloud2` on `/livox/lidar`.
`odom -> pelvis` comes from `g1_state_estimation` reading `/sportmodestate`.

**It is off by default, provisionally.** With sensors on the walking suites pass, alone and under
combined load, but `test_arm_command` misses its slew-limited convergence window. That was measured
on a machine whose CPU was pinned at roughly 14% of peak clock (`powersave` governor, `quiet`
platform profile), and the test is timing-sensitive, so the result is **unproven rather than a
property of the sensor stack**. Re-measure on an unthrottled machine before drawing a conclusion.

Keeping the default off in the meantime costs nothing: `test_lidar_geometry` enables sensors
explicitly and passes, so the sensor path stays covered either way.

**Why the sweep lives inside the simulator.** It needs the scene: geometry, meshes and current pose,
all of which live in `mjData` inside that process. No DDS topic carries it, so no companion process
can compute it. The finished cloud is what crosses the boundary, over a local socket.

**Why a separate node publishes it.** `unitree_sdk2` and `rmw_cyclonedds` both call
`dds_create_domain` unconditionally, and CycloneDDS allows exactly one explicit domain creation per
domain id per process. They cannot coexist, in either order. So the simulator links no ROS at all and
the relay owns the ROS side.

**Frame is `mid360_link`, not `livox_frame`.** `g1_sim`, the planar sandbox, publishes in
`livox_frame` because that is `livox_ros_driver2`'s own default, so swapping sim for hardware is a
driver launch rather than a pile of remaps. The converged track cannot do the same: the frame is not
ours to name. It comes from Unitree's vendored URDF, which calls the link `mid360_link`, and
renaming it means patching a description we otherwise consume verbatim. Adding a second link as an
alias would put two frames on one physical sensor, which is worse.

So the portability principle moves **one layer up**. The real driver's frame id is a launch
parameter, so hardware bring-up sets `livox_ros_driver2`'s `frame_id` to `mid360_link` (or remaps at
launch) instead of the sim contorting to match a default. Same guarantee at the swap, enforced at
launch rather than in the model.

**What is not validated here:** the Mid360's non-repetitive scan pattern (this is a uniform
azimuth/elevation grid), intensity, per-point timestamps, noise, dropout and motion distortion.

### Known limitation: the viewer's Reload button is fatal with `sensors:=true`

**Clicking Reload (or dragging a model onto the window) while sensors are running kills the
simulator with SIGSEGV. Relaunch instead.** Drag-and-drop takes the same code path and is
equally unsafe.

The sampler reads the model outside `sim.mtx` on purpose: a ~4.5 ms render or ~32 ms sweep
under the lock would stall physics. The reload path frees the model on the main thread, so
no lock-based check can make it safe. The reload hook therefore stops the sampler with a
blocking join before `mj_deleteModel` and never restarts it, logging `SENSORS DISABLED`.
That much is verified: the first reload after launch is survivable and stops sensors
cleanly.

**A second reload still crashes**, after the sampler is already stopped and none of this
code runs. The cause was not identified. Restarting the sampler against the new model was
tried and also faulted intermittently, so it was removed rather than shipped.

Accepted rather than fixed: Reload is a debugging-only button, the workaround is to
relaunch, and sensors do not survive a reload either way.

### D435i

Depth and colour come from **one** `mjr_render` of the `d435i` fixed camera, so they share a pose,
a timestamp and intrinsics by construction. A real D435i only gets that alignment from its
`align_depth_to_color` step, which is why the depth topic is named as if it had run.

| Topic | Type | Notes |
|---|---|---|
| `/camera/aligned_depth_to_color/image_raw` | `Image` `32FC1` | metres; misses are NaN, not 0 |
| `/camera/color/image_raw` | `Image` `rgb8` | |
| `/camera/aligned_depth_to_color/camera_info`, `/camera/color/camera_info` | `CameraInfo` | same intrinsics; `fovy` is carried in the frame so they cannot drift from the MJCF |

Published in `camera_depth_optical_frame` / `camera_color_optical_frame`, the REP-145 optical frames
added by `g1_description`. **Not `d435_link`:** that is a body frame (x forward, y left, z up) and
every depth consumer assumes the optical convention (z forward, x right, y down), so publishing in
the link projects the cloud rotated 90 degrees.

**What is not validated here:** depth noise, stereo dropout on textureless surfaces, IR projector
behaviour, auto-exposure and colour response. One MuJoCo camera carries one set of intrinsics, so
the colour stream matches the *depth* FOV rather than a real D435i's wider colour FOV.

## Pelvis pin: debugging aid

`pin_pelvis` defaults `false`. Setting it true welds the pelvis to the world **and** disables the
walking policy: the weld and the policy are the two possible owners of the legs and are never both
active. Keep it for exercising the arm bridge with nothing else driving the legs.

The weld is a `weld` equality constraint (not `connect`, which would still allow toppling rotation)
in `mjcf/g1_pinned_scene.xml`, an overlay that composes the vendored G1 model via `<include>`. The
vendored files are not modified. It constrains the pelvis body only, so all 29 actuated joints stay
driven via `/lowcmd`. The overlay is copied next to the vendored model at launch and removed on
shutdown, because MuJoCo 3.3.6 does not reliably resolve the model's relative `meshdir` when the
including file lives elsewhere.

## Configuration

`motion_service_sim.yaml` and `walk_policy.yaml` moved to `g1_motion_service_sim` with the node
that reads them; `sim.launch.py` still passes both. The three files here are the ones this
package's own launch graph owns.

`config/controllers.yaml`: `controller_manager` at 200 Hz. `G1ArmSdkSystem` starts inactive.
`joint_state_broadcaster` is spawned active, `arm_trajectory_controller` inactive, covering the 14
arm joints with relaxed sim tolerances. Re-tighten those against real hardware dynamics.

Hand joints stay inert. `g1_description`'s URDF includes the DEX3 joints for kinematic structure,
but they get no `ros2_control` interfaces and `unitree_mujoco`'s MJCF has no hand joints at all.

## Tests

| Test | Kind | Covers |
|---|---|---|
| `test_sim_bringup` | launch | Bring-up topics, rates, controller and component states (welded). |
| `test_arm_command` | launch | Ordered activation, weight ramp, closed-loop trajectory, slew clamp, rogue-publisher guard (welded). |
| `test_loco` | launch | LocoClient protocol end to end over DDS (welded). |
| `test_walk_stand` | launch | The policy stands the robot up unwelded and holds it. Entry transient bounded, single `/lowcmd` writer. |
| `test_walk_teleop` | launch | The real authority path: `7301` before `Start`, dead-man, Damp release, randomized and whiplash sequences. |
| `test_walk_and_arm` | launch | Acceptance: walking under `cmd_vel` while an arm trajectory converges, one session. |
| `sim_settle_gap` | ctest | A 5 s sleep holding the sim resource lock so each DDS graph drains. |
| `ruff_check_g1_bringup` | ctest | Python lint and import order. |

Every suite here launches `sim.launch.py`, which is why they stay in this package rather than
following `motion_service_sim`'s source: `g1_bringup` depends on `g1_motion_service_sim`, so
hosting them there would be a dependency cycle. They also share one `RESOURCE_LOCK`, and ctest
only honours that within a single invocation. The pure unit tests that need no simulator went
with their source.

```bash
colcon build --symlink-install --packages-select g1_bringup
colcon test --packages-select g1_bringup
colcon test-result --verbose
```

### Load sensitivity: read before trusting a red full-suite run

Every launch suite starts a real `unitree_mujoco`. The sim syncs its clock to CPU time and re-syncs
when it falls behind, while the walking policy is paced on a wall timer, so on a loaded machine the
two drift apart and the robot can topple. The leading hypothesis is a harness limitation rather than
a policy defect, stated as a hypothesis deliberately, since inspection on this path has already
produced two real defects.

**A full sweep does not reliably pass.** Suites that fail inside
`colcon test --packages-select g1_bringup` pass every time they are run alone. The isolated pass is
the current evidence of correctness, alongside hand-verified walking, teleop and arm motion in the
GUI. Do not treat a green full sweep as a precondition for trusting this package.

Mitigations are deliberately lightweight: `RESOURCE_LOCK` serialises the suites, a settle gap lets
each DDS graph drain, and `COST` ordering puts `test_walk_stand` last so the core balance claim runs
on the quietest machine. Real isolation belongs with the deferred CI work. Re-run a failing suite
alone before treating it as a regression:

```bash
colcon test --packages-select g1_bringup --ctest-args -R test_walk_stand
```

## Language

All Python, and no C++ at all since `motion_service_sim` moved out. ROS 2 provides no C++ path for
launch or `launch_testing`, and the `activate_arm`/`deactivate_arm` scripts are one-shot sequencing
tools rather than control loops. Nothing here runs at control rate; the packages this one launches
carry that.
