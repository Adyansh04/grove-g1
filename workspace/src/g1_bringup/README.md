# g1_bringup

Sim bring-up for the G1 arm bridge milestone: launches `unitree_mujoco` alongside the
`ros2_control` stack (`g1_description` + `g1_hardware_interface`), plus a sim-only node that
stands in for the onboard motion service so the simulated robot doesn't collapse. `ament_cmake`,
C++17 node + Python launch files and integration tests.

## Nodes and launch files

| File | What it does |
|---|---|
| `launch/sim.launch.py` | The main entry point. Env fail-fast, then `unitree_mujoco` + `motion_service_sim` + `control.launch.py`. Args: `headless` (default `true`), `pin_pelvis` (default `true`, see "Pelvis pin"), `sim_start_delay_s` (default `2.0`). |
| `launch/control.launch.py` | Composition-pure: `robot_state_publisher` + `ros2_control_node` + spawners. No sim, no bridge -- carries over unchanged to hardware bring-up. |
| `launch/activate_arm.launch.py` | Runs `scripts/activate_arm`: the explicit, ordered acquire step. |
| `launch/deactivate_arm.launch.py` | Runs `scripts/deactivate_arm`: the explicit, ordered release step. |
| `motion_service_sim` (executable) | SIM-ONLY node, see below. |

### Topics (beyond what `g1_hardware_interface`'s README already documents for `/lowstate`/`/arm_sdk`)

| Topic | Direction | Type | QoS | Published/consumed by |
|---|---|---|---|---|
| `/lowcmd` | out | `unitree_hg/msg/LowCmd` | best-effort, keep-last(1), volatile | `motion_service_sim` -> `unitree_mujoco` |
| `/joint_states` | out | `sensor_msgs/msg/JointState` | default (reliable, keep-last) | `joint_state_broadcaster`, ~200 Hz (`controller_manager`'s `update_rate`) |
| `/robot_description` | out | `std_msgs/msg/String` | transient-local | `robot_state_publisher` |

## Operating procedure

```bash
# 1. Bring the sim + bridge + control stack up (headless by default).
ros2 launch g1_bringup sim.launch.py

# 2. Once it's settled (robot standing, controllers loaded), acquire control
#    authority in the mandatory order (component, then controller):
ros2 launch g1_bringup activate_arm.launch.py

# 3. Command the arms, e.g. via a FollowJointTrajectory goal to
#    arm_trajectory_controller, or MoveIt/Servo in a later milestone.

# 4. Release control authority in the mandatory reverse order (controller,
#    then component) before tearing down:
ros2 launch g1_bringup deactivate_arm.launch.py

# 5. Stop the launch (Ctrl-C).
```

**Deactivate before killing the launch is the documented clean stop.** `deactivate_arm.launch.py`
blocks for roughly `blend_ramp_down_s` (2.0 s default) while `G1ArmSdkSystem`'s own `on_deactivate`
ramps the blend weight to 0 synchronously -- that's by design (see `g1_hardware_interface`'s
README), not a hang. **Ctrl-C while the component is still active also ramps down safely**: Humble's
`controller_manager` runs the component's `on_deactivate` (the same ~2 s clean ramp) before
`on_shutdown` on SIGINT/SIGTERM, and `sim.launch.py`'s `RegisterEventHandler` on the sim process's
exit additionally guarantees a dead sim tears down the whole launch rather than leaving controllers
commanding nothing.

**Acquire/release order is mandatory, not stylistic:** Humble ties command-interface availability to
hardware component state, so activating the controller before the component (or deactivating the
component before the controller) can fail the switch, or leave a controller claiming interfaces out
from under a deactivating component. `activate_arm`/`deactivate_arm` encode the correct order so
this is never left to be gotten right by hand.

## Domain/DDS story

The whole container runs on `ROS_DOMAIN_ID=1` (set unconditionally by `.devcontainer/Dockerfile`)
with CycloneDDS pinned to the `lo` interface, for this sim-first milestone -- this is a dedicated
local domain so a real robot on the network (default domain, its own interface) is never at risk of
crosstalk. `unitree_mujoco`'s own `simulate/config.yaml` independently defaults to `domain_id: 1`,
`interface: "lo"` -- the same values, so its bare-DDS layer and our ROS graph can see each other.
Switching from sim to hardware is a domain/interface change, not a code change.
`sim.launch.py` asserts `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, and `ROS_DOMAIN_ID` up front and
fails fast with an actionable message if the container isn't configured as expected, rather than
leaving you to debug a silently empty ROS graph.

### A footgun specific to launching `unitree_mujoco` from a ROS environment

`unitree_mujoco` is a native `unitree_sdk2` DDS application, not a ROS node, and links its own
build of CycloneDDS from `/opt/unitree_robotics/lib`. Sourcing a ROS environment (as any shell
running `ros2 launch` already has) prepends `/opt/ros/humble/lib*` to `LD_LIBRARY_PATH` ahead of
that directory; since the binary's own rpath is a `RUNPATH` (resolved *after* `LD_LIBRARY_PATH`,
unlike the older `RPATH`), this shadows the correct build with ROS's own, separately-built
`libddsc.so.0` -- an ABI-incompatible copy that crashes the sim on its very first DDS write
(observed directly: a heap-corruption/assertion abort inside `libddsc.so.0`; a `gdb` backtrace
confirmed the crash was inside `/opt/ros/humble/lib/.../libddsc.so.0`, not
`/opt/unitree_robotics/lib`). `sim.launch.py` works around this by prepending
`/opt/unitree_robotics/lib` back onto `LD_LIBRARY_PATH` for the sim process specifically (not
replacing it -- `xvfb-run`/GL still need the rest of the path).

## `motion_service_sim` -- SIM-ONLY, read this before running anything

**This node is never to be launched near real hardware.** On the real G1, the onboard motion
service is the sole owner of `/lowcmd` and provides the weight-blended `/arm_sdk` interface
`g1_hardware_interface` talks to. `unitree_mujoco` does not emulate that service: it only emulates
the low-level device (subscribes `/lowcmd`, publishes `/lowstate`). Nothing in sim services
`/arm_sdk`, and with nothing commanding the legs, the simulated robot collapses.

`motion_service_sim` closes that gap kinematically, sim-side only:

- Subscribes `/lowstate` (`unitree_hg/msg/LowState`, best-effort, keep-last(1), volatile) and, on
  the first sample received, captures the full-body measured pose as a frozen **hold pose** --
  the reference every subsequent tick holds legs/waist against and blends arms toward at weight 0.
- Subscribes `/arm_sdk` (`unitree_hg/msg/LowCmd`, reliable, keep-last(1), volatile -- matching
  `g1_hardware_interface::G1ArmSdkSystem`'s publisher exactly, the sole publisher on that topic).
- Publishes `/lowcmd` (`unitree_hg/msg/LowCmd`, best-effort, keep-last(1), volatile -- matching
  `unitree_mujoco`'s own `rt/lowcmd` subscription QoS) at a fixed `publish_rate_hz`, once the hold
  pose has been captured:
  - **Legs (motors 0-11) + waist (12-14):** stiff-held at the captured pose (`q` = hold value,
    `dq`/`tau` = 0, per-group `kp`/`kd`) -- this is the emulated stand-in for the onboard balance
    controller.
  - **Arms (motors 15-28):** blended between the captured hold pose and the incoming `/arm_sdk`
    command, on `q` and on `kp`/`kd` alike:
    `published = hold * (1 - w) + commanded * w`, where `w` is the effective blend weight (see
    below) and `commanded_kp`/`commanded_kd` come straight from the incoming `/arm_sdk` message.
  - The weight slot (`motor_cmd[29].q`, `G1Arm7JointIndex::NOT_USED_JOINT`) is set to the effective
    weight.
  - Outgoing CRC is computed via `g1_hardware_interface`'s vendored, exported
    `computeLowCmdCrc()` -- the same routine `G1ArmSdkSystem` uses.
- **`mode`/`mode_pr`/`mode_machine` are deliberately never set.** Read directly from
  `unitree_mujoco`'s own source (`simulate/src/unitree_sdk2_bridge.h`, `RobotBridge::run()`): its
  actuator torque is computed purely as `tau_ff + kp * (q_des - q_meas) + kd * (dq_des - dq_meas)`
  per motor slot -- the incoming `LowCmd`'s mode fields are never read for actuation in this sim.
  This is a **sim-specific finding, confirmed live** (the robot stands and the arm tracks with
  these fields left at their zero default); the real onboard motion service's use of these fields,
  if any, is unverified and stays a hardware re-validation item.

### `arm_sdk` staleness policy -- bridge policy, not vendor semantics

If the newest `/arm_sdk` message is older than `arm_sdk_timeout_ms` (default 500 ms), or none has
ever arrived, the *effective* blend weight decays toward 0 at a rate of
`1 / timeout_ramp_down_s` per second (default 1.0 s) instead of tracking the raw commanded weight
-- so a silent `g1_hardware_interface` (deactivated, crashed, or simply not yet launched) causes
the arms to ease back to the hold pose smoothly rather than freezing at whatever weight/position
they last held. A fresh message resumes tracking from whatever the effective weight currently sits
at, never snapping. **This is this bridge's own policy, invented for sim test scaffolding, not a
documented property of the real motion service** -- what the real onboard controller does if its
`/arm_sdk` publisher goes silent at weight 1 is unverified and stays a hardware re-validation item.

## Pelvis pin -- SIM-ONLY standing scaffolding, not a balance controller

The real G1 stays upright because the **vendor's onboard controller owns balance and
locomotion** -- this stack only ever commands the arms (weight-blended via `rt/arm_sdk`, through
`motion_service_sim` here) and, in a later milestone, drives the legs at the velocity level
through Unitree's `LocoClient`. We never write balance ourselves. `unitree_mujoco` does **not**
emulate that onboard controller: it is a low-level device only. So in sim there is nothing
holding the robot up, and a joint-space stiff-hold cannot substitute for it -- a floating-base
biped is an inverted pendulum, and holding each joint stiff still lets the whole body topple
about the feet (no centre-of-mass/ZMP feedback). Measured directly: the robot tips to ~55-60 deg
within ~1.5 s of spawn even with the bridge commanding a full-body hold from the first tick, so
this is not a startup-timing gap -- it is a missing capability.

To validate the arm bridge in isolation until a real balance/locomotion controller exists,
`sim.launch.py` welds the pelvis to the world:

- **What:** a `weld` equality constraint (fixes the floating base in **both** position and
  orientation -- a `connect` point-pin would still allow toppling rotation) at the pelvis's spawn
  pose, in `mjcf/g1_pinned_scene.xml`, a `g1_bringup`-owned scene overlay that composes the
  vendored G1 model via `<include>`. The vendored model files are not modified.
- **Scope:** the weld constrains the pelvis body only. All 29 actuated joints -- the 14 arm
  joints included -- remain driven exactly as before via `/lowcmd`; a weld on the floating base
  adds no constraint to any actuated joint. Verified: with the pin active, a commanded arm
  trajectory still tracks to target while the pelvis holds at its spawn pose (z 0.793, < 1 deg
  tilt).
- **Toggle:** on by default; `ros2 launch g1_bringup sim.launch.py pin_pelvis:=false` disables it
  (the robot then topples on spawn, as above). **This is testing scaffolding, not balance** --
  expected to be removed or replaced when the `LocoClient` milestone adds genuine
  standing/locomotion.
- **Staging note:** the overlay is copied next to the vendored model at launch (and removed on
  shutdown) because MuJoCo 3.3.6 does not reliably resolve the model's relative `meshdir` when the
  including file lives in a foreign directory. See the overlay's header comment.

The integration tests assert the pelvis stays pinned (height and orientation within bounds)
across the whole bring-up -> idle -> activate -> command -> deactivate sequence, so a broken or
disabled pin fails the suite loudly rather than silently validating the arm bridge on a collapsed
robot.

### Why a plain node, not lifecycle-managed

This is sim test scaffolding standing in for an always-on vendor service that has no
activate/deactivate concept of its own on the real robot -- there's no meaningful inactive state
for it to sit in. It is launched exclusively by `sim.launch.py` and lives for exactly as long as
the sim does.

### Parameters (`config/motion_service_sim.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `publish_rate_hz` | `500.0` Hz | `/lowcmd` publish rate. |
| `leg_kp` / `leg_kd` | `100.0` / `1.0` | Stiff-hold gains for motors 0-11. |
| `waist_kp` / `waist_kd` | `50.0` / `1.0` | Stiff-hold gains for motors 12-14. |
| `arm_hold_kp` / `arm_hold_kd` | `40.0` / `1.0` | Arm gains used at blend weight 0 (this bridge's stand-in for the onboard controller's own default arm behavior). |
| `arm_sdk_timeout_ms` | `500.0` ms | `/arm_sdk` age beyond this is "stale" (bridge policy, see above). |
| `timeout_ramp_down_s` | `1.0` s | Weight decay/resume rate on staleness. |

`leg_kp`/`leg_kd`/`waist_kp`/`waist_kd` provenance: Unitree's own
`unitree_ros2/example/src/src/g1/lowlevel/g1_low_level_example.cpp` holds a captured "zero posture"
with `motor_cmd[i].kp = (i < 13) ? 100.0 : 50.0` and `kd = 1.0` for every motor -- i.e. legs and
`waist_yaw` (motor 12) at `kp=100`, `waist_roll`/`waist_pitch` (13, 14) at the gentler `kp=50`
alongside the arms. This bridge holds the whole waist group at that gentler value uniformly, the
more conservative of the two for a component that's locking a static pose rather than driving
locomotion. `arm_hold_kp`/`arm_hold_kd` match this repo's own
`g1_description/config/arm_sdk_params.yaml` shoulder/elbow default.

## `config/controllers.yaml`

`controller_manager` at `update_rate: 200`. `G1ArmSdkSystem` starts `inactive` (configured but not
publishing -- see the operating procedure above). `joint_state_broadcaster` is spawned active with
its own defaults (nothing to tune: it broadcasts whatever state interfaces the one hardware
component exports). `arm_trajectory_controller`
(`joint_trajectory_controller/JointTrajectoryController`, position command, position+velocity
state) is spawned `--inactive`, covering exactly the 14 arm joints from `g1_description`, with
relaxed, explicitly-commented sim tolerances (`stopped_velocity_tolerance`, `goal_time`) -- the
bridge's own weight/slew ramping adds latency the defaults don't expect; re-tighten against real
hardware dynamics when that milestone arrives.

## Hand joints stay inert this milestone

`g1_description`'s vendored URDF includes the DEX3 hand joints for correct kinematic structure, but
they get no `ros2_control` interfaces, and `unitree_mujoco`'s G1 MJCF has no hand joints or
feedback at all (see `g1_description/README.md`). Nothing in this package's launch/config touches
hand joints; their TF frames simply don't resolve to a live pose until the hand-control milestone.

## Building, testing, and a fresh clone

```bash
# From a fresh clone: pull in the vcs-imported externals. workspace/src/unitree_ros2 is
# gitignored by design -- this template keeps most ROS packages in their own repos,
# assembled into workspace/src via workspace.repos.
vcs import workspace/src < workspace.repos

colcon build --symlink-install --packages-select g1_bringup
colcon test --packages-select g1_bringup
colcon test-result --verbose
```

`test_blend_math` covers the pure weight decay/resume policy and the hold/commanded blend with no
sim or DDS required. `test_assemble_sim_low_cmd` covers `publishTick()`'s `/lowcmd` assembly
(extracted into `assembleSimLowCmd()`): leg/waist slots hold at the captured pose with their group
gains, arm slots blend hold and commanded values by weight, and the weight slot echoes the
effective weight -- same no-sim/no-DDS treatment as `test_blend_math`. `test/test_sim_bringup.launch.py`
and `test/test_arm_command.launch.py` are headless `launch_testing` integration suites against the
real sim (see their own docstrings for what each asserts). Plus `clang-format` against the repo
root's `.clang-format`, `ruff` against `ruff.toml` (launch files, scripts, and tests),
`ament_lint_cmake`, and `xmllint` on this package's own XML files.

## Language note

The bridge node is C++17 (a >50 Hz control-rate loop, squarely in the "always C++" category).
Everything else in this package is Python:

- **Launch files** (`launch/*.launch.py`): ROS 2 launch descriptions are authored in Python --
  there is no C++ path for this.
- **`launch_testing` suites** (`test/*.launch.py`): `launch_testing` is the standard ROS 2
  integration-test harness for exercising launch files against a live system, and it's
  Python-only.
- **`scripts/activate_arm`, `scripts/deactivate_arm`**: one-shot administrative sequencing tools
  (a handful of bounded-retry service calls, well under 1 Hz) -- the same category as the launch
  files they're siblings to, not a control loop. `rclpy`'s synchronous `spin_until_future_complete`
  ergonomics are a natural fit for "wait for a topic, then call two services in order, with
  retries"; a C++ rewrite would just be more code for the same one-shot sequencing logic with no
  real-time or performance argument for it.
