# g1_hardware_interface

`ros2_control` `SystemInterface` plugin bridging standard joint command/state interfaces onto the
Unitree G1's weight-blended `rt/arm_sdk` DDS channel, for the 14 arm joints only. Legs, waist, and
hands stay under the onboard controller at all times -- this component never touches them, and it
never publishes on raw `/lowcmd`.

`ament_cmake`, C++17.

## Why this exists

The G1's arms can be driven two ways: raw `/lowcmd` (you become the whole-body balance
controller -- dangerous, and not what this package does) or `rt/arm_sdk`, a channel that
weight-blends your commanded arm targets with the onboard controller's own arm behavior while the
onboard controller keeps the legs balancing. This plugin exports standard `ros2_control`
`position` command/state interfaces for the 14 arm joints and translates them onto `rt/arm_sdk`
(seen from ROS as `/arm_sdk`), so a `joint_trajectory_controller` (or MoveIt Servo, later) can drive
the arms exactly like any other `ros2_control`-backed manipulator, without ever touching legs,
waist, or the balance loop.

## Layout

- `include/g1_hardware_interface/arm_ramp_engine.hpp`, `src/arm_ramp_engine.cpp` -- the pure,
  ROS-free ramp/slew safety logic (blend-weight ramping, per-joint slew clamping, motor-index
  validation, the stale-feedback escalation decision). No ROS includes, so it's directly
  unit-testable without a live hardware component -- this is the safety-critical surface the unit
  tests target.
- `include/g1_hardware_interface/g1_arm_sdk_system.hpp`, `src/g1_arm_sdk_system.cpp` -- the
  `hardware_interface::SystemInterface` plugin itself: parameter parsing, lifecycle, DDS I/O, LowCmd
  assembly, and the concurrency contract that keeps exactly one thread publishing at a time.
- `include/g1_hardware_interface/motor_crc_hg.hpp`, `src/motor_crc_hg.cpp` -- vendored CRC (see
  below).
- `g1_hardware_interface.xml` -- pluginlib export description.
- `test/` -- gmock/gtest suite.

## Building and testing

```bash
colcon build --symlink-install --packages-select g1_hardware_interface
colcon test --packages-select g1_hardware_interface
colcon test-result --verbose
```

Three gmock binaries, no sim required:

- `test_pluginlib_loading` -- the pluginlib discovery proof (see "Exported plugin").
- `test_arm_ramp_engine` -- the safety-critical surface: weight monotonicity and slope bounds in
  both directions (including a ramp-down triggered mid-ramp-up), the emergency ramp's duration,
  the slew clamp (exact at the boundary, independent per joint), seed-from-measured, motor-index
  map validation, and the staleness-escalation decision (idempotent -- never oscillates or
  de-escalates on its own).
- `test_assemble_low_cmd` -- the outgoing `LowCmd` assembly: every non-arm, non-weight slot stays
  zeroed; arm slots get exactly `q`/`kp`/`kd` (never `mode`); the weight lands on `motor_cmd[29]`;
  `mode_pr`/`mode_machine` are never touched.

Plus `clang-format` (against the repo root's `.clang-format`), `ament_lint_cmake`, and `xmllint` on
the package's own XML files.

## Exported plugin

`g1_hardware_interface/G1ArmSdkSystem`, declared with `type="system"` in the robot's URDF
(`g1_description`'s `g1_arm_sdk.urdf.xacro` wires this up already, including every `<param>` this
package reads).

## Exported interfaces

Per arm joint (14 total: shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw, left and right):

| Interface | Direction | Notes |
|---|---|---|
| `position` | command | Position-only. `prepare/perform_command_mode_switch` are reserved for a later Servo velocity mode. |
| `position` | state | From `LowState.motor_state[motor_index].q`. |
| `velocity` | state | From `motor_state[motor_index].dq`. |
| `effort` | state | From `motor_state[motor_index].tau_est`. |

## Topics

| Topic | Direction | Type | QoS | Why |
|---|---|---|---|---|
| `/lowstate` | in | `unitree_hg/msg/LowState` | best-effort, keep-last(1), volatile | Compatible with either a best-effort or reliable publisher (the sim happens to publish RELIABLE; hardware may differ) and only the newest of the ~500-900 Hz stream ever matters. |
| `/arm_sdk` | out | `unitree_hg/msg/LowCmd` | reliable, keep-last(1), volatile | We're the sole authority on this channel (single writer by construction), so the DDS layer should retry a dropped command rather than silently swallow it; keep-last(1) because only the newest ramp/slew tick ever matters -- Unitree's own example uses depth 10, but nothing here needs history. |

The sim (`unitree_mujoco`) emulates only the low-level device: it subscribes `/lowcmd` and
publishes `/lowstate`, but nothing in sim subscribes `/arm_sdk` -- there's no onboard motion
service to close the loop with in simulation. Validating against the sim (see below) proves this
plugin's own output (ramp shape, hold accuracy, publish rate, CRC, safety behavior); a separate
sim-only bridge node (a later package) closes the loop kinematically for end-to-end sim testing.

## Parameters (all via the URDF's `<ros2_control><hardware><param>` tags)

Humble hardware plugins only receive parameters parsed from `HardwareInfo` -- `controller_manager`'s
own YAML never reaches them -- so every tunable below must come from the URDF's `<param>` tags
(routed from `g1_description/config/arm_sdk_params.yaml`). `on_init` validates all of them are
present, parseable, and strictly positive, and fails the whole component with a clear log message
otherwise.

| Param | Meaning |
|---|---|
| `command_publish_rate` (Hz) | Throttle on `/arm_sdk` publication, independent of the `controller_manager`'s own `update_rate`. |
| `blend_ramp_up_s` | Weight eases 0->1 over this long on activate. |
| `blend_ramp_down_s` | Weight eases 1->0 over this long on a clean deactivate. |
| `emergency_ramp_down_s` | Faster ramp-down on stale feedback, an error, or shutdown. |
| `max_joint_velocity_rad_s` | Slew clamp applied to every commanded joint, independent of the weight ramp. |
| `lowstate_timeout_ms` | `/lowstate` age beyond this is "stale": blocks activation and (while active) triggers the autonomous emergency ramp-down. |

Per joint: `motor_index` (position in the `LowCmd`/`LowState` motor array; must be unique and in
`[15, 28]`, the arm range of Unitree's `G1Arm7JointIndex`), `kp`, `kd`.

## Safety / authority model

**Single writer by construction.** One `<ros2_control>` System in the URDF means one
`G1ArmSdkSystem`, the only thing in this stack that ever publishes `/arm_sdk`. While active, a
~1 Hz advisory timer counts `/arm_sdk` publishers; if a second one ever appears, it logs the
conflict and escalates the shared `mode_` atomic from `ACTIVE` to `EMERGENCY_RAMP_DOWN` -- exactly
the same compare-and-swap `write()` performs on itself for stale feedback (see below) -- so the
still-ticking `write()` on the RT thread drives and finishes the ramp-down itself. The timer never
touches the ramp engine or the publisher directly, so there is never a second thread doing so
concurrently with `write()`. Advisory only -- real cross-process arbitration over control authority
is a future behavior-tree authority arbiter.

**Self-gated, not framework-gated.** `write()` publishes only while an internal
`ACTIVE`/`RAMP_DOWN`/`EMERGENCY_RAMP_DOWN` atomic (`mode_`) says so. Humble's `controller_manager`
still calls `write()` on an `INACTIVE` component, so "commands only flow while active" is enforced
here, not assumed from the framework.

**Ramp, never snap.** The blend weight (`motor_cmd[29].q`, `G1Arm7JointIndex::NOT_USED_JOINT`) eases
0->1 on activate and 1->0 on deactivate/error/shutdown/stale-feedback, at a rate that's purely a
function of the *current* mode each tick -- so a deactivate requested mid-ramp-up just changes
direction from wherever the weight already is, monotonically, with no special case. Every commanded
joint position is independently slew-clamped (`max_joint_velocity_rad_s`) toward its target,
starting from wherever it was last published -- never a jump.

**Hold-in-place on activate.** `on_activate` requires a fresh `/lowstate` sample (age <
`lowstate_timeout_ms`) or fails with `ERROR` and publishes nothing; on success, the measured arm
position is written directly into the exported command interfaces (so `write()` always has a valid
hold target even before any controller has sent a command) and the weight ramp starts from 0.

**Stale feedback while active is self-protecting.** `write()` checks `/lowstate` age every tick and
autonomously escalates to the emergency ramp-down the instant it goes stale -- it does not wait for
`read()`'s `ERROR` return or for `on_error` to be called. `read()` still returns `ERROR` while active
on stale feedback too, purely as belt-and-braces for whatever `resource_manager` does with it.

**Lifecycle authority and the concurrency contract.** `on_deactivate`/`on_error`/`on_shutdown` do
**not** ask `write()` to ramp down and wait for it -- they run the ramp-down themselves,
synchronously, before returning, via `rampDownSynchronously()`, which is reachable *only* from these
three lifecycle transitions (never from the advisory guard above, which only ever touches the shared
atomic). This is deliberate, not an oversight: `resource_manager` serializes a hardware component's
lifecycle transitions against its own `read()`/`write()` calls (confirmed directly during manual sim
validation -- `write()` provably never ticks while a transition callback is running). The original
design called for the transition to *request* a ramp-down via the shared atomic and block waiting
for `write()` to report weight 0; in practice that deadlocks (avoided only by a stall-detection
timeout firing early, which produced an instantaneous, unwanted snap to weight 0 -- observed
directly). Since the transition callback and `write()` can never run concurrently anyway, whichever
one currently holds the floor is unambiguously the sole writer for that window, so the transition
callback simply *is* the writer while it's in progress and drives the ramp to completion itself.
`rampDownSynchronously()` also never de-escalates: if `write()` has already autonomously escalated to
`EMERGENCY_RAMP_DOWN` (e.g. `/lowstate` went stale right as a clean deactivate starts), the
transition callback continues that faster ramp rather than downgrading to the slower one it was
asked for. A consequence worth flagging for later milestones: because `resource_manager` appears to
serialize transitions against its *entire* read/write loop, a ~2 s clean deactivate (or ~0.5 s
emergency ramp) blocks the whole `controller_manager` cycle for that duration -- fine for a single
active hardware component (this milestone), but worth re-examining once multiple concurrently-active
components exist.

**Activation/release ordering.** Acquire: activate this component *before* activating
`arm_trajectory_controller` (Humble ties command-interface availability to component state; the
reverse order can fail the controller switch). Release: deactivate the controller *before*
deactivating this component. Enforced in `g1_bringup`'s launch sequencing (a later package).

**Stop procedure.** Deactivate before killing the launch is the documented normal path. Killing the
process while active still ramps down safely: confirmed directly that Humble's `controller_manager`
runs `on_deactivate` (the normal ~2 s ramp) **before** `on_shutdown` on SIGINT/SIGTERM, so a Ctrl-C
exercises the clean ramp-down, not the emergency one; `on_shutdown`'s emergency ramp remains the
fallback for a kill path that reaches shutdown without deactivate.

**RT constraints.** `read()`/`write()` do zero allocation (fixed-size arrays, a preallocated
`LowCmd`), no blocking locks (`RealtimeBuffer`/`RealtimePublisher`'s `trylock()`), no exceptions, no
hot-path logging. The lifecycle-driven ramp-down above deliberately runs *off* the RT path (in the
transition callback itself), so it's allowed to block and log.

## CRC

Every outgoing `LowCmd` gets a CRC computed in `write()` via a vendored, unmodified copy of
Unitree's CRC routine (`unitreerobotics/unitree_ros2`, commit
`668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`, `example/src/{include/common,src/common}/motor_crc_hg.{h,cpp}`,
BSD-3-Clause), wrapped in this package's namespace so a shared library loaded via `pluginlib` can't
collide with another `.so`'s global symbols of the same name. Note: Unitree's own
`g1_arm_sdk_dds_example` does not actually call this CRC routine on its `/arm_sdk` publishes (it's
linked against the unrelated go2/b2 CRC variant and never invokes it) -- we compute it anyway, since
the design calls for it and it costs nothing on the write() path.

`assembleLowCmd()` touches only the 14 arm motor slots (`q`/`dq`/`tau`/`kp`/`kd`) and the weight
slot (`motor_cmd[29].q`); every other slot is left at the zero it was constructed with. It also does
**not** set `mode_pr`, `mode_machine`, or any motor's `mode` field -- mirroring exactly what
Unitree's own `g1_arm_sdk_dds_example` touches on the outgoing message and nothing else (their raw
low-level `/lowcmd` example does set `mode_machine` and each motor's `mode = 1`, but that's a
different, full-authority channel; the arm_sdk motion service evidently doesn't need them).

## Threading model

- **`controller_manager`'s RT thread**: `read()`/`write()` only. `read()` drains the `/lowstate`
  `RealtimeBuffer` (`readFromRT()`, never blocks). `write()` throttles publication to
  `command_publish_rate` using a period-guard accumulator (ramp/slew state still advances every
  tick from the real elapsed `period`, so it's correct at any `update_rate`); publishes via
  `RealtimePublisher::trylock()`/`unlockAndPublish()`.
- **The component's own hidden node + `SingleThreadedExecutor`** (started in `on_configure`, torn
  down in `on_cleanup`, idempotently rebuilt if `on_configure` runs again): services the `/lowstate`
  subscription and the ~1 Hz advisory publisher-count timer. Never added to `controller_manager`'s
  own executor. `on_configure` bounded-waits for `is_spinning()` before returning, so a later
  `cancel()` (from `on_cleanup`/the destructor) can never land before the executor thread has
  actually entered `spin()` -- Humble's `Executor::cancel()`/`spin()` both write the same internal
  flag with no ordering guarantee otherwise, and losing that race would spin forever and deadlock
  `executor_thread_.join()`.
- **`RealtimePublisher`'s own internal thread**: does the actual DDS `publish()` call, decoupled
  from the RT thread by the trylock/copy-then-publish pattern.
- **Lifecycle callbacks** (`on_activate`/`on_deactivate`/`on_error`/`on_shutdown`): run on whatever
  thread `resource_manager` invokes them from (confirmed serialized against the RT thread -- see
  the safety model above). `on_deactivate`/`on_error`/`on_shutdown` block that thread for the
  relevant ramp duration, via `rampDownSynchronously()`. `on_activate` does a single non-blocking
  `readFromRT()` (safe off the RT thread specifically because that serialization leaves the RT side
  quiescent for the transition's duration) and returns.
- **The advisory publisher-count timer** (on the hidden executor thread above): never spawns a
  thread of its own. It only ever compare-and-swaps the shared `mode_` atomic from `ACTIVE` to
  `EMERGENCY_RAMP_DOWN` -- the actual ramp-down is then driven by whichever thread already owns
  publishing, `write()` on the RT thread, exactly like the stale-feedback escalation path.

## Exercising this against the sim

Not yet wired into a launch file -- that's `g1_bringup`, the next package in this milestone. In the
meantime, manual validation used a scratch `controller_manager` (`ros2_control_node`) loaded with
`g1_description`'s expanded URDF and `robot_state_publisher`, against a headless `unitree_mujoco`
(G1). Representative numbers observed:

- `/lowstate` at ~900 Hz in sim; `/joint_states` (via `joint_state_broadcaster`) at the configured
  200 Hz with real, non-zero measured arm positions.
- Ramp-up on activate: weight 0->1 in ~1.99 s against a configured `blend_ramp_up_s` of 2.0 s, at an
  observed `/arm_sdk` publish rate of ~100.04 Hz against a configured `command_publish_rate` of
  100 Hz; the commanded arm position held exactly constant (no drift) throughout.
- Deactivate mid-steady-state: weight 1->0 in ~2.01 s (matches `blend_ramp_down_s`), smooth and
  monotonic, position held constant, publishing stopped immediately at weight 0 (confirmed by the
  external trace simply receiving no further messages).
- Killing the sim while active: `/lowstate` staleness (past `lowstate_timeout_ms` = 100 ms) triggered
  the autonomous emergency ramp entirely on `write()`'s own initiative -- weight 1->0 in ~0.49 s
  against a configured `emergency_ramp_down_s` of 0.5 s, then publishing stopped; `read()`'s belt-
  and-braces `ERROR` return was also observed to eventually drive the framework's own lifecycle
  state to a safe `UNCONFIGURED` via `on_error`, with no crash.
- A clean deactivate call was observed to block for ~2.02 s end-to-end (matching the ramp), and a
  process-level `SIGTERM` while active was confirmed to run `on_deactivate` then `on_shutdown` (see
  the safety model above) with a clean process exit.

## Language note

C++17 throughout -- this is a control-loop / hardware-interface package, squarely in the "always
C++" category (`>50 Hz`, real-time paths). No Python here.
