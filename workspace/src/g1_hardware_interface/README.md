# g1_hardware_interface

`ros2_control` `SystemInterface` plugin bridging standard joint command and state interfaces onto
the Unitree G1's weight-blended `rt/arm_sdk` DDS channel, for the 14 arm joints only. Legs, waist
and hands stay under the onboard controller at all times. This component never touches them and
never publishes on raw `/lowcmd`.

`ament_cmake`, C++17.

## Why this exists

The G1's arms can be driven two ways: raw `/lowcmd`, where you become the whole-body balance
controller, or `rt/arm_sdk`, which weight-blends your arm targets with the onboard controller's own
arm behaviour while that controller keeps the legs balancing. This plugin exports standard
`position` command and state interfaces for the 14 arm joints and translates them onto `/arm_sdk`,
so a `joint_trajectory_controller` (or MoveIt Servo later) drives the arms like any other
`ros2_control` manipulator without touching the balance loop.

## Layout

| Path | Contents |
|---|---|
| `arm_ramp_engine.{hpp,cpp}` | Pure, ROS-free ramp and slew safety logic: blend-weight ramping, per-joint slew clamping, motor-index validation, stale-feedback escalation. Directly unit-testable. |
| `g1_arm_sdk_system.{hpp,cpp}` | The plugin itself: parameters, lifecycle, DDS I/O, `LowCmd` assembly, threading contract. |
| `motor_crc_hg.{hpp,cpp}` | Vendored CRC. See below. |
| `g1_hardware_interface.xml` | pluginlib export description. |

## Exported plugin and interfaces

`g1_hardware_interface/G1ArmSdkSystem`, declared `type="system"` in the URDF.
`g1_description`'s `g1_arm_sdk.urdf.xacro` wires it up including every `<param>`.

Per arm joint (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw, both sides):

| Interface | Direction | Source |
|---|---|---|
| `position` | command | Position-only. Mode switching is reserved for a later Servo velocity mode. |
| `position` | state | `motor_state[motor_index].q` |
| `velocity` | state | `motor_state[motor_index].dq` |
| `effort` | state | `motor_state[motor_index].tau_est` |

## Topics

| Topic | Direction | Type | QoS |
|---|---|---|---|
| `/lowstate` | in | `unitree_hg/msg/LowState` | best-effort, keep-last(1), volatile |
| `/arm_sdk` | out | `unitree_hg/msg/LowCmd` | reliable, keep-last(1), volatile |

`/lowstate` is best-effort so it matches either a best-effort or reliable publisher (the sim
publishes reliable; hardware may differ), and only the newest of the 500 to 900 Hz stream matters.
`/arm_sdk` is reliable because this component is the sole writer, so a dropped command should be
retried rather than swallowed. Depth 1 because only the newest ramp tick matters.

Nothing in sim subscribes `/arm_sdk`: `unitree_mujoco` emulates only the low-level device. Testing
against the sim proves this plugin's own output (ramp shape, hold accuracy, rate, CRC, safety
behaviour); `g1_motion_service_sim`'s node closes the loop kinematically for end-to-end tests.

## Parameters

All parameters come from the URDF's `<ros2_control><hardware><param>` tags, routed from
`g1_description/config/arm_sdk_params.yaml`. Humble hardware plugins only receive parameters parsed
from `HardwareInfo`; `controller_manager`'s YAML never reaches them. `on_init` validates that every
one is present, parseable and strictly positive, failing the component with a clear message
otherwise.

| Param | Meaning |
|---|---|
| `command_publish_rate` | `/arm_sdk` publish rate, independent of `controller_manager`'s `update_rate`. |
| `blend_ramp_up_s` | Weight eases 0 to 1 over this long on activate. |
| `blend_ramp_down_s` | Weight eases 1 to 0 over this long on a clean deactivate. |
| `emergency_ramp_down_s` | Faster ramp on stale feedback, error, or shutdown. |
| `max_joint_velocity_rad_s` | Slew clamp on every commanded joint, independent of the weight ramp. |
| `lowstate_timeout_ms` | `/lowstate` age beyond this is stale: blocks activation, and while active triggers the emergency ramp. |

Per joint: `motor_index` (unique, within `[15, 28]`, the arm range of Unitree's
`G1Arm7JointIndex`), `kp`, `kd`.

## Safety and authority model

**Single writer by construction.** One `<ros2_control>` System means one `G1ArmSdkSystem`, the only
thing here that publishes `/arm_sdk`. While active, a 1 Hz advisory timer counts publishers; a
second one is logged and escalates the shared `mode_` atomic from `ACTIVE` to
`EMERGENCY_RAMP_DOWN`, so the still-ticking `write()` drives the ramp itself. The timer never
touches the ramp engine or publisher directly. Advisory only; real cross-process arbitration is a
future behavior-tree concern.

**Self-gated, not framework-gated.** `write()` publishes only while the internal mode atomic says
so. Humble's `controller_manager` still calls `write()` on an inactive component, so "commands only
flow while active" is enforced here rather than assumed.

**Ramp, never snap.** The blend weight (`motor_cmd[29].q`) eases 0 to 1 on activate and 1 to 0 on
deactivate, error, shutdown or stale feedback, at a rate that is purely a function of the current
mode each tick. A deactivate mid-ramp-up just changes direction from wherever the weight is,
monotonically, with no special case. Every commanded position is independently slew-clamped from
wherever it was last published.

**Hold in place on activate.** `on_activate` requires a `/lowstate` sample fresher than
`lowstate_timeout_ms` or fails with `ERROR` and publishes nothing. On success the measured arm
position is written into the command interfaces, so `write()` always has a valid hold target before
any controller sends a command.

**Stale feedback is self-protecting.** `write()` checks `/lowstate` age every tick and escalates to
the emergency ramp on its own initiative, without waiting for `read()`'s `ERROR` or for `on_error`.
`read()` still returns `ERROR` as belt and braces.

**Lifecycle transitions own the ramp.** `on_deactivate`, `on_error` and `on_shutdown` run the
ramp-down themselves, synchronously, via `rampDownSynchronously()`, which is reachable only from
those three transitions. This is deliberate: `resource_manager` serialises a component's lifecycle
transitions against its own `read()`/`write()` calls, so whichever holds the floor is unambiguously
the sole writer for that window. The original design had the transition request a ramp via the
shared atomic and block waiting for `write()`; that deadlocks in practice. The function also never
de-escalates: if `write()` already escalated to the emergency ramp, a clean deactivate continues
the faster ramp rather than downgrading it.

Consequence worth flagging: because transitions serialise against the whole read/write loop, a 2 s
clean deactivate blocks the `controller_manager` cycle for that duration. Fine with one active
component; re-examine when several exist.

**Ordering.** Activate this component before `arm_trajectory_controller`, and deactivate the
controller first. Humble ties command-interface availability to component state, so the reverse
order can fail the switch. `g1_bringup`'s `activate_arm` and `deactivate_arm` enforce this.

**Stopping.** Deactivate before killing the launch is the normal path. Killing the process while
active still ramps down safely: `controller_manager` runs `on_deactivate` before `on_shutdown` on
SIGINT/SIGTERM, so Ctrl-C exercises the clean ramp. `on_shutdown`'s emergency ramp is the fallback
for a kill path that skips deactivate.

**RT constraints.** `read()` and `write()` do no allocation (fixed-size arrays, a preallocated
`LowCmd`), take no blocking locks (`trylock()`), throw nothing, and log nothing on the hot path.
The lifecycle ramp deliberately runs off the RT path, so it may block and log.

## Threading

| Thread | Does |
|---|---|
| `controller_manager` RT thread | `read()` and `write()` only. `read()` drains the `/lowstate` buffer via `readFromRT()`. `write()` throttles to `command_publish_rate` with a period-guard accumulator, while ramp and slew state advance every tick from the real elapsed period. |
| Component's hidden node + `SingleThreadedExecutor` | Services the `/lowstate` subscription and the advisory timer. Started in `on_configure`, never added to `controller_manager`'s executor. `on_configure` bounded-waits for `is_spinning()` first, because Humble's `cancel()` and `spin()` write the same flag with no ordering guarantee and losing that race deadlocks the join. |
| `RealtimePublisher`'s internal thread | The actual DDS publish, decoupled by trylock-copy-publish. |
| Lifecycle callbacks | Whatever thread `resource_manager` uses, serialised against the RT thread. The deactivate paths block it for the ramp duration. |

## CRC

Every outgoing `LowCmd` gets a CRC computed in `write()` using a vendored, unmodified copy of
Unitree's routine (`unitreerobotics/unitree_ros2`, commit `668d1ec5`, BSD-3-Clause), wrapped in
this package's namespace so a pluginlib-loaded `.so` cannot collide with another's global symbols.
Unitree's own `g1_arm_sdk_dds_example` does not actually call this CRC on its `/arm_sdk` publishes.
We compute it anyway since it costs nothing on the write path.

`assembleLowCmd()` touches only the 14 arm slots and the weight slot; every other slot stays at its
constructed zero. It does not set `mode_pr`, `mode_machine`, or any motor `mode` field, mirroring
what Unitree's own arm_sdk example touches.

## Tests

```bash
colcon build --symlink-install --packages-select g1_hardware_interface
colcon test --packages-select g1_hardware_interface
colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `test_pluginlib_loading` | pluginlib discovery. |
| `test_arm_ramp_engine` | Weight monotonicity and slope in both directions including a reversal mid-ramp, emergency ramp duration, slew clamp (exact at the boundary, independent per joint), seed-from-measured, motor-index validation, idempotent staleness escalation. |
| `test_assemble_low_cmd` | Every non-arm, non-weight slot stays zero; arm slots get exactly `q`/`kp`/`kd`; the weight lands on `motor_cmd[29]`; mode fields untouched. |

No sim required. Plus `clang-format`, `ament_lint_cmake` and `xmllint`.

End-to-end validation lives in `g1_bringup`'s `test_sim_bringup.launch.py` and
`test_arm_command.launch.py`.

### Measured against the sim

Captured during manual validation against a headless `unitree_mujoco`:

- `/lowstate` at about 900 Hz; `/joint_states` at the configured 200 Hz with real measured
  positions.
- Ramp-up 0 to 1 in about 1.99 s against a configured 2.0 s, at about 100.04 Hz publish rate
  against a configured 100 Hz, with the commanded position held constant throughout.
- Deactivate 1 to 0 in about 2.01 s, monotonic, publishing stopped immediately at zero.
- Killing the sim while active: staleness past 100 ms triggered the autonomous emergency ramp on
  `write()`'s own initiative, 1 to 0 in about 0.49 s against a configured 0.5 s.
- A clean deactivate blocked for about 2.02 s end to end; `SIGTERM` while active ran
  `on_deactivate` then `on_shutdown` with a clean exit.
