# g1_bringup

Sim bring-up for the G1 arm bridge milestone: launches `unitree_mujoco` alongside the
`ros2_control` stack (`g1_description` + `g1_hardware_interface`), plus a sim-only node that
stands in for the onboard motion service so the simulated robot doesn't collapse. `ament_cmake`,
C++17 node + Python launch files and integration tests.

## `arm_sdk_sim_bridge` -- SIM-ONLY, read this before running anything

**This node is never to be launched near real hardware.** On the real G1, the onboard motion
service is the sole owner of `/lowcmd` and provides the weight-blended `/arm_sdk` interface
`g1_hardware_interface` talks to. `unitree_mujoco` does not emulate that service: it only emulates
the low-level device (subscribes `/lowcmd`, publishes `/lowstate`). Nothing in sim services
`/arm_sdk`, and with nothing commanding the legs, the simulated robot collapses.

`arm_sdk_sim_bridge` closes that gap kinematically, sim-side only:

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

### Why a plain node, not lifecycle-managed

This is sim test scaffolding standing in for an always-on vendor service that has no
activate/deactivate concept of its own on the real robot -- there's no meaningful inactive state
for it to sit in. It is launched exclusively by `sim.launch.py` and lives for exactly as long as
the sim does.

### Parameters (`config/arm_sdk_sim_bridge.yaml`)

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

## Building and testing

```bash
colcon build --symlink-install --packages-select g1_bringup
colcon test --packages-select g1_bringup
colcon test-result --verbose
```

`test_blend_math` covers the pure weight decay/resume policy and the hold/commanded blend with no
sim or DDS required. Plus `clang-format` against the repo root's `.clang-format`, `ament_lint_cmake`,
and `xmllint` on this package's own XML files.

## Language note

The bridge node is C++17 (a >50 Hz control-rate loop, squarely in the "always C++" category).
Python is used elsewhere in this package for launch files and integration tests -- ROS 2 launch
descriptions and `launch_testing` are Python-only surfaces, so there is no C++ path for either
of those.
