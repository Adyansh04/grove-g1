# g1_motion_service_sim

**SIM-ONLY. Never launch this near real hardware.**

Stand-in for the G1's onboard motion service. On the real robot that service owns `/lowcmd`
exclusively, provides the weight-blended `/arm_sdk` interface, and answers `/api/sport/*`.
`unitree_mujoco` emulates only the low-level device, so none of it exists in simulation. This node
fills all three gaps, sim-side only, and is launched exclusively by `g1_bringup`'s `sim.launch.py`.

`ament_cmake`, C++17.

## Why this is its own package

Everything here exists only because the simulator stops at the low-level device. That makes it one
deletion boundary rather than three features: at hardware bring-up you stop launching this package
and nothing else in the stack changes.

It is not in `g1_locomotion` because that package is the LocoClient *client* and this is the
LocoClient *server* — opposite ends of one wire, and `g1_locomotion`'s whole claim is that it
carries to hardware unchanged. It is not in `g1_sim` because that is the separate
`mujoco_ros2_control` sandbox track, which is under a sunset clause.

The sim-only-ness is structural, not a switch. The walking policy's `base_lin_vel` observation
comes from `/sportmodestate`, and the real G1 publishes `unitree_hg::SportModeState_`, which
carries no velocity field at all. See the Contract section.

## Running

Not launched directly. `g1_bringup`'s `sim.launch.py` starts it with both config files and the two
launch-computed overrides (`publish_lower_joint_states`, `walk_policy.enabled`):

```bash
ros2 launch g1_bringup sim.launch.py
```

### Topics

| Topic | Direction | Type | QoS |
|---|---|---|---|
| `/lowcmd` | out | `unitree_hg/msg/LowCmd` | best-effort, keep-last(1), volatile |
| `/lowstate` | in | `unitree_hg/msg/LowState` | best-effort, volatile |
| `/arm_sdk` | in | `unitree_hg/msg/LowCmd` | matches `G1ArmSdkSystem`'s publisher exactly |
| `/sportmodestate` | in | `unitree_go/msg/SportModeState` | best-effort, volatile |
| `/api/sport/request` | in | `unitree_api/msg/Request` | `QoS(10)` reliable, volatile |
| `/api/sport/response` | out | `unitree_api/msg/Response` | `QoS(1)` reliable, volatile |
| `/joint_states` | out | `sensor_msgs/msg/JointState` | keep-last(1), lower body only, `publish_lower_joint_states` |

It is a plain node rather than lifecycle-managed: it emulates an always-on vendor service that has
no activate/deactivate concept of its own.

## Arm path

- Captures a frozen hold pose from the first `/lowstate` sample.
- Subscribes `/arm_sdk`, matching `G1ArmSdkSystem`'s publisher QoS exactly.
- Publishes `/lowcmd` at `publish_rate_hz`, blending arms (motors 15-28) between the hold pose and
  the commanded values on `q`, `kp` and `kd` alike: `published = hold * (1 - w) + commanded * w`.
  The weight slot (`motor_cmd[29].q`) echoes the effective weight, and the CRC uses
  `g1_hardware_interface`'s vendored `computeLowCmdCrc()`.

`mode`, `mode_pr` and `mode_machine` are deliberately never set. `unitree_mujoco` computes actuator
torque purely as `tau_ff + kp * (q_des - q_meas) + kd * (dq_des - dq_meas)` and never reads them.
That is a sim-specific finding; what the real motion service does with these fields is unverified
and stays a hardware re-validation item.

**Staleness policy (bridge policy, not vendor semantics):** if the newest `/arm_sdk` message is
older than `arm_sdk_timeout_ms`, the effective blend weight decays toward zero at
`1 / timeout_ramp_down_s` per second, so a silent publisher eases the arms back to the hold pose
instead of freezing them. A fresh message resumes from wherever the weight sits, never snapping.
What the real controller does when its publisher goes silent at weight 1 is unverified.

## LocoClient wire responder

The same node answers `/api/sport/request`, mirroring the single-service reality on hardware. It
tracks an FSM state and applies the acceptance rules in `include/g1_motion_service_sim/loco_fsm.hpp`.
Those rules are load-bearing: an accepted `SET_VELOCITY` latches the command the walking policy
consumes, so this path drives motors 0-14. It never touches the arm slots and never publishes
`/lowcmd` itself.

**Reliability and durability on `/api/sport/*` are vendor-matched. Do not deviate.** History depth
is not RxO-matched, so it is picked per side: the request reader is deeper than the response
publisher so two requests in one DDS batch cannot overwrite each other in a depth-1 cache.

Every response echoes `header.identity` (both `id` and `api_id`). The correlator matches on `id`
only, and nothing on either side validates `api_id`, so a colliding `id` from another client on the
same domain would not be caught.

| API id | Name | Behaviour |
|---|---|---|
| `7001` | `GET_FSM_ID` | Returns `{"data": <fsm_id>}`, code `0`. |
| `7101` | `SET_FSM_ID` | Applies the legality table below. Code `0` or `7302`. |
| `7105` | `SET_VELOCITY` | Code `7301` unless the FSM is `Start`, else `0` and the command is latched. |
| `7106` | `SET_ARM_TASK` | Always rejected with `-2`. `WaveHand`/`ShakeHand` moves hand arm authority to the onboard controller, which would fight this stack's `rt/arm_sdk` blend weight. |
| anything else, or malformed JSON | | `-2` (`UT_ROBOT_TASK_UNKNOWN_ERROR`). |

FSM state starts at `Damp(1)`, matching the robot's boot state. Legal transitions:

```
Damp(1)    -> StandUp(4)
StandUp(4) -> Start(500), Damp(1)
Start(500) -> StandUp(4), Damp(1)
```

Every other edge is rejected `7302`. The vendor contract has no code specific to "illegal
transition", so reusing `7302` is this responder's approximation and stays a hardware
re-validation item.

## Walking policy

An RL walking policy that owns motors 0-14 (legs and waist) and balances the robot with no pelvis
weld. `/arm_sdk` keeps motors 15-28, so the two never contend: they are disjoint slices of one
`/lowcmd` message. The policy's own arm outputs are discarded.

**Provenance:** `policy/walker.onnx` and its external weights `walker.onnx.data` come from
[luckyrobots/g1-manipulation-challenge](https://github.com/luckyrobots/g1-manipulation-challenge),
which describes the policy as trained with RL in Isaac Lab. That repository publishes no licence
file and no attribution for the policy's own origin, so its redistribution terms are undeclared.
Recorded here rather than left implicit. The MJCF, meshes and scene from that repository are not
vendored and are not needed.

### Contract

- **Observation, 99 elements, fed raw:** `base_lin_vel(3)`, `base_ang_vel(3)`,
  `projected_gravity(3)`, `joint_pos(29, relative to the default posture)`, `joint_vel(29)`,
  `last_action(29)`, `command(3)`. The exported graph starts with `Sub(mean)` then `Div(std)`, so
  normalisation is already inside the model. Applying the `obs_mean`/`obs_std` arrays that ship
  alongside it would apply it twice.
- **Action:** `target = default_joint_pos + action * action_scales`, per joint.
- **Joint order:** identical to the Unitree DDS motor order (0-28), so `motor_state[i]` and
  `motor_cmd[i]` map straight through. Asserted at startup and in `test_walk_policy`.
- **Base linear velocity comes from `/sportmodestate`,** not `/lowstate`, which carries no such
  field. This is why the policy is structurally sim-only, and the reason is stronger than "that
  service is switched off": **the field does not exist on hardware at all.** `unitree_mujoco`
  publishes the **go2** `SportModeState` (16 fields, with `position` and `velocity` filled from
  MuJoCo `framepos`/`framelinvel` on the pelvis `imu` site) regardless of which robot is simulated.
  The real G1 publishes `unitree_hg::SportModeState_` on the same topic name, and that type carries
  only `fsm_id`, `fsm_mode`, `task_id` and `task_time`. No pose, no velocity. `rt/odommodestate` does
  not exist anywhere in Unitree's code. Supplying this observation on hardware needs real state
  estimation, which is a future milestone (see `g1_state_estimation`).
- Inference runs at 50 Hz on its own timer, about 0.02 ms per call.

Armature matching turned out not to matter. Ablating armature, frictionloss and damping against
`unitree_mujoco`'s `g1_29dof.xml` showed the model as shipped (uniform `armature=0.01`) performs
identically to per-joint values across standing, walking, turning and a 500 N perturbation. No MJCF
edits and no mesh vendoring are required.

### Measured behaviour: read before commanding velocities

There is a hard gait-initiation deadband with no hysteresis. Below it the robot stands still rather
than stepping, and kicking above it then dropping back stops the gait outright.

| Axis | No motion at or below | Steps from | Measured output |
|---|---|---|---|
| `vx` | 0.35 m/s | **0.40 m/s** | 0.5 to 0.35, 0.6 to 0.47, 1.0 to 0.93 m/s |
| `vy` | 0.30 m/s | **0.50 m/s** | 0.5 to 0.44, 1.0 to 0.93 m/s |
| `vyaw` (in place) | 0.60 rad/s | **1.50 rad/s** | 1.0 to 0.21, 1.5 to 1.08 rad/s |

Commands below threshold pass through unchanged and are logged, never scaled up. Turning a small
command into a large motion is exactly what this stack's control-mode rules exist to prevent.

Also measured:

- About 0.22 to 0.25 m/s of uncommanded lateral drift while walking forward, so straight-line
  walking curves.
- Turning collapses forward speed: `vx=0.6` with yaw commanded measures about -0.11 m/s.
- Balance is solid. The robot survives a 500 N, 50 ms lateral impulse while standing with about
  2.6 degrees of peak tilt.

Together these make it a teleop-grade locomotion source, not a planner-grade one. `g1_locomotion`'s
`g1_gait_shaper` is the consumer-side answer to this table.

### Authority

No new authority mechanism exists for the policy. The FSM legality table is the gate:
`SET_VELOCITY` is latched only when `checkVelocityAllowed()` accepted it, so outside `Start` it is
rejected `7301` and nothing reaches the policy. A successful transition away from `Start` clears
the latch, and the latch carries the request's own `duration` as a dead-man, so a silent bridge
stops the robot within a second.

The policy runs continuously from startup regardless of FSM state. Leg authority is gated on policy
freshness, not on the FSM, so releasing locomotion authority stops the walking without dropping the
robot. If inference goes stale the lower-body target freezes at the last policy output rather than
reverting to the captured spawn pose, which would be a large step away from the stance the robot is
actually in.

## Configuration

`config/motion_service_sim.yaml`:

| Param | Default | Meaning |
|---|---|---|
| `publish_rate_hz` | `500.0` | `/lowcmd` publish rate. |
| `leg_kp` / `leg_kd` | `100.0` / `1.0` | Stiff-hold gains, motors 0-11. |
| `waist_kp` / `waist_kd` | `50.0` / `1.0` | Stiff-hold gains, motors 12-14. |
| `arm_hold_kp` / `arm_hold_kd` | `40.0` / `1.0` | Arm gains at blend weight 0. |
| `arm_sdk_timeout_ms` | `500.0` | `/arm_sdk` age beyond this is stale. |
| `timeout_ramp_down_s` | `1.0` | Weight decay and resume rate. |

Gain provenance: Unitree's `g1_low_level_example.cpp` holds a captured posture with
`motor_cmd[i].kp = (i < 13) ? 100.0 : 50.0` and `kd = 1.0`. This bridge holds the whole waist group
at the gentler value, the more conservative choice for locking a static pose. The arm gains match
`g1_description/config/arm_sdk_params.yaml`.

`config/walk_policy.yaml` holds the policy's joint names, default posture, action scales, per-joint
gains, rates and limits. The file itself is commented; see it directly.

Two parameters are set by `sim.launch.py` rather than by either file, because both are launch
decisions: `publish_lower_joint_states` (on only with `sensors:=true`, since it costs work on the
1 kHz `/lowstate` path) and `walk_policy.enabled` (off when `pin_pelvis:=true`, since the weld and
the policy are the two possible owners of the legs and are never both active).

## Tests

| Test | Kind | Covers |
|---|---|---|
| `test_blend_math` | gmock | Blend-weight decay and resume, q/kp/kd blend. |
| `test_assemble_sim_low_cmd` | gmock | `/lowcmd` assembly: lower-body slots and gains, arm blend, weight-slot echo. |
| `test_loco_fsm` | gmock | FSM legality: every legal and illegal edge, `SET_VELOCITY`'s Start-only gate. |
| `test_walk_policy` | gmock | Policy contract (joint order, observation layout, un-normalised input, action mapping, dead-man) and the leg-authority fallback. |
| `test_walk_policy_session` | gmock | ONNX Runtime: shape contract, external-weight resolution, determinism, inference budget. |
| `clang_format_check_g1_motion_service_sim` | ctest | C++ formatting. |

```bash
colcon build --symlink-install --packages-select g1_motion_service_sim
colcon test --packages-select g1_motion_service_sim
colcon test-result --verbose
```

None of these need a simulator, DDS or a live graph, so they are fast and deterministic. The
launch-level suites that exercise this node against a real `unitree_mujoco` live in `g1_bringup`
(`test_loco`, `test_walk_stand`, `test_walk_teleop`, `test_walk_and_arm`), because they all include
`g1_bringup/launch/sim.launch.py` and this package cannot depend on `g1_bringup` without a cycle.

## Language

C++17: this is a 500 Hz control-rate loop with 50 Hz inference on it. No Python here at all — the
launch files and integration suites that drive it live in `g1_bringup`.
