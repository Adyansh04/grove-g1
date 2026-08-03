# g1_bringup

Sim bring-up for the G1 stack. Launches `unitree_mujoco` alongside the `ros2_control` stack
(`g1_description` + `g1_hardware_interface`) and `g1_locomotion`'s bridge, plus a sim-only node
that stands in for the robot's onboard motion service.

`ament_cmake`, C++17 node with Python launch files and integration tests.

## Launch files and nodes

| File | Purpose |
|---|---|
| `launch/sim.launch.py` | Main entry point. Checks the DDS environment, then starts `unitree_mujoco`, `motion_service_sim`, `control.launch.py` and `loco.launch.py`. Args: `headless` (default `true`), `sensors` (default `true`), `pin_pelvis` (default `false`), `sim_start_delay_s` (default `2.0`). |
| `launch/control.launch.py` | `robot_state_publisher`, `ros2_control_node` and spawners. No sim, no bridge, so it carries over to hardware unchanged. |
| `launch/loco.launch.py` | Starts `g1_loco_bridge` and drives it configure to active off its own lifecycle events. |
| `launch/activate_arm.launch.py` | Runs `scripts/activate_arm`, the ordered acquire step. |
| `launch/deactivate_arm.launch.py` | Runs `scripts/deactivate_arm`, the ordered release step. |
| `motion_service_sim` | SIM-ONLY node. See below. |

### Topics

`g1_hardware_interface`'s README covers `/lowstate` and `/arm_sdk`. This package adds:

| Topic | Direction | Type | QoS |
|---|---|---|---|
| `/lowcmd` | out | `unitree_hg/msg/LowCmd` | best-effort, keep-last(1), volatile |
| `/sportmodestate` | in | `unitree_go/msg/SportModeState` | best-effort, volatile |
| `/api/sport/request` | in | `unitree_api/msg/Request` | `QoS(10)` reliable, volatile |
| `/api/sport/response` | out | `unitree_api/msg/Response` | `QoS(1)` reliable, volatile |
| `/joint_states` | out | `sensor_msgs/msg/JointState` | default, ~200 Hz |
| `/robot_description` | out | `std_msgs/msg/String` | transient-local |
| `/livox/lidar` | out | `sensor_msgs/msg/PointCloud2` | sensor data, `sensors:=true` only |
| `/g1_sensor_relay/sensor_pose` | out | `geometry_msgs/msg/PoseStamped` | sensor data, diagnostics |
| `/tf` (`odom` -> `pelvis`) | out | `tf2_msgs/msg/TFMessage` | `sensors:=true` only |

## Running

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

## motion_service_sim: SIM-ONLY

**Never launch this near real hardware.** On the real G1 the onboard motion service owns `/lowcmd`
exclusively and provides the weight-blended `/arm_sdk` interface. `unitree_mujoco` emulates only the
low-level device, so nothing services `/arm_sdk` and nothing commands the legs. This node fills both
gaps, sim-side only, and is launched exclusively by `sim.launch.py`.

It is a plain node rather than lifecycle-managed: it emulates an always-on vendor service that has
no activate/deactivate concept of its own.

### Arm path

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

### LocoClient wire responder

The same node answers `/api/sport/request`, mirroring the single-service reality on hardware. It
tracks an FSM state and applies the acceptance rules in `include/g1_bringup/loco_fsm.hpp`. Those
rules are load-bearing: an accepted `SET_VELOCITY` latches the command the walking policy consumes,
so this path drives motors 0-14. It never touches the arm slots and never publishes `/lowcmd`
itself.

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

## Walking policy: SIM-ONLY

`motion_service_sim` runs an RL walking policy that owns motors 0-14 (legs and waist) and balances
the robot with no pelvis weld. `/arm_sdk` keeps motors 15-28, so the two never contend: they are
disjoint slices of one `/lowcmd` message. The policy's own arm outputs are discarded.

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

Also measured, and relevant to future Nav2 work:

- About 0.22 to 0.25 m/s of uncommanded lateral drift while walking forward, so straight-line
  walking curves.
- Turning collapses forward speed: `vx=0.6` with yaw commanded measures about -0.11 m/s.
- Balance is solid. The robot survives a 500 N, 50 ms lateral impulse while standing with about
  2.6 degrees of peak tilt.

Together these make it a teleop-grade locomotion source, not a planner-grade one.

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

## Sensors on the converged track: SIM-ONLY

`sensors:=true` (the default) stages a room with known geometry, runs a LiDAR sweep **inside** the
patched `unitree_mujoco`, and starts `g1_sensor_relay` to publish it as `PointCloud2` on
`/livox/lidar`. `odom -> pelvis` comes from `g1_state_estimation` reading `/sportmodestate`.

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
azimuth/elevation grid), intensity, per-point timestamps, noise, dropout and motion distortion. The
depth camera is **not** on this track yet, see the milestone notes.

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

`config/controllers.yaml`: `controller_manager` at 200 Hz. `G1ArmSdkSystem` starts inactive.
`joint_state_broadcaster` is spawned active, `arm_trajectory_controller` inactive, covering the 14
arm joints with relaxed sim tolerances. Re-tighten those against real hardware dynamics.

Hand joints stay inert. `g1_description`'s URDF includes the DEX3 joints for kinematic structure,
but they get no `ros2_control` interfaces and `unitree_mujoco`'s MJCF has no hand joints at all.

## Tests

| Test | Kind | Covers |
|---|---|---|
| `test_blend_math` | gmock | Blend-weight decay and resume, q/kp/kd blend. |
| `test_assemble_sim_low_cmd` | gmock | `/lowcmd` assembly: lower-body slots and gains, arm blend, weight-slot echo. |
| `test_loco_fsm` | gmock | FSM legality: every legal and illegal edge, `SET_VELOCITY`'s Start-only gate. |
| `test_walk_policy` | gmock | Policy contract (joint order, observation layout, un-normalised input, action mapping, dead-man) and the leg-authority fallback. |
| `test_walk_policy_session` | gmock | ONNX Runtime: shape contract, external-weight resolution, determinism, inference budget. |
| `test_sim_bringup` | launch | Bring-up topics, rates, controller and component states (welded). |
| `test_arm_command` | launch | Ordered activation, weight ramp, closed-loop trajectory, slew clamp, rogue-publisher guard (welded). |
| `test_loco` | launch | LocoClient protocol end to end over DDS (welded). |
| `test_walk_stand` | launch | The policy stands the robot up unwelded and holds it. Entry transient bounded, single `/lowcmd` writer. |
| `test_walk_teleop` | launch | The real authority path: `7301` before `Start`, dead-man, Damp release, randomized and whiplash sequences. |
| `test_walk_and_arm` | launch | Acceptance: walking under `cmd_vel` while an arm trajectory converges, one session. |
| `sim_settle_gap` | ctest | A 5 s sleep holding the sim resource lock so each DDS graph drains. |
| `clang_format_check_g1_bringup` | ctest | C++ formatting. |
| `ruff_check_g1_bringup` | ctest | Python lint and import order. |

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

The bridge node is C++17, a control-rate loop. Launch files, `launch_testing` suites, and the
`activate_arm`/`deactivate_arm` scripts are Python because ROS 2 provides no C++ path for launch or
`launch_testing`, and the scripts are one-shot sequencing tools rather than control loops.
