# g1_bringup

Sim bring-up for the G1 arm bridge milestone: launches `unitree_mujoco` alongside the
`ros2_control` stack (`g1_description` + `g1_hardware_interface`), plus a sim-only node that
stands in for the onboard motion service so the simulated robot doesn't collapse. `ament_cmake`,
C++17 node + Python launch files and integration tests.

## Nodes and launch files

| File | What it does |
|---|---|
| `launch/sim.launch.py` | The main entry point. Env fail-fast, then `unitree_mujoco` + `motion_service_sim` + `control.launch.py` + `loco.launch.py`. Args: `headless` (default `true`), `pin_pelvis` (default `true`, see "Pelvis pin"), `sim_start_delay_s` (default `2.0`). |
| `launch/control.launch.py` | Composition-pure: `robot_state_publisher` + `ros2_control_node` + spawners. No sim, no bridge -- carries over unchanged to hardware bring-up. |
| `launch/loco.launch.py` | Brings up `g1_locomotion`'s `g1_loco_bridge` and drives it configure -> active automatically (`RegisterEventHandler`/`OnStateTransition` chained off the node's own lifecycle events, not a timing guess). |
| `launch/activate_arm.launch.py` | Runs `scripts/activate_arm`: the explicit, ordered acquire step. |
| `launch/deactivate_arm.launch.py` | Runs `scripts/deactivate_arm`: the explicit, ordered release step. |
| `motion_service_sim` (executable) | SIM-ONLY node, see below. |

### Topics (beyond what `g1_hardware_interface`'s README already documents for `/lowstate`/`/arm_sdk`)

| Topic | Direction | Type | QoS | Published/consumed by |
|---|---|---|---|---|
| `/lowcmd` | out | `unitree_hg/msg/LowCmd` | best-effort, keep-last(1), volatile | `motion_service_sim` -> `unitree_mujoco` |
| `/api/sport/request` | in | `unitree_api/msg/Request` | `rclcpp::QoS(1)`, reliable, volatile | `g1_locomotion`'s bridge -> `motion_service_sim` (LocoClient wire responder, see below) |
| `/api/sport/response` | out | `unitree_api/msg/Response` | `rclcpp::QoS(1)`, reliable, volatile | `motion_service_sim` -> `g1_locomotion`'s bridge |
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

### LocoClient wire responder -- protocol-only, the other SIM-ONLY role of this same node

On the real robot, one onboard motion service owns `/lowcmd` **and** answers the `LocoClient` wire
protocol (`/api/sport/request`/`/api/sport/response`) -- `unitree_mujoco` emulates neither.
Mirroring that single-service reality, `motion_service_sim` is also this stack's `/api/sport/*`
responder, rather than a second node. This half of the node is **protocol-only**: it tracks an FSM
state and applies the same `SET_FSM_ID`/`SET_VELOCITY` acceptance rules a real onboard controller
would (see `include/g1_bringup/loco_fsm.hpp`). Those rules are now load-bearing rather than
advisory: an **accepted** `SET_VELOCITY` latches the command the walking policy consumes, so this
path does drive motors 0-14. It still never touches the arm slots, never publishes anything itself,
and outside FSM `Start` nothing is latched at all -- the legality table is the authority gate. See
`g1_locomotion`'s README for the bridge this responder talks to.

- Subscribes `/sportmodestate` (`unitree_go/msg/SportModeState`, best-effort, volatile) purely for
  the base linear velocity the walking policy observes -- `/lowstate` carries no such field, which
  is why the policy is structurally sim-only.
- Subscribes `/api/sport/request` (`unitree_api/msg/Request`, `rclcpp::QoS(10)` reliable, volatile)
  and publishes `/api/sport/response` (`unitree_api/msg/Response`, `rclcpp::QoS(1)` reliable,
  volatile) -- **RELIABILITY/DURABILITY are vendor-matched, do not deviate**, the same rule
  `g1_locomotion`'s bridge documents for its own side of this exchange. HISTORY depth is not an
  RxO-matched policy, so it's picked per side: this request reader goes deeper than the response
  publisher, so two requests landing in the same DDS write batch can't overwrite each other in a
  depth-1 KEEP_LAST cache before this callback drains them.
- **Echoes `header.identity` (both `id` and `api_id`) back on every response**, regardless of
  outcome. `g1_locomotion`'s `LocoRequestCorrelator` matches responses purely on
  `header.identity.id`, though -- the wire contract carries `api_id` alongside it in the same
  struct, but nothing on either side filters or validates it today, so a colliding `id` from
  another client on the same DDS domain would not be caught.
- FSM state starts at `Damp(1)` (matching the real robot's own boot state) and only changes on a
  successful `SET_FSM_ID`.

| API id | Name | Behavior |
|---|---|---|
| `7001` | `GET_FSM_ID` | `data = {"data": <fsm_id>}`, code `0`. |
| `7101` | `SET_FSM_ID` | Parses `{"data": <fsm_id>}`, applies the legality table below, code `0` or `7302`. |
| `7105` | `SET_VELOCITY` | Parses `{"velocity": [...], "duration": d}` (the values themselves are never read -- this responder never drives a leg); code `7301` unless the FSM is currently `Start`, else `0`. |
| `7106` | `SET_ARM_TASK` | **Always rejected, `-2`.** Deliberately unsupported: `WaveHand`/`ShakeHand`-style moves hand arm authority to the onboard controller, fighting this stack's `rt/arm_sdk` blend weight -- see `g1_locomotion`'s README, where the api id isn't even defined for the same reason. Rejecting it here (rather than a silent no-op `0`) makes the omission a tested fact this milestone, not an unverified assumption. |
| anything else, or malformed JSON | -- | `-2` (`UT_ROBOT_TASK_UNKNOWN_ERROR`). |

#### FSM legality table (`include/g1_bringup/loco_fsm.hpp`)

```
Damp(1)    -> StandUp(4)
StandUp(4) -> Start(500)
Start(500) -> StandUp(4)
Start(500) -> Damp(1)
StandUp(4) -> Damp(1)
```

Every other edge -- into `Squat(2)`/`Sit(3)`/`ZeroTorque(0)`, a same-state no-op, or any pair not
listed above -- is rejected `7302`. The vendor wire contract has no status code specific to
"illegal transition" (only `7301` "LocoState not available" and `7302` "invalid fsm id"); reusing
`7302` here is this responder's own approximation, not a verified vendor behavior, and stays a
hardware re-validation item.

## Walking policy -- SIM-ONLY, and the reason the pelvis weld is gone

`motion_service_sim` runs an RL walking policy that owns **motors 0-14** (legs 0-11 + waist 12-14)
and balances the robot without any weld. `/arm_sdk` keeps **motors 15-28**, so arms and legs never
contend: they are disjoint slices of the one `/lowcmd` message this node already published. The
policy's own arm outputs are discarded.

### Provenance

`policy/walker.onnx` (+ `walker.onnx.data`, the external weights) is taken from
**[luckyrobots/g1-manipulation-challenge](https://github.com/luckyrobots/g1-manipulation-challenge)**,
which describes it as trained via RL in Isaac Lab. That repository publishes **no licence file and
no attribution for the policy's own origin**, so its redistribution terms are undeclared -- recorded
here rather than left implicit. The MJCF, meshes and scene from that repository are **not** vendored
and are not needed (see below).

### Why it works when the previous attempt did not

The earlier attempt used `unitree_rl_gym`'s checkpoint, which was trained and sim-validated on a
**legs-only 12-DoF** model and cannot carry a 29-DoF upper body. This policy is trained for the full
29-DoF robot. Note what did **not** turn out to matter: **armature matching**. Ablating armature,
frictionloss and damping against `unitree_mujoco`'s own `g1_29dof.xml` showed the model *as shipped*
(uniform `armature=0.01`) performs identically to per-joint values across standing, walking, turning
and a 500 N perturbation. **No MJCF edits and no mesh vendoring were required.**

### Contract

- **Observation, 99 elements, fed RAW**: `base_lin_vel(3)` + `base_ang_vel(3)` +
  `projected_gravity(3)` + `joint_pos(29, relative to the default posture)` + `joint_vel(29)` +
  `last_action(29)` + `command(3)`. The exported graph starts with `Sub(mean)` then `Div(std)`, so
  normalisation is **already inside the model** -- applying the `obs_mean`/`obs_std` arrays that ship
  alongside it would apply it twice.
- **Action**: `target = default_joint_pos + action * action_scales`, per joint.
- **Joint order**: identical to the Unitree DDS motor order (0-28), so `motor_state[i]` and
  `motor_cmd[i]` map straight through. Asserted at startup and in `test_walk_policy`, never assumed.
- **Base linear velocity comes from `/sportmodestate`**, not `/lowstate` (which carries no such
  field). That is why this policy is **structurally sim-only**: on the real robot that topic is
  served by the onboard motion service, which is off whenever this stack owns the low-level channel.
- Inference runs at 50 Hz on its own timer, decimated from the 500 Hz `/lowcmd` publish, ~0.02 ms per
  call.

### Measured behaviour -- read this before commanding velocities

**There is a hard gait-initiation deadband with no hysteresis.** Below it the robot stands perfectly
still rather than stepping, and kicking above it then dropping back stops the gait outright:

| axis | no motion at or below | steps from | measured output |
|---|---|---|---|
| `vx` | 0.35 m/s | **0.40 m/s** | 0.5 -> 0.35, 0.6 -> 0.47, 1.0 -> 0.93 m/s |
| `vy` | 0.30 m/s | **0.50 m/s** | 0.5 -> 0.44, 1.0 -> 0.93 m/s |
| `vyaw` (in place) | 0.60 rad/s | **1.50 rad/s** | 1.0 -> 0.21, 1.5 -> 1.08 rad/s |

Commands below threshold are passed through **unchanged** and logged, never scaled up -- turning a
small command into a large motion is exactly the habit this stack's control-mode rules exist to
prevent, and it would be dangerous on hardware.

Also measured, and relevant to any future Nav2 work:

- **~0.22-0.25 m/s uncommanded lateral drift** while walking forward, so straight-line walking curves.
- **Turning collapses forward speed**: `vx=0.6` with yaw commanded measures ~-0.11 m/s.
- Balance itself is solid: survives a 500 N / 50 ms lateral impulse standing, ~2.6 deg peak tilt.

Together these make this a **teleop-grade** locomotion source, not a planner-grade one.

### Manual driving with `teleop_twist_keyboard`

Drives the same `~/cmd_vel` seam Nav2 will use, so this exercises the production interface:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/g1_loco_bridge/cmd_vel -p speed:=0.6 -p turn:=1.57
```

**Both overrides are load-bearing.** The package defaults are `speed=0.5` (only just clears the
0.40 m/s threshold, and measures 0.35 m/s) and `turn=1.0` -- and **1.0 rad/s does not clear the
1.50 rad/s in-place yaw threshold at all**, measuring 0.21 rad/s, so with the default the robot
simply will not turn on the spot. `1.57` is the bridge's own `max_velocity` yaw ceiling. Note also
that `q`/`z` scale both speeds by +/-10 %, so a few `z` presses drop you back under threshold and
the gait stops.

You must reach `Start` before any of this moves the robot:

```bash
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 4}"
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 500}"
```

### Authority

No new authority mechanism was built. The Milestone-2 FSM legality table in `loco_fsm.cpp` **is**
the gate: `SET_VELOCITY` is latched only when `checkVelocityAllowed()` already accepted it, so
outside `Start` it is rejected with `7301` and nothing reaches the policy. A successful transition
away from `Start` clears the latch, and the latch carries the request's own `duration` as its
dead-man, so a silent or dead bridge stops the robot within a second.

The policy itself runs **continuously from startup regardless of FSM state** -- leg authority is
gated on policy *freshness*, not on the FSM -- so releasing locomotion authority stops the walking
without dropping the robot. If inference ever goes stale the lower-body target **freezes at the last
policy output** rather than reverting to the captured spawn pose, which would be a large step away
from the stance the robot is actually in.

## Pelvis pin -- SIM-ONLY, now a debugging aid only

> **Superseded by the walking policy above.** `pin_pelvis` now defaults **false** and, when set
> true, also **disables the walking policy** -- the weld and the policy are the two possible owners
> of the legs and are never both active. Keep it for exercising the arm bridge with nothing else
> driving the legs. The rest of this section explains why the weld existed and why a stiff-hold
> could never replace it.

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

## Test inventory

| Test | Kind | What it pins |
|---|---|---|
| `test_blend_math` | gmock | Blend-weight decay/resume and the q/kp/kd blend. |
| `test_assemble_sim_low_cmd` | gmock | `/lowcmd` assembly: lower-body slots+gains, arm blend, weight-slot echo. |
| `test_loco_fsm` | gmock | LocoClient FSM legality: every legal/illegal edge, `SET_VELOCITY`'s Start-only gate. |
| `test_walk_policy` | gmock | Policy wire contract (DDS joint order, 99-element observation, raw/un-normalised, action->target, velocity dead-man) **and the leg-authority fallback**: stale ticks freeze at the last policy output, and an inference that throws yields no action instead of killing the process. |
| `test_walk_policy_session` | gmock | ONNX Runtime: 99->29 shape contract, external-weight resolution, determinism, inference inside the 50 Hz budget. |
| `test_sim_bringup` | launch | Bring-up: topics, rates, controller/component states (welded). |
| `test_arm_command` | launch | Ordered activation, weight ramp, closed-loop trajectory, slew clamp, rogue-publisher guard (welded). |
| `test_loco` | launch | LocoClient protocol end to end over real DDS (welded). |
| `test_walk_stand` | launch | The policy stands the robot up **unwelded** and holds it; entry transient bounded; single `/lowcmd` writer. |
| `test_walk_teleop` | launch | Driving through the real LocoClient authority path: `7301` before `Start`, dead-man, Damp release, randomized and whiplash command sequences. |
| `test_walk_and_arm` | launch | The acceptance bar: walking under `cmd_vel` while an arm trajectory converges, in one session. |

### Sim-suite load sensitivity -- read before trusting a red full-suite run

Every `launch` suite above starts a real `unitree_mujoco`. The sim syncs its clock to CPU time and
re-syncs when it falls behind, while the walking policy is paced on a wall timer -- so on a loaded
machine the two drift apart and the robot can topple. **This is a harness limitation, not a policy
defect.** Current evidence of correctness: each suite passes run in isolation, and walking, teleop
and arm motion are hand-verified in the GUI.

Mitigations here are deliberately lightweight -- `RESOURCE_LOCK` serialises the suites, a settle gap
lets each DDS graph drain, and `COST` ordering puts `test_walk_stand` last so the milestone's core
balance claim runs on the quietest machine. Real isolation belongs with the deferred CI work. If a
sim suite fails inside a full sweep, re-run it alone before treating it as a regression:

```bash
colcon test --packages-select g1_bringup --ctest-args -R test_walk_stand
```

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
effective weight -- same no-sim/no-DDS treatment as `test_blend_math`. `test_loco_fsm` covers the
LocoClient FSM legality table (`include/g1_bringup/loco_fsm.hpp`): every legal `SET_FSM_ID` edge,
every illegal one, and `SET_VELOCITY`'s Start-only gate -- same no-sim/no-DDS treatment again.
`test/test_sim_bringup.launch.py`, `test/test_arm_command.launch.py`, and `test/test_loco.launch.py`
are headless `launch_testing` integration suites against the real sim (see their own docstrings for
what each asserts -- the last one drives `g1_locomotion`'s bridge itself through this responder end
to end). Plus `clang-format` against the repo
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
