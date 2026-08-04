# g1_locomotion

The G1's `LocoClient` bridge. Translates `geometry_msgs/Twist` and `g1_msgs/SetLocoMode` goals into
Unitree's `LocoClient` wire contract, two plain topics (`/api/sport/request` and
`/api/sport/response`) carrying JSON-encoded `unitree_api::msg::Request`/`Response`, without ever
blocking an executor callback on the DDS round trip.

`ament_cmake`, C++17 throughout.

Kept out of `g1_hardware_interface` deliberately: that package is the pluginlib `.so` loaded into
`ros2_control_node`, and folding `unitree_api`, `nlohmann_json` and `rclcpp_action` in there would
push those dependencies onto every consumer of the arm hardware interface.

## Running

Normally this comes up as part of the sim stack rather than standalone:

```bash
ros2 launch g1_bringup sim.launch.py
```

That includes this package's `loco.launch.py`, which starts `g1_loco_bridge` and drives it
configure to active off the node's own lifecycle events. Once up:

```bash
# Drive the FSM to Start. StandUp is a required intermediate hop.
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 4}"
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 500}"

# Watch status. Transient-local, so this prints the latest immediately.
ros2 topic echo /g1_loco_bridge/status

# Drive forward once authority is HELD.
ros2 topic pub -r 10 /g1_loco_bridge/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"

# Release authority.
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 1}"
```

In sim the responder is `g1_motion_service_sim`'s `motion_service_sim`, and an accepted
`SET_VELOCITY` feeds
the walking policy that drives motors 0-14. See `g1_bringup`'s README for the dispatch table and
for the policy's measured velocity thresholds, which matter when choosing a `cmd_vel` value.

### Standalone

For developing this package in isolation:

```bash
ros2 run g1_locomotion g1_loco_bridge --ros-args \
  --params-file install/g1_locomotion/share/g1_locomotion/config/g1_loco_bridge.yaml

ros2 lifecycle set /g1_loco_bridge configure
ros2 lifecycle set /g1_loco_bridge activate
```

Not while `sim.launch.py` is running. Two bridges on the same names in one domain trips this
bridge's single-writer guard on `/api/sport/request` and force-releases velocity authority. With no
responder running at all, every request times out after `request_timeout_s`, so `~/status.authority`
shows `ACQUIRING` then falls back to `RELEASED` with `last_error_code` set to `-1`. Expected in that
configuration, not a bug.

## Interfaces

| Interface | Direction | Type | QoS |
|---|---|---|---|
| `~/cmd_vel` | in | `geometry_msgs/Twist` | reliable, keep-last(1), volatile |
| `~/status` | out | `g1_msgs/LocoStatus` | reliable, transient-local, keep-last(1) |
| `~/set_mode` | action | `g1_msgs/SetLocoMode` | n/a |
| `/api/sport/request` | out | `unitree_api/Request` | `QoS(1)` reliable, volatile |
| `/api/sport/response` | in | `unitree_api/Response` | `QoS(10)` reliable, volatile |

**Reliability and durability on `/api/sport/*` are vendor-matched. Do not deviate.** They are the
exact policies `BaseClient` uses, and hardware endpoint compatibility depends on them matching.
History depth is not RxO-matched, so the response reader goes deeper than the vendor's depth-1
default: a response landing in the same DDS batch as another read must not overwrite an unread
result. Measured about 20% `SET_VELOCITY` loss before this and the heartbeat phase offset below.

`~/cmd_vel` is node-relative, not bare `/cmd_vel`. Arbitrating multiple command sources (Nav2,
teleop, a behavior tree) belongs to a future orchestration layer. It maps `linear.x` to `vx`,
`linear.y` to `vy`, `angular.z` to `vyaw`, multiplied by `axis_sign` then clamped to
`max_velocity`.

`~/status` publishes on change plus a 1 Hz heartbeat. Transient-local so a monitor attaching after
activation sees current state immediately.

## Configuration (`config/g1_loco_bridge.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `request_timeout_s` | 5.0 | Correlator sweep timeout. Matches `BaseClient`'s own blocking timeout. |
| `max_pending` | 16 | Bound on requests tracked at once. |
| `velocity_reissue_hz` | 5.0 | `SetVelocity` re-issue rate. Must be `> 1.0`. |
| `cmd_vel_timeout_ms` | 500.0 | `~/cmd_vel` age beyond this is stale. |
| `failure_streak_limit` | 3 | Consecutive non-zero codes before releasing authority. |
| `max_velocity` | `[0.8, 0.5, 1.57]` | `[vx, vy, vyaw]` clamp. See below. |
| `axis_sign` | `[1.0, 1.0, 1.0]` | Sign flip applied before clamping. |

`max_velocity` is a **sim bring-up value**, chosen to clear the sim walking policy's measured
gait-initiation thresholds `[0.40, 0.50, 1.50]` with headroom. The earlier `[0.3, 0.2, 0.5]`
clamped every axis below its own threshold, so the robot could never step. The conservative
slow-by-default ceiling hardware wants is a separate decision to make at hardware bring-up against
the real controller's limits, not back-derived from these numbers.

`axis_sign` is the calibration knob hardware bring-up will need: the vendor's frame convention
relative to `Twist` is undocumented and not derivable in sim.

## Design notes

### Why not the vendored `BaseClient`

`unitree_ros2`'s `BaseClient::Call()` is unusable from inside an executor callback:

1. It blocks until its response arrives or 5 s elapse, so only one request can be in flight.
2. It creates a fresh `/api/sport/response` subscription per call and drops it on return. At 5 Hz
   that is DDS endpoint churn five times a second.
3. Its subscription callback captures a stack-local `std::promise` by reference and calls
   `set_value()` with no coordination against the caller's timeout, so a late response can write to
   a promise that has gone out of scope.
4. It deadlocks a single-threaded executor outright: the per-call subscription can never be
   serviced while the callback holds the executor. Unitree's own example avoids this only by
   running `LocoClient` on a separate thread.

`LocoRequestCorrelator` replaces it with a map from request id to pending callback, serviced by the
node's own subscription and timers. No blocking, no per-call churn, any number of requests in
flight.

### Thread ownership

One thread ever touches this node, and the guarantee is two-part. Every callback source this class
creates goes in one named `MutuallyExclusive` group, **and** `main()` spins on a
`SingleThreadedExecutor`. The group alone is not enough: `LifecycleNode`'s own transition and
parameter services are created by the base class, land in Humble's default group, and cannot be
redirected, so `on_configure`/`on_cleanup`/`on_deactivate` are kept out from under a running
callback only by the executor being single-threaded.

No locks and no atomics anywhere in `G1LocoBridge`, `LocoRequestCorrelator` or `VelocityGate`.
Contrast `g1_hardware_interface`'s `G1ArmSdkSystem`, which genuinely needs atomics because its RT
thread and its executor thread are two independently scheduled threads. Here every correlator
`send()` is a fire-and-forget publish and every outcome arrives back through the same executor as
an ordinary callback.

A `MultiThreadedExecutor` migration would need the base class's lifecycle services serialised
against the named group by some other means.

#### `g1_loco_authority` deviates, and only it

**`g1_loco_authority` runs a `MultiThreadedExecutor` with two threads.** It has to: its
`on_activate` blocks on a `SetLocoMode` result, and that result arrives as a callback on the same
node, so a single-threaded executor would deadlock on the transition that is waiting for it. Its
action client and status subscription sit in one `Reentrant` group; the base class's lifecycle
services stay in the default `MutuallyExclusive` one.

The deviation is bounded to **one atomic**, `latest_authority_` — written by the status callback,
read by the transition thread. There is no other shared state and no lock anywhere in the node.

**`G1LocoBridge`, `LocoRequestCorrelator` and `VelocityGate` are unaffected.** They keep the
single-threaded guarantee above, and the no-locks-no-atomics property with it. Nothing in this
paragraph loosens anything about the bridge.

The alternative — a non-blocking `on_activate` driven off a timer — avoids the executor entirely,
but then the lifecycle manager reports the node active while the robot is not yet walk-capable,
which destroys the only thing the bracket is for.

### Pure, ROS-free classes

The safety-bearing logic lives in plain classes with no node, executor or timer, so it is unit
tested with no DDS. Same pattern as `g1_hardware_interface`'s `ArmRampEngine`.

**`loco_payloads`** builds the exact JSON the wire contract expects via `nlohmann::json`, the same
library `BaseClient` uses, so output is byte-for-byte what the vendor's client would send.
`buildSetVelocityPayload` always uses `kVelocityDurationS` (1.0 s): there is no parameter for a
different value, so the vendor's 864000 s "continuous move" constant can never appear.
`duration` is a dead-man switch, and latching a long value defeats it. This bridge re-issues
continuously instead, so every request including the stop can carry the same short duration.

`7106 SET_ARM_TASK` has no builder, deliberately. `WaveHand`/`ShakeHand` make the onboard
controller move the arms, fighting whatever blend weight `rt/arm_sdk` currently holds, and no
arbitration rule between the two paths exists. Not even the numeric api id is defined here.

**`loco_request_correlator`** keys pending requests on `header.identity.id`. Ids are seeded once
from `steady_clock` at construction then post-incremented, so two sends in the same tick cannot
collide. `onResponse` matches and erases; an unmatched id is dropped and counted rather than
treated as an error, since a response can legitimately arrive just after its entry timed out.
`sweep` expires entries past `request_timeout_s` with code `-1`, two-phase (collect, erase, then
invoke) so a callback that calls `send()` cannot invalidate the iteration it runs inside.
`supersede` drops an entry with no callback, for when a fresher command has already replaced it.

**`velocity_gate`** is the authority state machine:

```
kReleased --beginAcquire()--> kAcquiring --onAcquireResult(true)--> kHeld
kAcquiring --onAcquireResult(false)--> kReleased
kHeld --beginRelease()--> kReleasing --onReleaseResult()--> kReleased   (always)
kHeld --(failure_streak_limit consecutive non-zero codes)--> kReleased
any state --forceRelease()--> kReleased
```

Every path out of `kAcquiring` and `kReleasing` lands in a defined state, satisfying the release
cleanly on success and failure rule.

`cmd_vel` is honoured only in `kHeld`. A stale or exactly-zero command sends exactly one
`SetVelocity(0,0,0)` then nothing further until a fresh non-zero command arrives. A live non-zero
command is re-issued every tick, which is what makes the short `duration` safe.

### Authority handling

A `START` goal calls `beginAcquire()` before publishing, and only once the correlator confirms a
request is in flight, so every `kAcquiring` is guaranteed a future callback. Any other accepted
goal calls `beginRelease()`, but only from `kHeld`: release keys on leaving `Start`, not on the
literal `DAMP` id, because `Start -> StandUp` is a legal edge and a `STAND_UP` goal from `kHeld`
means the robot is about to leave the state velocity authority depends on.

The one authority-promoting callback self-gates on `PRIMARY_STATE_ACTIVE`. The primary defence is
`on_deactivate()` itself, which supersedes the correlator entry for any in-flight goal and aborts
it before returning, so a late reply can never reach the result handler. The self-gate is a
backstop.

`fsm_id` in `~/status` is not inferred from goal outcomes alone: a `GET_FSM_ID` poll rides the 1 Hz
heartbeat, keeping it confirmed by the robot. That timer's first tick is phase-offset half a
re-issue period, because two wall timers with harmonically related periods would otherwise fire
together every second and publish `SET_VELOCITY` and `GET_FSM_ID` back to back on a channel whose
contract assumes one call in flight.

**Single-writer guard:** the same timer checks `count_publishers("/api/sport/request")`. A count
above 1 is always logged, and if this bridge holds authority it also sends one defensive
`SetVelocity(0,0,0)` and force-releases. Advisory only; real cross-process arbitration is a future
behavior-tree concern.

**Shutdown has no synchronous ramp,** unlike the arm bridge, because this node never actuates
directly. If re-issuing stops, including by this process dying, the onboard controller's own 1 s
dead-man stops the robot. That emergent property is why re-issuing short rather than latching
864000 s is non-negotiable.

**Teardown never wedges a goal.** `resetEntities()` aborts any in-flight goal with a terminal
result and clears the correlator before destroying the action server and timers. Ordering matters:
once those are gone nothing could resolve the goal, so skipping it would hang the client forever.
Most easily reached via `shutdown()` from `PRIMARY_STATE_ACTIVE`, which Humble allows directly,
bypassing `on_deactivate()`.

## Tests

```bash
colcon build --symlink-install --packages-select g1_locomotion
colcon test --packages-select g1_locomotion
colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `test_loco_payloads` | Exact JSON for `7101`/`7105`, `duration` always 1.0, response parsing including malformed input. |
| `test_loco_correlator` | Overlapping requests, out-of-order responses, sweep timeout, an orphaned response arriving after its entry was swept, `supersede()`, reentrant `send()` from a sweep callback, the `max_pending` bound. |
| `test_velocity_gate` | Re-issue above 1 Hz, the stale/zero single-stop policy, failure-streak release, `cmd_vel` ignored outside `kHeld`, every terminal path landing defined. |
| `test_loco_bridge_node` | The node itself, in-process against a fake responder on an isolated domain: a late reply after `on_deactivate` must not revive authority, a slow round trip must still advance the failure streak, `STAND_UP` from `kHeld` must release, and a goal in flight at teardown must terminate. |

Plus `clang-format`, `ament_lint_cmake` and `xmllint`.

`g1_bringup/test/test_loco.launch.py` validates the wiring between this bridge and a real
`/api/sport/*` responder end to end over DDS. It lives there because it needs the sim stack's
launch files; run it with `colcon test --packages-select g1_bringup`.

## `g1_gait_shaper`

Reduces a planner's `Twist` onto the three motions this gait can actually produce: stop, drive
straight, turn in place. Subtractive only — every output is the input unchanged, clamped smaller,
or zero, never larger. That invariant is the safety claim, and it is asserted as a swept property
rather than spot-checked.

Yaw is tested first, so a command carrying both axes becomes a pure turn: the measured combined
response is the worst case (a commanded (0.50, 0, 0.50) produced 0.30 m/s of uncommanded lateral).
Forward compares **signed**, so any negative velocity becomes zero at any magnitude — which is
independently why a reverse recovery behaviour cannot lurch the robot backwards.

Thresholds live in `config/g1_gait_shaper.yaml` and carry the same sim-only banner
`max_velocity` does: the deadband is a property of the sim walking policy, not the real G1's
onboard MPC. Hardware bring-up does not launch this node.

## `g1_loco_authority`

The lifecycle bracket around LocoClient velocity authority: active means the robot is
walk-capable, inactive means authority has been handed back. Exists because a planner publishes
`cmd_vel` and nothing else — it has no way to send the `SetLocoMode` goals the bridge requires.

Acquisition is deliberately **not** automatic on first `cmd_vel`. That would be implicit
acquisition, and a stray publisher would stand the robot up and walk it.

`on_activate` retries while the stack is still coming up, because everything launches at once and
the bridge can be answering before the simulator has stepped any physics. The retry is selective,
not blanket: `7301` (controller not in a state that can service the call) and a sweep timeout are
retried; `7302` (the controller's own transition table refusing) and an unknown error are not,
because repeating those would hide a real fault behind a timeout. Bounded by `acquire_timeout_s`
overall.

Release uses `StandUp`, never `Damp` — `Damp` drops the robot. It fires on deactivate, on
shutdown (Humble allows `shutdown()` straight from active, bypassing `on_deactivate`), and from
`on_activate`'s own failure path, since returning FAILURE leaves the node inactive and
`on_deactivate` never runs.
