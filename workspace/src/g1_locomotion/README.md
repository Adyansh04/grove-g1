# g1_locomotion

The G1's `LocoClient` bridge: translates `geometry_msgs/Twist` and `g1_msgs/SetLocoMode` goals
into Unitree's `LocoClient` wire contract -- two plain topics, `/api/sport/request` and
`/api/sport/response`, carrying JSON-encoded `unitree_api::msg::Request`/`Response` -- without
ever blocking an executor callback on the DDS round trip. `ament_cmake`, C++17.

Kept out of `g1_hardware_interface` deliberately: that package *is* the pluginlib `.so` loaded
into `ros2_control_node`. Folding `unitree_api`/`nlohmann_json`/`rclcpp_action` in there would push
those dependencies onto every `ros2_control` consumer of that plugin for a concern (locomotion)
that has nothing to do with the arm hardware interface.

## Why not the vendored `BaseClient`

`unitree_ros2`'s own `BaseClient::Call()` (see `example/src/include/common/base_client.hpp`) is
unusable from inside this bridge's executor callbacks, for four separate reasons:

1. **One request in flight per call.** `Call()` blocks until its own response arrives (or 5 s
   elapses) before returning -- there is no way to have two LocoClient requests outstanding at
   once through it.
2. **A fresh DDS subscription per call.** Every `Call()` creates a brand-new subscription to
   `/api/sport/response` and lets it go out of scope when the call returns -- at
   `velocity_reissue_hz` (5 Hz default) that's a new DDS endpoint created and destroyed five times
   a second, discovery churn a long-lived bridge shouldn't need.
3. **An unsynchronized stack-local promise.** The subscription's callback captures a
   `std::promise` *by reference* and calls `set_value()` on it with no coordination against the
   calling thread's own 5 s timeout -- if the timeout and a late-arriving response race, the
   callback can still fire after the promise it's writing to has gone out of scope.
4. **It blocks.** `response_future.wait_for(5s)` inside a single-threaded executor's callback
   deadlocks outright: the per-call subscription can never be serviced while the callback holds
   the executor. Unitree's own `loco_client_example.cpp` only gets away with this by running
   `LocoClient` calls on a separate `std::thread` while `rclcpp::spin()` runs on another -- not an
   option for a node whose whole point is a documented single-thread contract (see below).

`LocoRequestCorrelator` replaces it: a plain map from request id to a pending callback, serviced
by the node's own subscription and two timers -- no blocking, no per-call DDS churn, an arbitrary
number of requests genuinely in flight at once.

## Thread-ownership contract

Exactly one thread ever touches this node, but that guarantee is two-part, not one: every
callback source *this class itself creates* is placed without exception in one explicitly created
and named `MutuallyExclusive` callback group -- the `/api/sport/response` subscription, the ~50 ms
sweep timer, the velocity re-issue timer, the ~1 Hz heartbeat/FSM-poll/rogue-guard timer,
`~/cmd_vel`, and the `~/set_mode` action server -- **and** `main()` spins that group on a single
`rclcpp::executors::SingleThreadedExecutor` (load-bearing, not a default left alone -- see
`g1_loco_bridge_main.cpp`'s own comment). The group alone does not cover everything:
`rclcpp_lifecycle::LifecycleNode`'s own transition/parameter services (`~/change_state`,
`~/get_state`, `~/set_parameters`, ...) are created by the *base class*, land in Humble's default
callback group, and cannot be redirected -- so `on_configure`/`on_cleanup`/`on_deactivate` (which
reset every publisher, subscription, and timer) are only kept out from under a concurrently
running callback by the executor being single-threaded, not by the named group by itself. No
locks, no atomics, anywhere in `G1LocoBridge`, `LocoRequestCorrelator`, or `VelocityGate` --
moving one of this class's own callbacks to a different group would still need a deliberate,
visible edit inside `on_configure()`; a genuine `MultiThreadedExecutor` migration would
additionally need the base class's lifecycle services serialised against the named group by some
other means, since the group can't do that part on its own.

Contrast this with `g1_hardware_interface`'s `G1ArmSdkSystem`, which genuinely needs
`std::atomic`: its RT `read()`/`write()` thread (`controller_manager`'s own) and its hidden
executor thread are two real, independently-scheduled threads by construction, so shared state
there truly is concurrent. Nothing here is: every correlator `send()` is a fire-and-forget
publish, and every outcome -- a matched response, a timeout, or a superseded request -- always
arrives back through this same single executor as an ordinary callback, never synchronously.

## Pure, ROS-free classes

Mirroring `g1_hardware_interface`'s `ArmRampEngine` and `g1_bringup`'s `blend_math`: the
safety/logic-bearing surface lives in plain C++ classes with no `rclcpp::Node`, executor, or
timer of their own, so it's unit-tested with no DDS and no live node.

### `loco_payloads.*`

Builds the exact JSON bodies the wire contract expects, via `nlohmann::json` -- the same library
`BaseClient` itself uses, so output is byte-for-byte what the vendor's own client would send.

- `buildSetFsmIdPayload(fsm_id)` -- `7101 SET_FSM_ID`'s `{"data":<fsm_id>}`.
- `buildSetVelocityPayload(vx, vy, vyaw)` -- `7105 SET_VELOCITY`'s
  `{"velocity":[vx,vy,vyaw],"duration":1.0}`. Always uses `kVelocityDurationS` (`1.0F`) -- there is
  no parameter to pass a different value, so the vendor's 864000 s ("continuous move") constant
  can never appear here. `duration` is a dead-man switch: latching a long value defeats it (a dead
  publisher would leave the robot walking for as long as the latch lasts). This bridge instead
  re-issues continuously at `velocity_reissue_hz`, so every request -- including the stop request
  -- can safely carry the same short duration.
- `parseFsmIdResponse(data)` -- the inverse, for `7001 GET_FSM_ID`'s `{"data":<fsm_id>}`.

`7106 SET_ARM_TASK` has no builder here, deliberately: `WaveHand`/`ShakeHand` make the *onboard*
controller move the arms, fighting whatever blend weight our `rt/arm_sdk` publisher
(`g1_hardware_interface`) currently holds. No arbitration rule between the two control paths
exists, so this bridge never sends it -- not even the numeric API id 7106 is defined in this
package, so nothing here can send it by accident.

### `loco_request_correlator.*` -- `LocoRequestCorrelator`

The async replacement for `BaseClient` described above. Owns a single pending-request map keyed
on `header.identity.id`:

- `send(api_id, parameter, now, on_done)` -- builds a `Request` with a fresh id and returns it
  ready to publish, or `nullopt` if `max_pending` (16, belt-and-braces) is already reached.
  Ids are seeded once from `steady_clock`'s epoch at construction, then post-incremented per call
  -- unique even for two sends within the same clock tick, unlike `BaseClient`'s own
  `GetSystemUptimeInNanoseconds()`, recomputed (and therefore theoretically collidable) on every
  call.
- `onResponse(msg)` -- matches `msg.header.identity.id`, invokes and erases the matching entry.
  An id with no match -- already swept by a timeout, already superseded, or simply unknown -- is
  dropped and counted (`droppedResponseCount()`), never treated as an error: a response can
  legitimately arrive just after its entry was timed out.
- `sweep(now)` -- expires entries older than `request_timeout_s` (5.0, the vendor's own
  `BaseClient` timeout), invoking their callback with `(-1, "")` (`UT_ROBOT_TASK_TIMEOUT`). Run
  from a ~50 ms timer -- frequent relative to the 5 s timeout so an expiry is noticed promptly.
  Two-phase (collect every expired callback, erase all of them, *then* invoke) so a callback that
  itself calls `send()` -- inserting into, and potentially rehashing, this same pending map --
  can't invalidate the iteration it's running inside of.
- `supersede(id)` -- drops a pending entry with no callback invocation, for exactly the case where
  a fresher command has already replaced it (the velocity re-issue timer supersedes its own
  previous in-flight request every tick, and the FSM-poll timer does the same for its own poll) --
  only the newest outcome for that *kind* of request ever matters. `onReissueTick()` also feeds a
  synthetic `(kCodeTaskTimeout, "")` into `VelocityGate::onVelocityResult()` immediately before
  superseding a velocity request that's still pending, so a round trip slower than the re-issue
  period still counts against the failure streak instead of vanishing silently.
- `clear()` -- drops every pending entry with no callback invocation, all at once. Used only by
  `resetEntities()` when the node itself is tearing down (see "Teardown never wedges a goal"
  below) -- nothing left standing afterward could service a response or a `sweep()` timeout
  anyway, so leaving entries pending would just strand their captured callbacks.

### `velocity_gate.*` -- `VelocityGate` and `LocoAuthority`

The velocity re-issue and locomotion-authority state machine, entirely decoupled from ROS/DDS:

```
kReleased --beginAcquire()--> kAcquiring --onAcquireResult(true)--> kHeld
kAcquiring --onAcquireResult(false)--> kReleased
kHeld --beginRelease()--> kReleasing --onReleaseResult()--> kReleased   (always, win or lose)
kHeld --(failure_streak_limit consecutive non-zero onVelocityResult codes)--> kReleased
any state --forceRelease()--> kReleased   (rogue-publisher guard, on_deactivate)
```

Every path out of `kAcquiring`/`kReleasing` lands in a defined state -- never stuck mid-transition
-- satisfying the "release cleanly on success *and* failure" rule for a skill that has acquired
exclusive control authority.

`cmd_vel` is honoured *only* in `kHeld`: `tick()` (driven by the node's re-issue timer, at
`velocity_reissue_hz`, default 5.0, validated `> 1.0` at `on_configure` -- fatal otherwise, since
at or below 1 Hz `duration`'s 1 s dead-man could expire between re-issues) returns `nullopt` in
every other state. Within `kHeld`:

- A stale command (`cmd_vel_timeout_ms`, default 500 ms, since the last `~/cmd_vel` sample) or an
  exact-zero `Twist` sends exactly **one** `SetVelocity(0, 0, 0)`, then nothing further until a
  fresh non-zero command arrives -- a live re-issue loop is what actually keeps commanding
  non-zero, so once it has nothing new to say, saying it again and again is just noise.
- A live, fresh, non-zero command is returned every tick for as long as `tick()` keeps being
  called -- continuous re-issue is what makes `duration`'s short window safe in the first place.
- `failure_streak_limit` (default 3) consecutive non-zero `SetVelocity` error codes release
  authority on the gate's own initiative (`kHeld -> kReleased`) and record the code
  (`lastErrorCode()`) rather than continuing to retry a channel that's clearly not working.

## `G1LocoBridge` (`rclcpp_lifecycle::LifecycleNode`)

Wires the classes above to DDS. Configure-time-only construction of every subscription, publisher,
timer, and the action server (torn down and idempotently rebuilt on repeated `on_configure`,
mirroring `G1ArmSdkSystem`'s pattern in `g1_hardware_interface`) -- `on_activate` only lets the
node *accept* `SetLocoMode` goals and honour `~/cmd_vel`; it commands nothing on its own. Humble's
`rclcpp_action::create_server` on a `LifecycleNode` needs the interface-pointer overload
(`get_node_base_interface()`, `get_node_clock_interface()`, `get_node_logging_interface()`,
`get_node_waitables_interface()`) -- the node-pointer convenience overload doesn't exist for
`LifecycleNode`. That overload has no lifecycle awareness either way, so `handle_goal` self-gates
on `get_current_state()` explicitly, the same "self-gated, not framework-gated" principle
`G1ArmSdkSystem` documents for its own `write()`.

### Topics, action, and QoS

| Interface | Direction | Type | QoS | Why |
|---|---|---|---|---|
| `~/cmd_vel` | in | `geometry_msgs/Twist` | reliable, keep-last(1), volatile | Node-relative, **not** bare `/cmd_vel` -- arbitrating multiple command sources (Nav2, teleop, a future behavior tree) is that future orchestration layer's job, not this milestone's. |
| `~/status` | out | `g1_msgs/LocoStatus` | reliable, **transient-local**, keep-last(1) | On change + 1 Hz heartbeat. Transient-local so a monitor that attaches after activation sees the current state immediately rather than waiting for the next heartbeat. |
| `~/set_mode` | action | `g1_msgs/SetLocoMode` | n/a | See `g1_msgs`'s README for why an action rather than a service. |
| `/api/sport/request` | out | `unitree_api/Request` | `rclcpp::QoS(1)`, reliable, volatile | **RELIABILITY/DURABILITY vendor-matched -- do not deviate.** The exact QoS `BaseClient`'s own publisher uses; hardware endpoint compatibility depends on matching those two policies. |
| `/api/sport/response` | in | `unitree_api/Response` | `rclcpp::QoS(10)`, reliable, volatile | Reliability/durability match `BaseClient` the same way; HISTORY depth is *not* an RxO-matched policy, so this reader goes deeper than the depth-1 vendor default -- a response landing in the same DDS write batch as another read must not overwrite an unread result in a depth-1 cache (measured ~20% `SET_VELOCITY` loss before this and the heartbeat-timer phase-offset fix below). |

`~/cmd_vel` is mapped `linear.x -> vx`, `linear.y -> vy`, `angular.z -> vyaw`, each clamped to
`max_velocity` (default `[0.8, 0.5, 1.57]` -- a **sim bring-up value**, chosen to clear the sim
walking policy's measured gait-initiation thresholds `[0.40, 0.50, 1.50]` with headroom; the earlier
`[0.3, 0.2, 0.5]` clamped every axis *below* its own threshold, so the robot could never step. The
conservative slow-by-default ceiling hardware wants is a separate decision to be made at hardware
bring-up against the real controller's limits, not back-derived from these numbers) after being multiplied by `axis_sign` (default `[1, 1, 1]` -- the vendor's frame
convention relative to `Twist` is undocumented, so this is the calibration knob hardware bring-up
will need, not something derivable in sim).

### `SetLocoMode` handling

Only `fsm_id` in `{DAMP(1), STAND_UP(4), START(500)}` is accepted; anything else is rejected in
`handle_goal` with the reason logged (a rejected goal has no result to carry a message in, so this
is a server-side log, not something the client can inspect). At most one goal is in flight at a
time -- a second one is rejected outright rather than needing to unwind the first.

A `START` goal calls `VelocityGate::beginAcquire()` before publishing its `SET_FSM_ID(500)`
request (only once `LocoRequestCorrelator::send()` confirms a request is actually in flight, so
every `kAcquiring` is guaranteed a future callback). Any *other* accepted goal -- `DAMP` or
`STAND_UP` alike -- calls `beginRelease()` the same way, but only while authority is currently
`kHeld`: release is keyed on leaving `Start`, not on the literal `DAMP` id, because `Start ->
StandUp` is itself a legal FSM edge (see `g1_bringup`'s legality table) and a `STAND_UP` goal
accepted from `kHeld` means the robot is about to leave the state velocity authority depends on,
exactly as much as a `DAMP` goal does. `onSetLocoModeResult()` mirrors the same split: a `START`
result calls `onAcquireResult()`; every other result calls `onReleaseResult()` (a harmless no-op
if `beginRelease()` was never called for that particular goal).

The one authority-*promoting* callback -- a successful `START` result calling
`onAcquireResult(true)` -- additionally self-gates on `PRIMARY_STATE_ACTIVE`, the same standard
`cmdVelCallback()`/`onReissueTick()` already hold themselves to. The primary defence against a
late reply reviving stale authority is `on_deactivate()` itself: it supersedes the correlator
entry for any in-flight `SetLocoMode` goal and aborts that goal directly before returning, so a
reply delivered after deactivation can never reach `onSetLocoModeResult()` at all; the
`PRIMARY_STATE_ACTIVE` self-gate is the belt-and-braces backstop for that same property, not the
primary fix.

`fsm_id` in `~/status` is **not** solely inferred from goal outcomes: a `7001 GET_FSM_ID` poll
rides along on the ~1 Hz heartbeat timer (via `loco_payloads::parseFsmIdResponse`), keeping it
confirmed by the robot rather than only assumed. That timer's first tick is deliberately
phase-offset half a re-issue period from the velocity re-issue timer's own ticks: two
independently-created wall timers with harmonically related periods (the default 1000 ms
heartbeat is an exact multiple of the default 200 ms re-issue period) would otherwise start
counting from the same `on_configure()` instant and fire together once a second, publishing
`SET_VELOCITY` and `GET_FSM_ID` back to back on a channel whose vendor contract assumes one call
in flight.

### Single-writer advisory guard

The same 1 Hz timer checks `count_publishers("/api/sport/request")`; a count above 1 (something
else is also writing to this channel) is always logged, and if this bridge currently holds
velocity authority (`kHeld`), it also sends one defensive `SetVelocity(0,0,0)` and calls
`VelocityGate::forceRelease()`. Advisory only, like `G1ArmSdkSystem`'s equivalent guard -- real
cross-process control-authority arbitration is a future behavior-tree concern.

### Shutdown has no synchronous ramp

Unlike the arm bridge, `on_shutdown` does not block ramping anything down -- this node never
actuates directly; it only ever asks the onboard controller to move. If re-issuing simply stops
(this process dying included), the onboard controller's own `duration` dead-man (1 s) takes the
robot back to a stop on its own. That emergent safety property is exactly why re-issuing with a
short duration, rather than latching the vendor's 864000 s "continuous" value, is non-negotiable.

### Teardown never wedges a goal

`resetEntities()` -- called from `on_cleanup`, `on_shutdown`, `on_error`, and the top of
`on_configure` -- aborts any `SetLocoMode` goal still in flight (a terminal result, `success:
false`) and clears the correlator's pending map, both *before* destroying the action server and
every timer/subscription. Ordering matters: once those entities are gone, nothing left standing
could ever resolve that goal on its own (no sweep timer to time it out, no response subscription
to match a reply), so skipping this would hang the goal's client forever -- most easily reached
today via `shutdown()` from `PRIMARY_STATE_ACTIVE`, which Humble's lifecycle state machine allows
directly, bypassing `on_deactivate()`'s own (separate) goal-termination entirely.

## Parameters (`config/g1_loco_bridge.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `request_timeout_s` | 5.0 | `LocoRequestCorrelator` sweep timeout -- matches `BaseClient`'s own blocking timeout. |
| `max_pending` | 16 | Belt-and-braces bound on requests tracked at once. |
| `velocity_reissue_hz` | 5.0 | `SetVelocity` re-issue rate; must be `> 1.0`. |
| `cmd_vel_timeout_ms` | 500.0 | `~/cmd_vel` age beyond this is "stale". |
| `failure_streak_limit` | 3 | Consecutive non-zero `SetVelocity` codes before releasing authority. |
| `max_velocity` | `[0.8, 0.5, 1.57]` | `[vx, vy, vyaw]` clamp, m/s / m/s / rad/s. **Sim bring-up value** -- see above. |
| `axis_sign` | `[1.0, 1.0, 1.0]` | `[vx, vy, vyaw]` sign flip applied before clamping. |

## Running

The normal way to bring this bridge up is as part of the whole sim stack, not standalone:

```bash
ros2 launch g1_bringup sim.launch.py
```

`sim.launch.py` includes this package's own `loco.launch.py`, which starts `g1_loco_bridge` and
drives it `configure -> active` automatically, chained off the node's own lifecycle events (not a
timing guess -- see `g1_bringup/README.md`'s launch-file table). `motion_service_sim` (also
started by `sim.launch.py`) is this stack's `/api/sport/*` responder: it answers `7001`/`7101`/
`7105` **protocol-only** -- FSM state tracking and the `SET_VELOCITY` `Start`-only gate, exactly
per the wire contract -- but never actuates a leg; the robot stays pelvis-welded regardless of FSM
state or velocity requests (see `g1_bringup/README.md`'s "LocoClient wire responder" section for
the full dispatch table and why walking-in-sim is a separate, out-of-scope concern this
milestone). Once the stack is up:

```bash
# Drive the FSM to Start (StandUp is a required intermediate hop):
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 4}"
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 500}"

# Watch status (transient-local, so this immediately prints the latest even if
# nothing changes for a while):
ros2 topic echo /g1_loco_bridge/status

# Drive forward slowly once authority is HELD:
ros2 topic pub -r 10 /g1_loco_bridge/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Release authority:
ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "{fsm_id: 1}"
```

This is exactly the sequence `g1_bringup/test/test_loco.launch.py` drives end to end against a
real, live sim (see "Building and testing" below).

### Standalone, without the sim stack

`ros2 run g1_locomotion g1_loco_bridge` plus manual lifecycle transitions also works, for
developing this package in isolation:

```bash
ros2 run g1_locomotion g1_loco_bridge --ros-args \
  --params-file install/g1_locomotion/share/g1_locomotion/config/g1_loco_bridge.yaml

# In another shell: configure then activate.
ros2 lifecycle set /g1_loco_bridge configure
ros2 lifecycle set /g1_loco_bridge activate
```

**Not while `sim.launch.py` is already running**, though: that would put two `g1_loco_bridge`
processes (and, if driven manually, two publishers) on the same `~/cmd_vel`/`~/set_mode`/
`/api/sport/*` names in the same domain, which trips the bridge's own single-writer advisory guard
on `/api/sport/request` the moment it notices and force-releases velocity authority. With no
`/api/sport/*` responder running at all (`motion_service_sim` or otherwise), every request above
times out after `request_timeout_s` (5 s) instead of succeeding: `~/status.authority` will show
`ACQUIRING` and then fall back to `RELEASED`, with `last_error_code` set to the correlator's own
timeout code (`-1`). That is expected in that specific configuration, not a bug in this package.

## Building and testing

```bash
colcon build --symlink-install --packages-select g1_locomotion
colcon test --packages-select g1_locomotion
colcon test-result --verbose
```

Four gmock binaries:

- `test_loco_payloads` -- exact JSON for `7101`/`7105`, `duration` always `1.0` (and `864000`
  never appears anywhere in this package's source), and response `data` parsing including
  malformed input. No sim or live DDS required.
- `test_loco_correlator` -- overlapping in-flight requests, out-of-order responses, a
  never-arrives sweep timeout, an orphaned response arriving *after* its entry was already swept
  (dropped safely, no crash, no double callback), `supersede()`, a `sweep()` callback that itself
  calls `send()` (reentrancy safety), and the `max_pending` bound. No sim or live DDS required.
- `test_velocity_gate` -- re-issue faster than 1 Hz, the stale/zero single-stop-then-idle policy,
  the failure-streak release, `cmd_vel` ignored outside `kHeld`, and every terminal path landing
  in a defined state. No sim or live DDS required.
- `test_loco_bridge_node` -- `G1LocoBridge` itself, in-process against a fake `/api/sport/*`
  responder on an isolated `ROS_DOMAIN_ID` (real DDS loopback, no sim, no `launch_testing`): a
  late `SetLocoMode` reply arriving after `on_deactivate` must not revive authority into the next
  session, a `SetVelocity` round trip slower than the re-issue period must still advance the
  failure streak, a `STAND_UP` goal from `kHeld` must release authority and stop velocity traffic,
  and a goal still in flight when the node tears down must terminate instead of hanging forever.

Plus `g1_bringup/test/test_loco.launch.py` -- a headless `launch_testing` suite (run via `colcon
test --packages-select g1_bringup`, since it needs the sim stack's own launch files) that drives
this bridge end to end against a real, live `motion_service_sim` over real DDS: the responder's
`Start`-only `SET_VELOCITY` gate, the `STAND_UP -> START` sequence, the re-issue loop's rate and
fixed duration, the zero-Twist stop, and `DAMP` releasing authority. Between the gmock suites
above (this package's own logic, no live DDS) and `test_loco_bridge_node` (this bridge's
lifecycle/authority behavior, live DDS but no sim), `test_loco.launch.py` is what actually
validates the wiring between this bridge and a real `/api/sport/*` responder. Leg dynamics (actual
walking) are out of scope for all of them and deferred to hardware -- `motion_service_sim` is
protocol-only by design (see `g1_bringup/README.md`).

Plus `clang-format` (against the repo root's `.clang-format`), `ament_lint_cmake`, and `xmllint`
on this package's own XML files.

## Language note

C++17 throughout. `rclcpp_action`/DDS glue and the pure engines it wraps are squarely in the
"always C++" category (a control-facing bridge translating into a real-time-adjacent robot
interface); there is no Python path here.
