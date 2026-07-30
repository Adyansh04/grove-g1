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

Exactly one thread ever touches this node: `main()` spins a single
`rclcpp::executors::SingleThreadedExecutor` over one explicitly created and named
`MutuallyExclusive` callback group that *every* callback source is placed in -- the
`/api/sport/response` subscription, the ~50 ms sweep timer, the velocity re-issue timer, the 1 Hz
heartbeat/FSM-poll/rogue-guard timer, `~/cmd_vel`, and the `~/set_mode` action server. No locks, no
atomics, anywhere in `G1LocoBridge`, `LocoRequestCorrelator`, or `VelocityGate`. This is
deliberate, not incidental: naming and populating the group explicitly (rather than relying on
every entity's implicit default group, which happens to coincide today) means a future
`MultiThreadedExecutor` swap can't silently start running two of these callbacks concurrently --
moving any one of them to a different group requires a visible edit inside `on_configure()`, not a
one-line change somewhere else entirely.

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
- `supersede(id)` -- drops a pending entry with no callback invocation, for exactly the case where
  a fresher command has already replaced it (the velocity re-issue timer supersedes its own
  previous in-flight request every tick, and the FSM-poll timer does the same for its own poll) --
  only the newest outcome for that *kind* of request ever matters.

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
| `/api/sport/request` | out | `unitree_api/Request` | `rclcpp::QoS(1)`, reliable, volatile | **Vendor-matched -- do not deviate.** The exact QoS `BaseClient`'s own publisher uses; hardware endpoint compatibility depends on matching it exactly. |
| `/api/sport/response` | in | `unitree_api/Response` | `rclcpp::QoS(1)`, reliable, volatile | Same as above. |

`~/cmd_vel` is mapped `linear.x -> vx`, `linear.y -> vy`, `angular.z -> vyaw`, each clamped to
`max_velocity` (default `[0.3, 0.2, 0.5]` -- deliberately conservative so a first hardware run is
slow by default) after being multiplied by `axis_sign` (default `[1, 1, 1]` -- the vendor's frame
convention relative to `Twist` is undocumented, so this is the calibration knob hardware bring-up
will need, not something derivable in sim).

### `SetLocoMode` handling

Only `fsm_id` in `{DAMP(1), STAND_UP(4), START(500)}` is accepted; anything else is rejected in
`handle_goal` with the reason logged (a rejected goal has no result to carry a message in, so this
is a server-side log, not something the client can inspect). At most one goal is in flight at a
time -- a second one is rejected outright rather than needing to unwind the first.

A `START` goal calls `VelocityGate::beginAcquire()` before publishing its `SET_FSM_ID(500)`
request (only once `LocoRequestCorrelator::send()` confirms a request is actually in flight, so
every `kAcquiring` is guaranteed a future callback); a `DAMP` goal from `kHeld` calls
`beginRelease()` the same way. `STAND_UP` never touches `VelocityGate` -- it's a prerequisite FSM
hop on the way to `Start`, not a change in who holds velocity-command authority.

`fsm_id` in `~/status` is **not** solely inferred from goal outcomes: a `7001 GET_FSM_ID` poll
rides along on the 1 Hz heartbeat timer (via `loco_payloads::parseFsmIdResponse`), keeping it
confirmed by the robot rather than only assumed.

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

## Parameters (`config/g1_loco_bridge.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `request_timeout_s` | 5.0 | `LocoRequestCorrelator` sweep timeout -- matches `BaseClient`'s own blocking timeout. |
| `max_pending` | 16 | Belt-and-braces bound on requests tracked at once. |
| `velocity_reissue_hz` | 5.0 | `SetVelocity` re-issue rate; must be `> 1.0`. |
| `cmd_vel_timeout_ms` | 500.0 | `~/cmd_vel` age beyond this is "stale". |
| `failure_streak_limit` | 3 | Consecutive non-zero `SetVelocity` codes before releasing authority. |
| `max_velocity` | `[0.3, 0.2, 0.5]` | `[vx, vy, vyaw]` clamp, m/s / m/s / rad/s. |
| `axis_sign` | `[1.0, 1.0, 1.0]` | `[vx, vy, vyaw]` sign flip applied before clamping. |

## Running

This package has no launch file of its own yet -- run the node directly and drive its lifecycle
by hand:

```bash
ros2 run g1_locomotion g1_loco_bridge --ros-args \
  --params-file install/g1_locomotion/share/g1_locomotion/config/g1_loco_bridge.yaml

# In another shell: configure then activate.
ros2 lifecycle set /g1_loco_bridge configure
ros2 lifecycle set /g1_loco_bridge activate

# Query the current mode / drive the FSM to Start (StandUp is a required intermediate hop):
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

**Nothing in `unitree_mujoco` serves `/api/sport/*` yet** -- it emulates only the low-level device
(subscribes `rt/lowcmd`, publishes `rt/lowstate`); there is no onboard motion service in sim to
answer a LocoClient request at all. So against the current sim every request above times out
after `request_timeout_s` (5 s) rather than succeeding; `~/status.authority` will show `ACQUIRING`
and then fall back to `RELEASED` with `last_error_code` set to the correlator's timeout code. That
is expected, not a bug in this package: this milestone validates the request/response bridge
itself (wire encoding, async correlation, the authority state machine) against unit tests with no
live DDS, exactly like `g1_hardware_interface`'s `ArmRampEngine` and `g1_bringup`'s `blend_math` are
validated. End-to-end validation against something that actually answers `/api/sport/request` --
either a future sim-side responder or real hardware -- is deferred to a later milestone.

## Building and testing

```bash
colcon build --symlink-install --packages-select g1_locomotion
colcon test --packages-select g1_locomotion
colcon test-result --verbose
```

Three gmock binaries, no sim required:

- `test_loco_payloads` -- exact JSON for `7101`/`7105`, `duration` always `1.0` (and `864000`
  never appears anywhere in this package's source), and response `data` parsing including
  malformed input.
- `test_loco_correlator` -- overlapping in-flight requests, out-of-order responses, a
  never-arrives sweep timeout, an orphaned response arriving *after* its entry was already swept
  (dropped safely, no crash, no double callback), `supersede()`, and the `max_pending` bound.
- `test_velocity_gate` -- re-issue faster than 1 Hz, the stale/zero single-stop-then-idle policy,
  the failure-streak release, `cmd_vel` ignored outside `kHeld`, and every terminal path landing
  in a defined state.

There is no sim/launch integration suite yet -- see "Running" above for why: nothing in
`unitree_mujoco` currently answers `/api/sport/request`, so there is nothing live to integrate
against until a sim-side responder or real hardware exists.

Plus `clang-format` (against the repo root's `.clang-format`), `ament_lint_cmake`, and `xmllint`
on this package's own XML files.

## Language note

C++17 throughout. `rclcpp_action`/DDS glue and the pure engines it wraps are squarely in the
"always C++" category (a control-facing bridge translating into a real-time-adjacent robot
interface); there is no Python path here.
