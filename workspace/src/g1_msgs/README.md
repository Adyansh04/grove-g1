# g1_msgs

Interfaces for the G1 LocoClient bridge (`g1_locomotion`): the `SetLocoMode` action and the
`LocoStatus` message. `ament_cmake` with `rosidl_default_generators`, no source of its own.

```bash
colcon build --symlink-install --packages-select g1_msgs
```

## `action/SetLocoMode.action`

| Field | Meaning |
|---|---|
| `fsm_id` (goal) | Target FSM id. Only `DAMP` (1), `STAND_UP` (4) and `START` (500) are accepted; the bridge rejects anything else in `handle_goal` and logs the reason. `Squat` (2), `Sit` (3) and `ZeroTorque` (0) have no constant here: nothing needs them, and their transition legality was never checked against a real onboard controller. |
| `success`, `error_code` (result) | `error_code` is the raw LocoClient wire status code: `0` on success, otherwise `7301`, `7302`, or a correlator timeout. |
| `message` (result) | Human-readable context for logs and CLI use. Not meant to be parsed. |

No feedback field: the exchange either completes or times out within a few seconds, with nothing
meaningful to report mid-flight.

## `msg/LocoStatus.msg`

| Field | Meaning |
|---|---|
| `stamp` | When this status was produced. |
| `fsm_id` | Last FSM id the bridge knows to be true. `-1` before it has any information. |
| `authority` | `RELEASED`, `ACQUIRING`, `HELD` or `RELEASING`: whether the bridge holds velocity-command authority. See `g1_locomotion`'s README for the state machine. |
| `last_error_code` | Most recent wire status code observed, from `SetLocoMode` or velocity requests alike. |
| `ignored_cmd_vel` | How many **non-zero** `~/cmd_vel` samples the bridge has discarded because it held no authority. Monotonic, never reset on acquire, so a reader can assert "stopped increasing". Zero commands are not counted: publishers idle at zero routinely, and an idle publisher is not a dropped intent. The counter exists because "the planner is running and the robot is stationary" is otherwise silent. |

## Why an action rather than a service

Completion is asynchronous. The LocoClient wire exchange is two plain topics: a request published
now, a response correlated to it later. A Humble service callback must populate its response before
returning, so waiting there for `/api/sport/response` would block the executor. An action's
`handle_accepted` returns immediately and reports the outcome later through the goal handle, which
matches the protocol's own asynchrony.
