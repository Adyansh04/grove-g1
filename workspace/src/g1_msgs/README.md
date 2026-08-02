# g1_msgs

Interfaces for the G1 LocoClient bridge (`g1_locomotion`): the `SetLocoMode` action and the
`LocoStatus` message. `ament_cmake` + `rosidl_default_generators` -- no C++/Python source of its
own, so there is no language-policy note to make here.

## Why an action, not a service

Completion of a `SetLocoMode` request must be asynchronous. The underlying LocoClient wire
exchange is two plain topics (a request published now, a response correlated to it later,
sometimes seconds later) -- see `g1_locomotion`'s README for the full rationale, including why
the vendored blocking client can't be used from an executor callback. A Humble service server's
callback must populate its response before returning; forcing that callback to synchronously wait
for the eventual `/api/sport/response` would reintroduce exactly the blocking-executor hazard this
milestone exists to remove. An action's `handle_accepted` returns immediately and reports the
outcome later through the goal handle, which matches the wire protocol's own asynchrony instead of
fighting it.

## `action/SetLocoMode.action`

| Field | Meaning |
|---|---|
| `fsm_id` (goal) | Target FSM id. Only `DAMP` (1), `STAND_UP` (4), `START` (500) are accepted -- `g1_locomotion`'s bridge rejects any other value in `handle_goal`, with the reason logged there. `Squat`(2)/`Sit`(3)/`ZeroTorque`(0) have no constant here: no caller needs them yet, and their transition legality from an arbitrary state was never checked against a real onboard controller. |
| `success`, `error_code` (result) | `error_code` is the raw LocoClient wire status code (`0` on success, e.g. `7301`/`7302`/a correlator timeout otherwise). |
| `message` (result) | Human-readable context for logs/CLI use -- not meant to be parsed. |

No feedback: the underlying exchange either completes or times out within a few seconds, with
nothing meaningful to report mid-flight.

## `msg/LocoStatus.msg`

| Field | Meaning |
|---|---|
| `stamp` | Time this status was produced. |
| `fsm_id` | Last FSM id `g1_locomotion` knows to be true; `-1` before it has any information. |
| `authority` | One of `RELEASED`/`ACQUIRING`/`HELD`/`RELEASING` -- whether the bridge currently holds velocity-command authority. See `g1_locomotion`'s README for the full state machine. |
| `last_error_code` | Most recent LocoClient wire status code the bridge observed (`SetLocoMode` or velocity requests alike). |

## Building

```bash
colcon build --symlink-install --packages-select g1_msgs
```
