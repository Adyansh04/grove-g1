# g1_inspire_hand

`ros2_control` hardware for the Inspire Robots RH56DFQ five-finger hand on the G1. Each hand is its
own system component on its own RS485 line, through a USB to RS485 adapter on the robot's
development computer, or through a pseudo-terminal the simulator answers on. C++, no DDS.

The hand has six actuators: one per finger and two for the thumb. Each moves to an angle register
at its own speed limit and stops closing when its finger presses at the force limit. The hand's
other six joints are linked to these mechanically; the URDF models them with `<mimic>` tags, so
`robot_state_publisher` and MoveIt derive them.

## Plugin

`g1_inspire_hand/G1InspireSystem`, loaded from `g1_description` with `hand_model:=inspire`, as
`G1InspireSystemLeft` and `G1InspireSystemRight`.

| Joint (left `L_`, right `R_`) | Actuator | Command | State |
|---|---|---|---|
| `*_pinky_proximal_joint` | pinky | `position` | `position`, `velocity`, `effort` |
| `*_ring_proximal_joint` | ring | `position` | `position`, `velocity`, `effort` |
| `*_middle_proximal_joint` | middle | `position` | `position`, `velocity`, `effort` |
| `*_index_proximal_joint` | index | `position` | `position`, `velocity`, `effort` |
| `*_thumb_proximal_pitch_joint` | thumb bend | `position` | `position`, `velocity`, `effort` |
| `*_thumb_proximal_yaw_joint` | thumb rotation | `position` | `position`, `velocity`, `effort` |

The joints come in the hand's register order, and the component refuses to start otherwise.
Positions are in radians, mapped linearly to the hand's angle register: 1000 is `open_rad`, 0 is
`closed_rad`. `effort` is the finger force the hand measures, in N m: the force acting
`lever_arm_m` from the joint.

## Behaviour

- Activation opens the line and reads the hand, waiting up to 2 s for an answer. It then clears
  the hand's latched errors, writes its speed and force limits, and seeds every command from the
  measured angle, so the first command moves nothing. A hand that does not answer fails
  activation with the port and the protocol in the message.
- A thread of its own runs the bus at `io_rate_hz`. Each cycle writes the six targets when one
  changed, then reads the angles and forces; every `diagnostics_every` cycles it also reads
  current, error, status and temperature, and logs an actuator whose error flags change.
  `read()` and `write()` only exchange values with it, without locking.
- Commands are slewed at `max_joint_velocity_rad_s` from where the last one left off.
- No good state for `state_timeout_ms` errors the component.
- A stop (deactivate, error, shutdown) sends nothing more. The actuators are lead screws and hold
  their last target, which the slew keeps within one step of the measured position: a stopped
  hand keeps what it holds and never opens.

## Protocol

Two framings over the same byte-addressed register map, chosen with `protocol`:

| `protocol` | Frame | Notes |
|---|---|---|
| `inspire` | `EB 90 id len cmd addr data sum` | The hand's own frame, what Unitree's G1 hand service speaks. |
| `modbus_rtu` | Modbus RTU, functions 0x03 and 0x10 | Newer firmware. The register address is the byte address; each register carries two bytes. |

Registers used: `ANGLE_SET` 1486, `FORCE_SET` 1498, `SPEED_SET` 1522, `ANGLE_ACT` 1546,
`FORCE_ACT` 1582, `CURRENT` 1594, `ERROR` 1606, `STATUS` 1612, `TEMP` 1618, `CLEAR_ERROR` 1004.
The map and the frame follow Inspire's RH56 manual and Unitree's
[dfx_inspire_service](https://github.com/unitreerobotics/dfx_inspire_service).

One master per line: stop Unitree's `inspire_g1` service on the robot before starting this, or
both garble the bus.

## Parameters

In `g1_description`'s `config/inspire_params.yaml`, passed as `<param>` tags:

| Parameter | Value | Meaning |
|---|---|---|
| `serial_port` | `/tmp/g1_inspire_<side>` in sim, `/dev/ttyUSB2` (left) and `/dev/ttyUSB1` (right) on the robot | The line. USB numbering follows plug order: check it on the robot. |
| `protocol` | `inspire` | See above. |
| `baud_rate`, `slave_id` | `115200`, `1` | The hand's defaults. |
| `io_rate_hz` | `50` | Bus cycles a second. A cycle is about 10 ms of bus time at 115200 baud. |
| `response_timeout_ms` | `25` | A reply later than this counts as lost. |
| `state_timeout_ms` | `300` | No good state for this long errors the component. |
| `diagnostics_every` | `10` | Cycles between diagnostics reads. |
| `speed` | `1000` | The hand's own speed limit, 0 to 1000. |
| `force_limit_g` | `300` | Grams at the fingertip: a closing finger stops here. The DFQ is rated 400 g a finger. |
| `max_joint_velocity_rad_s` | `3.0` | The command slew. |
| `open_rad`, `closed_rad`, `lever_arm_m` | per joint | The angle mapping and the effort's lever arm. |

## In simulation

The `unitree_mujoco` fork answers on the two pseudo-terminals as the hands do. Start the stack
with `hand_model:=inspire` (see `g1_bringup`).

## Tests

```bash
./scripts/manage.sh test g1_inspire_hand   # none of these needs a simulator
```

| Test | Checks |
|---|---|
| `test_protocol` | Both framings against golden frames: Unitree's service for `inspire`, the Modbus specification's CRC example. Replies refused for a bad checksum, another id or a Modbus exception. The angle and force mappings. |
| `test_serial_hand` | The serial line and the register client against a fake hand on a pseudo-terminal, in both framings: reads, writes, "no change" targets, the one-byte registers, clearing errors, a silent hand's timeout. |
| `test_system` | The component through its lifecycle against the fake: the plugin export, joints out of order refused, limits written and commands seeded at activation, the slew, a stop that sends nothing, a hand that stops answering, one that never does, and the same over Modbus. |
