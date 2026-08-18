# g1_hand_interface

`ros2_control` `SystemInterface` for one Unitree Dex3-1 hand, over the hand's own `unitree_sdk2`
channels.

`ament_cmake`, C++20. **Real hardware code** — it speaks Unitree's published Dex3 contract and
runs unchanged on the robot; the simulator answers the same channels.

The robot carries two state channels: `rt/dex3/<side>/state` and a lower-rate
`rt/lf/dex3/<side>/state`. Unitree's own code disagrees about which to read — their Dex3 example
subscribes to the `lf` one, their SDK bridge publishes the plain one. This is a control loop
with a freshness gate, so `state_topic` defaults to the full-rate name; it is a hardware
parameter (`g1_description/config/dex3_params.yaml`) so hardware bring-up can switch without a
rebuild if the robot turns out to populate only the other one.

Separate from `g1_hardware_interface` on purpose. The hand is a different device on different
channels with its own control authority, and one component per hand keeps a hand fault from
taking the arms down with it.

The component reaches the wire through the SDK's own CycloneDDS rather than through ROS. That is
why `CMakeLists.txt` pins `DT_RPATH` at the SDK's `lib/`: ROS ships a newer CycloneDDS under the
same SONAME, and binding both in one process corrupts the heap. `domain_id` and
`network_interface` must match `g1_hardware_interface`'s, because `ChannelFactory` is per process
and only its first `Init` takes effect.

## Interfaces

| Interface | Kind | Source |
|---|---|---|
| `position` | command | ramped, then clamped to the URDF limits |
| `position`, `velocity`, `effort` | state | `motor_state[i].q` / `.dq` / `.tau_est` |

| Channel | Direction | Type |
|---|---|---|
| `rt/dex3/<side>/cmd` | out | `unitree_hg::msg::dds_::HandCmd_` |
| `rt/dex3/<side>/state` | in | `unitree_hg::msg::dds_::HandState_` (`state_topic`) |

These are SDK channels, not ROS topics, so they carry the SDK's own QoS and do not appear in
`ros2 topic list`. `<side>` comes from the `side` parameter and must be `left` or `right`.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `side` | — | `left` or `right`. Picks the channels and the joint prefix. Required. |
| `domain_id` | — | SDK DDS domain. Must match `g1_hardware_interface`'s. Required. |
| `network_interface` | `""` | Must stay empty: a non-empty value makes the SDK discard `CYCLONEDDS_URI`. |
| `kp` | 1.5 | Finger motors, not arm motors. Roughly 300× smaller than the arm's gains. |
| `kd` | 0.2 | |
| `command_publish_rate` | 100.0 | Hz. What Unitree's own teleop uses. |
| `max_joint_velocity_rad_s` | 3.0 | Slew clamp on the commanded position. |
| `state_timeout_ms` | 200.0 | State older than this blocks activation, and errors the component if it goes stale once active. |
| `state_topic` | `rt/dex3/<side>/state` | Which of the robot's two state channels to read. See above. |

## Things the wire format will punish you for

**`motor_cmd` is an unbounded sequence, not a fixed array.** Write it unresized and DDS accepts
the message while nothing moves. `on_init` resizes the preallocated frame once, so the write path
never allocates.

**Joint order is positional**, and it is the URDF's own order: thumb_0, thumb_1, thumb_2,
middle_0, middle_1, index_0, index_1 — identical for both hands. `on_init` refuses to start if
the declared joints disagree, because the failure is otherwise a hand that closes the wrong
fingers. Unitree's own `Dex3_1_Right_JointIndex` enum lists index before middle and contradicts
their documented order; it is inert in their code but must not be copied here.

**There is no blend weight.** The first write takes full authority immediately. That is why this
component seeds its command from the measured position on activate and slews toward the target:
nothing upstream will soften a step for you.

**Timeout protection** is armed only on release. While driving, the controller is the heartbeat,
and arming it would stop the fingers a second after any hiccup in the control loop.

**Limits come from the URDF**, which matches Unitree's published spec. Their SDK example
disagrees on `thumb_1` (0.724 vs 0.611 rad) and its right hand says 0.742, which looks like a
transposed digit.

## In simulation

`unitree_mujoco` answers the same two channels, so this component is not swapped out for sim. The
responder is `workspace/vendor/unitree_mujoco/dex3_handler.cc`, registered by patch 004; the
finger joints themselves come from patch 003.

It runs the PD the hardware runs, from the `kp`/`kd` in the command, and clamps to the URDF's
effort limits. Two behaviours worth knowing:

- The fingers are driven through `qfrc_applied`, not through MuJoCo actuators. The vendored SDK
  bridge sizes itself from the actuator count and indexes a fixed 35-slot `LowCmd`, so 29 body
  motors plus 14 fingers would run it off the end of that array.
- `status = Lock` holds the finger where it is rather than going limp, which is what the
  hardware's own status byte means. The same applies before any command arrives and one second
  after the last one.

Finger contact is not simulated: the geometry is visual only, and grasping is modelled by
attaching the object in MoveIt's planning scene.

## Not yet verified on hardware

The real state publish rate, the press-sensor index-to-pad map, and the `thumb_1` upper limit.
None are documented publicly; each was measured against the hardware and the simulator.
