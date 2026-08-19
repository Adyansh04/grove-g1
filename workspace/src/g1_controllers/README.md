# g1_controllers

`ros2_control` controllers for the whole-body `rt/lowcmd` stack. `ament_cmake`, C++20.

| Controller | Does |
|---|---|
| `g1_controllers/G1AgileController` | Runs the AGILE velocity policy on 12 leg joints plus waist roll and pitch, tracking `/cmd_vel`. |
| `g1_controllers/G1SafetyController` | Chainable. Ramps the policy in from the pose held at activation, clamps joint rate, freezes on divergence. |
| `g1_controllers/G1FreezeController` | Captures each claimed joint's position on activation and holds it there under impedance control. |

## Attribution

The controller design and its interface naming are adapted from NVIDIA's
[isaac_ros_deploy](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_deploy) `InferenceController`,
`SafetyController` and `FreezeController` (Apache-2.0). The policy artifact under `policy/` is
NVIDIA's [WBC-AGILE](https://github.com/nvidia-isaac/WBC-AGILE) G1 velocity policy, also
Apache-2.0; its licence ships beside it.

Their reference-interface names
(`<controller>/<joint>/{position,velocity,effort,kp,kd}_raw`) and their `kp`/`kd`
command-interface spelling are kept deliberately, so their controllers chain onto this stack
unchanged.

## How the pieces fit

```
/cmd_vel ──► G1AgileController ──► G1SafetyController ──► G1LowCmdSystem ──► rt/lowcmd
              (policy, 50 Hz)       (blend + clamp)        (hardware component)
```

The policy claims 14 joints. The other 15 are held by two more controllers, so all 29 motors are
claimed at every instant. That matters because the component leaves any unclaimed joint unpowered.

| Controller | Joints | When active |
|---|---|---|
| `locomotion_safety_controller`, with `agile_controller` chained above it | 12 legs + waist roll/pitch | always |
| `waist_freeze_controller` | waist yaw | always |
| `arm_freeze_controller` | the 14 arm joints | until the arm is acquired |
| `arm_trajectory_controller` | the same 14 | while the arm is acquired |
| `locomotion_freeze_controller` | 12 legs + waist roll/pitch | emergency only; loaded inactive |

The last two rows are worth reading twice. `arm_freeze_controller` and `arm_trajectory_controller`
claim identical joints, so they trade in one `switch_controller` call rather than two, and
`ros2_control` applies that inside a single update cycle. `locomotion_freeze_controller` covers
exactly what the safety controller was driving and no more: a wider freeze could not activate
while the arm and waist controllers hold their own joints, so the emergency would fail in the one
situation it exists for.

`rt/lowcmd` is one message covering all 29 motors, so the component sends something for every joint
on every tick, claimed or not. Holding is therefore a controller rather than component behaviour:
it switches in and out at runtime, and the component stays free of policy.

## The policy contract

`policy/unitree_g1_velocity_e2e.onnx` is end-to-end. Three properties matter before touching
`AgilePolicy`:

- It is stateful. Seven of its twelve inputs are history tensors it emits again as outputs, so the
  runner feeds them straight back and keeps no ring buffers of its own.
- `action_joint_pos` is an absolute radian target, not a scaled action. The graph applies its own
  scale and default-pose offset.
- The gains come out of the graph, per joint, and are forwarded rather than configured.

Its two joint orderings differ from each other and from the SDK's motor order, so `agileObsIndex()`
and `agileActionIndex()` map by name. `AgilePolicy`'s constructor verifies the model's IO names and
widths against this contract and throws if they have drifted.

Inference costs 0.056 ms mean and 0.319 ms max, single-threaded on CPU, about 6% of a 200 Hz tick.
Plain `onnxruntime` is enough and no GPU runtime is involved.

## Parameters

`G1AgileController`:

| Parameter | Meaning |
|---|---|
| `model_path` | Empty resolves to the policy shipped in this package's share directory. |
| `cmd_vel_topic` | Where velocity commands come from. `/cmd_vel` by default, which is Nav2's output. |
| `decimation` | Controller-manager ticks per inference. 4, giving 50 Hz under 200 Hz. |
| `command_prefix` / `command_suffix` | Chain target. Empty writes straight to the component. |
| `cmd_vel_timeout` | Seconds before a silent publisher is treated as a zero command. |
| `max_linear_speed` / `max_angular_speed` | Command clamps; 0 disables. |

`G1SafetyController`:

| Parameter | Meaning |
|---|---|
| `blend_ratio` | 0 holds the activation pose, 1 follows the policy. Settable at runtime. |
| `max_blend_ratio_speed` | Rate limit on that ratio, per second. |
| `max_velocity` | Per-joint rad/s clamp; non-positive leaves a joint unclamped. |
| `kp` / `kd` | Fallback gains, used only on ticks where nothing upstream has written. |
| `mean_velocity_limit` / `max_velocity_limit` | Divergence thresholds; 0 disables the detector. |
| `emergency_controller` | Switched in when the detector fires. Empty disables the switch. |

`G1FreezeController` takes `joints`, `kp` and `kd`. Both gains must be positive, and
`on_configure` rejects them otherwise: a freeze with no stiffness is a disable with extra steps.

## Running

```bash
ros2 launch g1_bringup sim.launch.py
```

The pelvis is unpinned by default, because the policy balances the robot.

The policy and its safety controller must be spawned in one switch (`--activate-as-group`), which
`control.launch.py` does. A chainable controller's reference interfaces only become claimable as it
enters chained mode, and that happens inside the switch that activates it.

## What simulation does not validate

- Displacement. `test_agile_walk` runs without the sensor relay, so no ground truth reaches ROS and
  it asserts uprightness rather than distance travelled. The gait envelope is measured against
  MuJoCo directly instead.
- Hardware timing. The 200 Hz loop overruns when the perception stack shares the machine, and the
  policy is sensitive to that. On a robot this needs real-time scheduling.
- `MotionSwitcherClient`. Entry to `rt/lowcmd` on a real G1 is untested; see
  `g1_hardware_interface`.

## Tests

```bash
colcon test --packages-select g1_controllers
```

| Test | Covers |
|---|---|
| `test_agile_policy` | The ONNX contract and both joint tables, against the installed policy. |
| `test_safety_blend` | The blend-and-slew arithmetic, including that the rate clamp survives a blend-ratio step. |
| `test_freeze_pluginlib` | All three controllers resolve through pluginlib, the path `controller_manager` uses. |
| `test_joint_ownership` | Reads this package's controller config against `g1_description`'s joint list, so no body motor is left unclaimed. |

Behaviour is covered by `g1_bringup`'s `test_agile_walk`, which stands the robot on the policy with
an unpinned pelvis and walks it on command.
