# g1_controllers

`ros2_control` controllers for the whole-body `rt/lowcmd` stack. `ament_cmake`, C++20.

| Controller | Does |
|---|---|
| `g1_controllers/G1AgileController` | Runs the AGILE velocity policy on 12 leg joints plus waist roll and pitch, tracking `/cmd_vel`. |
| `g1_controllers/G1SafetyController` | Chainable. Ramps the policy in from the pose held at activation, clamps joint rate, freezes on divergence. |
| `g1_controllers/G1FreezeController` | Captures each claimed joint's position on activation and holds it there under impedance control. |

## Attribution

The controller design and interface naming are adapted from NVIDIA's
[isaac_ros_deploy](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_deploy) `InferenceController`,
`SafetyController` and `FreezeController` (Apache-2.0). Their reference-interface names
(`<controller>/<joint>/{position,velocity,effort,kp,kd}_raw`) and `kp`/`kd` command-interface
spelling are kept, so their controllers chain onto this stack unchanged. The policy under `policy/`
is NVIDIA's [WBC-AGILE](https://github.com/nvidia-isaac/WBC-AGILE) G1 velocity policy, also
Apache-2.0; its licence ships beside it.

## How the pieces fit

```
/cmd_vel ──► G1AgileController ──► G1SafetyController ──► G1LowCmdSystem ──► rt/lowcmd
              (policy, 50 Hz)       (blend + clamp)        (hardware component)
```

The component leaves any unclaimed joint unpowered, and `ros2_control` refuses a switch that gives
one joint two controllers, so `config/lowcmd_controllers.yaml` keeps all 29 motors claimed by
exactly one controller at every instant:

| Controller | Joints | When active |
|---|---|---|
| `locomotion_safety_controller`, with `agile_controller` chained above it | 12 legs + waist roll/pitch | always |
| `waist_freeze_controller` | waist yaw | always |
| `arm_freeze_controller` | the 14 arm joints | until the arm is acquired |
| `arm_trajectory_controller` | the same 14 | while the arm is acquired |
| `locomotion_freeze_controller` | 12 legs + waist roll/pitch | emergency only; loaded inactive |

`arm_freeze_controller` and `arm_trajectory_controller` claim identical joints and trade in one
`switch_controller` call, which `ros2_control` applies within a single update cycle; two calls
would leave the arms unclaimed, and so unpowered, in between. `locomotion_freeze_controller` claims
exactly the safety controller's joints: a wider set could not activate while the arm and waist are
owned, so the emergency switch would fail.

## Topics and services

| Name | Type | Used by | Notes |
|---|---|---|---|
| `/cmd_vel` (`cmd_vel_topic`) | `geometry_msgs/msg/Twist` | `agile_controller`, subscribes | Reliable, volatile, depth 1. |
| `~/inferring` | `std_msgs/msg/Bool` | `agile_controller`, publishes | Transient local, depth 1. False on activation, true from the first successful inference. |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | `locomotion_safety_controller`, calls | The emergency handover, sent off the update thread. |

## The policy contract

`policy/unitree_g1_velocity_e2e.onnx` is end-to-end:

- It is stateful. Seven of its twelve inputs (the last action and six histories) come back out as
  outputs, so the runner feeds them straight in again and keeps no ring buffers of its own.
- `action_joint_pos` is an absolute radian target, not a scaled action. The graph applies its own
  scale and default-pose offset.
- The gains come out of the graph, per joint, and are forwarded rather than configured.

Its two joint orderings differ from each other and from the SDK's motor order, so the controller
binds every interface by joint name. `AgilePolicy`'s constructor checks the model's input and
output names and counts, and throws on a mismatch. Inference is single-threaded on CPU with plain
`onnxruntime`, about 0.3 ms worst case against a 5 ms tick.

## Parameters

`G1AgileController`, with the values in `config/lowcmd_controllers.yaml`:

| Parameter | Value | Meaning |
|---|---|---|
| `model_path` | `""` | Empty resolves to the policy shipped in this package's share directory. |
| `cmd_vel_topic` | `/cmd_vel` | Velocity command source, Nav2's output. |
| `imu_sensor_name` | `imu` | Sensor whose orientation and angular velocity the policy observes. |
| `decimation` | 4 | Controller-manager ticks per inference: 50 Hz at the 200 Hz update rate. The controller warns if that is not the 50 Hz the policy was trained at. |
| `command_prefix` / `command_suffix` | `locomotion_safety_controller` / `_raw` | Chain target. Empty writes straight to the component. |
| `cmd_vel_timeout` | 0.5 | Seconds before a silent publisher is treated as a zero command; 0 disables. |
| `max_linear_speed` / `max_angular_speed` | 0.6 / 0.9 | Command clamps, m/s and rad/s; 0 disables. |

`G1SafetyController`:

| Parameter | Value | Meaning |
|---|---|---|
| `joints` | 12 legs + waist roll/pitch | Joints to blend. Their order fixes the reference-interface layout. |
| `blend_ratio` | 1.0 | 0 holds the activation pose, 1 follows the policy. Settable at runtime. The code default is 0: start first hardware bring-up there and raise it by hand. |
| `max_blend_ratio_speed` | 20.0 | Rate limit on that ratio, per second. Fast, so the blend completes before the simulator drops its startup pelvis weld. |
| `max_velocity` | `[-1.0]` | Per-joint rad/s clamp, one value or one per joint; non-positive leaves a joint unclamped. |
| `kp` / `kd` | `[100.0]` / `[2.0]` | Fallback gains: used where nothing upstream wrote, and for the hold after an emergency. |
| `mean_velocity_limit` / `max_velocity_limit` | 10.0 / 35.0 | Divergence thresholds in rad/s; non-positive disables that check. |
| `emergency_controller` | `locomotion_freeze_controller` | Switched in when the detector fires. Empty disables the switch. |

`G1FreezeController` takes `joints` and one `kp` and `kd` for all of them: 10/1 for
`waist_freeze_controller` and `locomotion_freeze_controller`, 300/4 for `arm_freeze_controller`.
The arm value matches the position-only gains of the shoulders and elbows, so the MoveIt handover
keeps their stiffness. `on_configure` rejects an empty joint list and non-positive gains.

## Running

```bash
ros2 launch g1_bringup sim.launch.py
```

The pelvis is unpinned by default, because the policy balances the robot. `pin_pelvis:=true`, for
simulation only, welds it and runs `locomotion_freeze_controller` in the policy's place.
`control.launch.py` activates the policy and its safety controller in one switch
(`--activate-as-group`): a chainable controller's reference interfaces only become claimable inside
the switch that activates it.

## What simulation does not validate

- Displacement. `test_agile_walk` runs with `sensors:=false`, so no simulator ground truth reaches
  ROS and it asserts uprightness rather than distance travelled. The gait envelope is measured
  against MuJoCo directly instead.
- Hardware timing. The update loop runs SCHED_FIFO (`thread_priority: 80`); whether the robot's own
  computer holds the 5 ms tick is unmeasured.
- `MotionSwitcherClient`. Entry to `rt/lowcmd` on a real G1 is untested; see
  [g1_hardware_interface](../g1_hardware_interface/README.md).

## Tests

```bash
./scripts/manage.sh test g1_controllers   # none of these needs a simulator
```

| Test | Covers |
|---|---|
| `test_agile_policy` | The ONNX contract and both joint tables, against the installed policy. |
| `test_safety_blend` | The blend-and-slew arithmetic, including that the rate clamp survives a blend-ratio step. |
| `test_freeze_pluginlib` | All three controllers resolve through pluginlib, the path `controller_manager` uses. |
| `test_joint_ownership` | Reads this package's controller config against `g1_description`'s joint list: every body motor has exactly one owner whether the arm is frozen or acquired, the arm pair claims identical joints, and the emergency freeze claims the safety controller's joints. |

Behaviour is covered by `g1_bringup`'s `test_agile_walk`, which stands the robot on the policy with
an unpinned pelvis, walks and turns it on command, and checks that the safety controller never
latched. It needs the simulator: see [g1_bringup](../g1_bringup/README.md#tests).
