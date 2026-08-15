# g1_controllers

`ros2_control` controllers for the whole-body `rt/lowcmd` stack. `ament_cmake`, C++20.

| Controller | Does |
|---|---|
| `g1_controllers/G1FreezeController` | Captures each claimed joint's position on activation and holds it there under impedance control. |

The AGILE policy controller and its safety wrapper land here too.

## Why a freeze controller exists

`rt/lowcmd` is one message covering all 29 motors, so the hardware component sends something for
every joint on every tick, claimed or not. Its answer for unclaimed joints is "unpowered", which
is correct but means nothing holds the robot up when no policy is running.

Holding is therefore a controller, not component behaviour: it can be switched in and out at
runtime, and the component stays free of policy. This mirrors NVIDIA's
`isaac_ros_deploy_ros2_control` split, where `FreezeController` holds and `DisableController`
lets go. Ours drops their regex gain patterns, because their own G1 config sets one value for
every joint anyway.

## Parameters

| Parameter | Meaning |
|---|---|
| `joints` | Which joints to hold. Must be non-empty. |
| `kp` | Position gain applied to all of them. Must be > 0. |
| `kd` | Damping applied to all of them. Must be > 0. |

Both gains must be positive: a freeze with no stiffness is a disable with extra steps, and
`on_configure` rejects it rather than silently letting the robot down.

`config/lowcmd_controllers.yaml` ships the bring-up set — this controller over 28 joints, plus a
`forward_command_controller` on one probe joint so a single command can be pushed through the
whole path and read back.

## Running

Comes up with the lowcmd stack:

```bash
ros2 launch g1_bringup sim.launch.py control_stack:=lowcmd pin_pelvis:=true
```

`pin_pelvis` matters until the policy controller lands: this holds joints, it does not balance.

## Tests

`test_freeze_pluginlib` proves the plugin resolves through pluginlib, which is the path
`controller_manager` uses to spawn it. Behaviour is covered by `g1_bringup`'s
`test_lowcmd_joint`, which asserts a held joint does not drift while a commanded one moves.
