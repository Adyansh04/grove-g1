# g1_description

Unitree G1 robot description: a vendored, kinematics-only URDF plus a xacro wrapper that adds the
`ros2_control` block for the 14 arm joints. No compiled code. `ament_cmake`, install-only.

## Contents

| Path | Purpose |
|---|---|
| `urdf/g1_29dof_with_hand_rev_1_0.urdf` | The vendored upstream description, unmodified. |
| `urdf/g1_arm_sdk.urdf.xacro` | Wraps the vendored URDF via `xacro:include` and appends a `<ros2_control>` block for the arms only. |
| `config/arm_sdk_params.yaml` | Every hardware-plugin tunable. Loaded with `xacro.load_yaml` and expanded into `<param>` tags, because Humble hardware plugins only receive parameters that way. |
| `test/test_arm_sdk_xacro.py` | Expands the xacro, validates with `check_urdf`, asserts the `<ros2_control>` block holds exactly the 14 arm joints. |

## Inspecting the model

```bash
colcon build --symlink-install --packages-select g1_description
source install/setup.bash

xacro $(ros2 pkg prefix g1_description)/share/g1_description/urdf/g1_arm_sdk.urdf.xacro \
  -o /tmp/g1_arm_sdk.urdf
check_urdf /tmp/g1_arm_sdk.urdf
```

`check_urdf` prints the link tree and confirms the document parses. It does not need meshes. To
look at actual geometry, use the MuJoCo GUI once `g1_bringup` launches the sim; that view loads
`unitree_mujoco`'s own MJCF, independent of this package.

```bash
colcon test --packages-select g1_description
colcon test-result --verbose
```

## Scope: arms only

The `<ros2_control name="G1ArmSdkSystem" type="system">` block exports command and state interfaces
for exactly the 14 arm joints (7 per arm: shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw).
Waist, legs and hands are deliberately absent and stay under the onboard controller.

`rt/arm_sdk` is a weight-blended channel that lets an external command drive the arms while the
onboard controller keeps the legs balanced. Commanding waist, legs or hands through the same system
would either do nothing or fight the balance controller. One `ros2_control` System per low-level
channel, one publisher per channel, is the invariant this scope preserves.

**Hand joints are present but inert.** The DEX3 joints stay in the model for correct TF structure
ahead of the hand-control milestone, but get no `ros2_control` interfaces: the hand has its own
command API on the real robot. `unitree_mujoco`'s G1 MJCF has no hand joints and reports no hand
feedback, so those TF frames have no live source until that milestone.

**No meshes.** The vendored URDF references meshes as plain relative paths, and they are not
vendored here. This description exists for `robot_state_publisher`, `controller_manager` and TF,
which need only link and joint kinematics. Meshes arrive when a later milestone needs them for
RViz or MoveIt planning-scene collision checking. Until then any tool that tries to resolve those
paths will fail to find them, which is expected.

## Parameters (`config/arm_sdk_params.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `command_publish_rate` | 100.0 Hz | `/arm_sdk` publish rate, independent of the `controller_manager` update rate. |
| `blend_ramp_up_s` | 2.0 s | Weight eases 0 to 1 on activate. |
| `blend_ramp_down_s` | 2.0 s | Weight eases 1 to 0 on a clean deactivate. |
| `emergency_ramp_down_s` | 0.5 s | Faster ramp on stale feedback or shutdown. Must fit inside the launch stack's SIGTERM window. |
| `max_joint_velocity_rad_s` | 1.0 rad/s | Slew clamp on every commanded joint. |
| `lowstate_timeout_ms` | 100 ms | `/lowstate` age beyond this trips the emergency ramp. |

Per-joint motor indices come from Unitree's `G1Arm7JointIndex`: legs 0-11, waist 12-14, left arm
15-21, right arm 22-28. The gains are Unitree's example-conservative values, a sim-safe starting
point.

| Joint | motor_index | kp | kd |
|---|---|---|---|
| `left_shoulder_pitch_joint` | 15 | 40 | 1 |
| `left_shoulder_roll_joint` | 16 | 40 | 1 |
| `left_shoulder_yaw_joint` | 17 | 40 | 1 |
| `left_elbow_joint` | 18 | 40 | 1 |
| `left_wrist_roll_joint` | 19 | 25 | 1 |
| `left_wrist_pitch_joint` | 20 | 25 | 1 |
| `left_wrist_yaw_joint` | 21 | 25 | 1 |
| `right_shoulder_pitch_joint` | 22 | 40 | 1 |
| `right_shoulder_roll_joint` | 23 | 40 | 1 |
| `right_shoulder_yaw_joint` | 24 | 40 | 1 |
| `right_elbow_joint` | 25 | 40 | 1 |
| `right_wrist_roll_joint` | 26 | 25 | 1 |
| `right_wrist_pitch_joint` | 27 | 25 | 1 |
| `right_wrist_yaw_joint` | 28 | 25 | 1 |

These are tunable in the YAML only, never hardcoded in the xacro or in plugin code.

## Vendored URDF provenance

- **Upstream:** https://github.com/unitreerobotics/unitree_ros, path `robots/g1_description/`
- **Commit:** `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`
- **Variant:** `g1_29dof_with_hand_rev_1_0.urdf`

Unitree ships several hand-equipped G1 variants. This one was chosen because
`g1_29dof_lock_waist_with_hand_rev_1_0.urdf` fixes `waist_roll` and `waist_pitch`, giving only 27
mobile joints; `g1_29dof_with_hand.urdf` carries an extra `waist_support_joint` not present in the
`_rev_1_0` line; and the hand joints here are Unitree's DEX3 three-finger hand rather than the
separate Inspire-hand URDFs also present upstream. All 3 waist joints are `revolute`, so this is
the full 29-DoF body plus DEX3 hands.

### Licence

Upstream `unitree_ros` is BSD-3-Clause:

```
BSD 3-Clause License

Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

This package's own files (xacro wrapper, YAML, build files) are BSD-3-Clause to match.
