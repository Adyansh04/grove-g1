# g1_description

Unitree G1 robot description for the arm-bridge bring-up milestone: a vendored, kinematics-only
URDF plus a xacro wrapper that adds the `ros2_control` block for the 14 arm joints. No compiled
code — `ament_cmake`, install-only.

## What's here

- `urdf/g1_29dof_with_hand_rev_1_0.urdf` — the vendored upstream description, **unmodified**.
- `urdf/g1_arm_sdk.urdf.xacro` — wraps the vendored URDF (via `xacro:include`, which splices in
  its links/joints and discards its own `<robot>` root) and appends a `<ros2_control>` block for
  the arms only.
- `config/arm_sdk_params.yaml` — every hardware-plugin tunable (ramp times, gains, motor index
  map). Loaded into the xacro via `xacro.load_yaml` and expanded into `<param>` tags — Humble
  hardware plugins only receive parameters this way; `controller_manager`'s own YAML never
  reaches them.
- `test/test_arm_sdk_xacro.py` — expands the xacro, validates it with `check_urdf`, and asserts
  the `<ros2_control>` block contains exactly the 14 arm joints with the right interfaces/params.

## Vendored URDF: provenance

- **Upstream:** `https://github.com/unitreerobotics/unitree_ros`, path `robots/g1_description/`.
- **Commit:** `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`.
- **Variant vendored:** `g1_29dof_with_hand_rev_1_0.urdf`.

Unitree ships several G1 hand-equipped variants in that directory. `g1_29dof_with_hand_rev_1_0.urdf`
was picked over the alternatives because:
- `g1_29dof_lock_waist_with_hand_rev_1_0.urdf` fixes `waist_roll`/`waist_pitch` (`type="fixed"`),
  giving only 27 mobile joints — not the full 29-DoF body.
- `g1_29dof_with_hand.urdf` (no `_rev_1_0` suffix) carries an extra `waist_support_joint`/link not
  present in the `_rev_1_0` line shared by the plain (`g1_29dof_rev_1_0.urdf`) and lock-waist
  revisions — `_rev_1_0` is the current, consistently-versioned revision across those siblings.
- The hand joints in this file (`*_thumb_0/1/2`, `*_index_0/1`, `*_middle_0/1`, 7 movable joints
  per hand plus a fixed palm mount) are Unitree's **DEX3** three-finger dexterous hand — the
  hand-equipped variant this milestone's design decision calls for, as opposed to the separate
  Inspire-hand URDFs also present upstream (`*_with_inspire_hand_*`, `inspire_hand/*.urdf`).

All 3 waist joints are `revolute` (full 29-DoF), matching `g1_29dof_rev_1_0.urdf`'s body plus the
DEX3 hands.

### License

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

`g1_description`'s own files (the xacro wrapper, YAML config, package/build files) are licensed
BSD-3-Clause to match and stay compatible with the vendored content.

## Kinematics-only: no meshes

The vendored URDF's `<visual>` tags reference mesh files as plain relative paths
(`meshes/*.STL`), not `package://` URIs — either way, the meshes themselves are **not vendored**
in this milestone. Visual/collision checks happen in the MuJoCo GUI (which loads its own MJCF, not
this URDF); this description exists for `robot_state_publisher`/`controller_manager`/TF, which only
need link/joint kinematics and don't touch `<visual>`/`<collision>` geometry. Meshes arrive when a
later milestone needs them for RViz/MoveIt (planning-scene collision checking, visualization). Until
then, any tool that *does* try to resolve those mesh paths will fail to find them — expected and
harmless for this milestone's use.

## ros2_control scope: arms only

The `<ros2_control name="G1ArmSdkSystem" type="system">` block in `g1_arm_sdk.urdf.xacro` exports
command/state interfaces for exactly the 14 arm joints (7 per arm: shoulder pitch/roll/yaw, elbow,
wrist roll/pitch/yaw). Waist, legs, and hands are **deliberately absent** — they stay under the
onboard controller. The G1's arm-control interface (`rt/arm_sdk`) is a weight-blended channel that
lets an external command drive the arms while the onboard controller keeps the legs balanced;
commanding waist/legs/hands through this same system would either do nothing (wrong interface) or
fight the balance controller. One `ros2_control` System per low-level channel, one publisher per
channel, is the safety invariant this package's scope preserves.

### Hand joints: present but inert this milestone

The vendored URDF's DEX3 hand joints stay in the model (for correct TF/kinematics structure ahead
of the hand-control milestone) but get **no `ros2_control` interfaces**: the hand has its own
separate command API on the real robot, out of scope here. In `unitree_mujoco`'s G1 MJCF, the
simulated robot has no hand joints and reports no hand feedback, so hand joint states have no
sim-side source; their TF frames won't resolve to a live pose until the hand milestone gives them
one (a static bringup default, or dropping them from the runtime model, whichever that milestone's
design favors — not decided here).

## Parameters (`config/arm_sdk_params.yaml`)

System-level:

| Param | Default | Meaning |
|---|---|---|
| `command_publish_rate` | `100.0` Hz | Throttle on `/arm_sdk` publication (independent of the `controller_manager` update rate). |
| `blend_ramp_up_s` | `2.0` s | Weight eases `0 -> 1` over this duration on activate. |
| `blend_ramp_down_s` | `2.0` s | Weight eases `1 -> 0` over this duration on a clean deactivate. |
| `emergency_ramp_down_s` | `0.5` s | Faster ramp-down on stale feedback or shutdown; must fit inside the launch stack's SIGTERM window. |
| `max_joint_velocity_rad_s` | `1.0` rad/s | Slew clamp applied to every commanded joint. |
| `lowstate_timeout_ms` | `100` ms | `/lowstate` age beyond this trips the emergency ramp-down. |

Per-joint (motor index into the LowCmd/LowState motor array, from Unitree's `G1Arm7JointIndex`
enum — legs occupy 0-11, waist 12-14, left arm 15-21, right arm 22-28; kp/kd are Unitree's
example-conservative gains, a sim-safe starting point):

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

All of these are tunable in the YAML only — never hardcoded in the xacro or in plugin code.

## Inspecting the model

Build the package, then expand the xacro and validate it with `check_urdf` (both ship with the
Humble desktop install):

```bash
colcon build --symlink-install --packages-select g1_description
source install/setup.bash

xacro $(ros2 pkg prefix g1_description)/share/g1_description/urdf/g1_arm_sdk.urdf.xacro \
  -o /tmp/g1_arm_sdk.urdf
check_urdf /tmp/g1_arm_sdk.urdf
```

`check_urdf` prints the link tree and confirms the document parses; it doesn't need meshes to
succeed. To eyeball the actual robot geometry, use the MuJoCo GUI once `g1_bringup` launches the
sim — that view loads `unitree_mujoco`'s own MJCF, independent of this package.

To run this package's own test suite (xacro validity, exact 14-joint `ros2_control` export,
ruff/pep257/CMake lint):

```bash
colcon test --packages-select g1_description
colcon test-result --verbose
```
