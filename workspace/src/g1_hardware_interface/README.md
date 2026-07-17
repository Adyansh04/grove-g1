# g1_hardware_interface

`ros2_control` `SystemInterface` plugin bridging standard joint command/state interfaces onto the
Unitree G1's weight-blended `rt/arm_sdk` DDS channel, for the 14 arm joints only. Legs, waist, and
hands stay under the onboard controller at all times -- this component never touches them.

`ament_cmake`, C++17.

## Status

This README grows alongside the package's four build commits (plugin skeleton, `/lowstate`
feedback, `/arm_sdk` command path with the ramp/slew safety engine, unit tests). Sections below are
filled in as each lands.

## Layout

- `include/g1_hardware_interface/arm_ramp_engine.hpp`, `src/arm_ramp_engine.cpp` -- the pure,
  ROS-free ramp/slew safety logic (blend-weight ramping, per-joint slew clamping, motor-index
  validation). No ROS includes, so it's directly unit-testable without a live hardware component.
- `include/g1_hardware_interface/g1_arm_sdk_system.hpp`, `src/g1_arm_sdk_system.cpp` -- the
  `hardware_interface::SystemInterface` plugin itself: parameter parsing, lifecycle, DDS I/O, and
  the concurrency contract that keeps exactly one thread publishing at a time.
- `g1_hardware_interface.xml` -- pluginlib export description.
- `test/` -- gmock/gtest suite.

## Building and testing

```bash
colcon build --symlink-install --packages-select g1_hardware_interface
colcon test --packages-select g1_hardware_interface
colcon test-result --verbose
```

## Exported plugin

`g1_hardware_interface/G1ArmSdkSystem`, declared with `type="system"` in the robot's URDF
(`g1_description`'s `g1_arm_sdk.urdf.xacro` wires this up already).

## Topics

| Topic | Direction | Type | QoS | Why |
|---|---|---|---|---|
| `/lowstate` | in | `unitree_hg/msg/LowState` | best-effort, keep-last(1), volatile | Compatible with either a best-effort or reliable publisher (the sim happens to publish RELIABLE; hardware may differ) and only the newest of the ~500-900 Hz stream ever matters. |

`/arm_sdk` (the command side) lands with the next commit.

## Threading model (so far)

`on_configure` starts a hidden `rclcpp::Node` (uniquely suffixed name, never added to the
`controller_manager`'s own executor) with its own `SingleThreadedExecutor` on a dedicated thread,
whose only job right now is servicing the `/lowstate` subscription into a
`realtime_tools::RealtimeBuffer`. `read()` (the `controller_manager`'s RT thread) drains that
buffer with `readFromRT()`, which never blocks. `on_cleanup` and the destructor both cancel the
executor and join the thread through the same idempotent teardown path -- `on_configure` itself
tears down and rebuilds from scratch, since it can run more than once per process (e.g. after an
error-triggered reset to `UNCONFIGURED`).

## Sim validation notes (manual, this commit)

Checked directly against `unitree_mujoco` (G1, headless) with a scratch `ros2_control_node` +
`g1_description`'s URDF + `joint_state_broadcaster`:

- `/lowstate` arrives at ~900 Hz in sim; `read()` maps each arm joint's `motor_state[motor_index]`
  into the exported `position`/`velocity`/`effort` state interfaces correctly -- `/joint_states`
  shows real, non-zero measured arm positions at the `controller_manager`'s configured 200 Hz.
- `controller_manager` calls `read()`/`write()` on every loaded hardware component every cycle
  regardless of lifecycle state (not just while active) -- confirmed directly, and the reason
  `read()` doesn't return `ERROR` on stale feedback yet (see the code comment): doing so
  unconditionally reset the component to `UNCONFIGURED` with no automatic recovery, because
  `resource_manager` reacts to any `read()`/`write()` `ERROR` by driving the hardware through
  `on_error`. That check is deferred to the commit that gives this component a real "active"
  concept to gate it on.
- Killing the process (SIGTERM) while active runs `on_deactivate` **then** `on_shutdown`, not
  `on_shutdown` alone -- confirmed directly. Deactivating cleanly first also runs `on_shutdown`
  afterwards regardless of the component already being `UNCONFIGURED`. Both are relevant to how the
  next commit's ramp-down contract behaves on process shutdown.
