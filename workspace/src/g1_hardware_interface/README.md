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
