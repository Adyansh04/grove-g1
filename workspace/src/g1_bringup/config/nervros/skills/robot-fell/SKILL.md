---
name: robot-fell
description: Use when robot_state says the robot cannot move, its tilt is large, or a mission was stopped because the robot fell or can no longer walk.
---

# The robot fell or cannot walk

A humanoid that is down must not be asked to walk: the executor refuses any plan that moves the
base while `can_move` is false, and stops a walk that is running when the robot falls.

1. Call `robot_state` and read `cannot_move_reason` and `tilt_deg`.
2. Above about 45 degrees of tilt the robot is down. In simulation this most often happens at start,
   when the arms move as the simulator releases the robot (see the `nav2-not-up` skill).
3. If it says a controller is not running, call `log_tail` for `ros2_control_node` to find which
   one stopped and why.
4. Do not plan or retry any mission that moves the base or the arms. A hand that holds something
   keeps its grip; do not ask to open it.

Tell the operator plainly that the robot is down or cannot walk, why, and that it needs to be
reset: in simulation, stop the stack (`./scripts/clean-stack.sh`) and start it again.
