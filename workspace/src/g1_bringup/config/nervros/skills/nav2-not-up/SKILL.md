---
name: nav2-not-up
description: Use when a walk or GoToPlace is refused because navigation is not ready (Nav2 inactive, or its bring-up aborted), usually just after the stack started. Not for a walk stopped by something in the way, which its failure names.
---

# Nav2 never came up

On this robot Nav2 failing at start is almost always the robot, not Nav2: it fell while the
simulator released it, and Nav2 aborts its bring-up 60 s later because odometry never settles.

1. Call `robot_state`. If `can_move` is false or `tilt_deg` is large, the robot is down: follow the
   `robot-fell` skill instead and stop here.
2. Call `log_tail` for `lifecycle_manager_navigation`. "Aborting bringup" about 60 s after
   "Activating controller_server" means the costmap never got its transform: the robot was down
   or odometry was not latched.
3. Call `log_tail` for `g1_odometry_publisher`. "Not latching the odom origin at N degrees" means
   the body was tilted at start: the same fall.
4. Call `log_tail` for `ros2_control_node` and look for the locomotion safety controller tripping
   ("joint velocity left the policy's trained range"): the arms moved while the robot settled.
5. If none of these shows, call `ros_graph` and check that `/bt_navigator`, `/controller_server`
   and `/planner_server` exist. Missing nodes, or many controllers "unconfigured", can mean two
   simulator stacks run at once.

Tell the operator what you found in two sentences. The fix for a fall or a half-started stack is
to stop it (`./scripts/clean-stack.sh`) and start it again; never plan a walk until `robot_state`
says the robot can move.
