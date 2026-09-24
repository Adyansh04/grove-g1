# Navigation and arm planning

Two demos that share a world. The first builds a map and drives to a goal under Nav2; the second
plans arm motions against an octomap built from the same LiDAR. They can run together.

Every command here runs inside the development container. Start it and open a shell from the
repository root:

```bash
./scripts/manage.sh start
./scripts/manage.sh exec
```

Then, in that shell:

```bash
cd /root/workspace && source install/setup.bash
```

## Simulator on its own

Useful for checking the robot stands up before adding anything else.

```bash
ros2 launch g1_bringup bringup.launch.py headless:=false
```

`headless:=false` opens the MuJoCo viewer and needs a local X11 display. Leave it at the default
`true` over SSH or on a machine without one. Restart the simulator rather than resetting or
reloading it from the viewer: Reset (Backspace) re-applies the weld that holds the robot up at
startup and nothing releases it again, and Reload with sensors on stops the sensors for good and
can crash the simulator.

## Building a map

`mode:=mapping` adds the scan pipeline and slam_toolbox.

```bash
ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true
```

Drive the robot from a second container shell and watch the map fill in. The gait ignores linear
commands below about 0.15 m/s.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

A map of the `navigation` world is already committed. Saving over it is only needed after
changing the scene:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/workspace/src/g1_navigation/maps/facility
```

## Localizing and navigating

`mode:=localization` runs map_server and AMCL against the committed map. `nav:=true` adds the Nav2
servers and the base approach, and requires `mode:=localization`: navigating against a map
slam_toolbox is still building moves the goal under the planner, so the launch refuses it.

```bash
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true
```

Send a goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}"
```

The committed map belongs to the `navigation` world, which is the default. Localization against
any other scene will not converge.

To rule odometry out rather than debug it, `odometry:=ground_truth` swaps FAST-LIO for exact
MuJoCo state. `fast_lio` is the default because it is what the real robot runs.

## Arm planning

`moveit:=true` starts move_group. Planning works immediately; executing a plan needs the arm
acquired first.

```bash
ros2 launch g1_bringup bringup.launch.py moveit:=true sensors:=true rviz:=true
```

`sensors:=true` puts the LiDAR octomap in the planning scene. The navigation modes, `manipulation`
and `perception` turn it on themselves, so it only has to be passed for a MoveIt-only run.

Acquiring takes the arm and both hands together; a hand that is absent or unpowered is logged and
the arm stays usable. It then swings both arms out to the sides, since where they hang at bring-up
is inside the octomap of anything in front, and MoveIt cannot plan from a start in collision.

```bash
ros2 launch g1_bringup activate_arm.launch.py
```

Now plan and execute from RViz's MotionPlanning panel, or send `FollowJointTrajectory` goals to
`arm_trajectory_controller`, `left_hand_controller` or `right_hand_controller`. Release when done:

```bash
ros2 launch g1_bringup deactivate_arm.launch.py
```

There are planning groups for each arm, both arms together, and each Dex3-1 hand. The SRDF names
arm poses such as `ready`, `tucked` and `carry`, and hand postures `open`, `pinch_ready` and
`closed`. Execute named arm poses one arm at a time: `both_arms` fails to execute them on this
stack.

## Both at once

```bash
ros2 launch g1_bringup bringup.launch.py \
  mode:=localization nav:=true moveit:=true rviz:=true \
  activate_arm:=true activate_arm_delay_s:=40.0 headless:=false
```

`activate_arm:=true` runs the acquire once the stack is up, a simulation convenience. The delay
has to cover the hardware component loading and `/lowstate` starting to flow; an acquire that runs
too early fails loudly.

With both MoveIt and Nav2 up, `rviz:=true` opens two windows: MoveIt's for the arm, and one on the
navigation config for the map and costmaps.

## Checking the graph

From a second container shell:

```bash
ros2 topic list -t
```

If a run behaves strangely for no visible reason, the usual cause is a previous stack still on the
DDS graph. This tears it down and exits non-zero unless the graph is empty afterwards:

```bash
./scripts/clean-stack.sh
```

## Arguments used here

| Argument | Default | Notes |
|---|---|---|
| `mode` | `none` | `mapping` or `localization` add g1_navigation. `nav:=true` requires `localization`. |
| `nav` | `false` | Nav2 servers and the base approach. |
| `moveit` | `false` | move_group. Works with any mode. |
| `sensors` | `false` | LiDAR sweep, relay and the odom chain. Forced on by the navigation modes, `manipulation` and `perception`. |
| `rviz` | `false` | Config follows the mode. |
| `odometry` | `fast_lio` | `ground_truth` reads MuJoCo state directly. |
| `world` | `navigation` | The facility the committed map was built from. |
| `headless` | `true` | `false` opens the MuJoCo viewer and needs a display. |
| `activate_arm` | `false` | Simulation convenience. Needs `moveit:=true`. |
| `activate_arm_delay_s` | `25.0` | Raise it for larger stacks. |
