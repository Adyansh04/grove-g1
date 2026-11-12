# Navigation and arm planning

Two demos that share a world. The first builds a map and drives to a goal under Nav2; the second
plans arm motions against an octomap built from the same LiDAR. They can run together.

`./scripts/demos/navigation-and-moveit.sh <sim|mapping|navigate|arm|both>` tears down any stack
left running, then opens a section's commands at once, in split panes of tilix (a window each in
another terminal). `stop` ends it, and `--print` lists the commands instead. Arguments after the
section, such as `rviz:=false`, go to the bringup launch.

Every command here runs inside the development container. Start it and open a shell from the
repository root; the shell starts in `/root/workspace` with the workspace sourced:

```bash
./scripts/manage.sh start
./scripts/manage.sh exec
```

## Simulator on its own

Useful for checking the robot stands up before adding anything else.

```bash
ros2 launch g1_bringup bringup.launch.py headless:=false
```

`headless:=false` opens the MuJoCo viewer and needs a local X11 display, as `rviz:=true` does. The
launcher refuses either until the container may use the display, which `manage.sh start` grants
(`xhost +local:docker`). Leave `headless` at the default `true` over SSH or on a machine without a
display.

Restart the simulator rather than resetting or reloading it from the viewer:

- Reset (Backspace) re-applies the weld that holds the robot up at startup, and nothing releases it
  again.
- Reload with sensors on stops the sensors for good and can crash the simulator.

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

`mode:=localization` runs map_server and AMCL against the world's committed map. `nav:=true` adds
the Nav2 servers and the base approach, in either navigation mode.

Trees with fixed goals, such as the pick-and-place missions, want `localization`: on a map that
slam_toolbox is still building, a loop closure can move the goal under the planner. Exploration
runs on `mapping` ([world-model.md](world-model.md)).

```bash
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true
```

Send a goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}"
```

Route server: a world with a graph, `g1_navigation/config/route_graph_<world>.geojson` (the
apartment has one), also runs `route_server` under its own `lifecycle_manager_route`, for NervROS's
walks along fixed lanes. A graph that will not load leaves Nav2 up without them.

Keep-out zones: `keepout:=true` turns on Nav2's keepout filter in both costmaps, and starts its
`costmap_filter_info_server` with its own `lifecycle_manager_filters`, which exist only with it.
The filter reads the `OccupancyGrid` on `/keepout_filter_mask`, whose cells at 100 are off limits.
NervROS publishes it from its zones ([NervROS](nervros.md#keep-out-zones)). Leave `keepout` off
otherwise: with no mask, the filters warn every 2 s.

Committed maps exist for the `navigation` world, the default, and the `apartment` world.
Localization picks the one for `world:=`, or the file `map:=<map.yaml>` names; on any other scene
it will not converge.

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

Acquiring takes the arm and both hands together. A hand that is absent or unpowered is logged, and
the arm stays usable. The acquire then swings both arms out to the sides: where they hang at
bring-up is inside the octomap of anything in front, and MoveIt cannot plan from a start in
collision.

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

`activate_arm:=true` runs the acquire once the stack is up, a simulation convenience. It waits up
to a minute for the controllers that hold the body, then 3 s for the balance policy to settle,
before the arms move.

With both MoveIt and Nav2 up, `rviz:=true` opens two windows: MoveIt's for the arm, and one on the
navigation config for the map and costmaps.

## On the real robot

The robot's side (control, the LiDAR and FAST-LIO) runs on the G1's Orin. Nav2, MoveIt and RViz run
on this machine and reach the robot through `scripts/robot.sh operator`, which runs a command in the
development container on the robot's ROS domain. Two launcher sections open everything at once,
with the robot hanging in its stand, feet on the ground:

```bash
./scripts/demos/navigation-and-moveit.sh robot-mapping
./scripts/demos/navigation-and-moveit.sh robot-navigate map:=/root/data/maps/<name>.yaml
```

Both also start the robot's side: its container, control, which turns the waist straight as it
starts, and the sensors; the other panes wait on what they need. Press Enter in the `stand` pane to
hand the legs to the policy; it reloads the policy's controllers if their spawner hung. Then take
the stand away.

- `robot-mapping` adds SLAM, RViz and teleop (the teleop pane waits until the robot stands). Walk
  the robot round slowly, then press Enter in the `save map` pane: it saves
  `data/maps/map_<date>_<time>`, outside the repository. The scan's 0.30 to 1.50 m band sees legs, so
  the map keeps the footsteps of anyone who walked beside the robot: set those specks to free (254)
  in the `.pgm` before navigating on it.
- `robot-navigate` adds MoveIt, AMCL and Nav2 on that map, and RViz for each. Give the robot its
  start with 2D Pose Estimate in the navigation RViz, then send goals with 2D Goal Pose. Press
  Enter in the `arm` pane to hand the arms to MoveIt before it executes. MoveIt starts first
  because on the robot the collision monitor's high band reads its filtered cloud: the raw one
  holds the robot's own head ([g1_navigation](../../workspace/src/g1_navigation/README.md)).

`stop` ends this machine's side. With the robot back in its stand, stop the robot's side; it asks
first, puts the arms and legs back on their freezes, and releases the motors:

```bash
./scripts/demos/navigation-and-moveit.sh stop
./scripts/robot.sh release
```

MoveIt's model carries the Dex3 hands, so on a robot without them (`hands:=false`) it lacks their
joint states and does not plan.

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
| `mode` | `none` | `mapping` or `localization` add g1_navigation. `nav:=true` needs one of them. |
| `nav` | `false` | Nav2 servers and the base approach. |
| `moveit` | `false` | move_group. Works with any mode. |
| `sensors` | `false` | LiDAR sweep, relay and the odom chain. Forced on by the navigation modes, `manipulation` and `perception`. |
| `rviz` | `false` | Config follows the mode. |
| `odometry` | `fast_lio` | `ground_truth` reads MuJoCo state directly. |
| `world` | `navigation` | `navigation` (the facility) or `apartment`; each has a committed map for `mode:=localization`. |
| `keepout` | `false` | Nav2's keepout filter in both costmaps, fed by NervROS's `/keepout_filter_mask`. |
| `headless` | `true` | `false` opens the MuJoCo viewer and needs a display. |
| `activate_arm` | `false` | Simulation convenience. Needs `moveit:=true`. |
| `activate_arm_delay_s` | `25.0` | When the acquire starts waiting for the body's controllers. |
