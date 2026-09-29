# g1_moveit_config

MoveIt 2 planning for the G1's two 7-DoF arms and its Dex3 hands, layered on the controllers the
control stack already runs. `ament_cmake`, configuration and launch only: `move_group` and
`servo_node` are upstream.

```mermaid
flowchart LR
    JS["/joint_states<br/>body + fingers"] --> MG["move_group<br/>plan + collision check"]
    MG -- "FollowJointTrajectory" --> JTC["arm_trajectory_controller"]
    MG -- "FollowJointTrajectory" --> HC["left/right_hand_controller"]
    POL["balance policy<br/>legs + waist roll, pitch"] --> HW["G1LowCmdSystem"]
    JTC --> HW
    HC --> HH["G1Dex3System (one per hand)"]
    HW -- "rt/lowcmd" --> SIM["unitree_mujoco<br/>(the robot on hardware)"]
    SIM -- "rt/lowstate" --> HW
    HH -- "rt/dex3/side/cmd" --> SIM
```

MoveIt adds no command path. It is another client of actions the controllers already serve, so
the arm channel and each hand's topic keep exactly one writer.

## Launch files

| File | Starts |
|---|---|
| `move_group.launch.py` | `move_group`, planning against the description `robot_state_publisher` loads. With `servo:=true` (default `false`) it also starts `servo_node`, which streams onto the controller `move_group` executes through, so do not run it during a planned motion. `bringup.launch.py` sets it with `vla_execution_mode:=servo`. Nothing sim-specific. |
| `moveit_sim.launch.py` | `g1_bringup`'s `sim.launch.py` plus `move_group`. Arguments: `headless` (`true`), `world` (`navigation`), `sensors` (`false`), `pin_pelvis` (`false`), `sim_start_delay_s` (`4.0`). |
| `moveit_rviz.launch.py` | RViz with the MotionPlanning panel on `both_arms`. Argument: `rviz_config`, default `config/g1_moveit.rviz`. |

## Config

| File | Holds |
|---|---|
| `g1.srdf` | Planning groups, end effectors, named poses, collision matrix. |
| `kinematics.yaml` | `pick_ik` per arm, none for `both_arms`. |
| `joint_limits.yaml` | The velocity and acceleration limits trajectories are timed against. |
| `moveit_controllers.yaml` | The three `FollowJointTrajectory` controllers; `moveit_manage_controllers: false`. |
| `ompl_planning.yaml` | RRTConnect on the arm groups. |
| `sensors_3d.yaml` | `/livox/lidar` into a 25 mm octomap. |
| `servo.yaml` | MoveIt Servo on `both_arms`. Takes `control_msgs/JointJog` on `/servo_node/delta_joint_cmds` and `geometry_msgs/TwistStamped` on `/servo_node/delta_twist_cmds`, and streams to `/arm_trajectory_controller/joint_trajectory`. |

## Arm ownership

The hardware component leaves any unclaimed joint unpowered, so something always holds the arms:
`arm_freeze_controller` until the arm is acquired, then `arm_trajectory_controller`. The acquire
swaps the two in a single switch, so the arms are never unowned.
[`g1_controllers`](../g1_controllers/README.md) has the full ownership table.

The hands are separate components on their own SDK channels (`rt/dex3/<side>/{cmd,state}`).
`activate_arm` brings them up after the arm, best-effort: `G1Dex3System` refuses to activate
without hand state, so an absent hand stays inactive and the arm remains usable.

## Planning groups

| Group | Joints | Chain |
|---|---|---|
| `left_arm` | 7 | `torso_link` to `left_hand_palm_link` |
| `right_arm` | 7 | `torso_link` to `right_hand_palm_link` |
| `both_arms` | 14 | the two above, composed |
| `left_hand` | 7 | the Dex3 fingers, a tree rather than a chain |
| `right_hand` | 7 | as above |

`both_arms` is the only group that collision-checks one arm against the other and times a motion
so both hands arrive together. The per-arm groups stay because a 7-DoF search is far cheaper.

The hands are their own groups, never joints of an arm group: a hand is a separate device with its
own topics and control authority. Each is its arm's `end_effector` (`left_gripper`,
`right_gripper`), which lets `attachObject` derive touch links and RViz offer the hand as the arm's
gripper. `test_robot_model` pins both.

The arm groups are rooted at `torso_link`, not `pelvis`. `waist_freeze_controller` owns waist yaw
and the balance policy owns waist roll and pitch, so a group spanning them would plan motion
nothing executes.

The planning frame is `pelvis`, because the vendored URDF's floating base is commented out and the
SRDF declares no virtual joint. That is fine while the robot stands still to manipulate. Scene
objects fixed in `odom` would need a virtual joint.

## Named poses

Set a pose from the MotionPlanning panel's goal-state dropdown, or with
`move_group.setNamedTarget("tucked")`. Each arm pose exists for `left_arm`, `right_arm` and
`both_arms`.

| Pose | What it is |
|---|---|
| `home` | The arms' resting posture: nearly straight, angled forward, hands in front of the hips. |
| `zero` | Every arm joint at 0: upper arms down, forearms level and pointing forward, since the elbow's zero is a right angle. |
| `tucked` | Arms down, nearly straight, swung out clear of the hips. The posture to walk in. |
| `ready` | Hands forward of the hips, elbows slightly bent, clear of the torso. |
| `reach_front` | Hands out in front, just below shoulder height. |
| `carry` | Holds a picked object low beside the body, wrist flat. The in-place trees use it between pick and place. |

`test_robot_model` checks that the three per-group copies agree, that every pose sits inside the
joint limits, and that `tucked` and `carry` keep 0.20 rad of room before self-collision.

Arm posture disturbs the gait, so manipulate standing still and walk in `tucked`. Send named poses
one arm at a time: `both_arms` fails to execute them on this stack (see the
[navigation and arm planning guide](../../../docs/guides/navigation-and-moveit.md)).

The hands have three postures each, on `left_hand` and `right_hand`:

| Pose | What it is |
|---|---|
| `open` | Every finger joint at 0: fingers straight, thumb mid-range. |
| `pinch_ready` | What a pick descends in: fingers open, thumb pulled back behind the object's near face. |
| `closed` | The thumb swung forward against the curled fingers, short of the end stops so both can press into an object. |

At `open` the thumb reaches the table before the object does. Even retracted in `pinch_ready` the
hand hangs 113 mm below the palm origin, 63 mm below the grasp frame, which is the clearance
`g1_manipulation`'s `min_grip_height_m` exists for (0.080 m by default; `g1_bringup` sets it per
world). [`g1_manipulation`](../g1_manipulation/README.md) describes how a pick and place uses the
postures.

`closed` leaves thumb abduction (`thumb_0`) at zero: abducted, the thumb grazes the middle finger
late in the close, which stalls a finger in an empty hand and reads as a grasp to the grip check.

MoveIt's attachment is planner bookkeeping, not what carries the load. The fingers hold an object
by friction, with a simulator grasp weld as a stand-in in the navigation world
([pick and place guide](../../../docs/guides/pick-and-place.md)).

## Kinematics

`pick_ik` solves each arm. With 7 joints against a 6-DoF pose the solver picks from a null space,
and KDL's pseudo-inverse wanders it and clamps at joint limits. Swapping back is one line in
`config/kinematics.yaml`.

`both_arms` has no solver entry, on purpose. Chain solvers, `pick_ik` included, reject a group that
is not a chain. MoveIt instead routes a pose goal per hand through the per-arm solvers, and builds
that subgroup map only for groups with no solver of their own. An entry for `both_arms` silently
breaks its IK, and `test_moveit_config_drift` asserts there is none.

For two simultaneous Cartesian goals, call `setPoseTarget(pose, link)` once per hand.
`setPoseTargets` means something else: alternative goals for one link.

## Speed

`config/joint_limits.yaml` caps every arm joint at 0.8 rad/s and every finger at 2.0 rad/s. It also
adds the acceleration limits the URDF does not declare (2.0 rad/s^2 on the arms, 5.0 on the
fingers). Without them a plan succeeds with zero timestamps and fails only at execution.

The URDF's 22 to 37 rad/s are motor limits, not what the arm tracks. `arm_trajectory_controller`
claims position alone, so the arm runs on the component's `position_only_kp` (300 on the shoulders
and elbows, 150 on the wrists) with no gravity compensation. It falls behind a trajectory timed
against the motor, and the controller aborts on its goal-time tolerance.

The fingers have a real clamp: `G1Dex3System` slew-limits every finger at
`max_joint_velocity_rad_s` (3.0), so planning faster only stretches the motion until the goal-time
tolerance trips. `test_moveit_config_drift` keeps every finger limit under it. The body component
has no such clamp, deliberately: it would throttle the fast corrections the balance policy needs.

## Running

From the operator entry point:

```bash
ros2 launch g1_bringup bringup.launch.py moveit:=true pin_pelvis:=true rviz:=true
```

Or this package on its own (the integration tests launch `moveit_sim.launch.py`):

```bash
ros2 launch g1_moveit_config moveit_sim.launch.py pin_pelvis:=true
ros2 launch g1_moveit_config moveit_rviz.launch.py   # second shell, once the stack is up
```

`./scripts/demos/navigation-and-moveit.sh arm` launches `moveit:=true sensors:=true rviz:=true`,
acquires the arm once the stack is up and stages the release command.

`move_group` will not plan until every joint it models has a state; `joint_state_broadcaster`
publishes all of them, body and fingers.

With Nav2 as well (`mode:=localization nav:=true moveit:=true rviz:=true`), `rviz:=true` opens two
windows: this package's for the arms and `g1_navigation.rviz` for the map and costmaps. One
combined window segfaults rviz2 once Nav2 is running.

Planning works immediately. Execution waits until the arm is acquired with
`ros2 launch g1_bringup activate_arm.launch.py`; release with `deactivate_arm.launch.py` on
success or failure alike. Until then the controller refuses the goal, which is intended:
`moveit_manage_controllers` is false, so MoveIt never activates anything itself.

## Seeing the world

`config/sensors_3d.yaml` feeds `/livox/lidar` into an octomap, so plans route around real
obstacles. It comes up with `sensors:=true`:

```bash
ros2 launch g1_moveit_config moveit_sim.launch.py sensors:=true pin_pelvis:=true world:=perception
```

The octomap is built in the planning frame, `pelvis`; `octomap_frame` is never read. That holds
while the pelvis is pinned or the robot stands still. A walking pelvis drags the voxel grid with it
and the map smears. There is no time decay, so a voxel the sensor cannot currently see is never
forgotten; `/clear_octomap` (`std_srvs/Empty`) resets the map.

The updater reads the LiDAR, not the depth camera: `depth_image_proc`'s converter cannot receive
the camera relay's best-effort images.

## Collision matrix

The `disable_collisions` block in `config/g1.srdf` is not the Setup Assistant's:
`collisions_updater` does not finish on this model, because 38 of the URDF's 52 collision elements
are full visual meshes, about 525k triangles. The block holds 56 pairs, none sampled: links joined
by a joint, links touching at rest (from `/check_state_validity`), and each hand's thumb-over-wrist
pair, which a closing hand always touches.

It contains no cross-arm pair, because `both_arms` exists to collision-check one arm against the
other. `test_robot_model` asserts those stay enabled.

## Tests

| Test | Needs a simulator | Covers |
|---|---|---|
| `test_moveit_config_drift` | no | This package against `g1_controllers`' controller config and `g1_description`'s hand clamp, the composite-group solver rule, SRDF well-formedness and the octomap sensor config. |
| `test_robot_model` | no | Group composition and order, planning frame, no hand or waist joints in an arm group, the collision matrix's adjacent and cross-arm pairs, the end effectors and the named poses. |
| `test_launch_threading` | no | The arguments `g1_bringup`'s `moveit:=true` branch threads into the simulator, the RViz choice, and what `moveit_sim.launch.py` composes. |
| `test_octomap_blocks_a_plan` | yes | The octomap fills from the LiDAR and MoveIt collision-checks against it: a reach into a mapped obstacle is rejected, with `<octomap>` named in the contact. |
| `test_moveit_lowcmd` | yes | MoveIt with the pelvis unpinned: every motor claimed before the acquire, the freeze traded for the trajectory controller and back, both arms moving without the balance policy losing the robot, both hands activating, and the left hand closing and opening through MoveIt. |

```bash
./scripts/manage.sh test g1_moveit_config
./scripts/manage.sh test --sim g1_moveit_config   # the suites that need a simulator
```

Untested: arm groups composing through a turned waist. Nothing stands the torso off-square, and
`waist_freeze_controller` latches the yaw the scene starts at, so a scene with a turned waist would
cover it.
