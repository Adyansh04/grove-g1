# Pick and place

Pick and place are ROS actions served by `g1_manipulation`, and a BehaviorTree.CPP tree sequences
them with navigation. There are two versions: one where the object is already at arm's length, and
the full mission that drives across the facility to reach it.

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

## The short version

The `manipulation` world puts one block on a bench at arm's length, so nothing has to walk. This is
the quickest way to see a grasp.

```bash
ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true world:=manipulation pin_pelvis:=true \
  odometry:=ground_truth activate_arm:=true activate_arm_delay_s:=40.0
```

- `pin_pelvis:=true` welds the pelvis and freezes the legs, so the arm works from the same base
  every run. Simulation only, and only with `mode:=none`, the default.
- `odometry:=ground_truth` is required. The world is a bench and nothing else, so FAST-LIO has
  nothing to register against and never publishes `odom`, which leaves `/objects` without a frame.
- `manipulation:=true` needs `moveit:=true`; the launch refuses it otherwise.

Wait for `activation complete` in the launch output, then run the tree. It drives the skills
itself:

```bash
ros2 launch g1_orchestration mission.launch.py tree:=pick_and_place_in_place.xml
```

The robot picks the block up, holds it at the `carry` posture, sets it down beside where it
started, and tucks the arm.

## How the hand holds an object

The Dex3's palm and fingers have collision geometry, and in the `manipulation` and `tabletop`
worlds the object is held by contact alone. After the close, the finger joints have to show the
thumb and the index or middle finger pressing on something, or the pick fails. A failed pick leaves
the hand open and nothing attached, so the tree's retry starts clean.

What that means for your own props:

- The hand takes objects 20 to 75 mm across.
- The fingers close 26 mm below the object's top, or `min_grip_height_m` above the surface it
  stands on if that is higher, because the thumb hangs 63 mm below that point. The default is
  0.080 m, so an object has to stand taller than 80 mm for the fingers to close on it. The blocks
  in the `manipulation` and `tabletop` worlds are 90 mm tall.
- The arm is position-controlled with no gravity compensation, so it sags. The pick measures the
  hand in TF and corrects it above the object, in clear air. A descent that ends more than
  `settle_tolerance_m` off backs straight up and tries again, up to `settle_attempts` more times;
  still more than `max_grasp_offset_m` off, the pick refuses rather than close on air.
- After the lift the grip is only logged, because a cradled object loads the fingers too little to
  tell held from dropped. A drop during the carry fails the place, which confirms where the object
  landed on `/objects`.

The `navigation` world adds one simulator aid for the walk: a weld holds the ball to the palm while
the thumb and one other digit touch it within 0.16 m of the palm, and lets go 0.3 s after that
contact ends. The grasp that engages it is still a contact grasp. That world also lowers
`min_grip_height_m` to 0.035 m, because its ball sits by the desk edge, where the thumb hangs past
the desk.

The server's parameters, `min_grip_height_m` included, can be changed with
`ros2 param set /g1_manipulation_server <name> <value>` between goals; a set while a goal runs is
refused. `velocity_scaling`, `grasp_rpy`, `grasp_service`, `publish_markers`, `grasp_source` and
`graspgen_to_grasp_frame_xyz_rpy` are fixed at startup. `g1_manipulation/README.md` describes
them.

## The full mission

The facility world, the committed map and Nav2. The robot drives to a workbench, walks the last
stretch against the measured ball, picks it up, carries it across the building and drops it into a
box on the storage bench.

```bash
ros2 launch g1_bringup bringup.launch.py \
  mode:=localization nav:=true moveit:=true manipulation:=true perception:=true \
  world:=navigation rviz:=true activate_arm:=true activate_arm_delay_s:=40.0 headless:=false
```

```bash
ros2 launch g1_orchestration mission.launch.py tree:=pick_and_place.xml
```

`perception:=true` with the default `detector:=mock` needs no GPU. `detector:=vision` uses the real
models; see [Open-vocabulary perception](open-vocabulary-grasping.md). The tree asks the detector
for objects only at the two benches and idles it in between, so `/objects` is quiet during the
walks.

Nav2 parks within 0.5 m of a goal and the arm's reach window is about 0.11 m wide, so a base
approach skill closes the rest against the measured object rather than against the map.

## Watching the tree

The executor publishes to Groot2 over ZeroMQ on port 1667. Groot2 runs on the host: connect its
Monitor to `localhost:1667` while a tree is running.

To edit trees, open `workspace/src/g1_orchestration/trees/g1_orchestration.btproj` in Groot2 and,
once per project, import `g1_orchestration_nodes.xml` from the same directory with Import Models.
Trees are symlinked into the install space, so a tree saved from Groot2 is used by the next launch
without a rebuild. `g1_orchestration/README.md` covers adding a leaf and regenerating that palette.

`groot2_port:=0` turns the monitor off, and `tick_rate_hz:=<n>` changes the tick rate from its
default of 10 Hz.

## Available trees

| Tree | World | What it does |
|---|---|---|
| `pick_and_place_in_place.xml` | `manipulation` | Picks the block within reach, holds it at `carry`, sets it down beside where it was. |
| `pick_and_place.xml` | `navigation` | Navigate, approach, pick, carry, place in the box on the storage bench. |
| `sort_into_box.xml` | `tabletop` | Picks the red block from beside a blue one of the same shape and drops it in the box. |
| `vla_grasp_in_place.xml` | `manipulation` | The learned grasp, covered in [its own guide](learned-grasping.md). |

`sort_into_box.xml` runs on the short version's launch with `world:=tabletop perception:=true` in
place of `world:=manipulation`.

## Where object poses come from

`object_source` defaults to `sim_ground_truth`, which reads MuJoCo bodies directly.
`perception:=true` measures them from the camera instead; see
[Open-vocabulary perception](open-vocabulary-grasping.md). `object_source:=hardware` refuses to
configure, because the robot has no detector of its own.

## Arm authority

The skills execute only while the arm is acquired. Every tree acquires it first, and the executor
releases it on every exit, including a failed tree and Ctrl-C. `activate_arm:=true` also acquires
it at bring-up and swings both arms out to the sides, where MoveIt can plan from; it is a
simulation convenience. By hand:

```bash
ros2 launch g1_bringup activate_arm.launch.py
ros2 launch g1_bringup deactivate_arm.launch.py
```

`activate_arm_delay_s` has to cover the hardware component loading and `/lowstate` starting to
flow; an acquire that runs too early fails loudly. Raise it if the stack is slow to come up.

## When a run misbehaves

Leftover nodes from a previous run cause most failures that make no sense. This tears them down
and exits non-zero unless the ROS graph is empty afterwards:

```bash
./scripts/clean-stack.sh
```
