# Pick and place

Pick and place are served as ROS actions, and a BehaviorTree.CPP tree sequences them with
navigation. There are two versions: one where the object is already at arm's length, and the full
mission that drives across the facility to reach it.

Every command runs inside the development container:

```bash
./scripts/manage.sh start
./scripts/manage.sh exec
cd /root/workspace && source install/setup.bash
```

## The short version

The `manipulation` world puts one cube on a bench at arm's length, so nothing has to walk
anywhere. This is the quickest way to see a grasp.

```bash
ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true world:=manipulation pin_pelvis:=true \
  odometry:=ground_truth activate_arm:=true activate_arm_delay_s:=40.0
```

Three of those need explaining:

- `pin_pelvis:=true` welds the pelvis and freezes the legs. It is a simulation-only aid that gives
  the arm a base that does not move between runs, and it only works with `mode:=none`, which is
  the default.
- `odometry:=ground_truth` is required here, not optional. The bench sits at arm's length with the
  pelvis pinned, so the Mid360 returns nothing, FAST-LIO logs `No point, skip this scan!` forever
  and never publishes odom, which leaves the object-pose source with no frame to place into.
- `manipulation:=true` needs `moveit:=true`, since the skills plan through move_group. The launch
  refuses the combination otherwise.

Then run the tree. It drives the skills itself; nothing else needs starting:

```bash
ros2 launch g1_orchestration mission.launch.py tree:=pick_and_place_in_place.xml
```

## The full mission

The facility world, a map, and Nav2. The robot drives to a workbench, closes the last stretch
under closed-loop control, picks the cube up, carries it across the building and puts it down.

```bash
ros2 launch g1_bringup bringup.launch.py \
  mode:=localization nav:=true moveit:=true manipulation:=true world:=navigation \
  rviz:=true activate_arm:=true activate_arm_delay_s:=40.0 headless:=false
```

```bash
ros2 launch g1_orchestration mission.launch.py tree:=pick_and_place.xml
```

Nav2 parks within about 0.5 m of a goal and the arm's usable window is roughly 0.2 m wide, which
is why there is a base approach skill: it closes the remaining gap against the measured object
rather than against the map.

## Watching the tree

The mission publishes to Groot2 over ZeroMQ on port 1667. Connect from the host at
`localhost:1667` to watch it tick live. The node palette is generated from the registered leaves,
so Groot2 can edit these trees without hand-written XML:

```bash
ros2 run g1_orchestration g1_bt_node_model
```

Set `groot2_port:=0` to turn the monitor off, and `tick_rate_hz:=<n>` to change the tick rate from
its default of 10 Hz.

## Available trees

| Tree | What it does |
|---|---|
| `pick_and_place_in_place.xml` | Grasps an object already within reach. |
| `pick_and_place.xml` | Navigate, approach, pick, carry, place. |
| `vla_grasp_in_place.xml` | The learned-grasp variant, covered in [its own guide](learned-grasping.md). |

## Where object poses come from

`object_source` defaults to `sim_ground_truth`, which reads MuJoCo bodies directly. There is no
object-detection pipeline yet, and `object_source:=hardware` deliberately refuses to configure
rather than pretending otherwise. A real detector replaces it without the skills changing.

## Arm authority

The skills need the arm acquired before they can execute anything. `activate_arm:=true` does it
automatically after `activate_arm_delay_s` seconds, which is a simulation convenience. Done by
hand it is:

```bash
ros2 launch g1_bringup activate_arm.launch.py
# ... run the mission ...
ros2 launch g1_bringup deactivate_arm.launch.py
```

The delay exists because the hardware component has to be loaded and `/lowstate` flowing first.
Too early and the acquire fails loudly rather than silently. Raise it if the stack is slow to
come up on your machine.

## When a run misbehaves

Leftover nodes from a previous run are the most common cause of failures that make no sense.
Clear them and confirm the graph is actually empty:

```bash
./scripts/clean-stack.sh
```
