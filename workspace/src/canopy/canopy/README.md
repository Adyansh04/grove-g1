# canopy

What a robot knows about its surroundings on top of the SLAM map: the rooms and their doorways,
the objects in them, and how much of each room its cameras have actually seen. It plans the
viewpoints that finish the job, and answers "where is the dustbin" and "how do I get there", so
missions name targets instead of carrying coordinates.

## Node: `canopy` (executable `world_model`)

| | Name | Type |
|---|---|---|
| Sub | `map` | `nav_msgs/OccupancyGrid` (latched) |
| Sub | `<camera>/depth/image_raw`, `<camera>/depth/camera_info`, `<camera>/color/image_raw` | each camera's aligned depth and colour |
| Sub | `<camera>/instance_masks` | `canopy_msgs/InstanceMaskArray` for that camera, from canopy_perception's `detector` or `mock_detector` |
| Sub | `cloud` | `sensor_msgs/PointCloud2`, the LiDAR, for the walls-only grid |
| Sub | `odom` | `nav_msgs/Odometry`, for the stillness gate |
| Sub | `descriptions` | `canopy_msgs/Description`, from canopy_perception's `object_describer` |
| Pub | `~/rooms` | `canopy_msgs/RoomArray` (latched): outline, doorways, type, coverage |
| Pub | `~/objects` | `canopy_msgs/WorldObjectArray` (latched) |
| Pub | `~/coverage` | `nav_msgs/OccupancyGrid` for RViz's costmap palette: 90 still to see, 99 written off, 0 the rest |
| Pub | `~/trail` | `nav_msgs/Path` (latched): where the robot has walked |
| Pub | `~/walls` | `nav_msgs/OccupancyGrid` (latched): the walls-only map rooms are cut on |
| Pub | `~/floor_plan` | `nav_msgs/OccupancyGrid` (latched): the map with each room closed and furniture solid, as saved |
| Pub | `~/markers` | `visualization_msgs/MarkerArray` for RViz: rooms, doorways, objects, glimpses, every viewpoint in order, the camera's view on the floor |
| Pub | `~/describe_requests` | `canopy_msgs/DescribeRequest`, with `describe:=true` |
| Srv | `~/next_viewpoint`, `~/report_viewpoint` | the exploration loop |
| Srv | `~/find_objects`, `~/get_approach_pose` | queries for missions |
| Srv | `~/save` | `std_srvs/Trigger`: writes the world to `world_dir` (also every minute) |

Parameters and their reasons are in `config/canopy.yaml`; room type likelihoods, and the synonyms
that map other words onto the detector's, in `config/room_types.yaml`.

Cameras are configuration only: `cameras` lists them, each reads the four topics above under
`camera.<name>.prefix` (`<name>/` by default), and its height, pitch, heading and field of view
come from TF and `camera_info`. Every camera whose `camera.<name>.coverage` is true credits
coverage, and the planner scores each heading by what all of them see. The camera pass waits
until TF has given at least one camera's mount.

## How it works

- **Rooms** (`room_segmentation`): peaks of the clearance map that stand well above the doorway
  they drain through (0-dimensional persistence), with a global pass that splits off corridors
  no 2.4 m disc fits in. The node segments a walls-only grid: occupied cells the LiDAR has looked
  across between 1.4 and 2.2 m without a return are furniture, not walls, and so is any blob
  standing clear of the walls with no return at wall height, unseen inside and all (`~/walls`
  shows the grid). Room ids survive re-segmentation by overlap.
- **Coverage** (`coverage_map`): floor cells, faces (obstacle cells beside free space) and the tops
  of tables and shelves whose top a view has taken in: a shelf taller than the camera looks is cut
  off at the image's top edge, and its highest voxel is no top. A top is the highest voxel layer
  dense enough to be the furniture, not the mug on it its mask took in. Every depth sample credits the
  target it lands on with a quality from range, incidence and distance from the image centre. Only
  frames taken after the base has been still for `settle_s` count, because a simulator's relay can
  stamp images on arrival. When SLAM redraws a wall a cell or two off, as after a loop closure, its faces keep
  the credit of the old ones beside them.
- **Viewpoints** (`viewpoint_planner`): frontier mode walks to the edge of the known map, where the
  known map opens onto real unknown space rather than the shadow behind a sofa. Unknown the robot
  has already had in plain view from a viewpoint it reached, and that is still unknown once SLAM has
  caught up, is shadow too, so it never walks into a corner of furniture after it. Coverage mode
  samples standing poses, predicts per heading which pending targets would be seen well enough,
  keeps up to three headings, and ranks poses by gain per second of walking, turning and dwelling.
  Targets that no reachable pose can see, such as a shelf above camera height, are written off and
  reported per room, and a room whose viewpoints Nav2 keeps refusing is given up. Once the camera
  has seen what it can, the robot looks into the unknown pockets left inside the building, such as
  the floor behind a wardrobe or a notch in a wall, from the few spots that see their edges, so
  their walls close in the map.
- **Objects** (`object_map`): masks lifted with depth into voxels and matched to mapped objects on
  overlap, label votes and embeddings, tolerant of a localiser's decimetre drift. Parts of one object
  seen from different sides merge when they touch, and only then is a sighting nothing confirmed
  within two minutes dropped (a sighting is an instant: two cameras catching one moment count
  once); until a second sighting confirms it, it is neither published nor
  drawn, and the camera pass goes back to where it was seen to look at it once more. Objects
  whose place the camera sees through collect misses, and are marked
  stale and then removed. Boxes lie along the walls unless an object is clearly turned from them;
  furniture on the floor then takes the outline the occupancy map shows under it, so an approach
  pose is computed from where the sofa is, not from where the camera happened to see it.

Everything but the node itself is plain C++ with no ROS dependency, so it is unit tested directly.

## What is saved

`world_dir` holds, side by side, everything needed to see and reuse what the robot learnt:

| File | |
|---|---|
| `map.pgm`, `map.yaml` | The floor plan of the map the world was built on, as `map_saver` writes it: after a mapping run, the map SLAM made, with each wall closed where furniture hid it from the scan band (from the LiDAR's hits at wall height, over the furniture) and the pockets that shuts in settled, so furniture is solid and every room a closed outline. Floor no sensor reached behind a wardrobe is drawn as floor unless an object the camera mapped stands there. |
| `semantic_map.png` | That map with each room tinted and named, and every object's box and label. |
| `world.yaml` | Rooms and objects in plain text: ids, names, types, boxes, what rests on what. |
| `objects.bin`, `coverage.bin`, `crops/` | Object voxels, what the camera has seen (and the map SLAM made), and each object's best view. |
| `wall_hits.png` | The LiDAR's returns at wall height, north up like `map.pgm`: with `coverage.bin` they rebuild the floor plan offline. |

A later run with the same `world_dir` on the same map picks all of it up and carries on.

## Running it

```bash
ros2 launch canopy world_model.launch.py world_dir:=/data/worlds/home rviz:=true \
  cameras:=head=/camera,chest=/chest_camera odom_topic:=/odom cloud_topic:=/lidar/points \
  detector:=true describe:=true params_file:=my_robot_canopy.yaml
```

- `cameras` names each camera and its RealSense-style driver namespace. Masks come from
  `/detector_<name>/instance_masks`: `detector:=true` starts canopy_perception's detector there,
  against the host model server in `servers/` (start it first); a simulator can run
  `mock_detector` under that name instead.
- `params_file` carries the robot's own values over `config/canopy.yaml`: its footprint radius,
  walking and turning speeds, and how long to dwell for the detector.
- The robot needs a latched `/map` (a SLAM node's or `map_server`'s), TF from `map` to its base
  and cameras, and something to walk it: the exploration loop below.

### The exploration loop

Whatever drives the robot (a behavior tree, a state machine, an agent) repeats:

1. `~/next_viewpoint` with mode `frontier` until it answers done, walking to each pose it returns
   and facing each heading, then `~/report_viewpoint` with whether the pose was reached.
2. The same with mode `coverage`, dwelling a couple of seconds per heading for the detector.
3. `~/save`.

Nav2's `NavigateToPose` and `Spin` are enough for the walking; grove-g1's `explore.xml` is one such
tree.

## Tools

In `scripts/`, run with `ros2 run canopy <tool> --help` for every option. Python, because they
are small ROS probes; none runs in the loop.

| Tool | What it gives |
|---|---|
| `track_run.py PREFIX` | Records a run: the robot's pose, each room's coverage over time, the rooms' outlines. |
| `snapshot_map.py OUT.png` | SLAM's map as it stands, with the frontier cells the frontier pass plans on. |

`doc/` holds a six-room flat a simulated Unitree G1 explored, and pictures of it.

## Tests

Unit tests only, no ROS graph: `test_room_segmentation` (hand-drawn plans and two saved maps in
`test/maps`), `test_viewpoint_planner` (a ray-cast camera covering two rooms and a corridor),
`test_object_map` and `test_world_parts`. The end-to-end acceptance run lives with a robot:
grove-g1 explores its simulated flat and scores rooms, coverage and objects against ground truth.

## Credits

Adapted from published work, reimplemented here:

| Idea | Source |
|---|---|
| Rooms from clearance | Hydra, MIT SPARK: https://github.com/MIT-SPARK/Hydra |
| Detection-to-object association | ConceptGraphs: https://github.com/concept-graphs/concept-graphs |
| Duplicate merging | OVO: https://github.com/tberriel/OVO |
| Absence evidence, "not found" answers | DynaMem: https://github.com/hello-robot/stretch_ai |
| Image-edge weight | VLFM: https://github.com/bdaiinstitute/vlfm |
| Viewpoints with headings, room-level ordering | FUEL: https://github.com/HKUST-Aerial-Robotics/FUEL, TARE: https://github.com/caochao39/tare_planner |
| Merge rules for small regions | Bormann et al., ICRA 2016: https://github.com/ipa320/ipa_coverage_planning |
