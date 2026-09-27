# g1_world_model

What the robot knows about its surroundings on top of the SLAM map: the rooms and their doorways,
the objects in them, and how much of each room the head camera has actually seen. It plans the
viewpoints that finish the job, and answers "where is the dustbin" and "how do I get there" for
the behavior tree, so missions name targets instead of carrying coordinates.

## Node: `g1_world_model`

| | Name | Type |
|---|---|---|
| Sub | `map` | `nav_msgs/OccupancyGrid` (latched) |
| Sub | `<camera>/depth/image_raw`, `<camera>/depth/camera_info`, `<camera>/color/image_raw` | each camera's aligned depth and colour |
| Sub | `<camera>/instance_masks` | `g1_msgs/InstanceMaskArray` for that camera, from `g1_detector` or the mock |
| Sub | `cloud` | `sensor_msgs/PointCloud2`, the LiDAR, for the walls-only grid |
| Sub | `odom` | `nav_msgs/Odometry`, for the stillness gate |
| Sub | `descriptions` | `g1_msgs/Description`, from `g1_object_describer` |
| Pub | `~/rooms` | `g1_msgs/RoomArray` (latched): outline, doorways, type, coverage |
| Pub | `~/objects` | `g1_msgs/WorldObjectArray` (latched) |
| Pub | `~/coverage` | `nav_msgs/OccupancyGrid` for RViz's costmap palette: 90 still to see, 99 written off, 0 the rest |
| Pub | `~/trail` | `nav_msgs/Path` (latched): where the robot has walked |
| Pub | `~/walls` | `nav_msgs/OccupancyGrid` (latched): the walls-only map rooms are cut on |
| Pub | `~/floor_plan` | `nav_msgs/OccupancyGrid` (latched): the map with each room closed and furniture solid, as saved |
| Pub | `~/markers` | `visualization_msgs/MarkerArray` for RViz: rooms, doorways, objects, glimpses, every viewpoint in order, the camera's view on the floor |
| Pub | `~/describe_requests` | `g1_msgs/DescribeRequest`, with `describe:=true` |
| Srv | `~/next_viewpoint`, `~/report_viewpoint` | the exploration loop |
| Srv | `~/find_objects`, `~/get_approach_pose` | queries for missions |
| Srv | `~/save` | `std_srvs/Trigger`: writes the world to `world_dir` (also every minute) |

Parameters and their reasons are in `config/g1_world_model.yaml`; room type likelihoods in
`config/room_types.yaml`.

Cameras are configuration only: `cameras` lists them, each reads the four topics above under
`camera.<name>.prefix` (`<name>/` by default), and its height, pitch, heading and field of view
come from TF and `camera_info`. Every camera whose `camera.<name>.coverage` is true credits
coverage, and the planner scores each heading by what all of them see. `world_model.launch.py
cameras:=head,chest` wires the simulator's head and chest cameras; another robot needs only its
own topic remaps.

## How it works

- **Rooms** (`room_segmentation`): peaks of the clearance map that stand well above the doorway
  they drain through (0-dimensional persistence), with a global pass that splits off corridors
  no 2.4 m disc fits in. The node segments a walls-only grid: occupied cells the LiDAR has looked
  across between 1.4 and 2.2 m without a return are furniture, not walls, and so is any blob
  standing clear of the walls with no return at wall height, unseen inside and all (`~/walls`
  shows the grid). Room ids survive re-segmentation by overlap.
- **Coverage** (`coverage_map`): floor cells, faces (obstacle cells beside free space) and the tops
  of tables and shelves whose top a view has taken in: a shelf taller than the camera looks is cut
  off at the image's top edge, and its highest voxel is no top. Every depth sample credits the
  target it lands on with a quality from range, incidence and distance from the image centre. Only
  frames taken after the base has been still for `settle_s` count, because the relay stamps images
  on arrival. When SLAM redraws a wall a cell or two off, as after a loop closure, its faces keep
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
  overlap, label votes and embeddings, tolerant of AMCL's decimetre drift. Parts of one object
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

## Running it in simulation

`mode:=mapping` explores from no map at all: slam_toolbox builds it while the tree walks to
frontiers, then the camera pass runs on it. `mode:=localization` uses the committed map instead.

```bash
ros2 launch g1_bringup bringup.launch.py mode:=mapping nav:=true world:=apartment headless:=true
ros2 run g1_perception g1_mock_detector --ros-args -r __node:=g1_detector \
  -p "phrases:=['sofa','dustbin','bed','desk','chair']" \
  -r object_poses:=/g1_sensor_relay/object_poses \
  -r depth/image_raw:=/camera/aligned_depth_to_color/image_raw \
  -r camera_info:=/camera/color/camera_info -r "~/instance_masks:=/g1_perception/instance_masks"
ros2 launch g1_world_model world_model.launch.py world_dir:=/root/data/worlds/apartment rviz:=true
ros2 run g1_orchestration g1_bt_executor --ros-args \
  -p tree_file:=$(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/explore.xml
```

With the real detector, start `scripts/semantic_server.py` on the host and load
`g1_perception/config/indoor_vocabulary.yaml` for `g1_detector`; add `describe:=true` to name
objects through the server's VLM, and to type the rooms whose objects leave their type in doubt
from a few whole frames taken where the robot stood in them.

## Tests

- Unit (CI): `test_room_segmentation` (hand-drawn plans and the committed facility and apartment
  maps), `test_viewpoint_planner` (a ray-cast camera covering two rooms and a corridor),
  `test_object_map`, `test_world_parts`.
- Simulation (`-L simulator`): `test_world_model_explore` explores the apartment on the committed
  map and scores rooms, coverage, objects and how well the boxes fit, against
  `g1_bringup/worlds/apartment.truth.yaml`, then walks to an object by name.
  `test_world_model_explore_mapping` does the same from no map, on the one SLAM builds; truth is
  moved into that map through the simulator's own pose of the robot.

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
