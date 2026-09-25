# g1_world_model

What the robot knows about its surroundings on top of the SLAM map: the rooms and their doorways,
the objects in them, and how much of each room the head camera has actually seen. It plans the
viewpoints that finish the job, and answers "where is the dustbin" and "how do I get there" for
the behavior tree, so missions name targets instead of carrying coordinates.

## Node: `g1_world_model`

| | Name | Type |
|---|---|---|
| Sub | `map` | `nav_msgs/OccupancyGrid` (latched) |
| Sub | `depth/image_raw`, `depth/camera_info`, `color/image_raw` | aligned D435i streams |
| Sub | `instance_masks` | `g1_msgs/InstanceMaskArray`, from `g1_detector` or the mock |
| Sub | `cloud` | `sensor_msgs/PointCloud2`, the LiDAR, for the walls-only grid |
| Sub | `odom` | `nav_msgs/Odometry`, for the stillness gate |
| Sub | `descriptions` | `g1_msgs/Description`, from `g1_object_describer` |
| Pub | `~/rooms` | `g1_msgs/RoomArray` (latched): outline, doorways, type, coverage |
| Pub | `~/objects` | `g1_msgs/WorldObjectArray` (latched) |
| Pub | `~/coverage` | `nav_msgs/OccupancyGrid`: best view quality per target, 0-100 |
| Pub | `~/markers` | `visualization_msgs/MarkerArray` for RViz |
| Pub | `~/describe_requests` | `g1_msgs/DescribeRequest`, with `describe:=true` |
| Srv | `~/next_viewpoint`, `~/report_viewpoint` | the exploration loop |
| Srv | `~/find_objects`, `~/get_approach_pose` | queries for missions |
| Srv | `~/save` | `std_srvs/Trigger`: writes the world to `world_dir` |

Parameters and their reasons are in `config/g1_world_model.yaml`; room type likelihoods in
`config/room_types.yaml`.

## How it works

- **Rooms** (`room_segmentation`): peaks of the clearance map that stand well above the doorway
  they drain through (0-dimensional persistence), with a global pass that splits off corridors
  no 2.4 m disc fits in. The node segments a walls-only grid: occupied cells the LiDAR has looked
  across between 1.4 and 2.2 m without a return are furniture, not walls. Room ids survive
  re-segmentation by overlap.
- **Coverage** (`coverage_map`): floor cells, faces (obstacle cells beside free space) and the tops
  of tables and shelves. Every depth sample credits the target it lands on with a quality from
  range, incidence and distance from the image centre. Only frames taken after the base has been
  still for `settle_s` count, because the relay stamps images on arrival.
- **Viewpoints** (`viewpoint_planner`): frontier mode walks to the edge of the known map; coverage
  mode samples standing poses, predicts per heading which pending targets would be seen well
  enough, keeps up to three headings, and ranks poses by gain per second of walking, turning and
  dwelling. Targets that no reachable pose can see, such as a shelf above camera height, are
  written off and reported per room, and a room whose viewpoints Nav2 keeps refusing is given up.
- **Objects** (`object_map`): masks lifted with depth into voxels and matched to mapped objects on
  overlap, label votes and embeddings, tolerant of AMCL's decimetre drift. Parts of one object
  seen from different sides merge when they touch, and a sighting nothing confirms within half a
  minute is dropped. Objects whose place the camera sees through collect misses, and are marked
  stale and then removed.

Everything but the node itself is plain C++ with no ROS dependency, so it is unit tested directly.

## Running it in simulation

```bash
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true world:=apartment headless:=true
ros2 run g1_perception g1_mock_detector --ros-args -r __node:=g1_detector \
  -p "phrases:=['sofa','dustbin','bed','desk','chair']" \
  -r object_poses:=/g1_sensor_relay/object_poses \
  -r depth/image_raw:=/camera/aligned_depth_to_color/image_raw \
  -r camera_info:=/camera/color/camera_info -r "~/instance_masks:=/g1_perception/instance_masks"
ros2 launch g1_world_model world_model.launch.py world_dir:=/root/data/worlds/apartment
ros2 run g1_orchestration g1_bt_executor --ros-args \
  -p tree_file:=$(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/explore.xml
```

With the real detector, start `scripts/semantic_server.py` on the host and load
`g1_perception/config/indoor_vocabulary.yaml` for `g1_detector`; add `describe:=true` to name
objects through the server's VLM.

## Tests

- Unit (CI): `test_room_segmentation` (hand-drawn plans and the committed facility and apartment
  maps), `test_viewpoint_planner` (a ray-cast camera covering two rooms and a corridor),
  `test_object_map`, `test_world_parts`.
- Simulation (`-L simulator`): `test_world_model_explore` explores the apartment and scores rooms,
  coverage and objects against `g1_bringup/worlds/apartment.truth.yaml`, then walks to an object
  by name.

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
