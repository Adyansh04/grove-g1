# Exploring a building and asking what is where

The world model (`g1_world_model`) builds on the SLAM map: rooms and doorways, the objects in
each room, and how much of every room the head camera has seen. An exploration tree walks the
robot through the building until the camera has seen what it can, and missions then name their
targets ("the dustbin in the office") instead of carrying coordinates.

## In simulation, with ground-truth masks

The apartment world has six rooms and 44 textured props, with a ground-truth file for scoring.
Fetch its assets once on the host (about 330 MB; nothing is committed):

```bash
./scripts/setup-world-assets.sh
```

Then, inside the container:

```bash
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true world:=apartment headless:=true
ros2 run g1_perception g1_mock_detector --ros-args -r __node:=g1_detector \
  -p "phrases:=['sofa','dustbin','bed','desk','chair','dining table','bookshelf','mug']" \
  -r object_poses:=/g1_sensor_relay/object_poses \
  -r depth/image_raw:=/camera/aligned_depth_to_color/image_raw \
  -r camera_info:=/camera/color/camera_info -r "~/instance_masks:=/g1_perception/instance_masks"
ros2 launch g1_world_model world_model.launch.py world_dir:=/root/data/worlds/apartment
ros2 run g1_orchestration g1_bt_executor --ros-args \
  -p tree_file:=$(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/explore.xml
```

The mock detector cuts masks from simulator ground truth, because open-vocabulary models do not
work on flat MuJoCo renders. It matches a phrase to every numbered body of that class: "chair"
finds `chair_1` to `chair_5`.

`explore.xml` first walks to frontiers until the LiDAR map is closed (on a saved map this ends at
once), then visits viewpoints until the camera has seen every room, and saves the world. In RViz,
add `/g1_world_model/markers` for rooms, doorways and object boxes, and `/g1_world_model/coverage`
for what the camera has seen.

## Asking the world model

```bash
ros2 service call /g1_world_model/find_objects g1_msgs/srv/FindObjects "{query: dustbin, room: office}"
ros2 service call /g1_world_model/get_approach_pose g1_msgs/srv/GetApproachPose "{target: 'dining table'}"
ros2 topic echo --qos-durability transient_local /g1_world_model/rooms
```

A room can be named by its id (`R3`), its name (`room C`) or its type (`office`). A search that
finds nothing reports how much of the searched area the camera has seen, so "not found" means "not
in the 96 % of the office the camera saw", not "absent".

In a tree, `ResolveTarget` turns a target into a pose facing it, for `NavigateToPose`:

```xml
<ResolveTarget target="dustbin" room="office" goal="{goal}"/>
<NavigateToPose goal="{goal}"/>
```

## With real models

The host semantic server serves detection (YOLOE-26), image embeddings (SigLIP 2) and object and
room descriptions (Gemini on its free tier, falling back to Qwen3.5-4B in llama.cpp):

```bash
./scripts/setup-semantic.sh
./scripts/start-vlm.sh start            # the offline VLM, on 127.0.0.1:8080
~/ref/grove-semantic/.venv/bin/python scripts/semantic_server.py
```

Gemini needs a free API key in `~/.config/grove/gemini.env`. The server caps its own use at 5
requests a minute and 100 a day, and stops for the day on any quota answer. Then run `g1_detector`
with `g1_perception/config/indoor_vocabulary.yaml` in place of the mock, and launch the world model
with `describe:=true`: each object gets a name and a caption, and a room whose objects do not
settle its type is typed from the frames the robot took standing in it.

## What the camera cannot see

The D435i is pitched 47.6° down on a head that cannot turn. Standing, it sees floor only 0.3–3.6 m
ahead, a table top only within about 1.4 m, and nothing above camera height. The world model
writes off what no standing pose can see, and each room reports those cells as
`unobservable_cells`, so a shelf's top tier is listed as unseen rather than silently missing.
