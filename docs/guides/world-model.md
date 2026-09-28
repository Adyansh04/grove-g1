# Exploring a building and asking what is where

The world model builds on the SLAM map: rooms and doorways, the objects in each room, and how much
of every room the cameras have seen. It is canopy (`workspace/src/canopy`, its own repository),
which `g1_bringup/launch/world_model.launch.py` wires to the G1's topics. An exploration tree walks the
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
ros2 launch g1_bringup bringup.launch.py mode:=mapping nav:=true world:=apartment headless:=true \
  arms_at_sides:=true
ros2 run canopy_perception mock_detector --ros-args -r __node:=detector_head \
  -p "phrases:=['sofa','dustbin','bed','desk','chair','dining table','bookshelf','mug']" \
  -r object_poses:=/g1_sensor_relay/object_poses \
  -r depth/image_raw:=/camera/aligned_depth_to_color/image_raw \
  -r camera_info:=/camera/color/camera_info
ros2 launch g1_bringup world_model.launch.py world_dir:=/root/data/worlds/apartment rviz:=true
ros2 run g1_orchestration g1_bt_executor --ros-args \
  -p tree_file:=$(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/explore.xml
```

The mock detector cuts masks from simulator ground truth, so a run scores the mapping rather than a
detector on renders. Named `detector_<camera>`, it publishes where canopy reads that camera's
masks. It matches a phrase to every numbered body of that class: "chair" finds
`chair_1` to `chair_5`. `arms_at_sides:=true` hangs the arms beside the thighs: at zero the
forearms point forward into both cameras' views, and a real detector maps the hands.

With `mode:=mapping` there is no map to start from: `explore.xml` first walks to frontiers while
slam_toolbox builds one, then visits viewpoints until the camera has seen every room, and saves
the world. `mode:=localization` uses the committed map instead, and the frontier pass ends at once.
Last, the robot looks into the pockets of unknown left inside the building, such as the corner
behind a wardrobe, from the few spots that see them, so the floor plan's walls close there too.
Before each walk and each turn the tree runs `StepClear`: Nav2 will not move a robot with furniture
inside its 0.45 m circle, and the gait's drift can leave it there, so `g1_base_approach` first
steps it a few centimetres clear. When the robot is already clear it does nothing.

`rviz:=true` opens the world model's own view: each room tinted and named with how much of it the
camera has seen, doorways, object boxes in their room's colour, what the camera still has to see
(red) and what it has written off (cyan), the robot's trail and every viewpoint in order, the one
it is walking to with the headings it will face, and where each camera looks on the floor.

When the tree finishes, `world_dir` holds the map SLAM made as a floor plan, each room closed and
the furniture solid (`map.pgm`, `map.yaml`), a picture of it with the rooms and objects drawn in
(`semantic_map.png`), and `world.yaml` with every room and object in plain text.

## Checking the map by hand

The models get some of it wrong. canopy's map editor lists what deserves a second look, with the
camera's view of each object: a phantom seen twice, the floor mapped as a desk, a label the
describer disagrees with, a room typed with little confidence. Each fix is a click or two. Run it
on the host once the run is over:

```bash
python3 workspace/src/canopy/editor/canopy_editor.py data/worlds/apartment   # http://127.0.0.1:8765/
ros2 service call /canopy/reload std_srvs/srv/Trigger   # in the container, if canopy is still up
```

What you fix stays fixed in later runs on that world. Until it reloads, canopy refuses to save over
your edits. See [canopy's editor/README.md](../../workspace/src/canopy/editor/README.md).

## Asking the world model

```bash
ros2 service call /canopy/find_objects canopy_msgs/srv/FindObjects "{query: dustbin, room: office}"
ros2 service call /canopy/get_approach_pose canopy_msgs/srv/GetApproachPose "{target: 'dining table'}"
ros2 topic echo --qos-durability transient_local /canopy/rooms
```

A room can be named by its id (`R3`), its name (`room C`) or its type (`office`), and an object
by the detector's word or a synonym from canopy's room table ("dustbin" finds a trash can). A search
that
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
cd workspace/src/canopy
./servers/setup.sh
./servers/start-vlm.sh start            # the offline VLM, on 127.0.0.1:8080
~/.local/share/canopy/.venv/bin/python servers/semantic_server.py
```

Gemini needs a free API key in `~/.config/canopy/gemini.env`. The free tier limits each model
separately, so the server spreads the calls over several, each under its own caps (canopy's
`servers/README.md` lists them). Then launch the world
model with `detector:=true describe:=true` in place of the mock: a detector per camera asks the
server, each object gets a name and a caption, and a room whose objects do not settle its type is
typed from the frames the robot took standing in it.

The acceptance test (`g1_bringup`'s `test_explore_apartment`) runs this way with
`G1_EXPLORE_TEST_DETECTOR=semantic`. On the apartment
renders it finds about three quarters of the objects against the mock's all: YOLOE confuses
furniture of one material (desk, cabinet, TV stand) and misses mugs and bowls on tables, while the
describer usually names them right. A run makes 100 to 250 describe calls; Gemini's free tier
covers several runs a day before the local VLM takes over.

## What the camera cannot see

The D435i is pitched 47.6° down on a head that cannot turn. Standing, it sees floor only 0.3–3.6 m
ahead, a table top only within about 1.4 m, and nothing above camera height. The world model
writes off what no standing pose can see, and each room reports those cells as
`unobservable_cells`, so a shelf's top tier is listed as unseen rather than silently missing.

In simulation a second D435i sits on the chest, 1.03 m up and 20° down (`g1_description`'s
`config/cameras.yaml`), and sees walls, counters and shelves from the side. Pass
`cameras:=head,chest` to both `bringup.launch.py` and `world_model.launch.py`, and run a mock
detector per camera (`detector_chest`, on `/chest_camera/*` and
`/g1_sensor_relay/chest/object_poses`). `cameras:=chest` alone saves the head camera's
render and detector, but the chest camera sees little floor near the robot, so the camera pass
takes longer.
