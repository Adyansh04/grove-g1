# Exploring a building and asking what is where

The world model builds on the SLAM map: rooms and doorways, the objects in each room, and how much
of every room the cameras have seen. It is canopy (`workspace/src/canopy`, its own repository),
which `g1_bringup/launch/world_model.launch.py` wires to the G1's topics. An exploration tree walks the
robot through the building until the camera has seen what it can, and missions then name their
targets ("the dustbin in the office") instead of carrying coordinates.

`./scripts/demos/world-model.sh <mock|real|editor>` opens a section's commands at once, in split panes of
your terminal, after tearing down any stack left running. `stop` ends it, and `--print` lists
the commands instead.

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
ros2 launch g1_bringup world_model.launch.py world_dir:=/root/data/worlds/apartment detector:=mock \
  rviz:=true
ros2 run g1_orchestration g1_bt_executor --ros-args \
  -p tree_file:=$(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/explore.xml
```

`detector:=mock` starts a mock detector per camera, which cuts masks from simulator ground truth,
so a run scores the mapping rather than a detector on renders, and needs no GPU or model
server. It looks for every label in
`g1_bringup/worlds/apartment.truth.yaml`, each matching every numbered body of that class: "chair"
finds `chair_1` to `chair_5`, while `armchair_1` is an armchair. `arms_at_sides:=true` hangs the
arms beside the thighs: at zero the forearms point forward into both cameras' views, and a real
detector maps the hands.

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

The models get some of it wrong. Before the world is saved, `explore.xml`'s `CleanUpWorld` has
canopy take out what it can tell is no object: the floor mapped as a table, or a piece of a sofa
seen as a chair inside it. Those two rules took out no real object in four runs of the test flat.
The rest is for a person. canopy's map editor lists what deserves a second look, with the camera's
view of each object: a weakly seen phantom, a label the describer disagrees with, a room typed with
little confidence. Each fix is a click or two, and what the clean-up took out is listed with why,
to restore. Run it on the host once the run is over:

```bash
python3 workspace/src/canopy/editor/canopy_editor.py data/worlds/apartment   # http://127.0.0.1:8765/
ros2 service call /canopy/reload std_srvs/srv/Trigger   # in the container, if canopy is still up
```

What you fix stays fixed in later runs on that world. Until it reloads, canopy refuses to save over
your edits. The [editor's guide](../../workspace/src/canopy/editor/doc/guide.md) shows each feature
and what to check, with screenshots.

## Asking the world model

```bash
ros2 service call /canopy/find_objects canopy_msgs/srv/FindObjects "{query: dustbin, room: office}"
ros2 service call /canopy/get_approach_pose canopy_msgs/srv/GetApproachPose "{target: 'dining table'}"
ros2 topic echo --qos-durability transient_local /canopy/rooms
```

A room can be named by its id (`R3`), its name (`room C`) or its type (`office`), and an object
by the detector's word or a synonym from canopy's room table ("dustbin" finds a trash can). A search
that finds nothing reports how much of the searched area the camera has seen, so "not found" means "not
in the 96 % of the office the camera saw", not "absent".

In a tree, `ResolveTarget` turns a target into a pose facing it, for `NavigateToPose`. The pose is
0.72 m from the target, out of the steep part of Nav2's inflation, square to one of its sides and
as near that side's middle as a short walk allows: each metre off the middle counts as 3 m of
walking. It takes a corner only when no side is clear, and stands further off only when nothing at
0.72 m is. Nav2 stops up to 0.5 m and 0.5 rad off, so a `TurnTo` squares the robot up once it
arrives:

```xml
<ResolveTarget target="dustbin" room="office" goal="{goal}" yaw="{yaw}"/>
<NavigateToPose goal="{goal}"/>
<TurnTo yaw="{yaw}"/>
```

## With real models

The host semantic server serves detection (SAM 3.1), image embeddings (SigLIP 2) and object and
room descriptions (Gemini on its free tier):

```bash
./workspace/src/canopy/servers/setup.sh   # once
./scripts/serve.sh canopy --describer gemini
```

SAM 3.1's weights are gated: request access at https://huggingface.co/facebook/sam3.1 and sign in
with `hf auth login` before the setup. Gemini needs a free API key in
`~/.config/canopy/gemini.env`. The free tier limits each model separately, so the server spreads
the calls over several, each under its own caps (canopy's `servers/README.md` lists them). Then
launch the world model with `detector:=true describe:=true` in place of the mock: a detector per
camera asks the server about the frames the robot took standing still, each object gets a name and
a caption, and a room whose objects do not settle its type is typed from those frames.

SAM 3.1 and SigLIP 2 use about 8.5 GB of VRAM while mapping, which leaves no room on a 12 GB GPU
for the offline describer (Qwen3.5-4B in llama.cpp, about 4 GB) beside the simulator: hence
`--describer gemini`, and past Gemini's daily limits an object keeps its detector's word. YOLOE-26
needs about 1.7 GB and leaves room for it: serve with `./scripts/serve.sh canopy --detector
yoloe`, which starts the offline VLM first, and launch with
`detector_params:=/root/workspace/src/canopy/canopy_perception/config/detector_yoloe.yaml`.

The acceptance test (`g1_bringup`'s `test_explore_apartment`) runs this way with
`G1_EXPLORE_TEST_DETECTOR=semantic`, and `G1_EXPLORE_TEST_DETECTOR_PARAMS` for YOLOE's list:

| Detector | Objects found | Not in the flat | Boxes (median IoU) | Time |
|---|---|---|---|---|
| SAM 3.1, 34 words (two runs) | 40 and 43 of 46 | 4 and 3 | 0.72 and 0.68 | 31 min |
| YOLOE-26, 128 words | 39 of 46 | 22 | 0.67 | 34 min |

SAM 3.1's misses are mostly names: the wardrobe came back as a cabinet and a crate as a cardboard
box, and an object seen only once is dropped unconfirmed. YOLOE confuses furniture of one material
(desk, cabinet, TV stand) and misses mugs and bowls on tables. A run makes 100 to 250 describe
calls; Gemini's free tier covers several a day.

## What the camera cannot see

The D435i is pitched 47.6° down on a head that cannot turn. Standing, it sees floor only 0.3–3.6 m
ahead, a table top only within about 1.4 m, and nothing above camera height. The world model
writes off what no standing pose can see, and each room reports those cells as
`unobservable_cells`, so a shelf's top tier is listed as unseen rather than silently missing.

In simulation a second D435i sits on the chest, 1.03 m up and 20° down (`g1_description`'s
`config/cameras.yaml`), and sees walls, counters and shelves from the side. Pass
`cameras:=head,chest` to both `bringup.launch.py` and `world_model.launch.py`, and `detector:=mock`
starts a mock for each. `cameras:=chest` alone saves the head camera's
render and detector, but the chest camera sees little floor near the robot, so the camera pass
takes longer.
