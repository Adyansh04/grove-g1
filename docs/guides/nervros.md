# Talking to the robot with NervROS

NervROS (`workspace/src/nervros`, its own repository) is an agent you talk to in a desktop app. It
looks through the chest camera and marks what the detector finds, asks canopy where things are, and
does physical tasks as missions: behaviour trees that `nervros_executor` checks, you approve, and
the executor runs. It runs on the host and answers with a local model unless you choose otherwise.

`./scripts/demos/nervros.sh <app|chat|explore|facility>` opens a section's commands at once, in
split panes of your terminal, after tearing down any stack left running. `stop` ends it, and
`--print` lists the commands instead.

## Once

The local model is Qwen3.5-9B, quantized to 4 bits, in llama.cpp's server in its own container.
It needs about 7 GB of GPU memory. Fetch it once on the host:

```bash
hf download unsloth/Qwen3.5-9B-GGUF Qwen3.5-9B-Q4_K_M.gguf mmproj-F16.gguf
```

The app builds on the host with Rust, clang, mold and ROS 2 Jazzy; NervROS's README lists what to
install. The first build compiles the Rerun viewer and takes about twenty minutes.

## In the apartment: looking, finding, walking

The apartment runs with the semantic map canopy built of it, corrected by hand
(`workspace/src/canopy/canopy/doc/apartment`): six rooms with their types (living room, bedroom,
office, study, hallway, storage room) and 54 named objects, so the agent knows the rooms and where
things are from the start. canopy writes into its world, so the demo gives it a copy,
`data/worlds/nervros-apartment`, made once and kept; delete it to start again from the committed
map. The robot localizes on that map's own grid, which the saved world needs to resume.

In the container: the apartment with AMCL and Nav2, canopy on the copy with the mock detector, and
the executor.

```bash
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true world:=apartment \
  map:=/root/data/worlds/nervros-apartment/map.yaml headless:=true rviz:=false \
  arms_at_sides:=true cameras:=head,chest
ros2 launch g1_bringup world_model.launch.py world_dir:=/root/data/worlds/nervros-apartment \
  rviz:=false detector:=mock cameras:=head,chest
ros2 launch g1_orchestration nervros_executor.launch.py hands_empty_on_attach:=true
```

On the host, from the repository root:

```bash
workspace/src/nervros/scripts/local-llm.sh start
cd workspace/src/nervros
./scripts/build-overlay.sh ../g1_msgs ../canopy/canopy_msgs
export NERVROS_UDP_ONLY=1 NERVROS_EXTRA_IDL_PACKAGES="g1_msgs;canopy_msgs" ROS_DOMAIN_ID=1
export CARGO_TARGET_DIR=~/.cache/nervros/target   # outside the colcon workspace
source scripts/ros-env.sh
cargo run -p nervros-gui -- --profile ../g1_bringup/config/nervros/nervros.toml
```

- `NERVROS_UDP_ONLY=1` turns Fast DDS shared memory off. The container runs as root, and shared
  memory between two users fails without an error: topics appear, but no data arrives.
- The profile, `g1_bringup/config/nervros/nervros.toml`, names the G1's camera, detector, world
  model topics, the executor and the models file, and a short persona (`apartment.md`). NervROS's
  `docs/profile.md` explains each key.

Things to ask:

- "What do you see? Is there a sofa?" The agent calls `look` with the question: the detector's finds
  are drawn on the frame as numbered marks, the local vision model looks at that frame and
  answers, and you see the frame in the chat. It looks through the chest camera, about 1 m up and
  20 degrees down, which the profile tells the vision model; the head camera, pointing further
  down, feeds canopy with it.
- "Where is the dustbin?" `find_objects` asks canopy, which answers with ids, rooms, and how much of
  each room the camera has seen, so "not found" comes with how sure it is.
- "Walk to the living room." The agent writes a plan, and a plan card appears with its steps. Arm
  the robot with the switch in the top bar, then approve the plan in the chat or the dock. The card
  follows the mission step by step, and the agent tells you how it ended.

The viewer draws the map, the rooms coloured by how much of each the camera has seen, the objects,
the G1's model with its joints as they are, Nav2's path and the camera's view in 3D. Click an
object, a room or a point on the map there and a bar above the message box offers "Go there", "What
is it?" and the like; it fills in the message, which you send. The Mission tab under the camera
shows each step of the running mission as a lane on the timeline. The dock's World tab (Ctrl+2)
lists the rooms with how much of each has been seen.

The apartment has no arm stack, so only walking skills run there.

## Exploring: watching the map fill in

`nervros.sh explore` is the apartment with an empty world model: canopy knows the map and nothing
else. Ask "Explore the building", or press Explore in the World tab. The agent plans
`ExploreBuilding`, a skill of the executor that walks to viewpoints canopy picks until the camera
has seen every room; after your approval the rooms turn from red to green in the viewer, objects
appear, and the "Exploring" plot tracks the share seen. One run explores for up to 23 minutes and
saves the world; ask again to carry on. In the test run from empty, canopy called the apartment done
after 22 minutes: all 6 rooms and 32 objects, with about 80% of the floor and 62% of the walls
seen. canopy wrote the rest off as out of sight from anywhere the robot could reach.

## In the facility: pick and place

`nervros.sh facility` starts the pick-and-place stack from the [pick and place](pick-and-place.md)
guide, the executor, and the app with `g1_bringup/config/nervros/facility.toml`. The facility has
no world model: the profile names the two benches as places, `workbench` and `storage`, and a
short persona (`facility.md`) tells the model the detector's names for the objects, `red_block`
and `brown_box`. The viewer shows the map, Nav2's path and the G1's model, so you can watch the arm
reach and the hand close.

Ask: "Pick up the red_block from the workbench and put it into the brown_box on the storage
bench." The agent plans four steps: walk to the workbench, pick the ball, walk to the storage
bench, place it into the box. In the test run the plan passed its checks at once and the mission
took about four minutes; the ball came to rest 5 cm from its target in the box.

## Stopping

Stop is always on screen, top right, and so is Ctrl+Shift+S. It calls the executor's `StopAll`:
the tree halts, every goal its skills started is cancelled, the arms and legs hold where they are,
and a hand keeps what it holds. The button says so when it matters: "Stop · right hand keeps
red_block". A pick stopped once the hand has closed keeps the object in the hand. On the real
robot the remote's emergency stop is the one that counts.

## The executor

`nervros_executor` (`g1_orchestration`) runs one mission at a time. It checks every tree before
running it: the hash you approved, only its own macros and the plain control nodes, and bounded
timeouts and retries. It takes the arms before the first step when a skill needs them, and hands
them back when the mission ends unless a hand holds, or may hold, something: after a pick that was
cut off, the executor does not trust that hand until a place with it succeeds, and the agent plans
as if it were full. The skills a plan may use are in `config/catalog.yaml`, as macros in
`trees/library/`: `GoToTarget`, `GoToPose`, `PickObject`, `PlaceInto`, `TuckForTravel` and
`ExploreBuilding`.

A mission can also be sent by hand, which is how to check a tree without the agent:

```bash
ros2 run g1_orchestration send_mission.sh $(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/missions/pick_and_place.xml validate
```

`validate` checks it, `dry-run` runs it with every skill replaced by a timed stand-in, and
`execute` runs it.

## Models

The app starts with the local model. NervROS can use other providers, set in its models file; this
project uses free models only, and NervROS refuses a paid OpenRouter model at start-up.
