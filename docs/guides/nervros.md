# Talking to the robot with NervROS

NervROS (`workspace/src/nervros`, its own repository) is an agent you talk to in a desktop app. It
looks through the head camera and marks what the detector finds, asks canopy where things are, and
does physical tasks as missions: behaviour trees that `nervros_executor` checks, you approve, and
the executor runs. It runs on the host and answers with a local model unless you choose otherwise.

`./scripts/demos/nervros.sh <app|chat|facility>` opens a section's commands at once, in split panes
of your terminal, after tearing down any stack left running. `stop` ends it, and `--print` lists
the commands instead.

## Once

The local model is Qwen3.5-9B, quantized to 4 bits, in llama.cpp's server in its own container.
It needs about 7 GB of GPU memory. Fetch it once on the host:

```bash
hf download unsloth/Qwen3.5-9B-GGUF Qwen3.5-9B-Q4_K_M.gguf mmproj-F16.gguf
```

The app builds on the host with Rust, clang, mold and ROS 2 Jazzy; NervROS's README lists what to
install. The first build compiles the Rerun viewer and takes about twenty minutes.

## In the apartment: looking, finding, walking

In the container: the apartment with SLAM and Nav2, canopy with the mock detector, and the
executor.

```bash
ros2 launch g1_bringup bringup.launch.py mode:=mapping nav:=true world:=apartment headless:=true \
  rviz:=false arms_at_sides:=true cameras:=head
ros2 launch g1_bringup world_model.launch.py world_dir:=/root/data/worlds/nervros rviz:=false \
  detector:=mock cameras:=head
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
  model topics, the executor and the models file. NervROS's `docs/profile.md` explains each key.

Things to ask:

- "What do you see?" The agent calls `look`: a camera frame with the detections drawn as numbered
  marks, which appears in the chat. It answers by mark number and label.
- "Where is the dustbin?" `find_objects` asks canopy, which answers with ids, rooms, and how much of
  each room the camera has seen, so "not found" comes with how sure it is.
- "Go to the kitchen." The agent writes a plan, and a plan card appears with its steps. Arm the
  robot with the switch in the top bar, then approve the plan in the chat or the dock. The card
  follows the mission step by step, and the agent tells you how it ended.

The apartment has no arm stack, so only walking skills run there.

## In the facility: pick and place

`nervros.sh facility` starts the pick-and-place stack from the [pick and place](pick-and-place.md)
guide, the executor, and the app with `g1_bringup/config/nervros/facility.toml`. The facility has
no world model: the profile names the two benches as places, `workbench` and `storage`, and a
short persona (`facility.md`) tells the model the detector's names for the objects, `red_block`
and `brown_box`.

Ask: "Pick up the red_block from the workbench and put it into the brown_box on the storage
bench." The agent plans four steps: walk to the workbench, pick the ball, walk to the storage
bench, place it into the box. In the test run the plan passed its checks at once and the mission
took about four minutes; the ball came to rest 5 cm from its target in the box.

## Stopping

Stop is always on screen, top right, and so is Ctrl+Shift+S. It calls the executor's `StopAll`:
the tree halts, every goal its skills started is cancelled, the arms and legs hold where they are,
and a hand keeps what it holds. The button says so when it matters: "Stop · right hand keeps
red_block". A pick stopped while it lifts still drops the object: the pick action opens the hand
when it is cancelled. On the real robot the remote's emergency stop is the one that counts.

## The executor

`nervros_executor` (`g1_orchestration`) runs one mission at a time. It checks every tree before
running it: the hash you approved, only its own macros and the plain control nodes, and bounded
timeouts and retries. It takes the arms before the first step when a skill needs them, and hands
them back when the mission ends unless a hand holds, or may hold, something: after a pick that was
cut off, the executor does not trust that hand until a place with it succeeds, and the agent plans
as if it were full. The skills a plan may use are in `config/catalog.yaml`, as macros in
`trees/library/`: `GoToTarget`, `GoToPose`, `PickObject`, `PlaceInto` and `TuckForTravel`.

A mission can also be sent by hand, which is how to check a tree without the agent:

```bash
ros2 run g1_orchestration send_mission.sh $(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/missions/pick_and_place.xml validate
```

`validate` checks it, `dry-run` runs it with every skill replaced by a timed stand-in, and
`execute` runs it.

## Models

The app starts with the local model. NervROS can use other providers, set in its models file; this
project uses free models only, and NervROS refuses a paid OpenRouter model at start-up.
