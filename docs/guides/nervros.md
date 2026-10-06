# Talking to the robot with NervROS

NervROS (`workspace/src/nervros`, its own repository) is an agent you talk to in a desktop app. It
looks through the chest or head camera and marks what the detector finds, segments whatever you
name in the view, asks canopy where things are, and
does physical tasks as missions: behaviour trees that `nervros_executor` checks, you approve, and
the executor runs. It runs on the host and answers with a local model unless you choose otherwise.

`./scripts/demos/nervros.sh <app|chat|explore|map>` opens a section's commands at once, in
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
- The profile, `g1_bringup/config/nervros/nervros.toml`, names the G1's cameras, detectors, world
  model topics, the executor and the models file (`models.toml` beside it), and a short persona
  (`apartment.md`). NervROS's `docs/profile.md` explains each key.

Things to ask:

- "What do you see? Is there a sofa?" The agent calls `look` with the question: the detector's finds
  are drawn on the frame as numbered marks, the local vision model looks at that frame and
  answers, and you see the frame in the chat. It looks through the chest camera, about 1 m up and
  20 degrees down, unless you name the head camera, which points further down at the floor ahead;
  the profile tells both models what each camera sees.
- "Give me a segmented image of just the floor." `segment` masks whatever you name, including
  things no detector marks, such as the floor or a wall, and shows it cut out of the frame, or
  tinted over it if you ask for an overlay. By default Gemini's free tier outlines it: 3.8 Flash in
  5 to 30 s, up to 10 times a day, then Flash-Lite in about 2 s, coarser. The local server gives
  pixel masks from SAM 3.1 in about half a second (`./scripts/serve.sh canopy --detector sam3.1
  --embedder none --describer none`, and "segment the floor with the local server"): on the
  apartment's renders its floor masks matched the truth at IoU 0.95, Flash-Lite's outlines at 0.80.
  It does not fit on the GPU beside the local chat model: use it with a cloud chat model, or with
  none from the command line, where
  `./scripts/demos/nervros.sh --print chat`'s last command with `segment "the floor" --backend
  service --out floor.jpg` in place of `chat` segments once. Without `~/.config/grove/gemini.env`
  (the bare key) only the local server segments.
- "Where is the dustbin?" `find_objects` asks canopy, which answers with ids, rooms, and how much of
  each room the camera has seen, so "not found" comes with how sure it is.
- "Point at the fridge's handle." `point` asks Gemini's free tier to point at what you name in the
  newest frame, including things no detector marks, and draws rings where it points; a point on one
  of the detector's marks says which, so the agent can use that object with other tools. "Look
  closer at mark 2: what does its label say?" crops the mark from the full frame at a higher
  resolution for the vision model.
- "What happened to the mug today?" `recall` reads the missions the robot ran (kept in
  `~/.local/state/nervros/`), canopy's history of the object (when it appeared, moved, went missing
  or was seen again) and the notes you asked it to keep. A failed pick's report says where the
  object was last seen too.
- "Walk to the living room." The agent writes a plan, and a plan card appears with its steps. Arm
  the robot with the switch in the top bar, then approve the plan in the chat or the dock: that is
  the one approval, and the agent never asks "shall I?" first, so deny what is wrong. The card
  follows the mission step by step, and the agent tells you how it ended. When a mission fails, the
  agent looks into why and proposes a changed plan for you to approve, up to twice.
- "Walk 1 metre forward, then turn left 90 degrees." Small exact moves are `WalkStraight` (0.1 to
  2 m, forward or backward) and `TurnInPlace` (up to 180 degrees). They go through Nav2's
  behaviors, which project the motion on the local costmap and stop the robot before it hits
  something: the step then fails and the agent says so. They never walk around anything.
- "Check the robot's health." `health_check` runs the connection check, measures each camera's
  frame rate, checks the robot knows where it is on the map and reads the executor's state, and
  the agent tells you what is wrong first.
- "Tell me if the chest camera drops below 5 Hz." `watch` keeps an eye on a topic in the
  background, its rate, a field crossing a value or a log line, and the agent tells you in the chat
  when it happens. "Which watches are running?" lists them, and you can cancel them.
- "Plot the robot's forward speed." `plot` draws a number from a topic over time in the viewer's
  Plots tab, below the 3D view.
- "Remember this spot as the reading corner." `tag_place` saves where the robot stands and which
  way it faces as a place, for "walk to the reading corner" later, in this session or the next. It
  is kept in `~/.local/state/nervros/places/`, one file per robot; "forget the reading corner"
  drops it.
- "Remember that the kitchen door sticks." `memory` keeps what you ask it to across sessions, in
  `~/.local/state/nervros/memory/`; the Agent tab (Ctrl+6) lists the notes with Forget.
- "Every 30 minutes, walk to the bedroom and back, 4 times." `schedule` runs a plan again and
  again: you approve it once for all its runs. A run is skipped while the robot is disarmed or busy,
  and Stop cancels every schedule. The Mission tab lists them with Cancel. A request with a clock is
  always a schedule, and the agent's every and times must match your words.
- "When a mug appears in the kitchen, walk to the hallway." A schedule can wait for something
  instead: an object of a kind turning up in a room, or a condition on a topic as `watch` takes
  it. It runs once each time that becomes true, not for what holds already, for up to an hour.

The viewer draws the map, the rooms coloured by how much of each the camera has seen, the objects,
the G1's model with its joints as they are, Nav2's path and the chest camera's view in 3D. Beside
it are the chest camera and, below it in tabs, the head camera and the last frame the agent looked
at, each with the detector's boxes. The profile adds what the world-model RViz view shows: canopy's
walls and floor plan, its doorways, viewpoints and visits, and the laser scan. The Layers tab
(Ctrl+3) switches each of them on and off. Object names start off, since 50 of them bury the map;
hover a box to see its name. Click an object, a room or a point on the map there and a bar above
the message box offers "Go there", "What is it?" and the like; it fills in the message, which you
send. The strip under the views holds the agent's log, the running mission's steps as lanes on the
timeline, the exploration's progress and the plots. The dock's World tab (Ctrl+2) lists the rooms
with how much of each has been seen.

## Plans and approvals

A plan comes as a card with its steps, before anything moves. The executor previews it: the viewer
draws where each walk ends and Nav2's path to it, and a step the preview finds unreachable is
marked on the card. Each step shows how it went before on this robot ("11 of 11 · 2 s"), from the
mission ledger. Before you see a plan, NervROS checks it against your words: left or right, forward
or back, how far, how much and which hand. A plan that turns right when you said left goes back to
the model once; if it comes back unchanged, the card shows the concern and Apply fixes. Edit, on the
card, changes a step's argument or drops or moves a step; the changed plan is checked again before
you can approve it. "Pick it up" as the first thing you say is not guessed at: the agent asks which
thing.

Approving a tool that does not move the robot, such as a parameter change, can cover the rest of
the session (Allow for session); what moves the robot asks every time. When the app closes with a
plan waiting, the next start shows it with Ask again, which checks it anew. A mission's end comes
as a toast too, and the Mission tab keeps the recent missions with their steps and times, plans you
asked to save by name, and the requests no skill could do (`nervros-cli missions`, `replay` and
`gaps` read the same from the command line). When the local model's plans for one request fail
their checks twice, a stronger model (Gemini's free tier, when the key is there) advises it how to
fix them.

Ctrl+K opens the command palette; its commands also run from the message box by name, such as
`/compact` or `/stop`. An image pasted or dropped on the window goes with your next message, for
the agent to look at.

## The robot tab

The dock's Robot tab (Ctrl+8) shows whether the robot is upright and able to walk, its tilt, where
it is and in which room, what each hand holds, and its hottest motors from `/diagnostics`. Drive,
there, hands the base to you: the executor takes it while no mission runs, W and S or the up and
down arrows walk, A and D or the side arrows turn, Q and E step sideways, at a capped speed and
through Nav2's collision monitor, which still stops the robot short of what it sees. The base
stops as soon as the commands pause; Slow halves the speed.

## Fixing the world model

`nervros.sh app` also starts canopy's world editor on the saved world. Edit world, in the top bar,
swaps the viewer for the world on its floor plan, the way canopy's editor page draws it: click an
object or a room, drag a box by its corners, its handle or its middle to resize, turn or move it,
Add object and Split off drag out a box, Merge with joins two objects, and the World tab holds the
selected thing's label, name, votes and the camera's view of it. C marks it checked, N goes to the
next thing to review, Ctrl+Z undoes, Ctrl+S saves. The agent edits the same world when you ask:
"the chair by the window is a stool", "O31 and O32 are one sofa", "call the study Anna's room". It
reads with `review_world` and `inspect_object`, edits with `edit_world` and saves with
`world_edits`, which has canopy read the world again. Edits ask for your approval but need no
arming: they change what the robot knows, not what it does.

Both go through the editor: the app shows the agent's edits within two seconds, and the agent
reads yours when it next looks. canopy saves the
world itself every minute while its detector runs, so a save may first replay the edits over
canopy's newer save; that is automatic.

- "Bring the small red mug from the dining table to the tray on the office desk." `app` runs the
  arm stack too: MoveIt, the manipulation skills and a mock detector that knows the mug and the
  tray by their names in the scene, `mug_4` and `tray_1`. The agent walks to `dining_table_side`,
  picks the mug with the left hand, carries it to `office_desk_tray` and sets it in the tray: in
  the test run, 177 s. The world model's ids for the two do not reach that detector, so the
  executor accepts only these names in a pick or a place (`arg_choices`), and a plan that names
  another comes back with the names it takes. It pins their phrases too, "red mug" and "wooden
  tray", which a real detector finds them by.

## Exploring: watching the map fill in

`nervros.sh explore` is the apartment with an empty world model: canopy knows the map and nothing
else. Ask "Explore the building", or press Explore in the World tab. The agent plans
`ExploreBuilding`, a skill of the executor that walks to viewpoints canopy picks until the camera
has seen every room; after your approval the rooms turn from red to green in the viewer, objects
appear, and the "Exploring" plot tracks the share seen. One run explores for up to 23 minutes and
saves the world; ask again to carry on. In the test run from empty, canopy called the apartment done
after 22 minutes: all 6 rooms and 32 objects, with about 80% of the floor and 62% of the walls
seen. canopy wrote the rest off as out of sight from anywhere the robot could reach.

## Mapping a world from nothing

`nervros.sh map` builds the map as the robot goes: SLAM instead of the saved map, and an empty
world model in `data/worlds/mapped-apartment` (`name:=` names it, `world:=` picks another simulated
world). Ask "Explore the building" and arm the robot: `ExploreBuilding` walks to canopy's frontiers
first, while the viewer shows the map growing, the rooms turning green as the camera covers them,
and the objects and viewpoints as they come. canopy saves the world, floor plan included, every
minute and at the end of each run of up to 23 minutes; ask again to carry on. The editor serves the
new world once it is first saved. Later, `nervros.sh app saved:=mapped-apartment` runs the robot on
it, localizing on its floor plan. A name already mapped is kept: pick another or delete it.

## Stopping

Stop is always on screen, top right, and so is Ctrl+Shift+S. It calls the executor's `StopAll`:
the tree halts, every goal its skills started is cancelled, the arms and legs hold where they are,
and a hand keeps what it holds. The button says so when it matters: "Stop · right hand keeps
red_block". A pick stopped once the hand has closed keeps the object in the hand. On the real
robot the remote's emergency stop is the one that counts.

A mission stops by itself in two cases. The app sends the executor a heartbeat several times a
second while a mission runs; if they stop for 2 s, because the app closed, crashed or hung, the
executor stops the mission and the robot holds. And the executor refuses to start a walk, or stops
one, when the robot has fallen or its balance controller is not running.

## The executor

`nervros_executor` (`g1_orchestration`) runs one mission at a time. It checks every tree before
running it: the hash you approved, only its own macros and the plain control nodes, and bounded
timeouts and retries. It takes the arms before the first step when a skill needs them, and hands
them back when the mission ends unless a hand holds, or may hold, something: after a pick that was
cut off, the executor does not trust that hand until a place with it succeeds, and the agent plans
as if it were full. The skills a plan may use are in `config/catalog.yaml`, as macros in
`trees/library/`: `GoToTarget`, `GoToPose`, `WalkStraight`, `TurnInPlace`, `PickObject`,
`PlaceInto`, `TuckForTravel` and `ExploreBuilding`.

A mission can also be sent by hand, which is how to check a tree without the agent:

```bash
ros2 run g1_orchestration send_mission.sh $(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/missions/pick_and_place.xml validate
```

`validate` checks it, `dry-run` runs it with every skill replaced by a timed stand-in, and
`execute` runs it.

## Sessions and the model's context

Every conversation is saved next to its log in `~/.local/state/nervros/logs/`. The Agent tab lists
the recent ones, and Resume carries one on; from the command line, `nervros-cli chat --resume
last`. The local model holds 16k tokens: the status bar shows how full the latest request was, and
how many model calls the session made with the share read from llama.cpp's prompt cache. Before the
conversation fills, the results of earlier requests are cut to a line; if that is not enough, the
model sums up the older part as goal, done, open and facts. `/compact` in the message box condenses
it at once, and right-clicking one of your messages condenses everything up to it. Replies stream
into the chat as they are written.

For the G1's rare troubles the agent has skills, short procedures in
`g1_bringup/config/nervros/skills/` that it reads when one fits: Nav2 that never came up, a camera
or detector that gives nothing, a robot lost on its map, and a robot that fell.

## Testing the agent

`g1_bringup/config/nervros/eval/apartment.toml` holds 44 requests an operator might make, from "what
can you see" to a full pick and place, each with what the agent should do. Against the apartment
stack with the arm (as `nervros.sh app` starts it) and the local model, from
`workspace/src/nervros` with the demo's environment:

```bash
cargo run -p nervros-cli -- --profile ../g1_bringup/config/nervros/nervros.toml \
  eval ../g1_bringup/config/nervros/eval/apartment.toml
```

Each case runs in a fresh session with every approval granted, and the robot carries its place
and what it holds from case to case. Moves and turns are judged on where the simulator says the
pelvis is, not on the robot's own estimate. The report lands in `~/.local/state/nervros/evals/`;
`--only <text>` runs the cases whose ids hold it, `--repeat 3` runs each three times for pass^3,
and `--models a,b` compares two entries of `models.toml`. The report gives each model's pass rate
with its interval, its model calls, time and tokens per case, and the llama.cpp build and template
it ran on.

The window can be driven the same way: NervROS's `live_eval` test types prompts into the real app
against the running stack, approves its cards, and saves the window at each step for a person to
look over (its header says how).

## Models

The app starts with the local model. NervROS can use other providers, set in its models file; this
project uses free models only, and NervROS refuses a paid OpenRouter model at start-up. With
Gemini's key in `~/.config/grove/gemini.env`, its free tier outlines for `segment`, points for
`point` and advises the planner; without it those fall back or stay off, and the rest runs locally.
When the local model is not running, as when SAM 3.1 holds the GPU (the two do not fit in 12 GB
together), the same free tier talks and looks instead: Gemini 3.1 Flash-Lite, then Gemma 4. Both
have tight limits: Flash-Lite takes 15 requests a minute, and Gemma 16K input tokens a minute,
which one or two agent calls use up. A `home` privacy mode keeps those models out.
