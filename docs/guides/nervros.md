# Talking to the robot with NervROS

NervROS (`workspace/src/nervros`, its own repository) is an agent you talk to in a desktop app. It
runs on the host and answers with a local model unless you choose otherwise. It can:

- look through the chest or head camera and mark what the detector finds,
- segment whatever you name in the view,
- ask canopy where things are,
- do physical tasks as missions: behaviour trees that `nervros_executor` checks, you approve, and
  the executor runs.

`./scripts/demos/nervros.sh <app|chat|explore|map>` tears down any stack left running, then opens
a section's commands at once, in split panes of tilix (a window each in another terminal). `stop`
ends it, and `--print` lists the commands instead.

- `chat` is `app`'s stack, with NervROS in the terminal instead of the window.
- `saved:=<a world under data/worlds>` runs `app`, `chat` or `explore` on another saved world.
- `map` takes `name:=` and `world:=`.
- Any other `name:=value` goes to the bringup launch.

## Once

The local model is Qwen3.5-9B, quantized to 4 bits, in llama.cpp's server in its own container.
It needs about 7 GB of GPU memory. Fetch it once on the host:

```bash
hf download unsloth/Qwen3.5-9B-GGUF Qwen3.5-9B-Q4_K_M.gguf mmproj-F16.gguf
```

The app builds on the host with Rust, clang, mold and ROS 2 Jazzy; NervROS's README lists what to
install. The first build compiles the Rerun viewer and takes about twenty minutes. The apartment's
meshes are fetched once too, on the host, about 330 MB: `./scripts/setup-world-assets.sh`.

## In the apartment: looking, finding, walking

The apartment runs with the semantic map canopy built of it, corrected by hand
(`workspace/src/canopy/canopy/doc/apartment`). It has six rooms with their types (living room,
bedroom, office, study, hallway, storage room) and 54 named objects, so the agent knows the rooms
and where things are from the start.

canopy writes into its world, so the demo gives it a copy, `data/worlds/nervros-apartment`, made
once and kept. Delete it to start again from the committed map. The robot localizes on that map's
own grid, which the saved world needs to resume.

In the container, the demo runs the apartment with AMCL, Nav2 and the arm stack (MoveIt and the
pick and place skills), canopy on the copy with the mock detector, and the executor. The mock
detector needs no GPU. SAM 3.1 (5.6 GB, 8.5 GB while mapping) and the local model (about 7 GB) do
not fit beside the simulator on a 12 GB GPU, and with SAM 3.1 the chat moves to free cloud models
(see [Models](#models)).

```bash
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true keepout:=true world:=apartment \
  map:=/root/data/worlds/nervros-apartment/map.yaml headless:=true rviz:=false cameras:=head,chest \
  moveit:=true manipulation:=true perception:=true detector:=mock activate_arm:=true \
  activate_arm_delay_s:=8.0 phrases:='mug_4=red mug,tray_1=wooden tray'
ros2 launch g1_bringup world_model.launch.py rviz:=false detector:=mock cameras:=head,chest \
  segmenter:=true world_dir:=/root/data/worlds/nervros-apartment
ros2 launch g1_orchestration nervros_executor.launch.py hands_empty_on_attach:=true \
  learned_skills_dir:=/root/data/skills/g1 \
  arg_choices:='PickObject.object_id=mug_4;PickObject.phrase=red mug;PlaceInto.container_id=tray_1;PlaceInto.phrase=wooden tray;FindInView.phrase=mug|tray'
```

The launcher starts the executor once Nav2 is up and the arm has swung clear (`wait_for_nav2` and
`wait_for_arm` in `scripts/lib/panes.sh`); by hand, wait for the same. `nervros.sh explore` runs
without the arm stack: `arms_at_sides:=true` instead, canopy on
`/root/data/worlds/nervros-explore`, and no `arg_choices`.

On the host, from the repository root:

```bash
workspace/src/nervros/scripts/local-llm.sh start
python3 workspace/src/canopy/editor/canopy_editor.py data/worlds/nervros-apartment &   # the world editor
cd workspace/src/nervros
./scripts/build-overlay.sh ../g1_msgs ../canopy/canopy_msgs
export NERVROS_UDP_ONLY=1 NERVROS_EXTRA_IDL_PACKAGES="g1_msgs;canopy_msgs" ROS_DOMAIN_ID=1 \
  ROS_PACKAGE_PATH=$PWD/../../build
export CARGO_TARGET_DIR=~/.cache/nervros/target   # outside the colcon workspace
source scripts/ros-env.sh
cargo run -p nervros-gui -- --profile ../g1_bringup/config/nervros/nervros.toml
```

- `NERVROS_UDP_ONLY=1` turns Fast DDS shared memory off. The container runs as root, and shared
  memory between two users fails without an error: topics appear, but no data arrives.
- `ROS_PACKAGE_PATH` points at the colcon build, where the viewer finds the G1's meshes. `install/`
  only links to the container's paths.
- The profile, `g1_bringup/config/nervros/nervros.toml`, names the G1's cameras, detectors, world
  model topics, the executor and the models file (`models.toml` beside it), and a short persona
  (`apartment.md`). NervROS's `docs/profile.md` explains each key.

### Things to ask: looking and finding

- "What do you see? Is there a sofa?" The agent calls `look` with the question. The detector's
  finds are drawn on the frame as numbered marks, the local vision model looks at that frame and
  answers, and you see the frame in the chat. It looks through the chest camera, about 1 m up and
  20 degrees down, unless you name the head camera, which points further down at the floor ahead.
  The profile tells both models what each camera sees.
- "Give me a segmented image of just the floor." `segment` masks whatever you name, including
  things no detector marks, such as the floor or a wall. It shows the mask cut out of the frame, or
  tinted over it if you ask for an overlay. See [Segmenting](#segmenting) for which model outlines.
- "Where is the dustbin?" `find_objects` asks canopy, which answers with ids, rooms, and how much
  of each room the camera has seen, so "not found" comes with how sure it is.
- "Point at the fridge's handle." `point` asks Gemini's free tier to point at what you name in the
  newest frame, including things no detector marks, and draws rings where it points. A point on one
  of the detector's marks says which mark, so the agent can use that object with other tools.
- "Look closer at mark 2: what does its label say?" crops the mark from the full frame at a higher
  resolution for the vision model.
- "What happened to the mug today?" `recall` reads the missions the robot ran (kept in
  `~/.local/state/nervros/`), canopy's history of the object (when it appeared, moved, went missing
  or was seen again) and the notes you asked it to keep. A failed pick's report also says where the
  object was last seen.

#### Segmenting

By default Gemini's free tier outlines: 3.8 Flash in 5 to 30 s, up to 10 times a day, then
Flash-Lite in about 2 s, coarser. The local server gives pixel masks from SAM 3.1 in about half a
second (`./scripts/serve.sh canopy --detector sam3.1 --embedder none --describer none`, and
"segment the floor with the local server"). On the apartment's renders, its floor masks matched the
truth at IoU 0.95, and Flash-Lite's outlines at 0.80.

SAM 3.1 does not fit on the GPU beside the local chat model: use it with a cloud chat model, or with
none from the command line. There, `./scripts/demos/nervros.sh --print chat`'s last command, with
`segment "the floor" --backend service --out floor.jpg` in place of `chat`, segments once. Without
`~/.config/grove/gemini.env` (the bare key), only the local server segments.

### Things to ask: walking and the arm

- "Walk to the living room." The agent writes a plan, and a plan card appears with its steps. Arm
  the robot with the switch in the top bar, then approve the plan in the chat or the dock. That is
  the one approval: the agent never asks "shall I?" first, so deny what is wrong. The card follows
  the mission step by step, and the agent tells you how it ended. When a mission fails, the agent
  looks into why and proposes a changed plan for you to approve, up to twice.
- "Walk 1 metre forward, then turn left 90 degrees." Small exact moves are `WalkStraight` (0.1 to
  2 m, forward or backward) and `TurnInPlace` (up to 180 degrees). They go through Nav2's
  behaviors, which project the motion on the local costmap and stop the robot before it hits
  something; the step then fails and the agent says so. They never walk around anything.
- "Bring the small red mug from the dining table to the tray on the office desk." `app` runs the
  arm stack too: MoveIt, the manipulation skills and a mock detector that knows the mug and the
  tray by their names in the scene, `mug_4` and `tray_1`. The agent walks to `dining_table_side`,
  picks the mug with the left hand, carries it to `office_desk_tray` and sets it in the tray; in the
  test run this took 177 s.
  - The world model's ids for the two do not reach that detector, so the executor accepts only
    these names in a pick or a place (`arg_choices`). A plan that names another comes back with the
    names it takes.
  - It also pins their phrases, "red mug" and "wooden tray", which a real detector finds them by,
    and a look to `mug` or `tray`.
- "Walk to the red mug, look for a mug there, and pick up the one you find with your left hand."
  One mission walks, looks with `FindInView` and picks (see [The executor](#the-executor)).

### Things to ask: the robot and ROS

- "Which Nav2 nodes are running?" or "What is the controller server's maximum forward speed?" The
  ROS tools list nodes under the component container that hosts them (AMCL and the map server run
  in `/nav2_container`, Nav2's servers outside it), and give each parameter's value with its
  description.
- "Check the robot's health." `health_check` runs the connection check, measures each camera's
  frame rate, checks that the robot knows where it is on the map, and reads the executor's state.
  The agent tells you what is wrong first.
- "Tell me if the chest camera drops below 5 Hz." `watch` keeps an eye on a topic in the
  background (its rate, a field crossing a value, or a log line), and the agent tells you in the
  chat when it happens. "Which watches are running?" lists them, and you can cancel them.
- "Plot the robot's forward speed." `plot` draws a number from a topic over time in the viewer's
  Plots tab, below the 3D view.

### Things to ask: remembering and schedules

- "Remember this spot as the reading corner." `tag_place` saves where the robot stands and which
  way it faces as a place, for "walk to the reading corner" later, in this session or the next. It
  is kept in `~/.local/state/nervros/places/`, one file per robot, and "forget the reading corner"
  drops it.
- "Remember that the kitchen door sticks." `memory` keeps what you ask it to across sessions, in
  `~/.local/state/nervros/memory/`. The Agent tab (Ctrl+6) lists the notes with Forget.
- "Keep the robot in the bedroom." `goal` keeps an outcome across missions and sessions, with what
  holds once it is met in the world model's ids, such as `at(R3)` or `inside(O27, O261)`. NervROS
  checks it itself after every mission the agent runs, and at start; the agent plans the missions
  toward it, the first at once, which you approve as usual, until it holds or its five missions are
  used. The world model may give an object the robot moved another id, so for `inside` and `on` an
  object of the same kind seen there after the goal's own object was last seen counts, once that
  object is gone from where it was or when the one there was seen after the goal was set. The Agent
  tab lists the goals with Drop.

https://github.com/user-attachments/assets/2cbd5b3b-22d3-4f12-84ce-f697941a21d9

In the clip, "set a goal to keep the red mug on the wooden tray, then work toward it": the goal is
approved, one pick-and-place runs toward it (four times faster), the camera agrees the mug is in
the tray, and the report ends "goal 1 (keep the red mug on the wooden tray): met".

- "Tell me when someone puts a box on the sofa." A watch can wait for what a camera shows: a
  vision model looks every 15 seconds, and two yes in a row post to the chat.
- "When did the red mug leave the dining table?" The cameras keep a frame each time their view
  changes, for a day in this demo, and `when` asks a vision model which first shows it, around the
  moments the world model saw the mug move; a second model then checks that frame alone, or its
  neighbours until one shows it. It answers with the time and the frame.

https://github.com/user-attachments/assets/1ebe06cc-de7a-490f-b902-597ea91f72d6

In the clip "tell me when the head camera shows a red mug" fires once the robot stands at the
dining table, and "when did the head camera first show the red mug?" answers with the frame where
the mug comes into view.

- "Check on the living room and the bedroom." `patrol` walks the round, turns once around in each
  room so the cameras sweep it, and reports what it saw there and what moved or went missing in
  the world model meanwhile.
- "How did today go?" `digest` counts the missions by how they ended, sets apart the runs the
  robot itself cut short, and gives each skill's record, the incidents and the goals.
  `nervros-cli digest` prints the same.

https://github.com/user-attachments/assets/e0343475-e62f-4f6a-8e6a-5a693125325d

In the clip the patrol looks around each room and reports what it saw, room by room ("the bedroom
(R3): a bed, a clock, a nightstand and a potted plant"), and the digest then sums up the day.
Gemini 3.5 Flash-Lite answered in the goal and patrol clips, the local Qwen3.5-9B in the watch.

- "Every 30 minutes, walk to the bedroom and back, 4 times." `schedule` runs a plan again and
  again, and you approve it once for all its runs. A run is skipped while the robot is disarmed or
  busy, and Stop cancels every schedule. The Mission tab lists them with Cancel. A request with a
  clock is always a schedule, and the agent's every and times must match your words.
- "When a mug appears in the kitchen, walk to the hallway." A schedule can wait for something
  instead: an object of a kind turning up in a room, or a condition on a topic as `watch` takes it.
  It runs once each time that becomes true, not for what holds already, for an hour unless you say
  longer, at most a day.

### The viewer

The viewer draws the map, the rooms coloured by how much of each the camera has seen, the objects,
the G1's model with its joints as they are, Nav2's path and the chest camera's view in 3D. Beside it
is the chest camera and, below it in tabs, the head camera and the last frame the agent looked at,
each with the detector's boxes. The profile adds what the world-model RViz view shows: canopy's
walls and floor plan, its doorways, viewpoints and visits, and the laser scan.

- The Layers tab (Ctrl+3) switches each of these on and off. Object names start off, since 50 of
  them bury the map; hover a box to see its name.
- Click an object, a room or a point on the map, and a bar above the message box offers "Go there",
  "What is it?" and the like. Go there plans the walk at once, for you to approve; the others fill
  in a message for you to send.
- The strip under the views holds the agent's log, the running mission's steps as lanes on the
  timeline, the exploration's progress and the plots.
- The dock's World tab (Ctrl+2) lists the rooms with how much of each has been seen.

## Plans and approvals

A plan comes as a card with its steps, before anything moves.

- The executor previews it: the viewer draws where each walk ends and Nav2's path to it, and a step
  the preview finds unreachable is marked on the card.
- Each step shows how it went before on this robot ("11 of 11 · 2 s"), from the mission ledger.
- Before you see a plan, NervROS checks it against your words: left or right, forward or back, how
  far, how much and which hand. A plan that turns right when you said left goes back to the model
  once. If it comes back unchanged, the card shows the concern, and Apply fixes it.
- A plan with a step that has worked at most half the time before also goes to the scene check,
  a vision model shown what the cameras see now. It can only add doubts, which show under the step
  and in the approval.
- **Edit**, on the card, changes a step's argument or drops or moves a step. The changed plan is
  checked again before you can approve it.
- The check also covers keep-out zones and steps that take an earlier step's result.
- "Pick it up" as the first thing you say is not guessed at: the agent asks which thing.

https://github.com/user-attachments/assets/91718cec-719c-4fad-96ff-6c210f32c287

In the clip `FindInView` for the tray had worked once in five, and the scene check's doubt, "there
is no tray in the current view", shows on the card before anything moves.

Approving a tool that does not move the robot, such as a parameter change, can cover the rest of
the session (Allow for session). What moves the robot asks every time. When the app closes with a
plan waiting, the next start shows it with Ask again, which checks it anew.

A mission's end comes as a toast too. The Mission tab keeps the recent missions with their steps
and times, the plans you asked to save by name, and the requests no skill could do.
`nervros-cli missions` and `gaps` read the same from the command line, and `replay` prints a
session. When the local model's plans for one request fail their checks twice, a stronger model
(Gemini's free tier, when the key is there) advises it how to fix them.

Ctrl+K opens the command palette. Its commands also run from the message box by name, such as
`/compact` or `/stop`. An image pasted or dropped on the window goes with your next message, for
the agent to look at.

## Programs, searches and taught skills

https://github.com/user-attachments/assets/c46fbd2f-89fd-4069-83ee-c0dcf27b8c6f

For anything that takes a loop, a count or a measurement, the agent writes a short program and runs
it as a job.

- "Count the chairs in every room" or "which room has the most objects?" reads canopy's world model
  and moves nothing, so it runs at once.
- "Run a program that looks for a mug in the office, room R4, then at the dining table, and stops
  at the first place where the camera sees one" walks and looks. The approval names what it may
  run, "may move the base (GoToPlace) and look (FindInView), for at most 5 min", and the chat shows
  a card with its latest lines and Stop.
- Every mission a program sends is checked like a plan, and an empty look in one place does not
  end the others: in the clip the office look saw nothing, and the program found the mug at the
  dining table.
- In the apartment demo the mock detector knows only `mug` and `tray`, so a look asks for one of
  those.

https://github.com/user-attachments/assets/51157255-5ddb-487b-b77f-5181f9481dd7

"Find the red mug" ranks where to look: where canopy saw it and how long ago, where such things
usually are, the rooms the camera has seen least of, and how far each is. It then walks to each
place and looks until the detector sees it. "Where could my keys be? Don't move" lists the places
without walking: canopy has never seen keys, so the clip ranks the rooms keys are usually in.

https://github.com/user-attachments/assets/4e1bf457-9f63-498c-b293-d18661542491

The executor lets the agent teach the robot a skill no macro has, from the leaves listed in
`g1_orchestration`'s `learnable_leaves`: Nav2's walk, the bounded walk and turn, the detector's
look, named arm postures, `Wait`, and `MoveHand`. `MoveHand` plans one hand with MoveIt to a point,
or onto the line from the shoulder to it, within 0.48 m of the shoulder and at 60 % of the arm's
usual speed.

- "Point at the red mug with your right hand" has no skill, so the agent teaches `PointAt`. The
  executor checks it, and you approve the teaching, then the plan with its first try.
- `MoveHand` takes a `target` or a robot `frame`, never both. Models sometimes give it both, or an
  object's id as the frame; the executor finds that only when the step runs, so that first try
  fails and the agent teaches the skill again.
- Once that mission succeeds, the skill is kept in `data/skills/g1` (`nervros.sh` sets
  `learned_skills_dir`) and loads with the executor from then on.
- A taught skill holds no authority of its own: the mission takes the arm as for any arm skill,
  and a stop cancels its MoveIt goal.
- The Mission tab lists the taught skills with Forget.

Gemini 3.5 Flash-Lite answered in the programs and teaching clips, the local Qwen3.5-9B in the
search.

## The robot tab

The dock's Robot tab (Ctrl+8) shows whether the robot is upright and able to walk, its tilt, where
it is and in which room, what each hand holds, and its hottest motors from `/diagnostics`.

**Drive**, there, hands the base to you: the executor takes it while no mission runs. W and S walk,
A and D turn, and Q and E step sideways, at a capped speed and through Nav2's collision monitor,
which still stops the robot short of what it sees. The base stops as soon as the commands pause,
and Slow halves the speed.

## Fixing the world model

`nervros.sh app` also starts canopy's world editor on the saved world. **Edit world**, in the top
bar, swaps the viewer for the world on its floor plan, the way canopy's editor page draws it:

- Click an object or a room. Drag a box by its corners, its handle or its middle to resize, turn
  or move it.
- Add object and Split off drag out a box, and Merge with joins two objects.
- The World tab holds the selected thing's label, name, votes and the camera's view of it.
- C marks it checked, N goes to the next thing to review, Ctrl+Z undoes, and Ctrl+S saves.

The agent edits the same world when you ask: "the chair by the window is a stool", "O31 and O32
are one sofa", "call the study Anna's room". It reads with `review_world` and `inspect_object`,
edits with `edit_world`, and saves with `world_edits`, which has canopy read the world again. Edits
ask for your approval but need no arming: they change what the robot knows, not what it does.

Both go through the editor: the app shows the agent's edits within two seconds, and the agent
reads yours when it next looks. canopy saves the world itself every minute while its detector
runs, so a save may first replay the edits over canopy's newer save; that is automatic.

## Exploring: watching the map fill in

`nervros.sh explore` is the apartment with an empty world model: canopy knows the map and nothing
else. Ask "Explore the building", or press Explore in the World tab. The agent plans
`ExploreBuilding`, a skill of the executor that walks to viewpoints canopy picks until the camera
has seen every room.

After your approval, the rooms turn from red to green in the viewer, objects appear, and the
"Exploring" plot tracks the share seen. One run explores for up to 23 minutes and saves the world;
ask again to carry on. In the test run from empty, canopy called the apartment done after 22
minutes: all 6 rooms and 32 objects, with about 80% of the floor and 62% of the walls seen. canopy
wrote the rest off as out of sight from anywhere the robot could reach.

## Mapping a world from nothing

`nervros.sh map` builds the map as the robot goes: SLAM instead of the saved map, and an empty
world model in `data/worlds/mapped-apartment` (`name:=` names it, `world:=` picks another simulated
world).

Ask "Explore the building" and arm the robot. `ExploreBuilding` walks to canopy's frontiers first,
while the viewer shows the map growing, the rooms turning green as the camera covers them, and the
objects and viewpoints as they come. canopy saves the world, floor plan included, every minute and
at the end of each run of up to 23 minutes; ask again to carry on. The editor serves the new world
once it is first saved.

Later, `nervros.sh app saved:=mapped-apartment` runs the robot on it, localizing on its floor plan.
A name already mapped is kept: pick another or delete it.

## Stopping

Stop is always on screen, top right, and so is Ctrl+Shift+S. It calls the executor's `StopAll`: the
tree halts, every goal its skills started is cancelled, the arms and legs hold where they are, and a
hand keeps what it holds. The button says so when it matters: "Stop · right hand keeps red_block".
A pick stopped once the hand has closed keeps the object in the hand. On the real robot, the
remote's emergency stop is the one that counts.

A mission also stops by itself in two cases:

- The app sends the executor a heartbeat several times a second while a mission runs. If the beats
  stop for 2 s, because the app closed, crashed or hung, the executor stops the mission and the
  robot holds.
- The executor refuses to start a walk, or stops one, when the robot has fallen or its balance
  controller is not running.

https://github.com/user-attachments/assets/7bf2d1ef-5c5e-4873-b48f-7222fd3dc6f3

**Pause** (beside Stop while a mission runs, and on the plan card), typing "pause", "wait" or "hold
on", or `/pause` in the terminal halts the running step instead. The robot stops where it is, the
arms hold and the hands keep their grip, and a pick or a place finishes first. **Resume**, or
"resume", "continue", "carry on" or "go on", lets it go on: a walk to a place sets out again, and a
straight walk or a turn does only what was left of it.

- "Wait", "continue" and the like count only while a mission runs; otherwise they go to the model.
  A plain "pause" or "resume" always goes to the executor, which says so when nothing is running.
- The heartbeat still stops a paused mission whose window closed.
- A pause longer than ten minutes ends the mission (`max_pause_s` in
  `g1_orchestration/config/nervros_executor.yaml`).
- The executor's side is the `/nervros_executor/pause_mission` service.

## Keep-out zones

https://github.com/user-attachments/assets/3df05f4a-344a-4d54-a824-6ecfa8bebf57

"Keep the robot out of the bedroom", or out of a metre around the vase, adds a keep-out zone around
a room, an object, a place or corners on the map. The agent then refuses a walk that would end in a
zone, a straight walk that would enter one, and a navigation goal inside one.

Nav2 plans around the zones too. The demo launches Nav2 with `keepout:=true`, and NervROS publishes
the zones as a keep-out mask on `/keepout_filter_mask` for both costmaps' keepout filter.

- `keepout` is off by default. Only with it does `nav2.launch.py` start the filter's info server
  and its `lifecycle_manager_filters`.
- Without it, the agent still refuses walks into a zone, but Nav2's paths cross it.
- With it, Nav2 warns every 2 s until NervROS publishes the mask, which is expected. The Doctor tab
  (Ctrl+7) or `nervros-cli doctor` then says "Nav2 reads it".

The 3D view draws the zones in red, and the World tab lists them with Add and Remove. Removing one
asks for your approval every time. A plan saved or scheduled before a zone was added is checked
again when it runs. Zones added in chat are kept for the next session, in
`~/.local/state/nervros/zones/`, one file per robot; the profile's own (`[[geofence.zone]]`) stay
until the profile changes.

## The executor

`nervros_executor` (`g1_orchestration`) runs one mission at a time. It checks every tree before
running it: the hash you approved, only its own macros and the plain control nodes, and bounded
timeouts and retries.

- It takes the arms before the first step when a skill needs them, and hands them back when the
  mission ends, unless a hand holds, or may hold, something.
- After a pick that was cut off, the executor does not trust that hand until a place with it
  succeeds, and the agent plans as if the hand were full.
- Each result says when the robot or the executor, not the plan, ended a mission (`cause`): the
  robot could not move or fell mid-walk, the deadman, a shutdown, a skill server that was not up,
  teleop or another mission. `RobotState.boot_id` is new each time the executor starts. NervROS
  keeps such runs out of its skills' records and does not count them against the request, and a
  mission it was running when it stopped is kept as interrupted.
- The skills a plan may use are in `config/catalog.yaml`, as macros in `trees/library/`:
  `GoToTarget`, `GoToPose`, `WalkStraight`, `TurnInPlace`, `FindInView`, `PickObject`, `PlaceInto`,
  `TuckForTravel` and `ExploreBuilding`, plus the ones an agent taught (above).

https://github.com/user-attachments/assets/3c3d470d-7af2-498e-b83c-8c70945c5c65

In the clip a walk NervROS was running when it stopped is marked at start, and a walk approved
while the base is driven by hand from the Robot tab comes back with `cause` teleop: the report says
it could not start, and the Mission tab dims both runs.

https://github.com/user-attachments/assets/99be5972-bc7b-457b-ae5c-feb447bf00e3

A step can take what an earlier one found, so "find a mug and pick it up" is one mission.
`FindInView` hands on the name the detector reports an object under, and a walk to a kind of
object, among several, hands on the id of the nearest one it walked to.

- The plan gives the producing step an id, and a later step takes `{id.output}`, as in
  `{"id": "look", "skill": "FindInView", "args": {"phrase": "mug"}}` followed by
  `PickObject(object_id="{look.object_id}")`. The card shows the id before its step.
- The key must come from an earlier step that runs to its end (not one made optional, under a
  `ForceSuccess` or in a `Fallback` branch), and fit the argument it goes to.
- The preview marks a walk to such a target as known only when it runs.
- The demo's mock detector knows things by their kind in the scene, so there a look is for `mug`
  or `tray`, and the executor refuses other words. In the clip, the first plan looked for "red mug"
  and came back.

A mission can also be sent by hand, which is how to check a tree without the agent:

```bash
ros2 run g1_orchestration send_mission.sh $(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/missions/pick_and_place_apartment.xml validate
```

`validate` checks it, `dry-run` runs it with every skill replaced by a timed stand-in, and
`execute` runs it.

## When something goes wrong

https://github.com/user-attachments/assets/eb499b8c-4e19-487d-939e-51b98aa5eefa

In the clip the head camera's detector is stopped from outside. The incident opens by itself, "Why
can't the robot find anything with its head camera?" names the stopped `detector_head`, and the
Doctor tab shows the evidence.

NervROS records the robot's last minutes and opens an incident when:

- a mission fails,
- a camera or the lidar goes quiet,
- a watched node stops (the detectors, the executor, the manipulation server, Nav2's controller),
- a node logs a burst of errors.

It then runs the playbooks that may explain it: NervROS's own, and this project's in
`g1_bringup/config/nervros/playbooks/`. Each is a few read-only checks for a trap this stack has
fallen into before:

| Playbook | What it checks |
|---|---|
| `stale-shared-memory` | `g1_object_geometry` finding no depth frame for a mask's stamp again and again, while the chest camera's images come under 6 Hz and its `camera_info` keeps its rate: Fast DDS shared memory left stale by `kill -9`. A loaded machine also makes depth miss its masks now and then; the dropped images tell the two apart. |
| `detector-too-slow` | A detector skipping frames as too old: it is asked about too much at once. |
| `robot-lost` | `map` to `pelvis` against the relay's `base_state`, the simulator's truth: more than 1 m or 30 degrees apart, since a map starts where the robot stood when it was made, up to half a metre from the simulator's origin. |
| `robot-down`, `fell-at-start` | The executor's `can_move`, and the odometry, the locomotion safety trip and Nav2's aborted bring-up in the log. |
| `camera-silent`, `detector-down`, `executor-down` | A camera's `camera_info` under 0.5 Hz, a detector or the executor gone from the graph. |

"Why did that fail?" or "is something wrong with the robot?" gets the cause the checks found, what
each saw, what they ruled out, and the minutes before it. The Doctor tab lists the incidents, and
`nervros-cli --profile ../g1_bringup/config/nervros/nervros.toml why` checks the stack from a
terminal. The cameras are watched by their `camera_info`, which the relay sends with every frame,
so watching them costs none of an image's bandwidth.

## Sessions and the model's context

Every conversation is saved next to its log in `~/.local/state/nervros/logs/`. The Agent tab lists
the recent ones, and Resume carries one on; from the command line, use
`nervros-cli chat --resume last`.

The local model holds 16k tokens. The status bar shows how full the latest request was, and how
many model calls the session made, with the share read from llama.cpp's prompt cache. Before the
conversation fills, the results of earlier requests are cut to a line; if that is not enough, the
model sums up the older part as goal, done, open and facts. `/compact` in the message box condenses
it at once, and right-clicking one of your messages condenses everything up to it. Replies stream
into the chat as they are written.

For the G1's rare troubles the agent has skills: short procedures in
`g1_bringup/config/nervros/skills/` that it reads when one fits. They cover Nav2 that never came up,
a camera or detector that gives nothing, a robot lost on its map, and a robot that fell. The
playbooks above check the same troubles by themselves and name the skill with the long form.

## The robot's own description

https://github.com/user-attachments/assets/32d93917-9227-47be-83e7-5b408e125657

The persona, `g1_bringup/config/nervros/apartment.md`, tells the model what the robot is.
**Draft from the robot** in the Doctor tab, or
`nervros-cli --profile ../g1_bringup/config/nervros/nervros.toml whoami`, gathers the skills,
sensors, body, rooms and places from the live robot. The model then drafts `apartment.draft.md`
beside it, in its style, never over it. Read the draft and keep what is right. `whoami --facts`
prints what was gathered, without a model.

## Testing the agent

`g1_bringup/config/nervros/eval/apartment.toml` holds 46 requests an operator might make, from
"what can you see" to a full pick and place, each with what the agent should do. Run it against the
apartment stack with the arm (as `nervros.sh app` starts it) and the local model, from
`workspace/src/nervros` with the demo's environment:

```bash
cargo run -p nervros-cli -- --profile ../g1_bringup/config/nervros/nervros.toml \
  eval ../g1_bringup/config/nervros/eval/apartment.toml
```

- Each case runs in a fresh session with every approval granted. The robot carries its place and
  what it holds from case to case.
- Moves and turns are judged on where the simulator says the pelvis is, not on the robot's own
  estimate.
- The report lands in `~/.local/state/nervros/evals/`. It gives each model's pass rate with its
  interval, its mean model calls, time and tokens per trial, and the llama.cpp build and template it
  ran on.
- `--only <text>` runs the cases whose ids contain it, `--repeat 3` runs each three times for
  pass^3, and `--models a,b` compares two entries of `models.toml`.
- Each model and trial keeps its own robot files (keep-out zones, missions, notes, programs,
  incidents) under the run's `state/`, so the two keep-out cases, run in order
  (`--only keep-out`), never touch your own.

Two smaller suites sit beside it:

- `eval/jobs.toml` asks for programs, searches and taught skills, each more than one way. A kept
  skill carries to the cases after it, so they run in order.
- `eval/incidents.toml` breaks one thing per case with the case's `inject` (a detector stopped with
  SIGSTOP, the sensor relay paused, localization told the robot faces a quarter turn away, a second
  node under a detector's name), mends it with `restore`, and wants the agent to name the cause.

Without the simulator, `nervros-cli --profile ../g1_bringup/config/nervros/nervros.toml record
g1.toml` writes the running stack as a scripted robot: the graph, the nodes each container hosts,
the parameters, the topics the profile names and the executor's catalog. Then
`eval ../g1_bringup/config/nervros/eval/apartment.toml --robot g1.toml --models a,b` compares
models on it in minutes, with no simulator running. Nothing moves there and every plan is accepted,
so the cases that measure the robot's motion cannot pass: pick the others with `--only`.

The window can be driven the same way: NervROS's `live_eval` test types prompts into the real app
against the running stack, approves its cards, and saves the window at each step for a person to
look over (its header says how).

## Models

The app starts with the local model. NervROS can use other providers, set in its models file. This
project uses free models only, and NervROS refuses a paid OpenRouter model at start-up.

- With Gemini's key in `~/.config/grove/gemini.env`, its free tier outlines for `segment`, points
  for `point` and advises the planner. Without it those fall back or stay off, and the rest runs
  locally.
- With the same key, Gemini Robotics-ER 2 checks a finished mission through both cameras' start
  and end frames when the detector cannot settle it, finds the frame for `when`, and looks at a
  plan whose steps have failed before. Its free 20 requests a day are split by use (10, 6 and 4),
  and a use whose share is spent goes to the Flash-Lites, then Gemma 4, then the local model. A
  check's "no" flags a success; its "yes" never clears what the world model found missing.
- When the local model is not running, as when SAM 3.1 holds the GPU (the two do not fit in 12 GB
  together), free models elsewhere talk and look instead: Gemini 3.5 and 3.1 Flash-Lite, then
  Nemotron 3 Super on OpenRouter's free tier (with `~/.config/openrouter.key`; NervROS refuses any
  model there that is not free), then Gemma 4.
- Each Flash-Lite takes 10 requests a minute and 250 a day here, and Gemma 16K input tokens a
  minute, which one or two agent calls use up. When every model is at its limit for the minute, a
  turn waits for the first to free up.
- On the apartment suite this chain passed 42 of the 44 cases it had then, as the local model does
  in most runs.
- A `home` privacy mode keeps these models out.
- After a step fails again, the `plan` model may propose a lasting fix: a note, a keep-out zone
  or a parameter. The Agent tab lists it to accept, which runs it as yours, or to dismiss.

https://github.com/user-attachments/assets/c23c8e5b-65c4-4367-88e8-5c94d21a6e42

In the clip the Agent tab holds two proposed fixes: Accept on the note adds it to Memory, with the
step's record before and since, and Dismiss turns the parameter change down.
