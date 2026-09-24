# Open-vocabulary perception

Name an object in plain text and get its 3D pose, with no dataset and no training. Two halves: a
vision server on the host turns an RGB frame plus a list of noun phrases into one mask per object,
and `g1_perception` in the container lifts those masks into the object poses the manipulation
skills consume. The same server can turn an instruction into phrases, and a second host server
proposes grasps.

Host commands run from the repository root. `ros2` commands run in a container shell with the
workspace sourced, as in the other guides.

## Running it in the stack

The stand-in detector cuts masks out of simulator ground truth, so everything below the mask runs
without a GPU or a server:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true detector:=mock
```

With the real models, once the server below is running:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true detector:=vision \
  phrases:="red block,white cup"
```

`perception:=true` makes the object-pose source take measured poses instead of the simulator's,
and raises how old a pose the skills accept from 1 s to 8 s, because the detector answers in
seconds. Each phrase costs the server a model pass, so ask only for what the task needs; the
mission trees narrow the list themselves.

Objects arrive on `/objects` as `<phrase>_<index>`, such as `red_block_0`. The bare phrase,
`red_block`, is an alias for the first object seen alone under that phrase, held until its track
retires. It is withheld from any frame where that object is unseen or another object with the same
phrase is in view, so it never jumps between objects.

## Watching it in RViz

`rviz:=true` opens MoveIt's window with these displays, and starts what feeds them:

| Display | Topic | Shows |
|---|---|---|
| Perception image | `/g1_perception_visualizer/annotated_image` | The frame each detection was cut from: masks tinted per object, fitted boxes, `id score` labels. An instance the geometry rejected reads `unmeasured` and has no box. |
| Object poses | `/object_markers` | What the skills act on: a box and a label per object. |
| Ground truth | `/g1_perception_visualizer/ground_truth` | The simulator's own boxes, each labelled with how far the perceived object is from it. |
| Grasp plan | `/g1_manipulation_server/grasp_plan` | During a pick, every candidate weighed and the grasp taken. |

The switch behind all four is `visualization`, which follows `rviz`. `visualization:=false` keeps
them off with RViz open, and then none of their nodes or publishers exist.

## The vision server

On the host. It needs `uv`:

```bash
./scripts/setup-vision.sh
```

This creates a virtualenv at `~/ref/grove-vision/.venv`, sharing the torch wheel cache with
`scripts/setup-groot.sh`. The weights download on first run: about 900 MB for Grounding DINO base
and 180 MB for SAM 2.1 small.

```bash
~/ref/grove-vision/.venv/bin/python scripts/vision_server.py --port 5560
```

Compose is host-networked, so the container reaches it at `tcp://127.0.0.1:5560`. The default
backend needs about 2.4 GiB of VRAM on top of the simulator's.

To check it against saved frames without binding a socket:

```bash
~/ref/grove-vision/.venv/bin/python scripts/vision_server.py \
  --self-test frame.png --phrases "red block,green cylinder"
```

The self-test prints each instance's score, region of interest and pixel count, then the peak
VRAM, and exits non-zero when a phrase found nothing. Run it on frames from a new scene before
trusting the detector there.

## The protocol

msgpack, with `msgpack_numpy` for the arrays, over a ZeroMQ REQ/REP socket, the same shape the
GR00T policy server uses. A request is `{"endpoint": <name>, "data": {...}}`.

| Endpoint | Data | Reply |
|---|---|---|
| `ping` | nothing | `status`, `backend`, `device` |
| `segment` | `image` uint8 (H, W, 3), `phrases`, optional `box_threshold` and `text_threshold` | `model`, `elapsed_ms`, `instances` |
| `ground` | `image`, `instruction`; needs `--vlm` | `phrases`, `target`, `points`, `model`, `elapsed_ms` |

Each instance carries `label` (the phrase it was asked for, not the model's own wording), `score`,
`roi` as `[x, y, width, height]`, and `mask`, a uint8 crop of that rectangle holding 0 or 255.

Any failure comes back as `{"error": "..."}`. A missing reply would strand the client's REQ
socket, so the server answers even when it cannot do the work.

## Swapping the model

`--backend sam3` runs SAM 3 instead, one model from text straight to masks. Its weights are gated:
request access at https://huggingface.co/facebook/sam3, then sign in with

```bash
~/ref/grove-vision/.venv/bin/hf auth login
```

Nothing in the ROS workspace changes with the backend; the instance-mask message is the same
either way.

## Grasps

A second host server, [GraspGenX](https://github.com/NVlabs/GraspGenX), turns the same masks into
six-degree-of-freedom grasps for the Dex3-1, with no CAD model of anything. Its model conditions on
a gripper's sweep volume, the space the fingers fill open and half closed, so it works for a hand
it never trained on. The adapter sends the Dex3-1's, from GraspGenX's own `unitree_g1`
description.

On the host, again with `uv`:

```bash
./scripts/setup-graspgen.sh
```

It tracks upstream `main` and prints the commit it checked out. Upstream does not version the wire
protocol, so once a commit works, pin it: `GRASPGEN_REF=<sha> ./scripts/setup-graspgen.sh`. Serve
it; the first run downloads a few gigabytes of checkpoints into `~/ref/GraspGenX/ext`:

```bash
cd ~/ref/GraspGenX
uv run python client-server/graspgenx_server.py \
  --config ext/graspgenx_checkpoints/release --assets_dir ext/gripper_descriptions --port 5556
```

Then ask for candidates:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true \
  detector:=mock grasp_engine:=graspgen
```

```bash
ros2 service call /g1_grasp_engine/generate_grasps g1_msgs/srv/GenerateGrasps \
  "{object_id: red_block_0, hand: right}"
```

That call only returns candidates; the arm side filters and executes them. Add
`grasp_source:=generated activate_arm:=true activate_arm_delay_s:=40.0 rviz:=true` to the launch
and send a pick:

```bash
ros2 action send_goal /g1_manipulation_server/pick g1_msgs/action/Pick \
  "{object_id: red_block, arm: right}" --feedback
```

`grasp_plan` then draws each candidate as an arrow along its approach: green for the one taken, red
for too tilted, orange for out of reach.

`grasp_engine:=mock` answers the same service from `/objects` alone, with three sensible grasps and
one reaching up through the table, so the filtering can be tested without a GPU.

### The one calibration

GraspGenX returns poses of its own gripper frame, where +z is the approach and the fingers close
along x. That frame is not a link of this robot. Run a pick against a known object: `grasp_plan`
draws the candidates in the generator's frame and the goal's axes after the offset. Read the offset
to `right_hand_grasp_frame` off the two and pass it as `grasp_offset`, xyz then rpy; the arm side
applies it. Left-hand requests are refused rather than mirrored: a mirrored sweep volume is a
different gripper.

## Instructions

A detector takes a noun phrase. Turning "pick up the mug to the left of the bowl" into one is a
different job, and a vision-language model does it. Start the server with one:

```bash
~/ref/grove-vision/.venv/bin/python scripts/vision_server.py \
  --vlm Qwen/Qwen3-VL-2B-Instruct --port 5560
```

The model loads on the first grounding request rather than at startup, so segmentation is
unaffected until something asks. Then run the grounder beside the detector:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true \
  detector:=vision grounding:=true
```

```bash
ros2 service call /ground_instruction g1_msgs/srv/GroundInstruction \
  "{instruction: 'pick up the white cup next to the green cylinder'}"
```

The answer holds the phrases, the one the instruction is about, and image points where the model
could give them. The grounder writes the phrases onto the detector, so look for the target on
`/objects` a second or two later, after one detection pass.
