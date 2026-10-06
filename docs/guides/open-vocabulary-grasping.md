# Open-vocabulary perception

Name an object in plain text and get its 3D pose, with no dataset and no training. Two halves:
SAM 3.1 on the host turns an RGB frame plus a list of noun phrases into one mask per object, and
`g1_perception` in the container lifts those masks into the object poses the manipulation skills
consume. A vision-language model can turn an instruction into phrases, and a second host server
proposes grasps.

`./scripts/demos/open-vocabulary-grasping.sh <mock|vision|gsam2|graspgen|grounding>` opens a
section's commands at once, in split panes of your terminal, after tearing down any stack left
running. `stop` ends it, and `--print` lists the commands instead.

Host commands run from the repository root. `ros2` commands run in a container shell, opened with
`./scripts/manage.sh exec`, as in the other guides.

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
  phrases:="red block,white cylinder"
```

Name things as they look: SAM 3.1 reads the words. The tabletop's `white_cup` is a plain white
cylinder, so "white cup" finds nothing, where Grounded SAM 2 would take it anyway; the same goes
for colours ("red mug" is never a yellow one).

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

## The detector server

SAM 3.1 runs in canopy's semantic server, which also serves the world model's detector. On the
host, with `uv`; its checkpoint is gated, so first request access at
https://huggingface.co/facebook/sam3.1 and sign in with `hf auth login`:

```bash
workspace/src/canopy/servers/setup.sh
./scripts/serve.sh canopy --detector sam3.1 --embedder none --describer none
```

Compose is host-networked, so the container reaches it at `tcp://127.0.0.1:5561`. Without the
embedder and describers it needs about 5.6 GB of VRAM on top of the simulator's, and answers in
about 0.45 s for the frame plus 0.04 s for each phrase on a laptop RTX 4080: ask only for what the
task needs. To check it against saved frames without binding a socket:

```bash
~/.local/share/canopy/.venv/bin/python workspace/src/canopy/servers/semantic_server.py \
  --detector sam3.1 --embedder none --describer none \
  --self-test frame.png --phrases "red block,green cylinder"
```

Run it on frames from a new scene before trusting the detector there.

## The protocol

msgpack, with `msgpack_numpy` for the arrays, over a ZeroMQ REQ/REP socket, the same shape the
GR00T policy server uses. A request is `{"endpoint": <name>, "data": {...}}`.

| Endpoint | Data | Reply |
|---|---|---|
| `ping` | nothing | `status`, `backend`, `device` |
| `segment` | `image` uint8 (H, W, 3), `phrases`, optional `box_threshold` and `text_threshold` | `model`, `elapsed_ms`, `instances` |
| `ground` (vision server only) | `image`, `instruction`; needs `--vlm` | `phrases`, `target`, `points`, `model`, `elapsed_ms` |

Both servers answer `segment` in this shape, so either can stand behind the detector.

Each instance carries `label` (the phrase it was asked for, not the model's own wording), `score`,
`roi` as `[x, y, width, height]`, and `mask`, a uint8 crop of that rectangle holding 0 or 255.

Any failure comes back as `{"error": "..."}`. A missing reply would strand the client's REQ
socket, so the server answers even when it cannot do the work.

## Swapping the model

Every server answers `segment` with the same instance masks, so nothing in the ROS workspace
changes with the model. Grounding DINO with SAM 2.1 runs in grove-g1's vision server
(`./scripts/setup-vision.sh`, then `./scripts/serve.sh vision`, on 5560); the `gsam2` variant runs
it on 5561, where the detector already looks. `./scripts/serve.sh canopy --detector yoloe` serves
YOLOE-26 there instead.

How they compare on 88 close-ups of the apartment's small objects, rendered from the G1's cameras,
and live in the tabletop world:

| | SAM 3.1 | Grounding DINO + SAM 2.1 | YOLOE-26 |
|---|---|---|---|
| Found by name: precision, recall (threshold) | 1.00, 0.83 (0.4) | 0.93, 0.88 (0.4) | 1.00, 0.66 (0.3) |
| Mask IoU | 0.98 | 0.97 | 0.88 |
| "red mug" asked of a yellow one | refused, every time | matched, every time | matched 3 % |
| Time a request, GPU memory | 0.53 s, 5.6 GB | 0.74 s, 3.2 GB | 23 ms, 1 GB |
| Live: detections a second, pose error | 0.8-1.0, 2.2 mm | 0.4, 2.2 mm | |

SAM 3.1 is the default for its masks and for reading a description; Grounded SAM 2 is as good on a
bare name at about half the memory. YOLOE wants `box_threshold` 0.3 and misses a third of the
objects close up, but it is fast enough to run on every frame.

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
./scripts/serve.sh graspgen
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
different job, and a vision-language model does it, in the vision server. Beside SAM 3.1 it needs
no segmentation model of its own:

```bash
./scripts/serve.sh vision --backend none --vlm Qwen/Qwen3-VL-2B-Instruct
```

The model loads on the first grounding request rather than at startup. Then run the grounder
beside the detector:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true \
  detector:=vision grounding:=true
```

```bash
ros2 service call /ground_instruction g1_msgs/srv/GroundInstruction \
  "{instruction: 'pick up the red block next to the white cylinder'}"
```

The answer holds the phrases, the one the instruction is about, and image points where the model
could give them. The grounder writes the phrases onto the detector, so look for the target on
`/objects` a second or two later, after one detection pass.
