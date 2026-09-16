# Open-vocabulary perception

Name an object in plain text and get its 3D pose, with no dataset and no training. Two halves: a
vision server on the host that turns an RGB frame plus a list of noun phrases into one mask per
object, and `g1_perception` in the container, which lifts those masks into the object poses the
manipulation skills consume.

## Running it in the stack

The stand-in detector cuts masks out of simulator ground truth, so the whole pipeline below the
mask runs without a GPU or a server:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true detector:=mock
```

With the real models, once the server below is running:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true detector:=vision \
  phrases:="red cube,white cup"
```

`perception:=true` makes the object-pose source take measured poses instead of the simulator's,
and widens the staleness window the skills judge against, because a detector answers in seconds
rather than milliseconds. Objects arrive on `/objects` as `red_cube_0`, plus a bare `red_cube`
while only one of them is in view.

Measured against the simulator's own poses in the tabletop world: all five objects land within
1.5 mm, and their sizes within 4.4 mm.

## The vision server

```bash
./scripts/setup-vision.sh
```

Creates a virtualenv at `~/ref/grove-vision/.venv`, reusing the torch wheels that
`scripts/setup-groot.sh` caches. The model weights download on first run: about 900 MB for
Grounding DINO base and 180 MB for SAM 2.1 small.

Run it:

```bash
~/ref/grove-vision/.venv/bin/python scripts/vision_server.py --port 5560
```

Compose is host-networked, so the container reaches it at `tcp://127.0.0.1:5560`.

To check it against saved frames without binding a socket:

```bash
~/ref/grove-vision/.venv/bin/python scripts/vision_server.py \
  --self-test frame.png --phrases "red cube,green cylinder"
```

The self-test prints one line per instance with its score, region of interest and pixel count, then
the peak VRAM. It exits non-zero when a phrase found nothing, so it works as a smoke test.

## The protocol

msgpack with `msgpack_numpy` for the arrays, over a ZMQ REQ/REP socket, the same shape the GR00T
policy server uses.

| Endpoint | Request | Reply |
|---|---|---|
| `ping` | nothing | `status`, `backend`, `device` |
| `segment` | `image` uint8 (H, W, 3), `phrases`, optional `box_threshold` and `text_threshold` | `model`, `elapsed_ms`, `instances` |

Each instance carries `label` (the phrase it was asked for, not the model's own wording), `score`,
`roi` as `[x, y, width, height]`, and `mask`, a uint8 crop of that rectangle holding 0 or 255. The
crop rather than a full frame: at 848x480 a full mask is 407 kB per object against about 3 kB for a
6 cm object at half a metre.

Any failure comes back as `{"error": "..."}`. A REQ socket is stranded by a missing reply, so the
server answers even when it cannot do the work.

## Measured

Three rendered frames per scene from the simulator's own head camera at 848x480, compared against
masks from MuJoCo's segmentation render. RTX 4080 Laptop, float32, no TensorRT.

| Scene | Phrases | Found | Score | Mask IoU | Latency | Peak VRAM |
|---|---|---|---|---|---|---|
| Five objects on a bench | 5 | 5 of 5, every frame | 0.85 to 0.91 | 0.97 to 0.99 | 1.45 to 1.72 s | 2.37 GiB |
| Manipulation world, one cube on a grey pedestal | 1 | 1 of 1, every frame | 0.81 | n/a | 1.43 to 1.75 s | 2.35 GiB |

Flat-shaded simulator renders were the open question, because published work reports these models
failing on non-photorealistic scenes. They do not fail on ours. Expect worse on textured household
objects, and check before trusting a new scene.

## Swapping the model

`--backend sam3` runs SAM 3 instead, one model from text straight to masks, and it scores better on
every published benchmark. Its weights are gated: request access at
https://huggingface.co/facebook/sam3, then sign in with

```bash
~/ref/grove-vision/.venv/bin/hf auth login
```

Nothing in the ROS workspace changes when the backend does. The instance-mask message is the same
either way, which is the point of putting the model behind a socket.

## Grasps

A second host server turns the same masks into six-degree-of-freedom grasps for the Dex3-1, with
no CAD model of anything. It is [GraspGenX](https://github.com/NVlabs/GraspGenX), whose released
model conditions on a gripper's *sweep volume*: two boxes describing the finger volume open and
half closed. That is why a hand it has never trained on works, and it ships the description of
this one, `unitree_g1`, which is the Dex3-1's seven joints.

```bash
./scripts/setup-graspgen.sh
```

It tracks upstream `main` and prints the commit it checked out. Nothing upstream versions the
wire protocol the adapter speaks, so once a commit works, pin it: `GRASPGEN_REF=<sha>
./scripts/setup-graspgen.sh`.

Run the server it prints, then ask for candidates:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true \
  detector:=mock grasp_engine:=graspgen
```

```bash
ros2 service call /g1_grasp_engine/generate_grasps g1_msgs/srv/GenerateGrasps \
  "{object_id: red_cube_0, hand: right}"
```

Candidates are published as arrows on `/grasp_candidates`, coloured red to green by confidence and
drawn along each grasp's approach axis. Nothing moves: this stage produces candidates, and what
filters and executes them is the arm side.

`grasp_engine:=mock` answers the same service from `/objects` alone, with three sensible grasps and
one deliberately reaching up through the table, so the filtering above it can be tested without a
GPU.

### The one calibration

GraspGenX returns poses of its own gripper frame, where +Z is the approach direction and +X the
closing direction. That is not a link in this robot's URDF. Look at the arrows in RViz against a
known object, read off the offset to `right_hand_grasp_frame`, and that number is what the arm
side applies. Left-hand requests are refused rather than mirrored: a mirrored sweep volume is a
different gripper, and the model was never asked about it.

## Instructions

A detector takes a noun phrase. Turning "pick up the mug to the left of the bowl" into one is a
different job, and a vision-language model does it. Start the server with one:

```bash
~/ref/grove-vision/.venv/bin/python scripts/vision_server.py \
  --vlm Qwen/Qwen3-VL-2B-Instruct --port 5560
```

It loads on the first grounding request rather than at startup, so segmentation is unaffected
until something asks. Then run the grounder beside the detector:

```bash
ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \
  odometry:=ground_truth moveit:=true manipulation:=true perception:=true \
  detector:=vision grounding:=true
```

```bash
ros2 service call /ground_instruction g1_msgs/srv/GroundInstruction \
  "{instruction: 'pick up the white cup next to the green cylinder'}"
```

The answer is the phrases, which of them the instruction was about, and points where the model
could give them. Those phrases are written straight onto the detector, so the next detection
looks for them. Ask for the target's object id on `/objects` a second or two later: the detector
needs one pass to find it.
