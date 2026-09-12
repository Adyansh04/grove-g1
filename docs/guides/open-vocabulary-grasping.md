# Open-vocabulary perception

Name an object in plain text and get instance masks back, with no dataset and no training. This is
the host-side half: a vision server that turns an RGB frame plus a list of noun phrases into one
mask per object. The ROS side that lifts those masks into object poses and grasps is built on top
of it.

## Setup

```bash
./scripts/setup-vision.sh
```

Creates a virtualenv at `~/ref/grove-vision/.venv`, reusing the torch wheels that
`scripts/setup-groot.sh` caches. The model weights download on first run: about 900 MB for
Grounding DINO base and 180 MB for SAM 2.1 small.

## Running it

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
