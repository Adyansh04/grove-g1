# Host model servers

Models that need torch and CUDA run on the host, outside the ROS image, and the container talks to
them over ZeroMQ on 127.0.0.1 (Compose is host-networked). Set each up once, then start it with
`./scripts/serve.sh`:

| Server | Setup | Start | Port | Serves |
|---|---|---|---|---|
| `groot_server.py` | `./scripts/setup-groot.sh` | `./scripts/serve.sh groot` | 5555 | The GR00T policy to `g1_vla`. |
| `vision_server.py` | `./scripts/setup-vision.sh` | `./scripts/serve.sh vision` | 5560 | Open-vocabulary masks to `g1_perception`, and instruction grounding with `--vlm`. |
| GraspGenX, upstream's | `./scripts/setup-graspgen.sh` | `./scripts/serve.sh graspgen` | 5556 | Grasps for the Dex3-1. |
| canopy's semantic server | `workspace/src/canopy/servers/setup.sh` | `./scripts/serve.sh canopy` | 5561 | Detection, embeddings and descriptions to the world model. |

Arguments after the server's name reach the server, after `serve.sh`'s defaults, so they
override them. Each server answers every request, failures included, since a dropped reply would
strand the client's REQ socket.

- `groot_server.py` wraps upstream's entry point to load the checkpoint in bf16; see
  [learned grasping](../docs/guides/learned-grasping.md).
- `vision_server.py` is the command line and `vision/` its parts: masks, the segmentation
  backends, grounding and the ZMQ loop. The protocol and the backends are in
  [open-vocabulary perception](../docs/guides/open-vocabulary-grasping.md).

## Why Python

The models ship as PyTorch checkpoints with Python processors, and these servers only load them,
batch a request and reply. The ROS side, which runs at rate, is C++ and never links torch.
