# Host model servers

Models that need torch and CUDA run on the host, outside the ROS image. The container talks to
them over ZeroMQ on 127.0.0.1 (Compose is host-networked). All are optional: the simulator tests use
a mock engine, detector or grasp source instead.

Set each up once, then start it with `./scripts/serve.sh`:

| Server | Setup | Start | Port | Serves |
|---|---|---|---|---|
| `groot_server.py` | `./scripts/setup-groot.sh` | `./scripts/serve.sh groot` | 5555 | The GR00T policy to `g1_vla`. |
| `vision_server.py` | `./scripts/setup-vision.sh` | `./scripts/serve.sh vision` | 5560 | Grounded SAM 2 masks, the alternative to SAM 3.1, and instruction grounding with `--vlm`. |
| GraspGenX, upstream's | `./scripts/setup-graspgen.sh` | `./scripts/serve.sh graspgen` | 5556 | Grasps for the Dex3-1 or the Inspire hand. |
| canopy's semantic server | `workspace/src/canopy/servers/setup.sh` | `./scripts/serve.sh canopy` | 5561 (5562 for `describe`) | Embeddings and descriptions to the world model, and YOLOE-26 masks with `--detector yoloe`. SAM 3.1 runs in TensorRT in the stack instead (`workspace/src/sam3_inference`). |

Arguments after the server's name reach the server, after `serve.sh`'s defaults, so they override
them.

`serve.sh canopy --detector yoloe`, or a `--describer` list that names `openai`, also starts canopy's
local VLM in a container on 127.0.0.1:8080 and leaves it running.
`workspace/src/canopy/servers/start-vlm.sh stop` ends it.

Beside SAM 3.1 and the simulator, a 12 GB GPU has no room for the VLM. So `serve.sh canopy`
stops one left running unless asked for YOLOE or `openai`, and the describer asks Gemini
(`--describer gemini`).

- `groot_server.py` wraps upstream's entry point to load the checkpoint in bf16. See [learned grasping](../docs/guides/learned-grasping.md).
- `vision_server.py` is the command line and `vision/` its parts: masks, the segmentation backends, grounding and the ZMQ loop. The protocol and the backends are in [open-vocabulary perception](../docs/guides/open-vocabulary-grasping.md).

There is no test suite here. `./scripts/manage.sh lint` runs ruff over this directory.

## Why Python

The models ship as PyTorch checkpoints with Python processors, and these servers only load them,
run a request and reply. The ROS side, which runs at rate, is C++ and never links torch.
