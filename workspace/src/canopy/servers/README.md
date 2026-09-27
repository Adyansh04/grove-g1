# Model servers

The models canopy asks, served on the host GPU over ZMQ, because the robot's image carries no torch
or CUDA. Not a ROS package (`COLCON_IGNORE`): canopy_perception's `detector` and
`object_describer` are the clients.

| File | Does |
|---|---|
| `semantic_server.py` | Detection (YOLOE-26 over a word list, one forward pass for the whole list), image and text embeddings (SigLIP 2), and object and room descriptions (Gemini, falling back to a local VLM). REQ/REP on `tcp://127.0.0.1:5561`, msgpack bodies. |
| `start-vlm.sh` | The local VLM behind the `openai` describer: Qwen3.5-4B in llama.cpp's server, in a container, on `127.0.0.1:8080`. |
| `setup.sh` | A uv virtualenv with pinned torch and model libraries, the YOLOE weights, the Hugging Face downloads and the llama.cpp image. |
| `test_semantic_server.py` | Unit tests with fake models: no GPU, no network, no weights. |

## Setup and running

```bash
./servers/setup.sh                        # once; CANOPY_HOME (default ~/.local/share/canopy)
./servers/start-vlm.sh start              # the offline describer, about 4 GB of VRAM
~/.local/share/canopy/.venv/bin/python servers/semantic_server.py
```

YOLOE-26l and SigLIP 2 take about 1.7 GB of VRAM together. Each backend is a flag (`--detector`,
`--embedder`, `--describer`) or a `--config` YAML shaped like the server's `DEFAULTS`, and
`--self-test frame.png` runs one frame through without a socket.

## Endpoints

| Endpoint | In | Out |
|---|---|---|
| `ping` | | backend and device |
| `segment` | RGB image, phrases, thresholds, `embed` | instances: label, score, roi, mask crop, optional embedding |
| `embed_text`, `embed_image` | texts or images | unit vectors from the same model |
| `describe` | `task` (object or room), up to 8 images, what the map believes | name, caption, `label_ok`, `room_type`, confidence |

The server answers one request at a time, so a describe call of a few seconds delays the next
segment by as much.

## Gemini

The `gemini` describer needs a free API key in `~/.config/canopy/gemini.env` (the bare key, or
`NAME=key`). The key goes only in the `x-goog-api-key` header and is redacted from errors. The
server caps its own use at the free tier's 5 requests a minute and 100 a day, counted in a file
beside the key across processes and restarts, and stops for the rest of the Pacific day on any
quota answer. One exploration of a six-room flat uses about 100 describe calls, so it falls through
to the local VLM partway.

## Models and libraries

Each keeps its own license; follow the link.

| What | Used for | Source |
|---|---|---|
| YOLOE-26l-seg, -seg-pf (Ultralytics, AGPL-3.0) | detection | https://docs.ultralytics.com/models/yoloe (arXiv 2503.07465) |
| MobileCLIP2-B | YOLOE's text encoder | https://github.com/apple/ml-mobileclip |
| CLIP tokenizer (Ultralytics' fork) | YOLOE's text prompts | https://github.com/ultralytics/CLIP |
| SigLIP 2 B/16-256 | embeddings | https://huggingface.co/google/siglip2-base-patch16-256 |
| Gemini 3.5 Flash-Lite | describer | https://ai.google.dev/gemini-api/docs/models |
| Qwen3.5-4B, Q4_K_M GGUF | local describer | https://huggingface.co/Qwen/Qwen3.5-4B, https://huggingface.co/unsloth/Qwen3.5-4B-GGUF |
| llama.cpp server | runs the local VLM | https://github.com/ggml-org/llama.cpp |
