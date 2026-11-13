# sam3_inference

SAM 3.1 open-vocabulary instance segmentation in C++ with TensorRT, served as a ROS 2 service:
an image and a list of phrases in, one mask per instance out. It depends on no grove-g1 or canopy
package, so this folder (with `sam3_inference_msgs` beside it) can move to a repository of its
own.

## Node

`sam3_segmentation` (component `sam3_inference::Sam3SegmentationNode`, executable
`sam3_segmentation_node`) answers `segment_objects` (`sam3_inference_msgs/srv/SegmentObjects`).
A request carries the image, or leaves it empty to take the newest frame of `image_topic`, and the
phrases; the answer holds each instance's phrase, score, box and `mono8` mask cropped to the box,
phrases in request order and the best first within each.

SAM 3.1's image detector runs as three TensorRT engines: the image encoder once per request, the
text encoder once per new phrase (its features stay on the GPU), and the decoder once per phrase.
The engines take about 2.6 GB of GPU memory.

## Parameters

In `config/sam3_segmentation.yaml`. The ones worth knowing:

| Parameter | Meaning |
|---|---|
| `model_dir` | The ONNX graphs from `scripts/export_sam3.py`. |
| `engine_dir` | Where engines are built from the graphs on the first start on a GPU (minutes), and again when a graph or the TensorRT version changes. |
| `precision_suffix` | `.bf16` for the graphs exported with `--bf16 --suffix .bf16`. |
| `min_score` | Instances under it are left out; a request's own `min_score` overrides it. |
| `idle_unload_s` | Frees the engines after this long without a request; 0 keeps them loaded. |

## Models

The graphs are exported once from Meta's SAM 3.1 checkpoint (gated on Hugging Face: request
access at https://huggingface.co/facebook/sam3.1). The export needs torch and Meta's `sam3`
package, which canopy's model server environment has (`workspace/src/canopy/servers/setup.sh`):

```bash
~/.local/share/canopy/.venv/bin/python workspace/src/sam3_inference/sam3_inference/scripts/export_sam3.py \
    --checkpoint ~/.local/share/canopy/weights/sam3.1_multiplex.pt --out data/models/sam3 \
    --bf16 --suffix .bf16
```

The tokenizer reads CLIP's BPE vocabulary, which ships in `resources/`.

## Build requirements

CUDA 13 and TensorRT 11 (`libnvinfer-dev`, `libnvonnxparsers-dev`), which the dev image installs
from NVIDIA's apt repository. Without them only the CPU parts (tokenizer, request checks) and
their tests build. TensorRT 10, as on JetPack 6, builds the graphs as strongly typed networks.

## Running

```bash
ros2 launch sam3_inference sam3_segmentation.launch.py
ros2 service call /segment_objects sam3_inference_msgs/srv/SegmentObjects "{prompts: ['red cube']}"
```

The second call needs `image_topic` set, since it sends no image of its own.

## Tests

```bash
./scripts/manage.sh test sam3_inference
```

runs the tokenizer and request-check tests, which need no GPU. `check_sam3_engine` runs the
engines on one image for a manual check on a GPU:

```bash
check_sam3_engine data/models/sam3 workspace/src/sam3_inference/sam3_inference/resources/bpe_simple_vocab_16e6.txt.gz frame.png "red cube"
```

## Credits

- SAM 3.1, Meta AI, https://github.com/facebookresearch/sam3, under the SAM License
  (`resources/SAM_LICENSE`).
- CLIP's BPE vocabulary, OpenAI, https://github.com/openai/CLIP, MIT (`resources/CLIP_LICENSE`).
