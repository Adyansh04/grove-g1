#!/usr/bin/env python3
"""Serves open-vocabulary instance masks to g1_perception, on the host rather than in the container.

The models need torch and CUDA, which the ROS image does not carry, so g1_detector talks to this
server over ZMQ, as servers/groot_server.py does for the policy.

SAM 3.1, g1_detector's default, runs in canopy's semantic server (./scripts/serve.sh canopy
--detector sam3.1, port 5561), which speaks this protocol too. This server keeps the alternatives, chosen with --backend:

  grounded-sam2  Grounding DINO boxes refined into masks by SAM 2.1. Ungated; the default here.
  none           No segmentation: only `ground`, beside canopy's SAM 3.1.

    ./scripts/serve.sh vision                 # --port 5560
    ./scripts/serve.sh vision --backend none --vlm Qwen/Qwen3-VL-2B-Instruct --dtype bfloat16

With --vlm, a vision-language model is loaded on the first `ground` request; in bfloat16 it fits
beside SAM 3.1 on a 12 GB GPU. It turns an
instruction such as "the mug left of the bowl" into noun phrases the detector can take.

Run scripts/setup-vision.sh first.

Wire protocol, msgpack with msgpack_numpy for the arrays:

    {"endpoint": "ping"}
      -> {"status": "ok", "backend": "grounded-sam2", "device": "cuda"}
    {"endpoint": "segment", "data": {"image": uint8 (H, W, 3), "phrases": ["red block", ...]}}
      -> {"model": "grounded-sam2",
          "instances": [{"label": "red block", "score": 0.71, "roi": [x, y, w, h],
                         "mask": uint8 (h, w), 0 or 255}]}

A failure of any endpoint is {"error": "..."}, never a dropped reply, which would strand the
client's REQ socket until its timeout.
"""

import argparse
import sys
import time

import numpy as np

VENV_HINT = "scripts/setup-vision.sh has not been run, or the wrong interpreter is being used"

try:
    import torch

    from vision.grounding import GroundingBackend
    from vision.segmentation import (
        DEFAULT_DETECTOR,
        DEFAULT_SEGMENTER,
        GroundedSam2Backend,
        NoBackend,
    )
    from vision.server import serve
except ImportError as error:  # pragma: no cover - depends on the host environment
    sys.exit(f"{error}. {VENV_HINT}")

DEFAULT_PORT = 5560
DTYPES = {"float32": torch.float32, "bfloat16": torch.bfloat16, "float16": torch.float16}


def build_backend(args, device, dtype):
    if args.backend == "none":
        return NoBackend()
    return GroundedSam2Backend(device, dtype, args.detector_model, args.segmenter_model)


def self_test(backend, args):
    """Runs the backend over image files and prints what it found, without binding a socket."""
    from PIL import Image

    phrases = [phrase.strip() for phrase in args.phrases.split(",") if phrase.strip()]
    failures = 0
    for path in args.self_test:
        image = np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
        started = time.perf_counter()
        instances = backend.segment(image, phrases, args.box_threshold, args.text_threshold)
        elapsed = (time.perf_counter() - started) * 1000.0
        found = {instance["label"] for instance in instances}
        missing = [phrase for phrase in phrases if phrase not in found]
        failures += bool(missing)
        print(f"{path}  {len(instances)} instances, {elapsed:.0f} ms")
        for instance in instances:
            roi = instance["roi"]
            pixels = int((instance["mask"] > 0).sum())
            print(
                f"    {instance['label']:20s} score {instance['score']:.2f}  roi {roi}  {pixels} px"
            )
        if missing:
            print(f"    MISSING: {', '.join(missing)}")
    if torch.cuda.is_available():
        print(f"peak VRAM {torch.cuda.max_memory_allocated() / 2**30:.2f} GiB")
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--backend", choices=["grounded-sam2", "none"], default="grounded-sam2")
    parser.add_argument("--detector-model", default=DEFAULT_DETECTOR)
    parser.add_argument("--segmenter-model", default=DEFAULT_SEGMENTER)
    parser.add_argument(
        "--vlm",
        default="",
        help="a vision-language model for the ground endpoint, e.g. Qwen/Qwen3-VL-2B-Instruct. "
        "Loaded on the first request; without it the endpoint says so.",
    )
    parser.add_argument("--vlm-max-new-tokens", type=int, default=256)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--dtype", choices=list(DTYPES), default="float32")
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument(
        "--self-test",
        nargs="+",
        metavar="IMAGE",
        help="run over these images and exit, instead of serving",
    )
    parser.add_argument(
        "--phrases",
        default="red block,green cylinder,blue sphere,yellow box,white cup",
        help="comma separated, used by --self-test",
    )
    args = parser.parse_args()

    device = args.device
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = DTYPES[args.dtype]
    try:
        backend = build_backend(args, device, dtype)
    except OSError as error:
        # A gated checkpoint fails here, and the message is about a 403 rather than about access.
        sys.exit(f"{error}\n\nthe checkpoint could not be loaded; {VENV_HINT}")

    if args.self_test:
        return self_test(backend, args)
    grounding = None
    if args.vlm:
        grounding = GroundingBackend(args.vlm, device, dtype, args.vlm_max_new_tokens)
    serve(backend, device, args, grounding)
    return 0


if __name__ == "__main__":
    sys.exit(main())
