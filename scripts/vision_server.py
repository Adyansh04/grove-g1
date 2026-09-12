#!/usr/bin/env python3
"""Serves open-vocabulary instance masks to g1_perception, on the host rather than in the container.

Same split as scripts/groot_server.py and for the same reason: the segmentation models need torch
and CUDA, the ROS image deliberately has neither, and g1_detector speaks this server's ZMQ protocol
instead of importing a model package.

Two backends, chosen with --backend, behind one reply format:

  grounded-sam2  Grounding DINO boxes refined into masks by SAM 2.1. Apache-2.0 and ungated, so it
                 is the default and the only one that works out of the box.
  sam3           One model, text straight to instance masks. Better on everything measured, and
                 gated: request access on the model page first. Once the weights are there this is
                 a flag, because the reply format below is all g1_perception ever sees.

    ./scripts/vision_server.py --port 5560
    ./scripts/vision_server.py --backend sam3 --port 5560

Run scripts/setup-vision.sh first.

Wire protocol, msgpack with msgpack_numpy for the arrays:

    {"endpoint": "ping"}
      -> {"status": "ok", "backend": "grounded-sam2", "device": "cuda"}
    {"endpoint": "segment", "data": {"image": uint8 (H, W, 3), "phrases": ["red cube", ...]}}
      -> {"model": "grounded-sam2",
          "instances": [{"label": "red cube", "score": 0.71, "roi": [x, y, w, h],
                         "mask": uint8 (h, w), 0 or 255}]}

A failure of any endpoint is {"error": "..."}, never a dropped reply: the client is a REQ socket
and a missing reply strands it until its timeout.
"""

import argparse
import sys
import time

import numpy as np

VENV_HINT = "scripts/setup-vision.sh has not been run, or the wrong interpreter is being used"

try:
    import msgpack
    import msgpack_numpy as mnp
    import torch
    import zmq
except ImportError as error:  # pragma: no cover - depends on the host environment
    sys.exit(f"{error}. {VENV_HINT}")

DEFAULT_PORT = 5560
DEFAULT_DETECTOR = "IDEA-Research/grounding-dino-base"
DEFAULT_SEGMENTER = "facebook/sam2.1-hiera-small"
DEFAULT_SAM3 = "facebook/sam3"


def _roi_and_crop(mask):
    """Full-frame boolean mask to (roi, cropped uint8 mask), or None when it is empty.

    The crop is what goes on the wire: a full frame at 848x480 is 407 kB per instance against
    about 3 kB for a 6 cm object at half a metre, and the consumer walks a rectangle either way.
    """
    rows = np.flatnonzero(mask.any(axis=1))
    cols = np.flatnonzero(mask.any(axis=0))
    if rows.size == 0 or cols.size == 0:
        return None
    y0, y1 = int(rows[0]), int(rows[-1]) + 1
    x0, x1 = int(cols[0]), int(cols[-1]) + 1
    crop = np.ascontiguousarray(mask[y0:y1, x0:x1].astype(np.uint8) * 255)
    return [x0, y0, x1 - x0, y1 - y0], crop


def _match_phrase(label, phrases):
    """Maps a backend's own label back onto the phrase that was asked for.

    Grounding DINO answers with the span of the prompt it matched, which is often a prefix
    ("red" for "a red cube"). The object id downstream is built from the phrase, so a drifting
    label would rename the object every time the match changed.
    """
    cleaned = label.strip().lower().removeprefix("a ").removeprefix("an ").removeprefix("the ")
    for phrase in phrases:
        if cleaned == phrase.lower():
            return phrase
    scored = [(len(set(cleaned.split()) & set(p.lower().split())), p) for p in phrases]
    overlap, best = max(scored, key=lambda pair: pair[0])
    return best if overlap else label


class GroundedSam2Backend:
    """Grounding DINO boxes, refined into masks by SAM 2.1."""

    name = "grounded-sam2"

    def __init__(self, device, dtype, detector_id=DEFAULT_DETECTOR, segmenter_id=DEFAULT_SEGMENTER):
        from transformers import (
            AutoModelForZeroShotObjectDetection,
            AutoProcessor,
            Sam2Model,
            Sam2Processor,
        )

        self._device = device
        self._detector_processor = AutoProcessor.from_pretrained(detector_id)
        self._detector = (
            AutoModelForZeroShotObjectDetection.from_pretrained(detector_id, dtype=dtype)
            .to(device)
            .eval()
        )
        self._segmenter_processor = Sam2Processor.from_pretrained(segmenter_id)
        self._segmenter = Sam2Model.from_pretrained(segmenter_id, dtype=dtype).to(device).eval()

    def segment(self, image, phrases, box_threshold, text_threshold):
        # One prompt holding every phrase: the detector runs its backbone once either way, and
        # asking per phrase costs a pass each.
        prompt = " ".join(f"a {phrase}." for phrase in phrases)
        inputs = self._detector_processor(images=image, text=prompt, return_tensors="pt").to(
            self._device
        )
        with torch.inference_mode():
            detections = self._detector_processor.post_process_grounded_object_detection(
                self._detector(**inputs),
                inputs.input_ids,
                threshold=box_threshold,
                text_threshold=text_threshold,
                target_sizes=[image.shape[:2]],
            )[0]

        boxes = detections["boxes"].float().cpu().numpy().tolist()
        if not boxes:
            return []
        labels = detections.get("text_labels", detections.get("labels"))
        scores = detections["scores"].float().cpu().numpy().tolist()

        prompted = self._segmenter_processor(
            images=image, input_boxes=[boxes], return_tensors="pt"
        ).to(self._device)
        with torch.inference_mode():
            predicted = self._segmenter(**prompted, multimask_output=False)
        masks = self._segmenter_processor.post_process_masks(
            predicted.pred_masks.cpu(), prompted["original_sizes"]
        )[0]
        masks = masks.numpy().astype(bool).reshape(len(boxes), *image.shape[:2])

        instances = []
        for mask, label, score in zip(masks, labels, scores, strict=True):
            cropped = _roi_and_crop(mask)
            if cropped is None:
                continue
            roi, data = cropped
            instances.append(
                {
                    "label": _match_phrase(str(label), phrases),
                    "score": float(score),
                    "roi": roi,
                    "mask": data,
                }
            )
        return instances


class Sam3Backend:
    """One model from text to instance masks. Gated weights; see the module docstring."""

    name = "sam3"

    def __init__(self, device, dtype, model_id=DEFAULT_SAM3):
        from transformers import Sam3Model, Sam3Processor

        self._device = device
        self._processor = Sam3Processor.from_pretrained(model_id)
        self._model = Sam3Model.from_pretrained(model_id, dtype=dtype).to(device).eval()

    def segment(self, image, phrases, box_threshold, text_threshold):
        del text_threshold  # one calibrated score, so the second threshold has nothing to do
        instances = []
        for phrase in phrases:
            inputs = self._processor(images=image, text=phrase, return_tensors="pt").to(
                self._device
            )
            with torch.inference_mode():
                outputs = self._model(**inputs)
            results = self._processor.post_process_instance_segmentation(
                outputs, threshold=box_threshold, mask_threshold=0.5,
                target_sizes=[image.shape[:2]],
            )[0]
            masks = results["masks"].cpu().numpy().astype(bool)
            scores = results["scores"].float().cpu().numpy().tolist()
            for mask, score in zip(masks, scores, strict=True):
                cropped = _roi_and_crop(mask)
                if cropped is None:
                    continue
                roi, data = cropped
                instances.append(
                    {"label": phrase, "score": float(score), "roi": roi, "mask": data}
                )
        return instances


def _build_backend(args):
    device = args.device if args.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu")
    dtype = {"float32": torch.float32, "bfloat16": torch.bfloat16, "float16": torch.float16}[
        args.dtype
    ]
    if args.backend == "sam3":
        return Sam3Backend(device, dtype, args.sam3_model), device
    return GroundedSam2Backend(device, dtype, args.detector_model, args.segmenter_model), device


def _segment_request(backend, payload, args):
    image = payload.get("image")
    phrases = payload.get("phrases") or []
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image must be an (H, W, 3) uint8 array")
    if image.dtype != np.uint8:
        raise ValueError(f"image must be uint8, got {image.dtype}")
    phrases = [str(phrase) for phrase in phrases if str(phrase).strip()]
    if not phrases:
        raise ValueError("at least one phrase is required")

    started = time.perf_counter()
    instances = backend.segment(
        image,
        phrases,
        payload.get("box_threshold", args.box_threshold),
        payload.get("text_threshold", args.text_threshold),
    )
    return {
        "model": backend.name,
        "elapsed_ms": (time.perf_counter() - started) * 1000.0,
        "instances": instances,
    }


def serve(backend, device, args):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://{args.host}:{args.port}")
    print(f"serving {backend.name} on {device} at tcp://{args.host}:{args.port}", flush=True)
    try:
        while True:
            request = msgpack.unpackb(socket.recv(), object_hook=mnp.decode, raw=False)
            endpoint = request.get("endpoint") if isinstance(request, dict) else None
            try:
                if endpoint == "ping":
                    reply = {"status": "ok", "backend": backend.name, "device": device}
                elif endpoint == "segment":
                    reply = _segment_request(backend, request.get("data") or {}, args)
                else:
                    raise ValueError(f"unknown endpoint {endpoint!r}")
            except Exception as error:  # noqa: BLE001 - the client gets every failure as a reply
                reply = {"error": f"{type(error).__name__}: {error}"}
                print(f"  {reply['error']}", file=sys.stderr, flush=True)
            socket.send(msgpack.packb(reply, default=mnp.encode))
    except KeyboardInterrupt:
        pass
    finally:
        socket.close(linger=0)
        context.term()


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
                f"    {instance['label']:20s} score {instance['score']:.2f}  "
                f"roi {roi}  {pixels} px"
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
    parser.add_argument("--backend", choices=["grounded-sam2", "sam3"], default="grounded-sam2")
    parser.add_argument("--detector-model", default=DEFAULT_DETECTOR)
    parser.add_argument("--segmenter-model", default=DEFAULT_SEGMENTER)
    parser.add_argument("--sam3-model", default=DEFAULT_SAM3)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--dtype", choices=["float32", "bfloat16", "float16"], default="float32")
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
        default="red cube,green cylinder,blue sphere,yellow box,white cup",
        help="comma separated, used by --self-test",
    )
    args = parser.parse_args()

    try:
        backend, device = _build_backend(args)
    except OSError as error:
        # A gated checkpoint fails here, and the message is about a 403 rather than about access.
        sys.exit(f"{error}\n\nthe checkpoint could not be loaded; {VENV_HINT}")

    if args.self_test:
        return self_test(backend, args)
    serve(backend, device, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
