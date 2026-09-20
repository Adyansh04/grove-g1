#!/usr/bin/env python3
"""Serves open-vocabulary instance masks to g1_perception, on the host rather than in the container.

Same split as scripts/groot_server.py and for the same reason: the segmentation models need torch
and CUDA, the ROS image deliberately has neither, and g1_detector speaks this server's ZMQ protocol
instead of importing a model package.

Two segmentation backends, chosen with --backend, behind one reply format:

  grounded-sam2  Grounding DINO boxes refined into masks by SAM 2.1. Apache-2.0 and ungated, so it
                 is the default and the only one that works out of the box.
  sam3           One model, text straight to instance masks. Better on everything measured, and
                 gated: request access on the model page first. Once the weights are there this is
                 a flag, because the reply format below is all g1_perception ever sees.

    ./scripts/vision_server.py --port 5560
    ./scripts/vision_server.py --backend sam3 --port 5560
    ./scripts/vision_server.py --vlm Qwen/Qwen3-VL-2B-Instruct --port 5560

A vision-language model is loaded only when --vlm names one, and only on the first `ground`
request, because it answers a different question: which objects an instruction is about. The
detector cannot parse "the mug left of the bowl"; this turns that into noun phrases it can.

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
import json
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


def _iou(a, b):
    """Intersection over union of two xyxy boxes."""
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    overlap = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    if overlap <= 0.0:
        return 0.0
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - overlap
    return overlap / union if union > 0.0 else 0.0


def _drop_cross_phrase_duplicates(boxes, labels, scores, iou_threshold=0.5):
    """Keeps one phrase per region: a patch of image is one object, whatever it is called.

    Each phrase is prompted separately, so nothing stops two of them claiming the same pixels.
    Observed on the tabletop: "brown box container" matched the red block at 0.41 on exactly the
    ROI "bright red plastic block" had at 0.82. Downstream that is two tracks answering to one
    phrase, which costs the phrase its bare-phrase alias, and the alias is the name the skills
    resolve an object by, so the bench simply stops existing for them.

    Scores from different prompts are not strictly comparable, but the case this exists for is
    not close: the phrase that actually names the thing wins by a wide margin.
    """
    order = sorted(range(len(boxes)), key=lambda i: scores[i], reverse=True)
    kept = []
    for i in order:
        if any(
            labels[i] != labels[j] and _iou(boxes[i], boxes[j]) > iou_threshold for j in kept
        ):
            continue
        kept.append(i)
    kept.sort()
    return ([boxes[i] for i in kept], [labels[i] for i in kept], [scores[i] for i in kept])


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
        # A prompt per phrase, run one at a time. Grounding DINO answers with the span of the
        # prompt it matched, and in a joint prompt that span can cross a phrase boundary:
        # "a red block a brown box" names two objects and matches neither, so the object id it
        # becomes is one nothing can address. A prompt per phrase makes the label exact by
        # construction rather than inferred from the returned text.
        #
        # Sequential, not batched: batching every phrase against its own copy of the image wants
        # 2.3 GB more than a 12 GB card has spare while the simulator renders on it too, and the
        # out-of-memory error loses the whole frame.
        boxes, labels, scores = [], [], []
        for phrase in phrases:
            inputs = self._detector_processor(
                images=image, text=f"a {phrase}.", return_tensors="pt"
            ).to(self._device)
            with torch.inference_mode():
                detection = self._detector_processor.post_process_grounded_object_detection(
                    self._detector(**inputs),
                    inputs.input_ids,
                    threshold=box_threshold,
                    text_threshold=text_threshold,
                    target_sizes=[image.shape[:2]],
                )[0]
            for box, score in zip(
                detection["boxes"].float().cpu().numpy().tolist(),
                detection["scores"].float().cpu().numpy().tolist(),
                strict=True,
            ):
                boxes.append(box)
                labels.append(phrase)
                scores.append(float(score))
        if not boxes:
            return []

        boxes, labels, scores = _drop_cross_phrase_duplicates(boxes, labels, scores)

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
                    "label": label,
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


GROUNDING_PROMPT = """Look at this image and read this instruction: "{instruction}"

Answer with JSON only, no prose:
{{"phrases": ["short noun phrase", ...], "target": "the phrase the instruction is about",
 "points": [{{"phrase": "...", "x": <pixel>, "y": <pixel>}}]}}

Each phrase names one kind of object in at most three words, with a colour or size if that tells
two apart. No relations between objects and no sentences: a detector will be asked for each
phrase on its own. Point at the target's centre if you can; leave "points" empty if you cannot.
"""


class GroundingBackend:
    """A vision-language model, loaded on the first request rather than at startup."""

    def __init__(self, model_id, device, dtype, max_new_tokens):
        self._model_id = model_id
        self._device = device
        self._dtype = dtype
        self._max_new_tokens = max_new_tokens
        self._model = None
        self._processor = None

    @property
    def name(self):
        return self._model_id

    def _load(self):
        if self._model is not None:
            return
        from transformers import AutoModelForImageTextToText, AutoProcessor

        print(f"loading {self._model_id} for grounding", flush=True)
        self._processor = AutoProcessor.from_pretrained(self._model_id)
        self._model = (
            AutoModelForImageTextToText.from_pretrained(self._model_id, dtype=self._dtype)
            .to(self._device)
            .eval()
        )

    def ground(self, image, instruction):
        self._load()
        from PIL import Image

        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": Image.fromarray(image)},
                    {"type": "text", "text": GROUNDING_PROMPT.format(instruction=instruction)},
                ],
            }
        ]
        inputs = self._processor.apply_chat_template(
            messages,
            add_generation_prompt=True,
            tokenize=True,
            return_dict=True,
            return_tensors="pt",
        ).to(self._device)
        with torch.inference_mode():
            generated = self._model.generate(**inputs, max_new_tokens=self._max_new_tokens)
        answer = self._processor.batch_decode(
            generated[:, inputs["input_ids"].shape[1] :], skip_special_tokens=True
        )[0]
        return parse_grounding(answer)


def parse_grounding(answer):
    """The JSON out of a model's answer, however much prose it wrapped around it."""
    start = answer.find("{")
    end = answer.rfind("}")
    if start < 0 or end <= start:
        raise ValueError(f"no JSON in the model's answer: {answer[:200]!r}")
    parsed = json.loads(answer[start : end + 1])
    phrases = [str(phrase).strip() for phrase in parsed.get("phrases", []) if str(phrase).strip()]
    if not phrases:
        raise ValueError("the model named no objects")
    target = str(parsed.get("target", "")).strip() or phrases[0]
    if target not in phrases:
        # A target nobody can be asked for is worse than a guess: the detector takes phrases.
        phrases.append(target)
    points = []
    for point in parsed.get("points", []):
        try:
            points.append(
                {
                    "phrase": str(point["phrase"]),
                    "x": int(round(float(point["x"]))),
                    "y": int(round(float(point["y"]))),
                }
            )
        except (KeyError, TypeError, ValueError):
            continue
    return {"phrases": phrases, "target": target, "points": points}


def _ground_request(grounding, payload):
    if grounding is None:
        raise ValueError("this server was started without --vlm, so it cannot ground anything")
    image = payload.get("image")
    instruction = str(payload.get("instruction", "")).strip()
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image must be an (H, W, 3) uint8 array")
    if not instruction:
        return {"error": "ValueError: an instruction is required"}
    started = time.perf_counter()
    result = grounding.ground(image, instruction)
    result["model"] = grounding.name
    result["elapsed_ms"] = (time.perf_counter() - started) * 1000.0
    return result


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


def serve(backend, device, args, grounding=None):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://{args.host}:{args.port}")
    print(f"serving {backend.name} on {device} at tcp://{args.host}:{args.port}", flush=True)
    if grounding is not None:
        print(f"  grounding with {grounding.name}, loaded on the first request", flush=True)
    try:
        while True:
            request = msgpack.unpackb(socket.recv(), object_hook=mnp.decode, raw=False)
            endpoint = request.get("endpoint") if isinstance(request, dict) else None
            try:
                if endpoint == "ping":
                    reply = {"status": "ok", "backend": backend.name, "device": device}
                elif endpoint == "segment":
                    reply = _segment_request(backend, request.get("data") or {}, args)
                elif endpoint == "ground":
                    reply = _ground_request(grounding, request.get("data") or {})
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
    parser.add_argument(
        "--vlm",
        default="",
        help="a vision-language model for the ground endpoint, e.g. Qwen/Qwen3-VL-2B-Instruct. "
        "Loaded on the first request; without it the endpoint says so.",
    )
    parser.add_argument("--vlm-max-new-tokens", type=int, default=256)
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
    grounding = None
    if args.vlm:
        dtype = {"float32": torch.float32, "bfloat16": torch.bfloat16, "float16": torch.float16}[
            args.dtype
        ]
        grounding = GroundingBackend(args.vlm, device, dtype, args.vlm_max_new_tokens)
    serve(backend, device, args, grounding)
    return 0


if __name__ == "__main__":
    sys.exit(main())
