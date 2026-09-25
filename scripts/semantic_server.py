#!/usr/bin/env python3
"""Serves detection, embeddings and descriptions for the semantic map, on the host GPU.

The ROS image carries no torch or CUDA, so the container reaches the models over ZMQ, as it does
scripts/vision_server.py. `segment` answers in the vision server's format, so g1_detector works
unchanged when pointed at this port. The difference is what `phrases` means: here it is the
detector's vocabulary, scored against every region in one forward pass rather than one pass per
phrase, so a list of a hundred names costs about what one does.

Each backend is chosen by a flag or by --config, and the replies name no model-specific fields:

  --detector   yoloe     YOLOE-26 prompted with the request's phrases (default)
               yoloe-pf  YOLOE-26 prompt-free: its built-in vocabulary; phrases are ignored
  --embedder   siglip2   SigLIP 2 image and text embeddings (default), or none
  --describer  gemini    Gemini REST API, free tier: at most 5 requests a minute and 100 a day
               openai    any OpenAI-compatible server; scripts/start-vlm.sh runs one locally
               none      echoes the detector's label at confidence 0, so describe never fails
               A list such as gemini,openai (the default) falls through in order when one is
               rate limited or unreachable.

Models:
  YOLOE-26l-seg, -seg-pf   https://docs.ultralytics.com/models/yoloe (arXiv 2503.07465)
  MobileCLIP2-B text       https://github.com/apple/ml-mobileclip (YOLOE's text encoder)
  SigLIP 2 B/16-256        https://huggingface.co/google/siglip2-base-patch16-256
  Gemini 3.5 Flash-Lite    https://ai.google.dev/gemini-api/docs/models
  Qwen3.5-4B Q4_K_M        https://huggingface.co/Qwen/Qwen3.5-4B, GGUF from
                           https://huggingface.co/unsloth/Qwen3.5-4B-GGUF

    ./scripts/semantic_server.py
    ./scripts/semantic_server.py --describer openai          # offline, after start-vlm.sh start
    ./scripts/semantic_server.py --config semantic.yaml --set yoloe.imgsz=800
    ./scripts/semantic_server.py --self-test frame.png --describe-test

Run scripts/setup-semantic.sh first. --config takes a YAML file shaped like DEFAULTS below.

Wire protocol, msgpack with msgpack_numpy for the arrays. Any failure is {"error": "..."}:

    {"endpoint": "ping"} -> {"status": "ok", "backend", "device", "embedder", "describers"}
    {"endpoint": "segment", "data": {"image": uint8 (H, W, 3) RGB, "phrases": ["chair", ...],
                                     "box_threshold": 0.3, "embed": false}}
      -> {"model", "elapsed_ms", "instances": [{"label", "score", "roi": [x, y, w, h],
          "mask": uint8 (h, w) 0 or 255, "embedding": float32 (D,), only with embed}]}
    {"endpoint": "embed_text", "data": {"texts": ["a mug", ...]}}
      -> {"model", "embeddings": float32 (N, D), L2-normalised}
    {"endpoint": "embed_image", "data": {"images": [uint8 (H, W, 3), ...]}} -> the same
    {"endpoint": "describe", "data": {"task": "object", "images": [crops],
                                      "context": {"labels": {"chair": 5, "stool": 1}}}}
    {"endpoint": "describe", "data": {"task": "room", "images": [views],
                                      "context": {"objects": {"bed": 1}, "room_type": "bedroom"}}}
      -> {"model", "backend", "name", "caption", "label_ok", "room_type", "confidence",
          "elapsed_ms"}

One request at a time: a describe call of a few seconds delays the next segment by as much.
"""

import argparse
import base64
import contextlib
import copy
import fcntl
import io
import json
import math
import os
import re
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

VENV_HINT = "scripts/setup-semantic.sh has not been run, or the wrong interpreter is being used"
REPO = Path(__file__).resolve().parents[1]
VOCABULARY = REPO / "workspace/src/g1_perception/config/indoor_vocabulary.yaml"
WEIGHTS = "~/ref/grove-semantic/weights"

DEFAULTS = {
    "host": "127.0.0.1",
    "port": 5561,
    "device": "auto",
    "detector": "yoloe",
    "embedder": "siglip2",
    "describer": "gemini,openai",
    # When a request carries none; g1_detector always sends its own.
    "box_threshold": 0.25,
    "yoloe": {"weights": f"{WEIGHTS}/yoloe-26l-seg.pt", "imgsz": 640, "half": True, "max_det": 100},
    "yoloe-pf": {
        "weights": f"{WEIGHTS}/yoloe-26l-seg-pf.pt",
        "imgsz": 640,
        "half": True,
        "max_det": 100,
    },
    "siglip2": {"model": "google/siglip2-base-patch16-256"},
    "gemini": {
        "model": "gemini-3.5-flash-lite",
        "key_file": "~/.config/grove/gemini.env",
        "usage_file": "~/.config/grove/gemini_usage.json",
        "per_minute": 5,
        "per_day": 100,
        "timeout_s": 20.0,
    },
    "openai": {"base_url": "http://127.0.0.1:8080/v1", "model": "qwen3.5-4b", "timeout_s": 60.0},
}

# Mid grey is zero after SigLIP's normalisation, so masked-out pixels carry no signal.
GREY = 128
# Longest side of an image sent to a describer: past this the tokens cost more than they tell.
MAX_IMAGE_SIDE = 512
# Eight 512 px images are about 2k tokens, half of the local VLM's context.
MAX_DESCRIBE_IMAGES = 8

ROOM_TYPES = (
    "kitchen",
    "living room",
    "dining room",
    "bedroom",
    "bathroom",
    "office",
    "storage room",
    "hallway",
    "other",
)
ROOM_SYNONYMS = {
    "lounge": "living room",
    "family room": "living room",
    "living": "living room",
    "dining": "dining room",
    "study": "office",
    "bath": "bathroom",
    "toilet": "bathroom",
    "restroom": "bathroom",
    "washroom": "bathroom",
    "storage": "storage room",
    "pantry": "storage room",
    "closet": "storage room",
    "garage": "storage room",
    "utility": "storage room",
    "corridor": "hallway",
    "hall": "hallway",
    "entry": "hallway",
    "foyer": "hallway",
}


class Unavailable(Exception):
    """A describer that cannot answer now: rate limited, out of quota, or unreachable."""


# Detectors: `name` and segment(image, phrases, box_threshold, text_threshold) -> instances, the
# vision server's interface. Its GroundedSam2Backend and Sam3Backend fit as they are, given an
# `import torch` and a DETECTORS entry.


def _roi_and_crop(mask):
    """Full-frame boolean mask to (roi, cropped uint8 mask), or None when it is empty."""
    rows = np.flatnonzero(mask.any(axis=1))
    cols = np.flatnonzero(mask.any(axis=0))
    if rows.size == 0 or cols.size == 0:
        return None
    y0, y1 = int(rows[0]), int(rows[-1]) + 1
    x0, x1 = int(cols[0]), int(cols[-1]) + 1
    crop = np.ascontiguousarray(mask[y0:y1, x0:x1].astype(np.uint8) * 255)
    return [x0, y0, x1 - x0, y1 - y0], crop


def _box_iou(a, b):
    """Intersection over union of two [x, y, w, h] boxes."""
    width = min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0])
    height = min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1])
    if width <= 0 or height <= 0:
        return 0.0
    overlap = width * height
    return overlap / (a[2] * a[3] + b[2] * b[3] - overlap)


def _suppress_duplicates(instances, iou_threshold=0.7):
    """Drops an instance whose box repeats a better-scored one's, whatever either is called.

    YOLOE-26's end-to-end head skips NMS, and neighbouring anchors can each report one object;
    every copy would become a track. 0.7 is ultralytics' own NMS threshold.
    """
    kept = []
    for instance in sorted(instances, key=lambda candidate: -candidate["score"]):
        if all(_box_iou(instance["roi"], other["roi"]) <= iou_threshold for other in kept):
            kept.append(instance)
    return kept


class YoloeDetector:
    """YOLOE-26 instance masks; text-prompted by the phrases, or prompt-free."""

    def __init__(
        self, model, device, weights_dir, prompt_free=False, imgsz=640, half=True, max_det=100
    ):
        self.name = "yoloe-pf" if prompt_free else "yoloe"
        self._model = model
        self._device = device
        self._weights_dir = Path(weights_dir)
        self._prompt_free = prompt_free
        self._imgsz = imgsz
        self._half = half
        self._max_det = max_det
        self._vocabulary = None

    def segment(self, image, phrases, box_threshold, text_threshold):
        del text_threshold  # one calibrated score per region
        if not self._prompt_free:
            self._use_vocabulary(tuple(phrases))
        result = self._model.predict(
            # Ultralytics reads a numpy image as OpenCV BGR; the wire carries RGB.
            np.ascontiguousarray(image[..., ::-1]),
            conf=box_threshold,
            imgsz=self._imgsz,
            quantize=16 if self._half else 32,
            device=self._device,
            max_det=self._max_det,
            # Full-resolution masks, cut to each box like the vision server's.
            retina_masks=True,
            # One label per region: otherwise the end-to-end head emits a region once per class
            # that scores above the threshold, and each copy would become a track.
            agnostic_nms=True,
            verbose=False,
        )[0]
        if result.masks is None:
            return []
        instances = []
        masks = result.masks.data.bool().cpu().numpy()
        for mask, score, cls in zip(
            masks, result.boxes.conf.tolist(), result.boxes.cls.tolist(), strict=True
        ):
            cropped = _roi_and_crop(mask)
            if cropped is None:
                continue
            roi, data = cropped
            instances.append(
                {"label": result.names[int(cls)], "score": float(score), "roi": roi, "mask": data}
            )
        return _suppress_duplicates(instances)

    def _use_vocabulary(self, vocabulary):
        if not vocabulary:
            raise ValueError("at least one phrase is required; --detector yoloe-pf takes none")
        if vocabulary == self._vocabulary:
            return
        # set_classes re-encodes every phrase and rebuilds the predictor, so only a new list pays.
        # The text encoder is looked up in the working directory.
        with contextlib.chdir(self._weights_dir):
            self._model.set_classes(list(vocabulary))
        self._vocabulary = vocabulary


def _build_yoloe(section, device, prompt_free):
    os.environ.setdefault("YOLO_AUTOINSTALL", "False")
    os.environ.setdefault("YOLO_OFFLINE", "True")  # no analytics, no surprise downloads
    from ultralytics import YOLOE

    weights = Path(section["weights"]).expanduser()
    if not weights.is_file():
        raise FileNotFoundError(f"{weights} is missing; {VENV_HINT}")
    model = YOLOE(str(weights))
    model.to(device)
    return YoloeDetector(
        model,
        device,
        weights.parent,
        prompt_free,
        int(section["imgsz"]),
        bool(section["half"]),
        int(section["max_det"]),
    )


# Embedders: `name`, embed_text(texts) and embed_images(images), each an L2-normalised
# float32 (N, D) array.


class Siglip2Embedder:
    """SigLIP 2: object crops and query text in one space, compared by dot product."""

    def __init__(self, device, model_id):
        import torch
        from transformers import AutoModel, AutoProcessor

        self.name = model_id
        self._device = device
        self._dtype = torch.float16 if device.startswith("cuda") else torch.float32
        self._processor = AutoProcessor.from_pretrained(model_id)
        self._model = AutoModel.from_pretrained(model_id, dtype=self._dtype).to(device).eval()

    def embed_text(self, texts):
        # Trained on lower-case text padded to 64 tokens; other shapes shift the embedding.
        inputs = self._processor(
            text=[text.lower() for text in texts],
            padding="max_length",
            max_length=64,
            truncation=True,
            return_tensors="pt",
        )
        return self._encode(
            self._model.get_text_features, input_ids=inputs["input_ids"].to(self._device)
        )

    def embed_images(self, images):
        # Arrays off the wire are read-only, and torch warns on wrapping them.
        images = [np.require(image, requirements="W") for image in images]
        inputs = self._processor(images=images, return_tensors="pt")
        return self._encode(
            self._model.get_image_features,
            pixel_values=inputs["pixel_values"].to(self._device, self._dtype),
        )

    @staticmethod
    def _encode(forward, **inputs):
        import torch

        with torch.inference_mode():
            output = forward(**inputs)
        features = getattr(output, "pooler_output", output)
        return torch.nn.functional.normalize(features.float(), dim=-1).cpu().numpy()


def masked_crop(image, roi, mask, pad=0.1):
    """The instance alone on a grey square with a margin: what gets embedded for it."""
    x, y, w, h = (int(value) for value in roi)
    side = int(round(max(w, h) * (1.0 + 2.0 * pad)))
    canvas = np.full((side, side, 3), GREY, np.uint8)
    top, left = (side - h) // 2, (side - w) // 2
    np.copyto(
        canvas[top : top + h, left : left + w],
        image[y : y + h, x : x + w],
        where=np.asarray(mask)[..., None] > 0,
    )
    return canvas


def padded_crop(image, roi, pad=0.1):
    """The instance's box and some surroundings: what a describer is shown."""
    x, y, w, h = (int(value) for value in roi)
    dx, dy = int(w * pad), int(h * pad)
    return image[max(y - dy, 0) : y + h + dy, max(x - dx, 0) : x + w + dx]


# Describers: `name` and describe(task, images, context) -> {"model", "name", "caption",
# "label_ok", "room_type", "confidence"}, raising Unavailable (or anything) to fall through.

PACIFIC = ZoneInfo("America/Los_Angeles")
# Hard ceilings for the free tier; config may lower them, never raise them.
GEMINI_MAX_PER_MINUTE = 5
GEMINI_MAX_PER_DAY = 100


class GeminiLimiter:
    """Client-side caps on Gemini's free tier, shared by every process through a locked file.

    The day is Google's quota day, which turns over at midnight Pacific. Every request counts,
    failed ones too, and the count is taken before the request is sent.
    """

    def __init__(
        self, path, per_minute=GEMINI_MAX_PER_MINUTE, per_day=GEMINI_MAX_PER_DAY, clock=time.time
    ):
        self._path = Path(path).expanduser()
        self.per_minute = min(int(per_minute), GEMINI_MAX_PER_MINUTE)
        self.per_day = min(int(per_day), GEMINI_MAX_PER_DAY)
        self._clock = clock

    def acquire(self):
        """Counts one request, or raises Unavailable without counting it."""

        def take(state, now):
            if state["exhausted"]:
                raise Unavailable("Gemini's quota is exhausted until midnight Pacific")
            if state["count"] >= self.per_day:
                raise Unavailable(f"Gemini's cap of {self.per_day} requests a day is reached")
            if len(state["recent"]) >= self.per_minute:
                raise Unavailable(f"Gemini's cap of {self.per_minute} requests a minute is reached")
            state["count"] += 1
            state["recent"].append(now)

        self._update(take)

    def mark_exhausted(self):
        """No more requests until the next Pacific day: Google has refused on quota."""
        self._update(lambda state, now: state.update(exhausted=True))

    def usage(self):
        return self._update(lambda state, now: dict(state))

    def _update(self, change):
        self._path.parent.mkdir(parents=True, exist_ok=True)
        # A separate lock file, because the state file is replaced rather than rewritten.
        with open(self._path.with_name(self._path.name + ".lock"), "a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            now = self._clock()
            state = self._load(now)
            result = change(state, now)
            staging = self._path.with_name(self._path.name + ".tmp")
            staging.write_text(json.dumps(state))
            # Atomic, so a crash never leaves a truncated count that reads as zero.
            os.replace(staging, self._path)
            return result

    def _load(self, now):
        today = datetime.fromtimestamp(now, PACIFIC).date().isoformat()
        try:
            state = json.loads(self._path.read_text())
            recent = [float(stamp) for stamp in state.get("recent", []) if now - stamp < 60.0]
            count, exhausted = int(state.get("count", 0)), bool(state.get("exhausted", False))
        except FileNotFoundError:
            state, recent, count, exhausted = {}, [], 0, False
        except (ValueError, TypeError, AttributeError) as error:
            # Fail closed: an unreadable count must not read as a fresh day.
            raise Unavailable(
                f"{self._path} is unreadable ({error}); delete it to reset"
            ) from error
        if state.get("day") != today:
            return {"day": today, "count": 0, "exhausted": False, "recent": recent}
        return {"day": today, "count": count, "exhausted": exhausted, "recent": recent}


def http_post(url, headers, body, timeout):
    """(status, body) of a POST; Unavailable when the server cannot be reached at all."""
    request = urllib.request.Request(url, data=body, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as error:
        return error.code, error.read()
    except OSError as error:  # refused, reset, timed out, DNS
        raise Unavailable(f"{url.split('/')[2]} is unreachable: {error}") from error


def _jpeg_base64(image):
    from PIL import Image

    picture = Image.fromarray(np.asarray(image, dtype=np.uint8))
    picture.thumbnail((MAX_IMAGE_SIDE, MAX_IMAGE_SIDE))
    buffer = io.BytesIO()
    picture.save(buffer, format="JPEG", quality=90)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def _votes(context, key):
    """{name: count}, most votes first, from a dict, a list of names, or [name, count] pairs."""
    raw = context.get(key) or {}
    pairs = (
        raw.items()
        if isinstance(raw, dict)
        else (
            (entry[0], entry[1]) if isinstance(entry, (list, tuple)) else (entry, 1)
            for entry in raw
        )
    )
    votes = {}
    for name, count in pairs:
        name = " ".join(str(name).split())
        if name:
            votes[name] = votes.get(name, 0) + float(count)
    return dict(sorted(votes.items(), key=lambda item: -item[1]))


def _listing(votes):
    return ", ".join(f"{name} ({count:g})" for name, count in votes.items()) or "none"


def _prompt(task, n_images, context):
    if task == "object":
        votes = _votes(context, "labels")
        if not votes:
            raise ValueError("an object's context needs the detector's labels")
        views = "The image shows" if n_images == 1 else f"The {n_images} images show"
        # The detector's label comes last and only for label_ok: named up front, it anchors the
        # model, which then calls a fridge a cabinet because the detector did.
        return (
            f"{views} one object a robot saw indoors, cropped from its camera.\n\n"
            "Answer in JSON:\n"
            "- name: what the object is, judged from the image alone: a common noun of 1 to 3 "
            'words, singular and lower case, such as "office chair", "mug" or "floor lamp".\n'
            "- caption: one short sentence on how it looks: colour, material, notable parts.\n"
            "- label_ok: an object detector, which is often wrong, called it "
            f'"{next(iter(votes))}". True if that is a fair name for what you see, else false.\n'
            "- confidence: 0 to 1, how sure you are of the name."
        )
    guess = _room_type(context.get("room_type")) if context.get("room_type") else ""
    views = " The images are views from its camera." if n_images else ""
    return (
        f"A robot mapped one room of a home or office.{views} Objects found in it "
        f"(counts in brackets): {_listing(_votes(context, 'objects'))}.\n\n"
        "Answer in JSON:\n"
        f"- room_type: one of {', '.join(ROOM_TYPES)}.\n"
        '- name: a short name for the room, 1 to 3 words, such as "home office".\n'
        "- caption: one sentence on what the room is used for.\n"
        + (
            f'- label_ok: true if "{guess}" suits the room, else false.\n'
            if guess
            else "- label_ok: true.\n"
        )
        + "- confidence: 0 to 1, how sure you are of room_type."
    )


def _schema(task):
    properties = {
        "name": {"type": "string"},
        "caption": {"type": "string"},
        "label_ok": {"type": "boolean"},
        "confidence": {"type": "number"},
    }
    if task == "room":
        properties["room_type"] = {"type": "string", "enum": list(ROOM_TYPES)}
    return {"type": "object", "properties": properties, "required": list(properties)}


def _gemini_schema(schema):
    """JSON Schema to the OpenAPI subset that Gemini's responseSchema takes."""
    converted = {"type": schema["type"].upper()}
    if "enum" in schema:
        converted["enum"] = schema["enum"]
    if "properties" in schema:
        converted["properties"] = {
            key: _gemini_schema(value) for key, value in schema["properties"].items()
        }
        converted["required"] = schema["required"]
    return converted


def _noun(value):
    words = re.sub(r"[^a-z0-9\s-]", " ", str(value or "").lower()).split()
    if words and words[0] in ("a", "an", "the"):
        words = words[1:]
    # English puts the head noun last: "large red office chair" keeps "red office chair".
    return " ".join(words[-3:])


def _room_type(value):
    text = " ".join(str(value or "").lower().replace("_", " ").replace("-", " ").split())
    for room in ROOM_TYPES[:-1]:  # not "other", which is inside words like "mother"
        if room in text:
            return room
    for word, room in ROOM_SYNONYMS.items():
        if word in text:
            return room
    return "other"


def _confidence(value):
    try:
        number = float(str(value).strip().rstrip("%"))
    except ValueError:
        return 0.5
    if not math.isfinite(number):
        return 0.5
    if number > 1.0:  # a percentage
        number /= 100.0
    return min(max(number, 0.0), 1.0)


def parse_answer(text, task):
    """A describer's JSON answer, normalised; ValueError when it holds nothing usable."""
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end <= start:
        raise ValueError(f"no JSON in the answer: {text[:200]!r}")
    raw = json.loads(text[start : end + 1])
    if not isinstance(raw, dict):
        raise ValueError(f"the answer is not a JSON object: {text[:200]!r}")
    fields = {str(key).strip().lower(): value for key, value in raw.items()}
    name = _noun(fields.get("name"))
    room_type = _room_type(fields.get("room_type")) if task == "room" else ""
    if task == "object" and not name:
        raise ValueError(f"the answer names no object: {text[:200]!r}")
    return {
        "name": name or room_type,
        "caption": " ".join(str(fields.get("caption") or "").split())[:300],
        "label_ok": str(fields.get("label_ok")).strip().lower() in ("true", "yes", "1"),
        "room_type": room_type,
        "confidence": _confidence(fields.get("confidence")),
    }


def _read_key(path):
    """The key from a file holding it bare or as NAME=key; None when there is no file."""
    try:
        lines = Path(path).expanduser().read_text().splitlines()
    except FileNotFoundError:
        return None
    for line in lines:
        line = line.strip()
        if line and not line.startswith("#"):
            return line.partition("=")[2].strip().strip("'\"") if "=" in line else line
    return None


class GeminiDescriber:
    """Gemini's REST API with JSON output, behind the free-tier limiter."""

    name = "gemini"
    URL = "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"

    def __init__(self, model, key_file, limiter, timeout_s=20.0, transport=http_post):
        self.model = model
        self._key_file = key_file
        self._key = _read_key(key_file)
        self._limiter = limiter
        self._timeout_s = float(timeout_s)
        self._transport = transport

    def describe(self, task, images, context):
        if not self._key:
            raise Unavailable(f"no Gemini key in {self._key_file}")
        parts = [
            {"inlineData": {"mimeType": "image/jpeg", "data": _jpeg_base64(image)}}
            for image in images
        ]
        body = {
            "contents": [
                {"role": "user", "parts": [*parts, {"text": _prompt(task, len(images), context)}]}
            ],
            "generationConfig": {
                "responseMimeType": "application/json",
                "responseSchema": _gemini_schema(_schema(task)),
                "mediaResolution": "MEDIA_RESOLUTION_LOW",
                "maxOutputTokens": 1024,
            },
        }
        # Last step before the network, and the request counts even if the call then fails.
        self._limiter.acquire()
        # The key goes in a header only: URLs end up in logs and error messages.
        status, reply = self._transport(
            self.URL.format(model=self.model),
            {"x-goog-api-key": self._key, "Content-Type": "application/json"},
            json.dumps(body).encode(),
            self._timeout_s,
        )
        if status != 200:
            try:
                error = json.loads(reply).get("error", {})
            except (ValueError, AttributeError):
                error = {}
            detail = self._redact(f"{error.get('status', '')} {error.get('message', '')}".strip())
            if status == 429 or "RESOURCE_EXHAUSTED" in detail or "quota" in detail.lower():
                self._limiter.mark_exhausted()
                raise Unavailable(
                    f"Gemini refused on quota (HTTP {status}): no more calls until midnight Pacific"
                )
            raise Unavailable(f"Gemini answered HTTP {status}: {detail[:200]}")
        data = json.loads(reply)
        candidate = (data.get("candidates") or [{}])[0]
        text = "".join(
            part.get("text", "")
            for part in candidate.get("content", {}).get("parts", [])
            if not part.get("thought")
        )
        answer = parse_answer(text, task)
        answer["model"] = data.get("modelVersion", self.model)
        return answer

    def _redact(self, text):
        return text.replace(self._key, "<key>") if self._key else text


class OpenAIDescriber:
    """An OpenAI-compatible chat completions server, such as llama.cpp's on this host."""

    name = "openai"

    def __init__(self, base_url, model, timeout_s=60.0, transport=http_post):
        self.model = model
        self._url = base_url.rstrip("/") + "/chat/completions"
        self._timeout_s = float(timeout_s)
        self._transport = transport

    def describe(self, task, images, context):
        content = [
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{_jpeg_base64(i)}"}}
            for i in images
        ]
        content.append({"type": "text", "text": _prompt(task, len(images), context)})
        body = {
            "model": self.model,
            "messages": [{"role": "user", "content": content}],
            "temperature": 0.0,
            "max_tokens": 256,
            "response_format": {
                "type": "json_schema",
                "json_schema": {"name": task, "schema": _schema(task)},
            },
        }
        status, reply = self._transport(
            self._url,
            {"Content-Type": "application/json"},
            json.dumps(body).encode(),
            self._timeout_s,
        )
        if status != 200:
            raise Unavailable(f"{self._url} answered HTTP {status}: {reply[:200]!r}")
        data = json.loads(reply)
        answer = parse_answer(data["choices"][0]["message"].get("content") or "", task)
        answer["model"] = data.get("model", self.model)
        return answer


class NoDescriber:
    """The detector's own label, or the caller's room guess, at confidence 0."""

    name = "none"

    def describe(self, task, images, context):
        del images
        if task == "object":
            votes = _votes(context, "labels")
            if not votes:
                raise ValueError("an object's context needs the detector's labels")
            name, room_type = next(iter(votes)), ""
        else:
            name = room_type = _room_type(context.get("room_type"))
        return {
            "model": "none",
            "name": name,
            "caption": "",
            "label_ok": True,
            "room_type": room_type,
            "confidence": 0.0,
        }


class DescriberChain:
    """Asks each describer in turn until one answers; the order is the preference."""

    def __init__(self, describers):
        self.describers = list(describers)

    @property
    def names(self):
        return [describer.name for describer in self.describers]

    def describe(self, task, images, context):
        failures = []
        for describer in self.describers:
            try:
                answer = describer.describe(task, images, context)
            except Exception as error:  # noqa: BLE001 - any failure moves on to the next one
                failures.append(f"{describer.name}: {error}")
                print(
                    f"  {describer.name} could not describe: {error}", file=sys.stderr, flush=True
                )
                continue
            answer["backend"] = describer.name
            return answer
        raise RuntimeError("no describer answered: " + ("; ".join(failures) or "none configured"))


# Registries: one entry per backend, built from its section of the config.

DETECTORS = {
    "yoloe": lambda config, device: _build_yoloe(config["yoloe"], device, prompt_free=False),
    "yoloe-pf": lambda config, device: _build_yoloe(config["yoloe-pf"], device, prompt_free=True),
}
EMBEDDERS = {
    "siglip2": lambda config, device: Siglip2Embedder(device, config["siglip2"]["model"]),
    "none": lambda config, device: None,
}
DESCRIBERS = {
    "gemini": lambda config: GeminiDescriber(
        config["gemini"]["model"],
        config["gemini"]["key_file"],
        GeminiLimiter(
            config["gemini"]["usage_file"],
            config["gemini"]["per_minute"],
            config["gemini"]["per_day"],
        ),
        config["gemini"]["timeout_s"],
    ),
    "openai": lambda config: OpenAIDescriber(
        config["openai"]["base_url"], config["openai"]["model"], config["openai"]["timeout_s"]
    ),
    "none": lambda config: NoDescriber(),
}


def _image(value, what="image"):
    if not isinstance(value, np.ndarray) or value.ndim != 3 or value.shape[2] != 3:
        raise ValueError(f"{what} must be an (H, W, 3) uint8 array")
    if value.dtype != np.uint8:
        raise ValueError(f"{what} must be uint8, got {value.dtype}")
    return value


class SemanticServer:
    """The request handlers, apart from the socket so they can be driven directly."""

    def __init__(self, detector, embedder, describer, device="cpu", box_threshold=0.25):
        self.detector = detector
        self.embedder = embedder
        self.describer = describer
        self.device = device
        self.box_threshold = box_threshold

    def handle(self, request):
        """A reply for every request, failures included: a dropped reply strands a REQ socket."""
        endpoint = request.get("endpoint") if isinstance(request, dict) else None
        handlers = {
            "ping": self._ping,
            "segment": self._segment,
            "embed_text": self._embed_text,
            "embed_image": self._embed_image,
            "describe": self._describe,
        }
        try:
            if endpoint not in handlers:
                raise ValueError(f"unknown endpoint {endpoint!r}")
            data = request.get("data") or {}
            if not isinstance(data, dict):
                raise ValueError("data must be a map")
            return handlers[endpoint](data)
        except Exception as error:  # noqa: BLE001 - the client gets every failure as a reply
            reply = {"error": f"{type(error).__name__}: {error}"}
            print(f"  {endpoint}: {reply['error']}", file=sys.stderr, flush=True)
            return reply

    def _ping(self, data):
        del data
        return {
            "status": "ok",
            "backend": self.detector.name,
            "device": self.device,
            "embedder": self.embedder.name if self.embedder else "",
            "describers": self.describer.names,
        }

    def _segment(self, data):
        image = _image(data.get("image"))
        embed = bool(data.get("embed", False))
        if embed and self.embedder is None:
            raise ValueError("this server was started with --embedder none")
        # Stripped and deduplicated, so equal vocabularies hit the same cache entry.
        phrases = list(
            dict.fromkeys(
                str(phrase).strip() for phrase in data.get("phrases") or [] if str(phrase).strip()
            )
        )
        started = time.perf_counter()
        instances = self.detector.segment(
            image,
            phrases,
            float(data.get("box_threshold", self.box_threshold)),
            float(data.get("text_threshold", 0.25)),
        )
        reply = {"model": self.detector.name, "instances": instances}
        if embed and instances:
            embedded = time.perf_counter()
            embeddings = self.embedder.embed_images(
                [masked_crop(image, instance["roi"], instance["mask"]) for instance in instances]
            )
            for instance, embedding in zip(instances, embeddings, strict=True):
                instance["embedding"] = embedding
            reply["embed_ms"] = (time.perf_counter() - embedded) * 1000.0
        reply["elapsed_ms"] = (time.perf_counter() - started) * 1000.0
        return reply

    def _embed_text(self, data):
        texts = [str(text) for text in data.get("texts") or []]
        if not texts or not all(text.strip() for text in texts):
            raise ValueError("texts must be a non-empty list of non-empty strings")
        return {"model": self._embedder().name, "embeddings": self._embedder().embed_text(texts)}

    def _embed_image(self, data):
        images = [_image(image, "each image") for image in data.get("images") or []]
        if not images:
            raise ValueError("images must be a non-empty list")
        return {"model": self._embedder().name, "embeddings": self._embedder().embed_images(images)}

    def _embedder(self):
        if self.embedder is None:
            raise ValueError("this server was started with --embedder none")
        return self.embedder

    def _describe(self, data):
        task = data.get("task")
        if task not in ("object", "room"):
            raise ValueError(f"task must be 'object' or 'room', got {task!r}")
        images = [_image(image, "each image") for image in data.get("images") or []]
        if task == "object" and not images:
            raise ValueError("an object needs at least one image")
        if len(images) > MAX_DESCRIBE_IMAGES:
            raise ValueError(f"at most {MAX_DESCRIBE_IMAGES} images per request")
        context = data.get("context") or {}
        if not isinstance(context, dict):
            raise ValueError("context must be a map")
        started = time.perf_counter()
        answer = self.describer.describe(task, images, context)
        answer["elapsed_ms"] = (time.perf_counter() - started) * 1000.0
        return answer


def _merge(base, update):
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(args):
    """DEFAULTS, then --config, then the backend flags, then each --set section.key=value."""
    import yaml

    config = copy.deepcopy(DEFAULTS)
    if args.config:
        _merge(config, yaml.safe_load(Path(args.config).read_text()) or {})
    for key in ("host", "port", "device", "detector", "embedder", "describer"):
        if getattr(args, key) is not None:
            config[key] = getattr(args, key)
    for assignment in args.set:
        key, _, value = assignment.partition("=")
        *sections, leaf = key.split(".")
        target = config
        for section in sections:
            target = target.setdefault(section, {})
        target[leaf] = yaml.safe_load(value)
    return config


def _pick(registry, name, kind):
    if name not in registry:
        sys.exit(f"unknown {kind} {name!r}; one of: {', '.join(registry)}")
    return registry[name]


def build_server(config):
    import torch

    device = config["device"]
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    names = [name.strip() for name in str(config["describer"]).split(",") if name.strip()]
    describer = DescriberChain(_pick(DESCRIBERS, name, "describer")(config) for name in names)
    detector = _pick(DETECTORS, config["detector"], "detector")(config, device)
    embedder = _pick(EMBEDDERS, config["embedder"], "embedder")(config, device)
    return SemanticServer(detector, embedder, describer, device, float(config["box_threshold"]))


def serve(server, config):
    try:
        import msgpack
        import msgpack_numpy as mnp
        import zmq
    except ImportError as error:
        sys.exit(f"{error}. {VENV_HINT}")

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://{config['host']}:{config['port']}")
    embedder = server.embedder.name if server.embedder else "no embedder"
    print(
        f"serving {server.detector.name}, {embedder}, describers "
        f"{','.join(server.describer.names) or 'none'} on {server.device} at "
        f"tcp://{config['host']}:{config['port']}",
        flush=True,
    )
    try:
        while True:
            message = socket.recv()
            try:
                request = msgpack.unpackb(message, object_hook=mnp.decode, raw=False)
            except Exception as error:  # noqa: BLE001 - a REP socket must answer every request
                reply = {"error": f"undecodable request: {error}"}
            else:
                reply = server.handle(request)
            socket.send(msgpack.packb(reply, default=mnp.encode))
    except KeyboardInterrupt:
        pass
    finally:
        socket.close(linger=0)
        context.term()


def _load_phrases(value):
    """Comma-separated phrases, or the phrases of a g1_detector parameter file."""
    if value.endswith((".yaml", ".yml")):
        import yaml

        return list(
            yaml.safe_load(Path(value).read_text())["g1_detector"]["ros__parameters"]["phrases"]
        )
    return [phrase.strip() for phrase in value.split(",") if phrase.strip()]


def self_test(server, args):
    """Runs segment (and describe) over image files and prints what came back, without a socket."""
    import torch
    from PIL import Image

    phrases = _load_phrases(args.phrases)
    print(
        f"{server.detector.name} with {len(phrases)} phrases, "
        f"embedder {server.embedder.name if server.embedder else 'none'}"
    )
    for path in args.self_test:
        image = np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
        reply = server.handle(
            {
                "endpoint": "segment",
                "data": {"image": image, "phrases": phrases, "embed": server.embedder is not None},
            }
        )
        if "error" in reply:
            print(f"{path}  {reply['error']}")
            continue
        instances = sorted(reply["instances"], key=lambda instance: -instance["score"])
        print(
            f"{path}  {image.shape[1]}x{image.shape[0]}  {len(instances)} instances, "
            f"{reply['elapsed_ms']:.0f} ms (embed {reply.get('embed_ms', 0.0):.0f} ms)"
        )
        for instance in instances:
            print(
                f"    {instance['label']:18s} score {instance['score']:.2f}  "
                f"roi {instance['roi']}  {int((instance['mask'] > 0).sum())} px"
            )
        if args.describe_test and instances:
            best = instances[0]
            answer = server.handle(
                {
                    "endpoint": "describe",
                    "data": {
                        "task": "object",
                        "images": [padded_crop(image, best["roi"])],
                        "context": {"labels": {best["label"]: 1}},
                    },
                }
            )
            if "error" in answer:
                print(f"    describe: {answer['error']}")
            else:
                print(
                    f"    describe {best['label']!r} via {answer['backend']} ({answer['model']}, "
                    f"{answer['elapsed_ms']:.0f} ms): {answer['name']!r}, label_ok "
                    f"{answer['label_ok']}, confidence {answer['confidence']:.2f}, "
                    f"{answer['caption']!r}"
                )
    if torch.cuda.is_available():
        print(
            f"peak VRAM {torch.cuda.max_memory_allocated() / 2**30:.2f} GiB allocated, "
            f"{torch.cuda.max_memory_reserved() / 2**30:.2f} GiB reserved by torch"
        )
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", help="YAML file shaped like DEFAULTS")
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="SECTION.KEY=VALUE",
        help="override one config value, e.g. gemini.model=gemini-3.5-flash",
    )
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--device", help="auto, cuda or cpu")
    parser.add_argument("--detector", help=f"one of {', '.join(DETECTORS)}")
    parser.add_argument("--embedder", help=f"one of {', '.join(EMBEDDERS)}")
    parser.add_argument("--describer", help=f"comma separated, from {', '.join(DESCRIBERS)}")
    parser.add_argument(
        "--self-test",
        nargs="+",
        metavar="IMAGE",
        help="run segment over these images and exit, instead of serving",
    )
    parser.add_argument(
        "--describe-test",
        action="store_true",
        help="with --self-test, also describe each image's best instance "
        "(counts against the Gemini caps)",
    )
    parser.add_argument(
        "--phrases",
        default=str(VOCABULARY),
        help="for --self-test: comma separated, or a g1_detector parameter file",
    )
    args = parser.parse_args()

    config = load_config(args)
    if args.self_test and not args.describe_test:
        config["describer"] = ""
    try:
        server = build_server(config)
    except (ImportError, OSError) as error:
        sys.exit(f"{error}\n\n{VENV_HINT}")
    if args.self_test:
        return self_test(server, args)
    serve(server, config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
