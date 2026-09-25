"""Unit tests for scripts/semantic_server.py: no GPU, no network, no model weights.

~/ref/grove-semantic/.venv/bin/python -m unittest scripts/tests/test_semantic_server.py
"""

import argparse
import base64
import io
import json
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import semantic_server as ss  # noqa: E402


def utc(*fields):
    return datetime(*fields, tzinfo=timezone.utc).timestamp()


class Clock:
    def __init__(self, now):
        self.now = now

    def __call__(self):
        return self.now


class Scratch(unittest.TestCase):
    """A temporary directory and a clock at noon Pacific (PDT) on 2026-09-24."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.dir = Path(directory.name)
        self.usage = self.dir / "gemini_usage.json"
        self.clock = Clock(utc(2026, 9, 24, 19, 0))

    def limiter(self, **caps):
        return ss.GeminiLimiter(self.usage, clock=self.clock, **caps)


class LimiterTest(Scratch):
    def test_minute_window(self):
        limiter = self.limiter()
        for _ in range(5):
            limiter.acquire()
        with self.assertRaisesRegex(ss.Unavailable, "a minute"):
            limiter.acquire()
        self.clock.now += 59
        with self.assertRaises(ss.Unavailable):
            limiter.acquire()
        self.clock.now += 2
        limiter.acquire()

    def test_daily_cap(self):
        limiter = self.limiter()
        for _ in range(100):
            limiter.acquire()
            self.clock.now += 13
        with self.assertRaisesRegex(ss.Unavailable, "a day"):
            limiter.acquire()
        self.assertEqual(limiter.usage()["count"], 100)

    def test_day_turns_over_at_midnight_pacific(self):
        self.clock.now = utc(2026, 9, 24, 23, 50)
        limiter = self.limiter(per_day=2)
        limiter.acquire()
        limiter.acquire()
        self.clock.now = utc(2026, 9, 25, 0, 30)  # past midnight UTC, 17:30 in California
        with self.assertRaisesRegex(ss.Unavailable, "a day"):
            limiter.acquire()
        self.clock.now = utc(2026, 9, 25, 6, 59)  # 23:59 PDT
        with self.assertRaises(ss.Unavailable):
            limiter.acquire()
        self.clock.now = utc(2026, 9, 25, 7, 1)  # 00:01 PDT
        limiter.acquire()
        self.assertEqual(limiter.usage()["day"], "2026-09-25")

    def test_day_follows_standard_time_in_winter(self):
        self.clock.now = utc(2026, 12, 2, 7, 59)  # 23:59 PST
        self.assertEqual(self.limiter().usage()["day"], "2026-12-01")
        self.clock.now = utc(2026, 12, 2, 8, 1)
        self.assertEqual(self.limiter().usage()["day"], "2026-12-02")

    def test_exhausted_until_the_next_pacific_day(self):
        limiter = self.limiter()
        limiter.mark_exhausted()
        with self.assertRaisesRegex(ss.Unavailable, "exhausted"):
            limiter.acquire()
        self.clock.now = utc(2026, 9, 25, 7, 1)
        limiter.acquire()

    def test_ceilings_cannot_be_raised(self):
        limiter = self.limiter(per_minute=50, per_day=1000)
        self.assertEqual((limiter.per_minute, limiter.per_day), (5, 100))
        for _ in range(5):
            limiter.acquire()
        with self.assertRaises(ss.Unavailable):
            limiter.acquire()

    def test_ceilings_can_be_lowered(self):
        limiter = self.limiter(per_minute=2)
        limiter.acquire()
        limiter.acquire()
        with self.assertRaises(ss.Unavailable):
            limiter.acquire()

    def test_state_persists_across_instances(self):
        first, second = self.limiter(), self.limiter()
        for _ in range(3):
            first.acquire()
        second.acquire()
        second.acquire()
        with self.assertRaises(ss.Unavailable):
            first.acquire()
        second.mark_exhausted()
        self.clock.now += 120
        with self.assertRaisesRegex(ss.Unavailable, "exhausted"):
            self.limiter().acquire()
        self.assertEqual(self.limiter().usage()["count"], 5)

    def test_unreadable_state_fails_closed(self):
        self.usage.write_text("{not json")
        with self.assertRaisesRegex(ss.Unavailable, "unreadable"):
            self.limiter().acquire()


def gemini_reply(answer):
    return json.dumps(
        {
            "modelVersion": "gemini-3.5-flash-lite",
            "candidates": [{"content": {"parts": [{"text": json.dumps(answer)}]}}],
        }
    ).encode()


MUG = {"name": "Mug", "caption": "A white ceramic mug.", "label_ok": False, "confidence": 0.9}
QUOTA = b'{"error": {"code": 429, "status": "RESOURCE_EXHAUSTED", "message": "Quota exceeded"}}'
OBJECT = {"labels": {"cup": 3, "mug": 1}}


class Transport:
    """Plays back (status, body) replies, or raises an exception given in their place."""

    def __init__(self, *replies):
        self.replies = list(replies)
        self.calls = []

    def __call__(self, url, headers, body, timeout):
        self.calls.append((url, headers, json.loads(body)))
        reply = self.replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply


class GeminiTest(Scratch):
    def setUp(self):
        super().setUp()
        self.key_file = self.dir / "gemini.env"
        self.key_file.write_text("GEMINI_API_KEY=test-key-123\n")

    def describer(self, transport, limiter=None):
        return ss.GeminiDescriber(
            "gemini-3.5-flash-lite", self.key_file, limiter or self.limiter(), transport=transport
        )

    def describe(self, describer):
        return describer.describe("object", [np.zeros((900, 600, 3), np.uint8)], OBJECT)

    def test_request(self):
        transport = Transport((200, gemini_reply(MUG)))
        answer = self.describe(self.describer(transport))
        self.assertEqual((answer["name"], answer["label_ok"]), ("mug", False))
        self.assertEqual(answer["model"], "gemini-3.5-flash-lite")

        url, headers, body = transport.calls[0]
        self.assertNotIn("test-key-123", url)
        self.assertEqual(headers["x-goog-api-key"], "test-key-123")
        config = body["generationConfig"]
        self.assertEqual(config["responseMimeType"], "application/json")
        self.assertEqual(config["mediaResolution"], "MEDIA_RESOLUTION_LOW")
        self.assertEqual(config["responseSchema"]["type"], "OBJECT")
        self.assertEqual(config["responseSchema"]["properties"]["label_ok"]["type"], "BOOLEAN")
        parts = body["contents"][0]["parts"]
        jpeg = base64.b64decode(parts[0]["inlineData"]["data"])
        self.assertEqual(Image.open(io.BytesIO(jpeg)).size, (341, 512))
        self.assertIn('called it "cup"', parts[-1]["text"])

    def test_bare_key(self):
        self.key_file.write_text("bare-key-456")
        transport = Transport((200, gemini_reply(MUG)))
        self.describe(self.describer(transport))
        self.assertEqual(transport.calls[0][1]["x-goog-api-key"], "bare-key-456")

    def test_quota_refusal_stops_calls_until_the_next_day(self):
        transport = Transport((429, QUOTA), (200, gemini_reply(MUG)))
        describer = self.describer(transport)
        with self.assertRaisesRegex(ss.Unavailable, "quota"):
            self.describe(describer)
        with self.assertRaisesRegex(ss.Unavailable, "exhausted"):
            self.describe(describer)
        self.assertEqual(len(transport.calls), 1)
        self.assertTrue(self.limiter().usage()["exhausted"])
        self.clock.now = utc(2026, 9, 25, 7, 1)
        self.assertEqual(self.describe(describer)["name"], "mug")

    def test_quota_error_under_another_status(self):
        body = b'{"error": {"code": 403, "status": "PERMISSION_DENIED", "message": "quota for this key"}}'
        with self.assertRaises(ss.Unavailable):
            self.describe(self.describer(Transport((403, body))))
        self.assertTrue(self.limiter().usage()["exhausted"])

    def test_server_error_does_not_exhaust(self):
        with self.assertRaisesRegex(ss.Unavailable, "HTTP 503"):
            self.describe(self.describer(Transport((503, b'{"error": {"message": "busy"}}'))))
        self.assertFalse(self.limiter().usage()["exhausted"])
        self.assertEqual(self.limiter().usage()["count"], 1)

    def test_limiter_refuses_before_any_network_io(self):
        transport = Transport((200, gemini_reply(MUG)))
        describer = self.describer(transport, self.limiter(per_minute=1))
        self.describe(describer)
        with self.assertRaisesRegex(ss.Unavailable, "a minute"):
            self.describe(describer)
        self.assertEqual(len(transport.calls), 1)

    def test_missing_key(self):
        self.key_file.unlink()
        transport = Transport()
        with self.assertRaisesRegex(ss.Unavailable, "no Gemini key"):
            self.describe(self.describer(transport))
        self.assertEqual(transport.calls, [])


class FakeDescriber:
    def __init__(self, name, answer=None, error=None):
        self.name = name
        self.answer = answer
        self.error = error
        self.calls = 0

    def describe(self, task, images, context):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return dict(self.answer)


ANSWER = {
    "model": "m",
    "name": "mug",
    "caption": "",
    "label_ok": True,
    "room_type": "",
    "confidence": 0.8,
}


class ChainTest(Scratch):
    def test_falls_through_an_unavailable_describer(self):
        chain = ss.DescriberChain(
            [FakeDescriber("a", error=ss.Unavailable("rate limited")), FakeDescriber("b", ANSWER)]
        )
        self.assertEqual(chain.describe("object", [], OBJECT)["backend"], "b")

    def test_first_answer_wins(self):
        second = FakeDescriber("b", ANSWER)
        chain = ss.DescriberChain([FakeDescriber("a", ANSWER), second])
        self.assertEqual(chain.describe("object", [], OBJECT)["backend"], "a")
        self.assertEqual(second.calls, 0)

    def test_all_failing_names_every_reason(self):
        chain = ss.DescriberChain(
            [
                FakeDescriber("a", error=ss.Unavailable("rate limited")),
                FakeDescriber("b", error=ValueError("no JSON in the answer")),
            ]
        )
        with self.assertRaisesRegex(RuntimeError, "a: rate limited; b: no JSON"):
            chain.describe("object", [], OBJECT)

    def test_gemini_quota_falls_back_to_the_local_vlm(self):
        key_file = self.dir / "gemini.env"
        key_file.write_text("key")
        local = Transport(
            (
                200,
                json.dumps(
                    {"model": "qwen3.5-4b", "choices": [{"message": {"content": json.dumps(MUG)}}]}
                ).encode(),
            )
        )
        chain = ss.DescriberChain(
            [
                ss.GeminiDescriber(
                    "g", key_file, self.limiter(), transport=Transport((429, QUOTA))
                ),
                ss.OpenAIDescriber("http://127.0.0.1:8080/v1", "qwen3.5-4b", transport=local),
            ]
        )
        answer = chain.describe("object", [np.zeros((64, 64, 3), np.uint8)], OBJECT)
        self.assertEqual(
            (answer["backend"], answer["model"], answer["name"]), ("openai", "qwen3.5-4b", "mug")
        )
        url, _, body = local.calls[0]
        self.assertEqual(url, "http://127.0.0.1:8080/v1/chat/completions")
        content = body["messages"][0]["content"]
        self.assertTrue(content[0]["image_url"]["url"].startswith("data:image/jpeg;base64,"))
        self.assertEqual(body["response_format"]["type"], "json_schema")

    def test_unreachable_local_vlm_is_unavailable(self):
        describer = ss.OpenAIDescriber(
            "http://127.0.0.1:8080/v1", "m", transport=Transport(ss.Unavailable("refused"))
        )
        with self.assertRaises(ss.Unavailable):
            describer.describe("room", [], {"objects": ["bed"]})

    def test_none_echoes_the_detector(self):
        answer = ss.NoDescriber().describe("object", [], OBJECT)
        self.assertEqual((answer["name"], answer["confidence"]), ("cup", 0.0))
        room = ss.NoDescriber().describe("room", [], {"objects": [], "room_type": "Kitchen"})
        self.assertEqual(room["room_type"], "kitchen")


class ParseAnswerTest(unittest.TestCase):
    def test_json_inside_prose_and_fences(self):
        text = 'Sure!\n```json\n{"name": "Mug", "caption": " A  mug. ", "label_ok": true}\n```'
        answer = ss.parse_answer(text, "object")
        self.assertEqual(
            (answer["name"], answer["caption"], answer["label_ok"]), ("mug", "A mug.", True)
        )

    def test_loose_types(self):
        def parse(**fields):
            return ss.parse_answer(json.dumps({"name": "mug", **fields}), "object")

        self.assertTrue(parse(label_ok="yes")["label_ok"])
        self.assertFalse(parse(label_ok="no")["label_ok"])
        self.assertFalse(parse()["label_ok"])
        self.assertAlmostEqual(parse(confidence="85%")["confidence"], 0.85)
        self.assertAlmostEqual(parse(confidence=85)["confidence"], 0.85)
        self.assertEqual(parse(confidence="high")["confidence"], 0.5)
        self.assertEqual(parse(confidence=-1)["confidence"], 0.0)
        self.assertEqual(
            ss.parse_answer('{"Name": "mug", "Confidence": NaN}', "object")["confidence"], 0.5
        )

    def test_name_is_a_short_noun(self):
        answer = ss.parse_answer('{"name": "A large red office chair."}', "object")
        self.assertEqual(answer["name"], "red office chair")

    def test_room_type_is_one_of_the_list(self):
        def room(value):
            return ss.parse_answer(json.dumps({"room_type": value}), "room")["room_type"]

        self.assertEqual(room("Living_Room"), "living room")
        self.assertEqual(room("master bedroom"), "bedroom")
        self.assertEqual(room("Lounge"), "living room")
        self.assertEqual(room("walk-in closet"), "storage room")
        self.assertEqual(room("spaceship"), "other")
        answer = ss.parse_answer('{"room_type": "kitchen"}', "room")
        self.assertEqual((answer["name"], answer["room_type"]), ("kitchen", "kitchen"))

    def test_unusable_answers(self):
        for text in ("no json here", "[1, 2]", '{"name": ""}', '{"caption": "a thing"}'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                ss.parse_answer(text, "object")


class FakeTensor:
    def __init__(self, array):
        self.array = np.asarray(array)

    def bool(self):
        return FakeTensor(self.array.astype(bool))

    def cpu(self):
        return self

    def numpy(self):
        return self.array

    def tolist(self):
        return self.array.tolist()


class FakeYoloe:
    """Records set_classes and predict; answers with fixed masks labelled by class index."""

    def __init__(self, masks=None, scores=(), classes=(), names=None):
        self.names = names or {}
        self.vocabularies = []
        self.predictions = []
        self._masks, self._scores, self._classes = masks, scores, classes

    def set_classes(self, classes):
        self.vocabularies.append(list(classes))
        self.names = dict(enumerate(classes))

    def predict(self, image, **kwargs):
        self.predictions.append((image, kwargs))
        masks = None if self._masks is None else SimpleNamespace(data=FakeTensor(self._masks))
        boxes = SimpleNamespace(conf=FakeTensor(self._scores), cls=FakeTensor(self._classes))
        return [SimpleNamespace(masks=masks, boxes=boxes, names=self.names)]


class YoloeTest(Scratch):
    def detector(self, model, prompt_free=False):
        return ss.YoloeDetector(model, "cpu", self.dir, prompt_free=prompt_free)

    def test_vocabulary_is_set_only_when_it_changes(self):
        model = FakeYoloe()
        detector = self.detector(model)
        image = np.zeros((4, 6, 3), np.uint8)
        detector.segment(image, ["chair", "table"], 0.3, 0.25)
        detector.segment(image, ["chair", "table"], 0.3, 0.25)
        self.assertEqual(model.vocabularies, [["chair", "table"]])
        detector.segment(image, ["table", "chair"], 0.3, 0.25)
        self.assertEqual(len(model.vocabularies), 2)
        self.assertEqual(len(model.predictions), 3)  # one pass per request, whatever the list

    def test_prompt_free_ignores_phrases(self):
        model = FakeYoloe(names={0: "armchair"})
        self.detector(model, prompt_free=True).segment(np.zeros((4, 6, 3), np.uint8), [], 0.3, 0.2)
        self.assertEqual(model.vocabularies, [])

    def test_text_prompted_needs_phrases(self):
        with self.assertRaisesRegex(ValueError, "yoloe-pf"):
            self.detector(FakeYoloe()).segment(np.zeros((4, 6, 3), np.uint8), [], 0.3, 0.25)

    def test_instances(self):
        masks = np.zeros((2, 4, 6), bool)
        masks[0, 1:3, 2:5] = True
        model = FakeYoloe(masks, scores=[0.9, 0.5], classes=[1, 0])
        image = np.zeros((4, 6, 3), np.uint8)
        image[..., 0] = 255  # red in RGB
        instances = self.detector(model).segment(image, ["chair", "mug"], 0.3, 0.25)
        self.assertEqual(len(instances), 1)  # the empty mask is dropped
        self.assertEqual((instances[0]["label"], instances[0]["roi"]), ("mug", [2, 1, 3, 2]))
        self.assertEqual(instances[0]["mask"].tolist(), [[255] * 3] * 2)
        sent, kwargs = model.predictions[0]
        self.assertEqual(sent[0, 0].tolist(), [0, 0, 255])  # BGR for ultralytics
        self.assertTrue(kwargs["retina_masks"] and kwargs["agnostic_nms"])
        self.assertEqual(kwargs["conf"], 0.3)

    def test_one_instance_per_region(self):
        masks = np.zeros((3, 20, 20), bool)
        masks[0, 0:10, 0:10] = True
        masks[1, 0:10, 0:9] = True  # the same object from a neighbouring anchor
        masks[2, 4:8, 4:8] = True  # something on top of it
        model = FakeYoloe(masks, scores=[0.4, 0.8, 0.5], classes=[0, 1, 1])
        instances = self.detector(model).segment(
            np.zeros((20, 20, 3), np.uint8), ["table", "desk"], 0.3, 0.25
        )
        self.assertEqual(
            [(i["label"], i["score"]) for i in instances], [("desk", 0.8), ("desk", 0.5)]
        )


class FakeDetector:
    name = "fake"

    def __init__(self, instances):
        self.instances = instances
        self.phrases = None

    def segment(self, image, phrases, box_threshold, text_threshold):
        self.phrases = phrases
        return [dict(instance) for instance in self.instances]


class FakeEmbedder:
    name = "fake-embedder"

    def __init__(self):
        self.images = None

    def embed_images(self, images):
        self.images = images
        return np.eye(len(images), 4, dtype=np.float32)

    def embed_text(self, texts):
        return np.ones((len(texts), 4), np.float32) / 2.0


class ServerTest(unittest.TestCase):
    def setUp(self):
        mask = np.array([[255, 0], [255, 255]], np.uint8)
        self.detector = FakeDetector(
            [{"label": "mug", "score": 0.8, "roi": [1, 1, 2, 2], "mask": mask}]
        )
        self.embedder = FakeEmbedder()
        chain = ss.DescriberChain([ss.NoDescriber()])
        self.server = ss.SemanticServer(self.detector, self.embedder, chain)
        self.image = np.full((4, 4, 3), 200, np.uint8)

    def test_segment_with_embeddings(self):
        reply = self.server.handle(
            {
                "endpoint": "segment",
                "data": {
                    "image": self.image,
                    "phrases": ["mug", " mug ", "", "bowl"],
                    "embed": True,
                },
            }
        )
        self.assertEqual(self.detector.phrases, ["mug", "bowl"])
        self.assertEqual(reply["instances"][0]["embedding"].tolist(), [1.0, 0.0, 0.0, 0.0])
        crop = self.embedder.images[0]
        self.assertEqual(crop.shape, (2, 2, 3))
        self.assertEqual(crop[0, 1].tolist(), [ss.GREY] * 3)  # outside the mask
        self.assertEqual(crop[1, 1].tolist(), [200] * 3)

    def test_segment_without_embed_matches_the_vision_server(self):
        reply = self.server.handle(
            {"endpoint": "segment", "data": {"image": self.image, "phrases": ["mug"]}}
        )
        self.assertEqual(set(reply), {"model", "elapsed_ms", "instances"})
        self.assertEqual(set(reply["instances"][0]), {"label", "score", "roi", "mask"})

    def test_masked_crop_is_a_padded_grey_square(self):
        image = np.full((20, 20, 3), 50, np.uint8)
        crop = ss.masked_crop(image, [0, 0, 10, 5], np.full((5, 10), 255, np.uint8))
        self.assertEqual(crop.shape, (12, 12, 3))
        self.assertEqual(crop[0, 0].tolist(), [ss.GREY] * 3)
        self.assertEqual(crop[6, 6].tolist(), [50] * 3)

    def test_embed_text(self):
        reply = self.server.handle({"endpoint": "embed_text", "data": {"texts": ["a mug"]}})
        self.assertEqual((reply["model"], reply["embeddings"].shape), ("fake-embedder", (1, 4)))

    def test_describe(self):
        reply = self.server.handle(
            {
                "endpoint": "describe",
                "data": {"task": "object", "images": [self.image], "context": OBJECT},
            }
        )
        self.assertEqual((reply["backend"], reply["name"]), ("none", "cup"))
        self.assertIn("elapsed_ms", reply)

    def test_failures_are_replies(self):
        for request in (
            {"endpoint": "nope"},
            "not a map",
            {"endpoint": "segment", "data": {"image": np.zeros((4, 4), np.uint8)}},
            {"endpoint": "describe", "data": {"task": "object", "images": []}},
            {"endpoint": "describe", "data": {"task": "garage"}},
            {"endpoint": "embed_text", "data": {"texts": [""]}},
        ):
            with self.subTest(request=request):
                self.assertIn("error", self.server.handle(request))
        self.server.embedder = None
        self.assertIn(
            "error",
            self.server.handle(
                {
                    "endpoint": "segment",
                    "data": {"image": self.image, "phrases": ["mug"], "embed": True},
                }
            ),
        )


class ConfigTest(Scratch):
    def test_flags_and_overrides(self):
        args = argparse.Namespace(
            config=None,
            host=None,
            port=None,
            device=None,
            detector="yoloe-pf",
            embedder=None,
            describer="openai",
            set=[
                "gemini.per_day=500",
                "yoloe.imgsz=800",
                f"gemini.key_file={self.dir}/none",
                f"gemini.usage_file={self.usage}",
            ],
        )
        config = ss.load_config(args)
        self.assertEqual((config["detector"], config["describer"]), ("yoloe-pf", "openai"))
        self.assertEqual((config["yoloe"]["imgsz"], config["yoloe"]["half"]), (800, True))
        self.assertEqual(ss.DESCRIBERS["gemini"](config)._limiter.per_day, 100)


if __name__ == "__main__":
    unittest.main()
