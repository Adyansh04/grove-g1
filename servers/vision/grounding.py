"""Instructions turned into noun phrases by a vision-language model, for the detector to take."""

import json

import torch

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
