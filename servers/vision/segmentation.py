"""The Grounded SAM 2 backend, and none for a server that only grounds instructions."""

import torch

from .masks import drop_cross_phrase_duplicates, roi_and_crop

DEFAULT_DETECTOR = "IDEA-Research/grounding-dino-base"
DEFAULT_SEGMENTER = "facebook/sam2.1-hiera-small"


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
        # A prompt per phrase: in a joint prompt the matched span can cross a phrase boundary and
        # name nothing. Sequential, because a batch of image copies does not fit in VRAM beside
        # the simulator.
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

        boxes, labels, scores = drop_cross_phrase_duplicates(boxes, labels, scores)

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
            cropped = roi_and_crop(mask)
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


class NoBackend:
    """For a server kept only to ground instructions: SAM 3.1 segments in canopy's server."""

    name = "none"

    def segment(self, image, phrases, box_threshold, text_threshold):
        raise ValueError(
            "this server segments nothing (--backend none); "
            "canopy's semantic server answers segment on 5561"
        )
