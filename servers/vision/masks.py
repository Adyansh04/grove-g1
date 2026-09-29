"""Masks as they go on the wire, and one region per phrase."""

import numpy as np


def roi_and_crop(mask):
    """Full-frame boolean mask to (roi, cropped uint8 mask), or None when it is empty.

    The crop is what goes on the wire: a few kB per object instead of a full frame.
    """
    rows = np.flatnonzero(mask.any(axis=1))
    cols = np.flatnonzero(mask.any(axis=0))
    if rows.size == 0 or cols.size == 0:
        return None
    y0, y1 = int(rows[0]), int(rows[-1]) + 1
    x0, x1 = int(cols[0]), int(cols[-1]) + 1
    crop = np.ascontiguousarray(mask[y0:y1, x0:x1].astype(np.uint8) * 255)
    return [x0, y0, x1 - x0, y1 - y0], crop


def iou(a, b):
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


def drop_cross_phrase_duplicates(boxes, labels, scores, iou_threshold=0.5):
    """Keeps one phrase per region: a patch of image is one object, whatever it is called.

    Phrases are prompted separately, so two can claim the same pixels, and the runner-up would
    become a second track. The higher score keeps the region; scores from different prompts are
    not strictly comparable, but the right phrase wins by a wide margin.
    """
    order = sorted(range(len(boxes)), key=lambda i: scores[i], reverse=True)
    kept = []
    for i in order:
        if any(labels[i] != labels[j] and iou(boxes[i], boxes[j]) > iou_threshold for j in kept):
            continue
        kept.append(i)
    kept.sort()
    return ([boxes[i] for i in kept], [labels[i] for i in kept], [scores[i] for i in kept])
