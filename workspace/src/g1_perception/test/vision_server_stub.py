#!/usr/bin/env python3
"""A vision server that answers the real wire protocol with a fixed rectangle per phrase.

Stands in for the models so the detector client's protocol, encoding and message building can be
tested without a GPU. It is as strict about the request as the real server, so a client that
sends a transposed or float image fails the test here rather than reading as a model problem.

Usage: vision_server_stub.py <port>
"""

import sys

import msgpack
import msgpack_numpy as mnp
import numpy as np

# Where each phrase's rectangle sits, as x, y, width, height. Distinct per phrase so a test can
# tell which answer it got.
BOXES = {
    "red cube": (100, 60, 40, 30),
    "blue sphere": (300, 200, 24, 24),
}
SCORE = 0.77


def _complaint(data):
    """What is wrong with a segment request, or None. The real server is this strict."""
    image = data.get("image")
    if not isinstance(image, np.ndarray):
        return f"image is {type(image).__name__}, not an array"
    if image.dtype != np.uint8:
        return f"image is {image.dtype}, not uint8"
    if image.ndim != 3 or image.shape[2] != 3:
        return f"image is {image.shape}, not (H, W, 3)"
    phrases = data.get("phrases")
    if not isinstance(phrases, list) or not phrases:
        return "phrases is empty or not a list"
    if any(not isinstance(phrase, str) for phrase in phrases):
        return "a phrase is not a string"
    return None


def _segment(data):
    complaint = _complaint(data)
    if complaint is not None:
        return {"error": complaint}
    instances = []
    for phrase in data["phrases"]:
        box = BOXES.get(phrase)
        if box is None:
            continue
        x, y, width, height = box
        instances.append(
            {
                "label": phrase,
                "score": SCORE,
                "roi": [x, y, width, height],
                "mask": np.full((height, width), 255, dtype=np.uint8),
            }
        )
    return {"model": "stub", "elapsed_ms": 1.0, "instances": instances}


def main():
    import zmq

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://127.0.0.1:{int(sys.argv[1])}")
    try:
        while True:
            request = msgpack.unpackb(socket.recv(), object_hook=mnp.decode, raw=False)
            endpoint = request.get("endpoint") if isinstance(request, dict) else None
            if endpoint == "ping":
                reply = {"status": "ok", "backend": "stub", "device": "cpu"}
            elif endpoint == "segment":
                reply = _segment(request.get("data") or {})
            else:
                reply = {"error": f"unknown endpoint {endpoint!r}"}
            socket.send(msgpack.packb(reply, default=mnp.encode))
    except KeyboardInterrupt:
        # A traceback here reads like a test failure.
        pass
    finally:
        socket.close(linger=0)
        context.term()


if __name__ == "__main__":
    main()
