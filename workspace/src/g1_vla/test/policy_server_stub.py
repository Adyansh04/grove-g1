#!/usr/bin/env python3
"""A policy server that answers the real wire protocol with fixed numbers.

Stands in for the model so the adapter's protocol, key mapping and action integration can be
tested without a GPU. It encodes the way the reference servers do, including the ModalityConfig
marker, so the adapter's decode path is exercised rather than bypassed.

Usage: policy_server_stub.py <port> <horizon> <delta>
"""

import json
import sys

import msgpack
import msgpack_numpy as mnp
import numpy as np

STATE_KEY = "state.test_arm"
ACTION_KEY = "action.test_arm"
VIDEO_KEY = "video.test_cam"
ANNOTATION_KEY = "annotation.human.action.task_description"


def _modality_config():
    """Each entry wrapped the way the server wraps a ModalityConfig."""

    def wrap(keys):
        return {
            "__ModalityConfig__": True,
            "as_json": json.dumps({"delta_indices": [0], "modality_keys": keys}),
        }

    return {
        "state": wrap([STATE_KEY]),
        "action": wrap([ACTION_KEY]),
        "video": wrap([VIDEO_KEY]),
        "annotation": wrap([ANNOTATION_KEY]),
    }


def main():
    import zmq

    port, horizon, delta = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
    socket = zmq.Context().socket(zmq.REP)
    socket.bind(f"tcp://127.0.0.1:{port}")
    print(f"stub policy server on {port}", flush=True)

    while True:
        request = msgpack.unpackb(socket.recv(), object_hook=mnp.decode, raw=False)
        endpoint = request.get("endpoint")
        if endpoint == "ping":
            reply = {"status": "ok"}
        elif endpoint == "get_modality_config":
            reply = _modality_config()
        elif endpoint == "reset":
            reply = {"status": "ok"}
        elif endpoint == "get_action":
            data = request.get("data", {})
            width = np.asarray(data[STATE_KEY]).shape[-1]
            # A constant offset per step, so the adapter's integration is checkable by hand.
            reply = {ACTION_KEY: np.full((horizon, width), delta, dtype=np.float64)}
        else:
            reply = {"error": f"unknown endpoint {endpoint}"}
        socket.send(msgpack.packb(reply, default=mnp.encode))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # SIGINT lands inside the blocking recv; a traceback here reads like a test failure.
        pass
