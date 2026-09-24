#!/usr/bin/env python3
"""A policy server that answers the real wire protocol with fixed numbers.

Tests the adapter without a GPU. It encodes the way the reference servers do, ModalityConfig
marker included, so the adapter's decode path really runs.

Usage: policy_server_stub.py <port> <horizon> <delta>
"""

import json
import sys

import msgpack
import msgpack_numpy as mnp
import numpy as np

# The key set and widths of the checkpoint's own G1 embodiment, not a convenient subset.
STATE_DIMS = {
    "left_wrist_eef_9d": 9,
    "right_wrist_eef_9d": 9,
    "left_hand": 7,
    "right_hand": 7,
    "left_arm": 7,
    "right_arm": 7,
    "waist": 3,
}
# Action keys that have nothing to do with an arm, kept so the adapter has to ignore them.
ACTION_ONLY_DIMS = {"base_height_command": 1, "navigate_command": 3}
VIDEO_KEY = "ego_view"
ANNOTATION_KEY = "annotation.human.task_description"
VIDEO_DELTAS = [-20, 0]


def _modality_config(horizon):
    """Each entry wrapped the way the server wraps a ModalityConfig."""

    def wrap(keys, deltas):
        return {
            "__ModalityConfig__": True,
            "as_json": json.dumps({"delta_indices": deltas, "modality_keys": keys}),
        }

    return {
        "state": wrap(list(STATE_DIMS), [0]),
        "action": wrap(list(STATE_DIMS) + list(ACTION_ONLY_DIMS), list(range(horizon))),
        "video": wrap([VIDEO_KEY], VIDEO_DELTAS),
        "language": wrap([ANNOTATION_KEY], [0]),
    }


def _complaint(data):
    """What is wrong with an observation, or None. The real server is this strict."""
    for modality in ("state", "video", "language"):
        if modality not in data:
            return f"no '{modality}' in the observation"

    for key, dim in STATE_DIMS.items():
        if key not in data["state"]:
            return f"no state '{key}'"
        shape = np.asarray(data["state"][key]).shape
        if shape != (1, 1, dim):
            return f"state '{key}' is {shape}, wanted (1, 1, {dim})"

    if VIDEO_KEY not in data["video"]:
        return f"no video '{VIDEO_KEY}'"
    frames = np.asarray(data["video"][VIDEO_KEY])
    if frames.ndim != 5 or frames.shape[:2] != (1, len(VIDEO_DELTAS)):
        return f"video '{VIDEO_KEY}' is {frames.shape}, wanted (1, {len(VIDEO_DELTAS)}, H, W, 3)"
    if frames.dtype != np.uint8:
        return f"video '{VIDEO_KEY}' is {frames.dtype}, wanted uint8"

    text = data["language"].get(ANNOTATION_KEY)
    if not isinstance(text, list) or not text or not isinstance(text[0], list):
        return f"language '{ANNOTATION_KEY}' is {text!r}, wanted [[str]]"
    return None


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
            reply = _modality_config(horizon)
        elif endpoint == "reset":
            reply = {"status": "ok"}
        elif endpoint == "get_action":
            # The real server passes data to its handler as keyword arguments, so the observation
            # arrives under "observation".
            data = request.get("data", {}).get("observation", {})
            complaint = _complaint(data)
            if complaint:
                reply = {"error": complaint}
            else:
                # Absolute answers one constant offset from the state sent, checkable by hand. The
                # real server likewise converts its relative predictions before replying.
                actions = {}
                for key, dim in STATE_DIMS.items():
                    base = np.asarray(data["state"][key], dtype=np.float64)[0, -1]
                    actions[key] = np.broadcast_to(base + delta, (1, horizon, dim)).copy()
                for key, dim in ACTION_ONLY_DIMS.items():
                    actions[key] = np.zeros((1, horizon, dim), dtype=np.float64)
                # The policy answers (action, info); msgpack delivers that as a list.
                reply = [actions, {}]
        else:
            reply = {"error": f"unknown endpoint {endpoint}"}
        socket.send(msgpack.packb(reply, default=mnp.encode))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # SIGINT lands inside the blocking recv; a traceback here reads like a test failure.
        pass
