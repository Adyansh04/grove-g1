#!/usr/bin/env python3
"""A grasp generator that answers GraspGenX's wire protocol with fixed poses.

Written from the protocol reference in NVlabs/GraspGenX's client-server README, and as strict
about the request as the real server: a depth frame in the wrong dtype, intrinsics that do not
match, a mask with no instances or sweep params that are not twelve numbers all come back as an
error, so a client that gets those wrong fails here rather than reading as a bad model.

Usage: graspgen_server_stub.py <port>
"""

import sys

import msgpack
import msgpack_numpy as mnp
import numpy as np

INSTANCE_ID = 1
# Two grasps, descending score, at distinguishable positions so a test can tell them apart.
GRASPS = [
    ([0.10, -0.20, 0.55], 0.91),
    ([0.11, -0.21, 0.56], 0.64),
]


def _complaint(data):
    """What is wrong with an infer_scene_depth request, or None."""
    depth = data.get("depth")
    intrinsics = data.get("intrinsics")
    mask = data.get("instance_mask")
    params = data.get("sweep_volume_params")
    if not isinstance(depth, np.ndarray) or depth.ndim != 2:
        return f"depth is {type(depth).__name__}, not an (H, W) array"
    if depth.dtype != np.float32:
        return f"depth is {depth.dtype}, not float32 metres"
    if not np.isfinite(depth).all():
        return "depth carries non-finite values; the server treats <= 0 as invalid instead"
    if not isinstance(intrinsics, np.ndarray) or intrinsics.shape != (3, 3):
        return "intrinsics is not a (3, 3) matrix"
    if not isinstance(mask, np.ndarray) or mask.shape != depth.shape:
        return "instance_mask does not match the depth frame's shape"
    if not (mask == INSTANCE_ID).any():
        return "instance_mask holds no instance 1"
    if not isinstance(params, dict):
        return "sweep_volume_params is not a map"
    for key in ("extents_open", "offset_open", "extents_mid", "offset_mid"):
        if len(params.get(key, [])) != 3:
            return f"sweep_volume_params[{key!r}] is not three numbers"
    return None


def _infer_scene_depth(data):
    complaint = _complaint(data)
    if complaint is not None:
        return {"error": f"ValueError: {complaint}"}
    poses = []
    scores = []
    for translation, score in GRASPS:
        matrix = np.eye(4, dtype=np.float32)
        # Reaching down: the approach axis is +z of the grasp frame, so it points at the floor.
        matrix[:3, :3] = np.diag([1.0, -1.0, -1.0])
        matrix[:3, 3] = translation
        poses.append(matrix)
        scores.append(score)
    return {
        "instance_ids": np.asarray([INSTANCE_ID], dtype=np.int32),
        "grasps": [np.stack(poses)],
        "confidences": [np.asarray(scores, dtype=np.float32)],
        "branch_tags": [["diff"] * len(poses)],
        "skipped_instance_ids": np.asarray([], dtype=np.int32),
        "timing": {"infer_ms": 1.0},
    }


def main():
    import zmq

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://127.0.0.1:{int(sys.argv[1])}")
    try:
        while True:
            request = msgpack.unpackb(socket.recv(), object_hook=mnp.decode, raw=False)
            action = request.get("action") if isinstance(request, dict) else None
            if action == "health":
                reply = {"status": "ok"}
            elif action == "metadata":
                reply = {"model": {"grasp_repr": "stub"}, "loaded_grippers": []}
            elif action == "infer_scene_depth":
                reply = _infer_scene_depth(request)
            else:
                reply = {"error": f"ValueError: unknown action {action!r}"}
            socket.send(msgpack.packb(reply, default=mnp.encode))
    except KeyboardInterrupt:
        # A traceback here reads like a test failure.
        pass
    finally:
        socket.close(linger=0)
        context.term()


if __name__ == "__main__":
    main()
