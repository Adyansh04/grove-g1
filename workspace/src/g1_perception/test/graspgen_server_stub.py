#!/usr/bin/env python3
"""Answers GraspGenX's wire protocol with fixed grasps.

As strict about requests as the real server (per NVlabs/GraspGenX's client-server README), so a
malformed request fails here rather than reading as a bad model.

Usage: graspgen_server_stub.py <port>
"""

import sys

import msgpack
import msgpack_numpy as mnp
import numpy as np

INSTANCE_ID = 1
# Two grasps, descending score, placed off the masked points' centroid so a test can tell which
# scene was sent. The first is the camera's frame turned half a turn about its x; the second is
# tilted 30 degrees about the camera's y, so a reflection of it shows.
TILT = np.radians(30.0)
GRASPS = [
    ([0.0, 0.0, 0.0], np.diag([1.0, -1.0, -1.0]), 0.91),
    (
        [0.01, -0.01, 0.01],
        np.array(
            [
                [np.cos(TILT), 0.0, np.sin(TILT)],
                [0.0, -1.0, 0.0],
                [np.sin(TILT), 0.0, -np.cos(TILT)],
            ]
        ),
        0.64,
    ),
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


def _centroid(depth, intrinsics, mask):
    """The masked pixels back-projected as GraspGenX does, x = (u - cx) z / fx, and averaged."""
    rows, cols = np.nonzero(mask == INSTANCE_ID)
    z = depth[rows, cols].astype(np.float64)
    x = (cols - intrinsics[0, 2]) * z / intrinsics[0, 0]
    y = (rows - intrinsics[1, 2]) * z / intrinsics[1, 1]
    return np.array([x.mean(), y.mean(), z.mean()])


def _infer_scene_depth(data):
    complaint = _complaint(data)
    if complaint is not None:
        return {"error": f"ValueError: {complaint}"}
    poses = []
    scores = []
    centroid = _centroid(data["depth"], data["intrinsics"], data["instance_mask"])
    for offset, rotation, score in GRASPS:
        matrix = np.eye(4, dtype=np.float32)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = centroid + np.asarray(offset)
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
