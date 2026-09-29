"""The request handlers and the ZMQ loop."""

import sys
import time

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq


def segment_request(backend, payload, args):
    image = payload.get("image")
    phrases = payload.get("phrases") or []
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image must be an (H, W, 3) uint8 array")
    if image.dtype != np.uint8:
        raise ValueError(f"image must be uint8, got {image.dtype}")
    # Each phrase costs a model pass, so a repeat is dropped.
    phrases = list(dict.fromkeys(str(phrase) for phrase in phrases if str(phrase).strip()))
    if not phrases:
        raise ValueError("at least one phrase is required")

    started = time.perf_counter()
    instances = backend.segment(
        image,
        phrases,
        payload.get("box_threshold", args.box_threshold),
        payload.get("text_threshold", args.text_threshold),
    )
    return {
        "model": backend.name,
        "elapsed_ms": (time.perf_counter() - started) * 1000.0,
        "instances": instances,
    }


def ground_request(grounding, payload):
    if grounding is None:
        raise ValueError("this server was started without --vlm, so it cannot ground anything")
    image = payload.get("image")
    instruction = str(payload.get("instruction", "")).strip()
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image must be an (H, W, 3) uint8 array")
    if not instruction:
        return {"error": "ValueError: an instruction is required"}
    started = time.perf_counter()
    result = grounding.ground(image, instruction)
    result["model"] = grounding.name
    result["elapsed_ms"] = (time.perf_counter() - started) * 1000.0
    return result


def serve(backend, device, args, grounding=None):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://{args.host}:{args.port}")
    print(f"serving {backend.name} on {device} at tcp://{args.host}:{args.port}", flush=True)
    if grounding is not None:
        print(f"  grounding with {grounding.name}, loaded on the first request", flush=True)
    try:
        while True:
            request = msgpack.unpackb(socket.recv(), object_hook=mnp.decode, raw=False)
            endpoint = request.get("endpoint") if isinstance(request, dict) else None
            try:
                if endpoint == "ping":
                    reply = {"status": "ok", "backend": backend.name, "device": device}
                elif endpoint == "segment":
                    reply = segment_request(backend, request.get("data") or {}, args)
                elif endpoint == "ground":
                    reply = ground_request(grounding, request.get("data") or {})
                else:
                    raise ValueError(f"unknown endpoint {endpoint!r}")
            except Exception as error:  # noqa: BLE001 - the client gets every failure as a reply
                reply = {"error": f"{type(error).__name__}: {error}"}
                print(f"  {reply['error']}", file=sys.stderr, flush=True)
            socket.send(msgpack.packb(reply, default=mnp.encode))
    except KeyboardInterrupt:
        pass
    finally:
        socket.close(linger=0)
        context.term()
