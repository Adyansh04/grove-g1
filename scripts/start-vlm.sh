#!/usr/bin/env bash
#
# Runs the local VLM behind scripts/semantic_server.py's `openai` describer: Qwen3.5-4B (Q4_K_M)
# and its vision projector in llama.cpp's server, in a container.
#
#   ./scripts/start-vlm.sh start     # returns once the model answers
#   ./scripts/start-vlm.sh status
#   ./scripts/start-vlm.sh stop
#
# OpenAI-compatible API at http://127.0.0.1:8080/v1, bound to localhost only. Run
# scripts/setup-semantic.sh first; it fetches the GGUF files and this image.
set -euo pipefail

NAME=grove-vlm
# Pinned, because llama.cpp's flags change between builds: build 11151 (bd4f514db).
IMAGE="${VLM_IMAGE:-ghcr.io/ggml-org/llama.cpp@sha256:014f721265464f38ccb247c1338d07d852c4bae7509a4b4734d07a2bbadc765c}"
PORT="${VLM_PORT:-8080}"
HF_CACHE="${HF_HOME:-${HOME}/.cache/huggingface}"
MODEL=Qwen3.5-4B-Q4_K_M.gguf
MMPROJ=mmproj-F16.gguf

running() { [ "$(docker inspect -f '{{.State.Running}}' "${NAME}" 2>/dev/null)" = true ]; }
healthy() { curl -sf "http://127.0.0.1:${PORT}/health" >/dev/null; }

start() {
    if running; then
        echo "${NAME} is already running"
        return
    fi
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    model=$(compgen -G "${HF_CACHE}/hub/models--unsloth--Qwen3.5-4B-GGUF/snapshots/*/${MODEL}" \
        | head -n 1 || true)
    if [ -z "${model}" ] || [ ! -e "$(dirname "${model}")/${MMPROJ}" ]; then
        echo "${MODEL} or ${MMPROJ} is missing from ${HF_CACHE}; run scripts/setup-semantic.sh" >&2
        exit 1
    fi
    snapshot="/hf/$(dirname "${model#"${HF_CACHE}"/}")"

    # One slot, so an image request gets the whole 4096-token context; no prompt cache in RAM,
    # since no two describe requests share a prefix worth keeping. Qwen3.5 thinks by default and
    # a caption does not need it.
    docker run -d --name "${NAME}" --gpus all --memory 8g --cpus 6 \
        -p "127.0.0.1:${PORT}:8080" -v "${HF_CACHE}:/hf:ro" "${IMAGE}" \
        --model "${snapshot}/${MODEL}" --mmproj "${snapshot}/${MMPROJ}" --alias qwen3.5-4b \
        --ctx-size 4096 --n-gpu-layers all --parallel 1 --reasoning off --cache-ram 0 \
        --no-webui --host 0.0.0.0 --port 8080 >/dev/null

    for _ in $(seq 180); do
        if healthy; then
            echo "${NAME} serves qwen3.5-4b at http://127.0.0.1:${PORT}/v1"
            return
        fi
        running || break
        sleep 1
    done
    docker logs --tail 30 "${NAME}" >&2
    echo "${NAME} did not come up" >&2
    exit 1
}

status() {
    if ! running; then
        echo "${NAME} is not running"
        return 1
    fi
    if healthy; then
        echo "${NAME} is up at http://127.0.0.1:${PORT}/v1"
    else
        echo "${NAME} is running but still loading"
    fi
    docker stats --no-stream --format '  {{.MemUsage}} RAM, {{.CPUPerc}} CPU' "${NAME}"
    nvidia-smi --query-compute-apps=process_name,used_memory --format=csv,noheader 2>/dev/null \
        | sed -n 's|.*llama-server, |  VRAM |p'
}

case "${1:-}" in
    start) start ;;
    stop) docker rm -f "${NAME}" >/dev/null 2>&1 && echo "${NAME} stopped" || echo "${NAME} was not running" ;;
    status) status ;;
    *)
        echo "usage: $0 start|stop|status" >&2
        exit 2
        ;;
esac
