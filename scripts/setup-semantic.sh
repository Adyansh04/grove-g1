#!/usr/bin/env bash
#
# Sets up the host side of the semantic map: scripts/semantic_server.py (YOLOE-26 detector,
# SigLIP 2 embedder, Gemini or local VLM describer) and the local VLM that
# scripts/start-vlm.sh runs in llama.cpp.
#
#   ./scripts/setup-semantic.sh
#   SEMANTIC_HOME=/opt/grove-semantic ./scripts/setup-semantic.sh
#
# Idempotent. Safe to re-run to repair a half-finished install.
#
# Not a ROS package: the models need torch and CUDA, which the container deliberately lacks, so
# the server runs on the host and g1_detector speaks its wire protocol. The torch wheels come
# from the cache that scripts/setup-vision.sh and scripts/setup-groot.sh share.
set -euo pipefail

SEMANTIC_HOME="${SEMANTIC_HOME:-${HOME}/ref/grove-semantic}"
WHEEL_CACHE="${WHEEL_CACHE:-${HOME}/ref/wheels}"
WEIGHTS="${SEMANTIC_HOME}/weights"
PYTHON_VERSION="3.12"

TORCH_INDEX="https://download.pytorch.org/whl/cu128"
TORCH_WHEEL="torch-2.9.0%2Bcu128-cp312-cp312-manylinux_2_28_x86_64.whl"
VISION_WHEEL="torchvision-0.24.0%2Bcu128-cp312-cp312-manylinux_2_28_x86_64.whl"

DEPS=(
    "ultralytics==8.4.162"
    "transformers==5.17.0"
    "accelerate==1.14.0"
    "pillow==11.3.0"
    "pyzmq==27.0.1"
    "msgpack==1.1.0"
    "numpy==2.2.6"
    "sentencepiece==0.2.2"
    "pyyaml==6.0.3"
    # YOLOE's text tokenizer. Ultralytics would pip-install it on first use, which the server
    # forbids (YOLO_AUTOINSTALL=False), so it is pinned here instead.
    "clip @ git+https://github.com/ultralytics/CLIP.git@a13192f8cb767260d7dfd98c843b0716593169e7"
)

# YOLOE-26 large, text-prompted and prompt-free, and the MobileCLIP2 text encoder that
# text prompting loads from the working directory.
YOLOE_RELEASE="https://github.com/ultralytics/assets/releases/download/v8.4.0"
YOLOE_FILES=(yoloe-26l-seg.pt yoloe-26l-seg-pf.pt mobileclip2_b.ts)

SIGLIP="google/siglip2-base-patch16-256"
SIGLIP_REVISION="3f9f96cb90da5dbc758b01813f2f6f1aee24c1ab"
GGUF_REPO="unsloth/Qwen3.5-4B-GGUF"
GGUF_REVISION="e87f176479d0855a907a41277aca2f8ee7a09523"
GGUF_FILES=(Qwen3.5-4B-Q4_K_M.gguf mmproj-F16.gguf)
# One pin for the image, kept in start-vlm.sh.
LLAMA_IMAGE="$(grep -o 'ghcr.io/ggml-org/llama.cpp@sha256:[0-9a-f]*' "$(dirname "$0")/start-vlm.sh")"

for tool in uv git curl docker; do
    command -v "${tool}" >/dev/null || {
        echo "${tool} is not installed" >&2
        exit 1
    }
done

echo "==> torch wheels in ${WHEEL_CACHE}"
mkdir -p "${WHEEL_CACHE}"
for wheel in "${TORCH_WHEEL}" "${VISION_WHEEL}"; do
    name="${wheel//%2B/+}"
    if [ -s "${WHEEL_CACHE}/${name}" ]; then
        echo "    have ${name}"
        continue
    fi
    echo "    fetching ${name} (resumable; re-run this script if it stalls)"
    curl -fL -C - --retry 20 --retry-all-errors -o "${WHEEL_CACHE}/${name}" \
        "${TORCH_INDEX}/${wheel}"
done

echo "==> virtualenv at ${SEMANTIC_HOME}/.venv"
mkdir -p "${SEMANTIC_HOME}"
[ -d "${SEMANTIC_HOME}/.venv" ] || uv venv --python "${PYTHON_VERSION}" "${SEMANTIC_HOME}/.venv"

echo "==> installing torch, then the rest"
VIRTUAL_ENV="${SEMANTIC_HOME}/.venv" uv pip install --quiet \
    "${WHEEL_CACHE}/${TORCH_WHEEL//%2B/+}" "${WHEEL_CACHE}/${VISION_WHEEL//%2B/+}"
VIRTUAL_ENV="${SEMANTIC_HOME}/.venv" uv pip install --quiet \
    --index-url https://pypi.org/simple "${DEPS[@]}"
# --no-deps: it pins an old numpy, and only a handful of its functions are used.
VIRTUAL_ENV="${SEMANTIC_HOME}/.venv" uv pip install --quiet --no-deps msgpack-numpy==0.4.8

echo "==> YOLOE-26 weights in ${WEIGHTS}"
mkdir -p "${WEIGHTS}"
for file in "${YOLOE_FILES[@]}"; do
    if [ -s "${WEIGHTS}/${file}" ]; then
        echo "    have ${file}"
        continue
    fi
    echo "    fetching ${file}"
    curl -fL -C - --retry 20 --retry-all-errors -o "${WEIGHTS}/${file}" \
        "${YOLOE_RELEASE}/${file}"
done

echo "==> SigLIP 2 and Qwen3.5-4B GGUF in the Hugging Face cache"
"${SEMANTIC_HOME}/.venv/bin/hf" download "${SIGLIP}" --revision "${SIGLIP_REVISION}" >/dev/null
"${SEMANTIC_HOME}/.venv/bin/hf" download "${GGUF_REPO}" "${GGUF_FILES[@]}" \
    --revision "${GGUF_REVISION}" >/dev/null
echo "    have ${SIGLIP} and ${GGUF_FILES[*]}"

echo "==> llama.cpp server image"
if docker image inspect "${LLAMA_IMAGE}" >/dev/null 2>&1; then
    echo "    have ${LLAMA_IMAGE}"
else
    docker pull -q "${LLAMA_IMAGE}"
fi

echo "==> checking the install"
"${SEMANTIC_HOME}/.venv/bin/python" - <<'PY'
import clip  # noqa: F401
import msgpack_numpy  # noqa: F401
import torch
import zmq  # noqa: F401
from transformers import AutoModel  # noqa: F401
from ultralytics import YOLOE  # noqa: F401

print(f"    torch {torch.__version__}, cuda available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    total = torch.cuda.get_device_properties(0).total_memory / 1024**3
    print(f"    {torch.cuda.get_device_name(0)}, {total:.1f} GiB")
PY

cat <<EOF

Done. Serve the detector, embedder and describers on port 5561 (YOLOE-26l and SigLIP 2 take
about 1.7 GB of VRAM):

  ${SEMANTIC_HOME}/.venv/bin/python scripts/semantic_server.py

Describers fall through in order, gemini,openai by default. Gemini reads its key from
~/.config/grove/gemini.env and stops at the free tier's 5 requests a minute and 100 a day. The
local VLM (Qwen3.5-4B in llama.cpp, about 4 GB of VRAM) serves the openai describer:

  ./scripts/start-vlm.sh start      # stop when done: ./scripts/start-vlm.sh stop
  ${SEMANTIC_HOME}/.venv/bin/python scripts/semantic_server.py --describer openai

Check it against saved frames instead of serving:

  ${SEMANTIC_HOME}/.venv/bin/python scripts/semantic_server.py --self-test frame.png

Point g1_detector at it by loading g1_perception/config/indoor_vocabulary.yaml after
g1_detector.yaml. Unit tests, no GPU needed:

  ${SEMANTIC_HOME}/.venv/bin/python -m unittest scripts/tests/test_semantic_server.py
EOF
