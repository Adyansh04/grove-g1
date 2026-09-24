#!/usr/bin/env bash
#
# Sets up the host-side vision server that g1_perception's detector talks to.
#
# Optional: the mock detector publishes the same instance masks from simulator ground truth, so
# every sim test runs without a GPU.
#
#   ./scripts/setup-vision.sh
#   VISION_HOME=/opt/grove-vision ./scripts/setup-vision.sh
#
# Idempotent. Safe to re-run to repair a half-finished install.
#
# Not a ROS package: the models need torch and CUDA, which the container deliberately lacks, so
# the server runs on the host and the ROS detector speaks its wire protocol.
#
# The torch wheels are fetched with curl into a cache shared with scripts/setup-groot.sh, because
# the ~900 MB download drops partway through and curl -C - resumes where uv cannot.
set -euo pipefail

VISION_HOME="${VISION_HOME:-${HOME}/ref/grove-vision}"
WHEEL_CACHE="${WHEEL_CACHE:-${HOME}/ref/wheels}"
PYTHON_VERSION="3.12"

TORCH_INDEX="https://download.pytorch.org/whl/cu128"
TORCH_WHEEL="torch-2.9.0%2Bcu128-cp312-cp312-manylinux_2_28_x86_64.whl"
VISION_WHEEL="torchvision-0.24.0%2Bcu128-cp312-cp312-manylinux_2_28_x86_64.whl"

# transformers 5.x for Sam3Model, which 4.x lacks; the Grounding DINO and SAM 2.1 classes are the
# same in both.
DEPS=(
    "transformers==5.17.0"
    "accelerate==1.14.0"
    "pillow==11.3.0"
    "pyzmq==27.0.1"
    "msgpack==1.1.0"
    "numpy==2.2.6"
)

command -v uv >/dev/null || {
    echo "uv is not installed: https://docs.astral.sh/uv/getting-started/installation/" >&2
    exit 1
}

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

echo "==> virtualenv at ${VISION_HOME}/.venv"
mkdir -p "${VISION_HOME}"
[ -d "${VISION_HOME}/.venv" ] || uv venv --python "${PYTHON_VERSION}" "${VISION_HOME}/.venv"

echo "==> installing torch, then the rest"
VIRTUAL_ENV="${VISION_HOME}/.venv" uv pip install --quiet \
    "${WHEEL_CACHE}/${TORCH_WHEEL//%2B/+}" "${WHEEL_CACHE}/${VISION_WHEEL//%2B/+}"
VIRTUAL_ENV="${VISION_HOME}/.venv" uv pip install --quiet \
    --index-url https://pypi.org/simple "${DEPS[@]}"
# --no-deps: it pins an old numpy, and only a handful of its functions are used.
VIRTUAL_ENV="${VISION_HOME}/.venv" uv pip install --quiet --no-deps msgpack-numpy==0.4.8

echo "==> checking the install"
"${VISION_HOME}/.venv/bin/python" - <<'PY'
import msgpack_numpy  # noqa: F401
import torch
import zmq  # noqa: F401
from transformers import AutoModelForZeroShotObjectDetection, Sam2Model  # noqa: F401

print(f"    torch {torch.__version__}, cuda available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    total = torch.cuda.get_device_properties(0).total_memory / 1024**3
    print(f"    {torch.cuda.get_device_name(0)}, {total:.1f} GiB")
try:
    from transformers import Sam3Model  # noqa: F401

    print("    Sam3Model present: --backend sam3 works once the weights are downloadable")
except ImportError:
    print("    Sam3Model missing: this transformers is too old for --backend sam3")
PY

cat <<EOF

Done. The weights download on first run into ~/.cache/huggingface: about 900 MB for Grounding
DINO base and 180 MB for SAM 2.1 small.

Serve the default backend:

  ${VISION_HOME}/.venv/bin/python scripts/vision_server.py --port 5560

Check it against saved frames instead of serving:

  ${VISION_HOME}/.venv/bin/python scripts/vision_server.py --self-test frame.png \\
      --phrases "red block,green cylinder"

SAM 3 is the better model and its weights are gated. Request access at
https://huggingface.co/facebook/sam3, sign in with

  ${VISION_HOME}/.venv/bin/hf auth login

and then serve it with --backend sam3. Nothing in the ROS workspace changes: the instance-mask
message is the same either way.
EOF
