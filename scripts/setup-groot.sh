#!/usr/bin/env bash
#
# Sets up the host-side GR00T policy server that g1_vla's groot engine talks to.
#
# Optional: the mock engine serves the same GetActionChunk service. Run this only to drive the
# learned-grasp path with a real model.
#
#   ./scripts/setup-groot.sh
#   GROOT_HOME=/opt/Isaac-GR00T ./scripts/setup-groot.sh
#
# Idempotent. Safe to re-run to repair a half-finished install.
#
# Not a ROS package: the model needs torch and CUDA, which the container deliberately lacks, so
# the server runs on the host and g1_vla_groot_adapter speaks its wire protocol.
#
# The torch wheels are fetched with curl because the ~900 MB download from download.pytorch.org
# drops partway through, and curl -C - resumes where uv cannot.
set -euo pipefail

# Pinned: the wire protocol and the server's entry point are not versioned anywhere upstream.
GROOT_COMMIT="376ba89"
GROOT_REPO="https://github.com/NVIDIA/Isaac-GR00T.git"
GROOT_HOME="${GROOT_HOME:-${HOME}/ref/Isaac-GR00T}"
WHEEL_CACHE="${WHEEL_CACHE:-${HOME}/ref/wheels}"
PYTHON_VERSION="3.12"

TORCH_INDEX="https://download.pytorch.org/whl/cu128"
TORCH_WHEEL="torch-2.9.0%2Bcu128-cp312-cp312-manylinux_2_28_x86_64.whl"
VISION_WHEEL="torchvision-0.24.0%2Bcu128-cp312-cp312-manylinux_2_28_x86_64.whl"

# Inference only. flash-attn compiles from source, and deepspeed and tensorrt are training and
# deployment concerns; the server falls back to sdpa attention and says so on startup.
INFERENCE_DEPS=(
    "transformers==4.57.3"
    "diffusers==0.35.1"
    "peft==0.17.1"
    "einops==0.8.1"
    "timm==1.0.28"
    "accelerate==1.14.0"
    "albumentations==1.4.18"
    "pyzmq==27.0.1"
    "msgpack==1.1.0"
    "tyro==0.9.17"
    "numpy==1.26.4"
)

command -v uv >/dev/null || {
    echo "uv is not installed: https://docs.astral.sh/uv/getting-started/installation/" >&2
    exit 1
}

echo "==> repository at ${GROOT_HOME}, pinned to ${GROOT_COMMIT}"
if [ ! -d "${GROOT_HOME}/.git" ]; then
    mkdir -p "$(dirname "${GROOT_HOME}")"
    git clone "${GROOT_REPO}" "${GROOT_HOME}"
fi
git -C "${GROOT_HOME}" fetch --quiet origin
git -C "${GROOT_HOME}" checkout --quiet "${GROOT_COMMIT}"

if ! git -C "${GROOT_HOME}" diff --quiet; then
    echo "    the checkout has local modifications; leaving them alone." >&2
    echo "    this repo does not need any: scripts/groot_server.py carries the one fix." >&2
fi

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

echo "==> virtualenv"
[ -d "${GROOT_HOME}/.venv" ] || uv venv --python "${PYTHON_VERSION}" "${GROOT_HOME}/.venv"

echo "==> installing torch, then the inference subset"
# `uv pip install`, never `uv sync`, which would fetch torch from download.pytorch.org again.
VIRTUAL_ENV="${GROOT_HOME}/.venv" uv pip install --quiet \
    "${WHEEL_CACHE}/${TORCH_WHEEL//%2B/+}" "${WHEEL_CACHE}/${VISION_WHEEL//%2B/+}"
VIRTUAL_ENV="${GROOT_HOME}/.venv" uv pip install --quiet \
    --index-url https://pypi.org/simple "${INFERENCE_DEPS[@]}"
VIRTUAL_ENV="${GROOT_HOME}/.venv" uv pip install --quiet --no-deps msgpack-numpy==0.4.8
VIRTUAL_ENV="${GROOT_HOME}/.venv" uv pip install --quiet --no-deps -e "${GROOT_HOME}"

echo "==> checking the install"
"${GROOT_HOME}/.venv/bin/python" - <<'PY'
import torch
import gr00t  # noqa: F401
from gr00t.eval.run_gr00t_server import ServerConfig  # noqa: F401
print(f"    torch {torch.__version__}, cuda available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    total = torch.cuda.get_device_properties(0).total_memory / 1024**3
    print(f"    {torch.cuda.get_device_name(0)}, {total:.1f} GiB")
PY

cat <<EOF

Done. Two things this script cannot do for you:

  1. nvidia/Cosmos-Reason2-2B is a gated repo and every GR00T checkpoint loads it. Sign in and
     accept its terms, then authenticate:

       ${GROOT_HOME}/.venv/bin/hf auth login
       ${GROOT_HOME}/.venv/bin/hf download nvidia/Cosmos-Reason2-2B config.json

  2. The weights are ~11 GB and download on first run into ~/.cache/huggingface.

Then serve the policy, from this repository so the load fix applies:

  ${GROOT_HOME}/.venv/bin/python scripts/groot_server.py \\
      --model-path nvidia/GR00T-N1.7-3B --embodiment-tag real_g1 --port 5555

and bring the stack up against it:

  ros2 launch g1_bringup bringup.launch.py vla:=true vla_engine:=groot \\
      manipulation:=true moveit:=true world:=manipulation

The base checkpoint reaches toward an object but does not grasp it; that needs post-training on
demonstrations from this robot, and there is no recorder yet.
EOF
