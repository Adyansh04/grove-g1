#!/usr/bin/env bash
#
# Fetches the apartment world's third-party meshes and textures into workspace/assets/ and
# converts them for MuJoCo. world:=apartment needs them; every other world runs without.
#
#   ./scripts/setup-world-assets.sh
#   ./scripts/setup-world-assets.sh --only ph_sofa_02 --force
#   WORLDGEN_HOME=/opt/grove-worldgen ./scripts/setup-world-assets.sh
#
# Idempotent: converted assets are skipped and downloads are cached in workspace/assets/.downloads,
# about 0.3 GB, safe to delete once converted. Arguments go to tools/world_assets.py.
#
# Runs on the host, not in the container: the converter needs mujoco, trimesh and pillow, which
# the dev image lacks. The container sees the result through its workspace mount.
set -euo pipefail

WORLDGEN_HOME="${WORLDGEN_HOME:-${HOME}/ref/grove-worldgen}"
PYTHON_VERSION="3.12"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# mujoco matches the simulator's 3.3.6, so a converted mesh loads exactly as it will in the sim.
DEPS=(
    "mujoco==3.3.6"
    "trimesh==5.1.0"
    "pillow==12.3.0"
    "numpy==2.5.3"
    "scipy==1.18.1"
    "pyyaml==6.0.3"
)

if [ ! -x "${WORLDGEN_HOME}/.venv/bin/python" ]; then
    command -v uv >/dev/null || {
        echo "uv is not installed: https://docs.astral.sh/uv/getting-started/installation/" >&2
        exit 1
    }
    echo "==> virtualenv at ${WORLDGEN_HOME}/.venv"
    mkdir -p "${WORLDGEN_HOME}"
    uv venv --python "${PYTHON_VERSION}" "${WORLDGEN_HOME}/.venv"
    VIRTUAL_ENV="${WORLDGEN_HOME}/.venv" uv pip install --quiet "${DEPS[@]}"
fi

exec "${WORLDGEN_HOME}/.venv/bin/python" \
    "${REPO}/workspace/src/g1_bringup/tools/world_assets.py" "$@"
