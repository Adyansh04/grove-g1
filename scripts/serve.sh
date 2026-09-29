#!/usr/bin/env bash
# Starts a model server on the host, with the interpreter its setup script installed.
#
#   ./scripts/serve.sh groot [args]      the GR00T policy for learned grasping, on 5555
#   ./scripts/serve.sh vision [args]     open-vocabulary masks and instruction grounding, on 5560
#   ./scripts/serve.sh graspgen [args]   GraspGenX grasps for the Dex3-1, on 5556
#   ./scripts/serve.sh canopy [args]     canopy's semantic server on 5561, and its offline VLM
#
# The arguments go to the server after the defaults here, so they override them:
#   ./scripts/serve.sh vision --vlm Qwen/Qwen3-VL-2B-Instruct
#
# The container reaches every one of them on 127.0.0.1: Compose is host-networked.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CANOPY="$ROOT/workspace/src/canopy"

# Where each setup script installs, overridable the same way.
GROOT_HOME="${GROOT_HOME:-${HOME}/ref/Isaac-GR00T}"
VISION_HOME="${VISION_HOME:-${HOME}/ref/grove-vision}"
GRASPGEN_HOME="${GRASPGEN_HOME:-${HOME}/ref/GraspGenX}"
CANOPY_HOME="${CANOPY_HOME:-${HOME}/.local/share/canopy}"

# Says which setup script to run, rather than failing later on an import.
require() {
    if [[ ! -e "$1" ]]; then
        echo "$1 is missing; run $2 first." >&2
        exit 1
    fi
}

server="${1:-}"
(( $# )) && shift
case "$server" in
    groot)
        require "$GROOT_HOME/.venv/bin/python" ./scripts/setup-groot.sh
        exec "$GROOT_HOME/.venv/bin/python" "$ROOT/servers/groot_server.py" \
            --model-path nvidia/GR00T-N1.7-3B --embodiment-tag real_g1 --port 5555 "$@"
        ;;
    vision)
        require "$VISION_HOME/.venv/bin/python" ./scripts/setup-vision.sh
        exec "$VISION_HOME/.venv/bin/python" "$ROOT/servers/vision_server.py" --port 5560 "$@"
        ;;
    graspgen)
        require "$GRASPGEN_HOME/client-server/graspgenx_server.py" ./scripts/setup-graspgen.sh
        cd "$GRASPGEN_HOME"
        exec uv run python client-server/graspgenx_server.py \
            --config ext/graspgenx_checkpoints/release --assets_dir ext/gripper_descriptions \
            --port 5556 "$@"
        ;;
    canopy)
        require "$CANOPY_HOME/.venv/bin/python" workspace/src/canopy/servers/setup.sh
        # The describer's fallback when Gemini refuses. It runs as a container of its own and is
        # left up for the next run; `start-vlm.sh stop` ends it.
        "$CANOPY/servers/start-vlm.sh" start
        exec "$CANOPY_HOME/.venv/bin/python" "$CANOPY/servers/semantic_server.py" "$@"
        ;;
    *)
        sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
        [[ "$server" == help || "$server" == -h || "$server" == --help ]]
        ;;
esac
