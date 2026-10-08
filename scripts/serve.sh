#!/usr/bin/env bash
# Starts a model server on the host, with the interpreter its setup script installed.
#
#   ./scripts/serve.sh groot [args]      the GR00T policy for learned grasping, on 5555
#   ./scripts/serve.sh vision [args]     Grounded SAM 2 masks and instruction grounding, on 5560
#   ./scripts/serve.sh graspgen [args]   GraspGenX grasps for the Dex3-1, on 5556
#   ./scripts/serve.sh canopy [args]     canopy's semantic server on 5561 (SAM 3.1, or YOLOE-26 with
#                                        --detector yoloe), and the offline VLM with YOLOE or when
#                                        a --describer list names openai
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
        # The offline describer's 4 GB fit beside YOLOE but not beside SAM 3.1 and the simulator: it
        # starts with YOLOE or a --describer list naming openai; beside SAM 3.1 one left up stops.
        describers=""
        detector="sam3.1"
        previous=""
        for arg in "$@"; do
            [[ "$previous" == --describer ]] && describers="$arg"
            [[ "$arg" == --describer=* ]] && describers="${arg#*=}"
            [[ "$previous" == --detector ]] && detector="$arg"
            [[ "$arg" == --detector=* ]] && detector="${arg#*=}"
            previous="$arg"
        done
        if [[ "$describers" == *openai* || ( -z "$describers" && "$detector" == yoloe* ) ]]; then
            "$CANOPY/servers/start-vlm.sh" start
        elif [[ "$detector" == sam3.1 ]]; then
            "$CANOPY/servers/start-vlm.sh" stop >/dev/null
        fi
        exec "$CANOPY_HOME/.venv/bin/python" "$CANOPY/servers/semantic_server.py" "$@"
        ;;
    *)
        sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
        [[ "$server" == help || "$server" == -h || "$server" == --help ]]
        ;;
esac
