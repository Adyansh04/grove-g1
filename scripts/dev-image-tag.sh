#!/usr/bin/env bash
# Prints the dev image's tag: a hash of everything the image is built from, the Dockerfile and the
# unitree_mujoco commit this repo records. CI publishes the image under this tag, and manage.sh
# pulls it, so a machine builds the image only when its inputs changed.
#
#   ./scripts/dev-image-tag.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

{
    cat "$ROOT/.devcontainer/Dockerfile"
    git -C "$ROOT" rev-parse HEAD:workspace/vendor/unitree_mujoco
} | sha256sum | cut -c1-16
