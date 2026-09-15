#!/usr/bin/env bash
#
# Sets up the host-side grasp generator that g1_perception's graspgen engine talks to.
#
# Optional. Nothing else in the stack needs it: the stand-in grasp source answers the same
# GenerateGrasps service from /objects alone, and every simulator test uses that one.
#
#   ./scripts/setup-graspgen.sh
#   GRASPGEN_HOME=/opt/GraspGenX ./scripts/setup-graspgen.sh
#
# Idempotent. Safe to re-run to repair a half-finished install.
#
# WHY THIS IS NOT A ROS PACKAGE. Same reason as scripts/setup-groot.sh and setup-vision.sh: the
# model needs torch and CUDA and the container has neither, so it runs on the host and the ROS
# node speaks its ZMQ protocol.
#
# WHAT THIS DOES NOT DO. The checkpoints and the gripper descriptions download themselves on the
# first import, several gigabytes of them, into ext/ inside the checkout. That is upstream's
# design; this script only gets you to the point where that import works.
set -euo pipefail

# Tracks main. Nothing upstream versions the wire protocol the adapter speaks, so a change to it
# arrives as a msgpack error; set GRASPGEN_REF to a commit that worked once you have one.
GRASPGEN_REPO="https://github.com/NVlabs/GraspGenX.git"
GRASPGEN_REF="${GRASPGEN_REF:-main}"
GRASPGEN_HOME="${GRASPGEN_HOME:-${HOME}/ref/GraspGenX}"

command -v uv >/dev/null || {
    echo "uv is not installed: https://docs.astral.sh/uv/getting-started/installation/" >&2
    exit 1
}

echo "==> repository at ${GRASPGEN_HOME}"
if [ ! -d "${GRASPGEN_HOME}/.git" ]; then
    mkdir -p "$(dirname "${GRASPGEN_HOME}")"
    git clone "${GRASPGEN_REPO}" "${GRASPGEN_HOME}"
fi
git -C "${GRASPGEN_HOME}" fetch --quiet origin
# origin/<ref> for a branch, the ref itself for a commit: plain `checkout <branch>` on an
# existing clone sits on whatever it was cloned at.
target="$(git -C "${GRASPGEN_HOME}" rev-parse --verify --quiet "origin/${GRASPGEN_REF}" ||
    echo "${GRASPGEN_REF}")"
git -C "${GRASPGEN_HOME}" checkout --quiet --detach "${target}"
# Printed so a working setup can be reproduced: this is the value to put in GRASPGEN_REF.
echo "    at $(git -C "${GRASPGEN_HOME}" rev-parse --short HEAD)"

echo "==> dependencies, including the serving extra the ZMQ server needs"
(cd "${GRASPGEN_HOME}" && uv sync --extra serve)

echo "==> checking the install"
(cd "${GRASPGEN_HOME}" && uv run python - <<'PY'
import torch

print(f"    torch {torch.__version__}, cuda available: {torch.cuda.is_available()}")
try:
    from graspgenx import get_gripper_descriptions_root

    print(f"    gripper descriptions: {get_gripper_descriptions_root()}")
except Exception as error:  # noqa: BLE001 - the first import is what downloads the assets
    print(f"    gripper descriptions not ready yet: {error}")
PY
)

cat <<EOF

Done. Serve the generator:

  cd ${GRASPGEN_HOME}
  uv run python client-server/graspgenx_server.py \\
      --config ext/graspgenx_checkpoints/release --assets_dir ext/gripper_descriptions --port 5556

The first run downloads the checkpoints and the gripper descriptions, a few gigabytes, into
${GRASPGEN_HOME}/ext.

Then bring the stack up against it:

  ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true \\
      odometry:=ground_truth moveit:=true manipulation:=true perception:=true \\
      grasp_engine:=graspgen

and look at /grasp_candidates in RViz. The hand this robot carries is GraspGenX's own
\`unitree_g1\` gripper, the Dex3-1, and g1_perception sends its sweep volume rather than its name,
so the server needs no assets for it.

The poses it returns are in its own gripper frame, which is not a link of this robot. Measuring
that offset against the arrows is the one calibration this path needs; see
docs/guides/open-vocabulary-grasping.md.
EOF
