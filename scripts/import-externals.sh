#!/usr/bin/env bash
#
# Pulls the externals listed in workspace.repos into workspace/src and puts the two that do
# not ship a buildable ROS 2 layout into one.
#
# Run after cloning, and again whenever workspace.repos changes. Idempotent.
#
#   ./scripts/import-externals.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${REPO_ROOT}/workspace/src"

command -v vcs >/dev/null || {
    echo "vcstool is not installed: pip install vcstool" >&2
    exit 1
}

vcs import "${SRC}" < "${REPO_ROOT}/workspace.repos"

# --- livox_ros_driver2 -------------------------------------------------------------------
# The repo carries both ROS 1 and ROS 2 sources and picks between them at build time via
# build.sh, which rewrites the checkout and then calls colcon itself. We cannot use it: it
# builds the whole workspace with its own flags and wipes ../../install on the way in. So do
# the three things it does that actually matter, and let our normal colcon build see a
# conventional package.
DRIVER="${SRC}/livox_ros_driver2"
if [ -d "${DRIVER}" ]; then
    cp -f "${DRIVER}/package_ROS2.xml" "${DRIVER}/package.xml"
    rm -rf "${DRIVER}/launch"
    cp -r "${DRIVER}/launch_ROS2" "${DRIVER}/launch"

    # ROS_EDITION/DISTRO_ROS are build.sh's, and both are load-bearing on Humble: without
    # DISTRO_ROS the CMakeLists takes the Foxy typesupport path and fails to link. A
    # colcon.pkg keeps them attached to this package instead of every build command in the
    # repo's docs having to carry them.
    cat > "${DRIVER}/colcon.pkg" <<'EOF'
{"cmake-args": ["-DROS_EDITION=ROS2", "-DDISTRO_ROS=humble"]}
EOF
    echo "prepared livox_ros_driver2 for ROS 2"
fi

# --- FAST_LIO_LOCALIZATION_HUMANOID ------------------------------------------------------
# We build FAST_LIO and nothing else. open3d_loc is a map-relocalization node we do not use
# (AMCL owns map -> odom) and is the only thing in the repo that needs Open3D, which ships as
# a 400 MB prebuilt blob.
FAST_LIO_REPO="${SRC}/fast_lio_humanoid"
if [ -d "${FAST_LIO_REPO}/open3d_loc" ]; then
    touch "${FAST_LIO_REPO}/open3d_loc/COLCON_IGNORE"
    echo "ignoring open3d_loc"
fi
