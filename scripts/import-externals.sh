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
    echo "vcstool is not installed: sudo apt install python3-vcstool" >&2
    exit 1
}

# Applies one sed to a vendored file, idempotently. Warns when the file matches neither the
# upstream nor the patched text, since a silent no-op would quietly restore upstream behaviour.
patch_upstream() {
    local file="$1" pattern="$2" applied="$3" script="$4" note="$5"
    [ -f "${file}" ] || return 0
    if grep -q "${pattern}" "${file}"; then
        sed -i "${script}" "${file}"
        echo "${note}"
    elif ! grep -q "${applied}" "${file}"; then
        echo "WARNING: ${file} matches neither the upstream text nor the patched text; the" >&2
        echo "         patch was NOT applied and upstream behaviour is back." >&2
    fi
}

vcs import "${SRC}" < "${REPO_ROOT}/workspace.repos"

# --- livox_ros_driver2 -------------------------------------------------------------------
# Upstream's build.sh picks ROS 1 or 2 by rewriting the checkout, then builds the whole workspace
# with its own flags and wipes ../../install. Do the three steps that matter and let colcon build
# it as a conventional package.
DRIVER="${SRC}/livox_ros_driver2"
if [ -d "${DRIVER}" ]; then
    cp -f "${DRIVER}/package_ROS2.xml" "${DRIVER}/package.xml"
    rm -rf "${DRIVER}/launch"
    cp -r "${DRIVER}/launch_ROS2" "${DRIVER}/launch"

    # build.sh normally passes these. Without DISTRO_ROS=jazzy the CMakeLists takes the Foxy
    # typesupport path and fails to link; colcon.pkg keeps them on this package alone.
    cat > "${DRIVER}/colcon.pkg" <<'EOF'
{"cmake-args": ["-DROS_EDITION=ROS2", "-DDISTRO_ROS=jazzy"]}
EOF
    echo "prepared livox_ros_driver2 for ROS 2"
fi

# --- FAST_LIO_LOCALIZATION_HUMANOID ------------------------------------------------------
# Only FAST_LIO is built. open3d_loc is a map relocalizer this stack does not use (AMCL owns
# map -> odom) and the only thing needing Open3D, a 400 MB prebuilt blob.
FAST_LIO_REPO="${SRC}/fast_lio_humanoid"
if [ -d "${FAST_LIO_REPO}/open3d_loc" ]; then
    touch "${FAST_LIO_REPO}/open3d_loc/COLCON_IGNORE"
    echo "ignoring open3d_loc"
fi

# FAST-LIO runs its whole update in one callback on a single-threaded executor, so a 10-deep IMU
# queue (50 ms at 200 Hz) overflows on a dense scan and the lost samples become heading error.
# 2000 is 10 s of buffer, at 300 bytes a sample.
LASER_MAPPING="${FAST_LIO_REPO}/FAST_LIO/src/laserMapping.cpp"
patch_upstream "${LASER_MAPPING}" \
    'create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk)' \
    'create_subscription<sensor_msgs::msg::Imu>(imu_topic, 2000, imu_cbk)' \
    's/create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk)/create_subscription<sensor_msgs::msg::Imu>(imu_topic, 2000, imu_cbk)/' \
    "raised FAST_LIO IMU queue depth to 2000 (the update blocks its own executor)"

# FAST-LIO fixes gravity from the first MAX_INI_COUNT IMU samples. Upstream's 10 is 50 ms at
# 200 Hz, one instant of a balancing robot's sway, and the tilt it bakes in marks the floor as an
# obstacle. 1000 averages 5 s, inside the launch delay that already waits out the spawn drop.
IMU_PROCESSING="${FAST_LIO_REPO}/FAST_LIO/src/IMU_Processing.hpp"
patch_upstream "${IMU_PROCESSING}" \
    '^#define MAX_INI_COUNT (10)' \
    '^#define MAX_INI_COUNT (1000)' \
    's/^#define MAX_INI_COUNT (10)/#define MAX_INI_COUNT (1000)/' \
    "raised FAST_LIO MAX_INI_COUNT to 1000 (5 s of gravity averaging at 200 Hz)"
