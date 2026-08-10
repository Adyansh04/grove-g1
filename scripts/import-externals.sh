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

# FAST-LIO fixes its gravity vector by averaging the first MAX_INI_COUNT IMU samples, and
# upstream sets that to 10. Its own paper asks for the sensor to be held STATIC for about two
# seconds while that happens; ten samples is only that long on a slow IMU, and at the Mid360's
# 200 Hz it is 50 ms.
#
# A balancing humanoid has no static pose. Fifty milliseconds does not average the sway, it
# samples one instant of it, so gravity comes out tilted by whatever the body was doing at
# that moment -- and every pose afterwards inherits that tilt, which lands on the floor plane
# and comes back as the costmap marking the floor.
#
# 1000 samples is five seconds at 200 Hz, rather more than the paper asks for and deliberately
# so: this robot is still settling for the first few seconds after spawn, and the extra
# averaging is free -- it sits inside the launch delay that already waits out the spawn drop.
# FAST-LIO subscribes to the IMU with a ten-deep queue and then does its entire update -- ikd-tree
# search, the iterated EKF, the map insert -- inside one timer callback on a single-threaded
# executor. Nothing takes IMU while that runs. Ten samples is 50 ms at a Mid360's 200 Hz, and an
# update on a dense indoor scan can exceed that, at which point the samples are gone: FAST-LIO
# integrates the whole gap on the last reading it saw. Standing that is nothing. Turning at
# 1.6 rad/s it is heading error, sized by how busy the machine happened to be, which is what made
# runs of the same configuration score anywhere between 1 % and 27 % drift.
#
# 2000 is two seconds of buffer. The samples are 300 bytes each.
LASER_MAPPING="${FAST_LIO_REPO}/FAST_LIO/src/laserMapping.cpp"
if [ -f "${LASER_MAPPING}" ] &&
   grep -q 'create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk)' "${LASER_MAPPING}"; then
    sed -i 's/create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk)/create_subscription<sensor_msgs::msg::Imu>(imu_topic, 2000, imu_cbk)/' \
        "${LASER_MAPPING}"
    echo "raised FAST_LIO IMU queue depth to 2000 (the update blocks its own executor)"
fi

IMU_PROCESSING="${FAST_LIO_REPO}/FAST_LIO/src/IMU_Processing.hpp"
if [ -f "${IMU_PROCESSING}" ] && grep -q '^#define MAX_INI_COUNT (10)' "${IMU_PROCESSING}"; then
    sed -i 's/^#define MAX_INI_COUNT (10)/#define MAX_INI_COUNT (1000)/' "${IMU_PROCESSING}"
    echo "raised FAST_LIO MAX_INI_COUNT to 1000 (2 s of gravity averaging at 500 Hz)"
fi
