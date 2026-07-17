# Extends your pre-built image (tag is provided by docker-compose.yml via BASE_IMAGE)

ARG BASE_IMAGE=adyansh04/ros-dev:humble
FROM ${BASE_IMAGE}

# Allow pip installs in distro-managed Python images (required on Ubuntu 24.04+/ROS Jazzy).
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# ============================================================================
# EDITOR TOOLING
# These are pre-configured tools for C++ and Python development.
# ============================================================================
RUN apt-get update && apt-get install -y clangd-14 clang-format-14 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-14 100 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-14 100 && \
    (python3 -m pip install --no-cache-dir --break-system-packages ruff || \
     python3 -m pip install --no-cache-dir ruff) && \
    rm -rf /var/lib/apt/lists/*

# ============================================================================
# PROJECT-SPECIFIC CUSTOMIZATION
# G1 sim-first foundation: ros2_control + Unitree SDK / unitree_mujoco
# bring-up deps: everything needed to build and run unitree_sdk2 and
# unitree_mujoco from source, installed in the layer below.
# ============================================================================

# ros2_control stack, CycloneDDS RMW, and the unitree_sdk2/unitree_mujoco
# source builds' own deps, in one layer. libgl1-mesa-dri/libglx-mesa0:
# software GL — without mesa, GLFW under Xvfb dies on context creation.
# libeigen3-dev/libfmt-dev aren't in unitree_sdk2's or unitree_mujoco's own
# published dep lists but are genuinely required transitively: the G1
# dds_wrapper headers unitree_mujoco includes pull in Eigen, and
# unitree_mujoco/simulate links libfmt directly. rosidl-generator-dds-idl:
# required by the unitree_ros2 message packages (unitree_api/go/hg) built in
# the workspace; not part of the desktop install.
RUN apt-get update && apt-get install -y \
        libboost-all-dev \
        libeigen3-dev \
        libfmt-dev \
        libgl1-mesa-dri \
        libglfw3-dev \
        libglx-mesa0 \
        libspdlog-dev \
        libyaml-cpp-dev \
        ros-humble-launch-testing \
        ros-humble-launch-testing-ament-cmake \
        ros-humble-realtime-tools \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ros2launch \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-xacro \
        xvfb \
    && rm -rf /var/lib/apt/lists/*

# --- unitree_sdk2 -----------------------------------------------------------
# Vendors its own CycloneDDS thirdparty libs; installs straight into
# /opt/unitree_robotics so unitree_mujoco's find_package(unitree_sdk2) below
# needs no extra CMAKE_PREFIX_PATH plumbing. Only include/lib are
# runtime-relevant, so the source/build tree is discarded once install
# finishes — unlike unitree_mujoco below, nothing reads it at runtime.
# Pinned 2026-07-17 — current tip of unitree_sdk2 at authoring time.
ARG UNITREE_SDK2_SHA=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git init -q /opt/unitree_robotics/unitree_sdk2 && \
    cd /opt/unitree_robotics/unitree_sdk2 && \
    git remote add origin https://github.com/unitreerobotics/unitree_sdk2.git && \
    git fetch --depth 1 origin ${UNITREE_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics \
             -DBUILD_EXAMPLES=OFF && \
    make -j"$(nproc)" && \
    make install && \
    rm -rf /opt/unitree_robotics/unitree_sdk2

# --- MuJoCo ------------------------------------------------------------------
# unitree_mujoco/simulate/CMakeLists.txt compiles MuJoCo's own "simulate"
# viewer sources (glfw_adapter.cc etc.) directly into our binary, not just a
# prebuilt library, so the release tarball is extracted whole rather than
# just headers + .so. Must match the exact version unitree_mujoco's own tip
# instructs extracting, not just "latest" MuJoCo.
# Pinned 2026-07-17 — the release unitree_mujoco's current tip's README
# instructs extracting (mujoco-3.3.6).
ARG MUJOCO_VERSION=3.3.6
RUN mkdir -p /opt/unitree_robotics/mujoco-${MUJOCO_VERSION} && \
    curl -fsSL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
      | tar -xz --strip-components=1 -C /opt/unitree_robotics/mujoco-${MUJOCO_VERSION}

# --- unitree_mujoco (the `simulate` app) ------------------------------------
# Same DDS topics as the real robot (unitree_hg, rt/arm_sdk), per
# unitreerobotics/unitree_mujoco upstream. The binary resolves its config and MJCF
# assets relative to its own on-disk location (simulate/config.yaml,
# ../unitree_robots/<robot>/...), so this source tree — including the
# extracted MuJoCo release moved under simulate/mujoco/ — stays in the
# image; only build byproducts are trimmed.
# `jstest` (a joystick self-test target we don't use — config.yaml ships
# with use_joystick: 0) fails to build against this unitree_sdk2 tip: its
# `xKeySwitchUnion` fields were renamed upstream since unitree_mujoco's last
# update. Unrelated to the `simulate` binary itself, so scope the build to
# the one target we ship.
# Pinned 2026-07-17 — current tip of unitree_mujoco at authoring time,
# resolved together with UNITREE_SDK2_SHA above.
ARG UNITREE_MUJOCO_SHA=ae6a8403e272733e9996ef59990880330496177f
RUN git init -q /opt/unitree_robotics/unitree_mujoco && \
    cd /opt/unitree_robotics/unitree_mujoco && \
    git remote add origin https://github.com/unitreerobotics/unitree_mujoco.git && \
    git fetch --depth 1 origin ${UNITREE_MUJOCO_SHA} && \
    git checkout -q FETCH_HEAD && \
    rm -rf .git && \
    rm -rf simulate/mujoco && \
    mv /opt/unitree_robotics/mujoco-${MUJOCO_VERSION} simulate/mujoco && \
    mkdir simulate/build && cd simulate/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_PREFIX_PATH=/opt/unitree_robotics/lib/cmake && \
    cmake --build . --target unitree_mujoco --parallel "$(nproc)" && \
    rm -rf CMakeFiles && \
    rm -rf ../mujoco/include ../mujoco/simulate ../mujoco/sample \
           ../mujoco/bin ../mujoco/doc ../mujoco/model
# Binary: /opt/unitree_robotics/unitree_mujoco/simulate/build/unitree_mujoco
# Config: /opt/unitree_robotics/unitree_mujoco/simulate/config.yaml, or
# override per-run with the binary's own -r/-i/-n/-s flags (robot/domain/
# interface/scene) — g1_bringup's ExecuteProcess uses the flags rather than
# mutating this file in place.

# --- CycloneDDS / ROS domain (unconditional -- required by Unitree SDK) -----
# Pin discovery to loopback so the ROS graph and unitree_mujoco's native DDS
# app land on the same interface — the sim's own config.yaml pins `lo` too;
# without this, both sides can silently pick different interfaces and never
# discover each other.
RUN mkdir -p /etc/cyclonedds && \
    cat > /etc/cyclonedds/cyclonedds.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
# Sim-only for this milestone — revisit at hardware bring-up when the real
# robot needs the default domain (see g1_bringup README).
ENV ROS_DOMAIN_ID=1

# The base image's /etc/profile.d/10-ros-env.sh re-exports ROS_DOMAIN_ID=0 in
# login shells, silently overriding the ENV above. 99- sorts after 10-, so this
# wins in every login shell and keeps interactive terminals on the sim domain.
RUN cat > /etc/profile.d/99-grove-g1-env.sh <<'EOF'
export ROS_DOMAIN_ID="1"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export CYCLONEDDS_URI="file:///etc/cyclonedds/cyclonedds.xml"
EOF

# The base image ships the pip package `em` (a terminal highlighter) in
# /usr/local, shadowing apt's python3-empy and breaking all rosidl generation
# ("cannot import name 'Interpreter' from 'em'"). Remove it; empy 3.3.4 from
# apt is what Humble's rosidl expects.
RUN python3 -m pip uninstall -y em

# Ensure default workspace exists (mounted from host: ./workspace -> /root/workspace)
RUN mkdir -p /root/workspace/src
RUN mkdir -p /root/data/

# Default working directory inside the container
WORKDIR /root/workspace
