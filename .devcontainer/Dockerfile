# Extends your pre-built image (tag is provided by docker-compose.yml via BASE_IMAGE)

ARG BASE_IMAGE=adyansh04/ros-dev:humble
FROM ${BASE_IMAGE}

# Allow pip installs in distro-managed Python images (required on Ubuntu 24.04+/ROS Jazzy).
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# ============================================================================
# EDITOR TOOLING
# These are pre-configured tools for C++ and Python development.
# ============================================================================
RUN apt-get update && apt-get install -y clangd-14 clang-format-14 clang-tidy-14 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-14 100 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-14 100 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-14 100 && \
    (python3 -m pip install --no-cache-dir --break-system-packages ruff || \
     python3 -m pip install --no-cache-dir ruff) && \
    rm -rf /var/lib/apt/lists/*

# ============================================================================
# PROJECT-SPECIFIC CUSTOMIZATION
# G1 sim-first foundation: ros2_control + Unitree SDK / unitree_mujoco
# bring-up deps: everything needed to build and run unitree_sdk2 and
# unitree_mujoco from source, installed in the layer below.
# ============================================================================

# ros2_control, unitree_sdk2/mujoco build deps. Mesa GL for headless Xvfb;
# libeigen3-dev/libfmt-dev are transitive deps of unitree_mujoco.
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
# Build and install unitree_sdk2; source tree discarded after install.
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
# Full tarball needed — unitree_mujoco compiles MuJoCo viewer sources directly.
# Pinned 2026-07-17 — the release unitree_mujoco's current tip's README
# instructs extracting (mujoco-3.3.6).
ARG MUJOCO_VERSION=3.3.6
RUN mkdir -p /opt/unitree_robotics/mujoco-${MUJOCO_VERSION} && \
    curl -fsSL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
      | tar -xz --strip-components=1 -C /opt/unitree_robotics/mujoco-${MUJOCO_VERSION}

# --- unitree_mujoco (the `simulate` app) ------------------------------------
# unitree_mujoco — DDS-based sim binary. Source tree stays in image (binary resolves
# assets relative to its own location). Only the `unitree_mujoco` target is built
# (jstest fails against current unitree_sdk2 tip — unrelated upstream rename).
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
# Binary at simulate/build/unitree_mujoco; g1_bringup overrides config via CLI flags.

# --- CycloneDDS / ROS domain (unconditional -- required by Unitree SDK) -----
# Pin DDS discovery to loopback so ROS and unitree_mujoco find each other.
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
    <Discovery>
      <!-- Default cap of 9 is too low for the multi-node sim stack; raise it. -->
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
# Sim-only domain; revisit at hardware bring-up.
ENV ROS_DOMAIN_ID=1

# Override base image's /etc/profile.d/10-ros-env.sh which resets ROS_DOMAIN_ID=0.
RUN cat > /etc/profile.d/99-grove-g1-env.sh <<'EOF'
export ROS_DOMAIN_ID="1"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export CYCLONEDDS_URI="file:///etc/cyclonedds/cyclonedds.xml"
EOF

# Remove pip `em` — it shadows apt's python3-empy and breaks rosidl generation.
RUN python3 -m pip uninstall -y em

# --- Milestone 2: LocoClient deps -------------------------------------------

# nlohmann-json: used for the LocoClient wire format (matches vendor's client).
RUN apt-get update && apt-get install -y nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Milestone 3: walking-policy deps ---------------------------------------

# ONNX Runtime CPU tarball. ldconfig (not RPATH) avoids LD_LIBRARY_PATH shadowing
# by ROS's /opt/ros/humble/lib.
ARG ONNXRUNTIME_VERSION=1.20.1
RUN mkdir -p /opt/onnxruntime && \
    curl -fsSL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz" \
      | tar -xz --strip-components=1 -C /opt/onnxruntime && \
    echo /opt/onnxruntime/lib > /etc/ld.so.conf.d/onnxruntime.conf && \
    ldconfig

# teleop_twist_keyboard for manual velocity testing via ~/cmd_vel.
RUN apt-get update && apt-get install -y ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# --- Milestone 4: perception sim track ---------------------------------------

# ros-controls/mujoco_ros2_control, built from source into its own overlay. This is the
# SECOND simulation track (sensors + a simplified mobility body); unitree_mujoco is
# untouched and stays the source of truth for arm/locomotion fidelity.
#
# Pinned 2026-08-02 to main, NOT to a release tag. Release 0.0.3 is apt-installable but
# ships no 3D lidar -- only a 2D LaserScan -- which does not match the robot's Mid360.
# The 3D PointCloud2 lidar exists only on main. Risk accepted deliberately: main is
# unreleased and was recently restructured, so this SHA is a snapshot, not a moving target.
# Verified to build on Humble at this SHA before pinning.
ARG MUJOCO_ROS2_CONTROL_SHA=ae0b235affe8792cf8584cb7074e10038f26f57b
RUN apt-get update && apt-get install -y \
        ros-humble-mujoco-vendor \
        ros-humble-ros2-control-cmake \
        ros-humble-backward-ros \
        ros-humble-control-toolbox \
        ros-humble-transmission-interface \
        ros-humble-tinyxml2-vendor \
        ros-humble-velocity-controllers \
    && rm -rf /var/lib/apt/lists/*

# mujoco_ros2_control_demos/_tests are skipped: they pull extra deps and ship example
# robots we don't use. The mujoco_3d_lidar extension registers itself under the
# ament resource index key "mujoco_plugins", which the node scans at startup via
# ament_index_cpp -- so no MUJOCO_PLUGIN_DIR is needed (and it does not work anyway).
RUN mkdir -p /opt/mujoco_ros2_control_ws/src && \
    git init -q /opt/mujoco_ros2_control_ws/src/mujoco_ros2_control && \
    cd /opt/mujoco_ros2_control_ws/src/mujoco_ros2_control && \
    git remote add origin https://github.com/ros-controls/mujoco_ros2_control.git && \
    git fetch --depth 1 origin ${MUJOCO_ROS2_CONTROL_SHA} && \
    git checkout -q FETCH_HEAD && \
    rm -rf .git && \
    cd /opt/mujoco_ros2_control_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-skip mujoco_ros2_control_demos mujoco_ros2_control_tests && \
    rm -rf build log src

# Overlay it for every shell. 99- already sorts after the base image's 10-ros-env.sh.
RUN printf '. /opt/mujoco_ros2_control_ws/install/setup.sh\n' \
      >> /etc/profile.d/99-grove-g1-env.sh

# Ensure default workspace exists (mounted from host: ./workspace -> /root/workspace)
RUN mkdir -p /root/workspace/src
RUN mkdir -p /root/data/

# Default working directory inside the container
WORKDIR /root/workspace
