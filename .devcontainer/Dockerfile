# Extends your pre-built image (tag is provided by docker-compose.yml via BASE_IMAGE)

ARG BASE_IMAGE=adyansh04/ros-dev:jazzy
FROM ${BASE_IMAGE}

# Pinned explicitly rather than inherited from the base image ENV: every `ros-${ROS_DISTRO}-*`
# apt name below resolves through it, and an unset value would silently install nothing.
ARG ROS_DISTRO=jazzy

# Allow pip installs in distro-managed Python images (required on Ubuntu 24.04+/ROS Jazzy).
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# ============================================================================
# EDITOR TOOLING
# These are pre-configured tools for C++ and Python development.
# ============================================================================
# ruff and clang-format are PINNED and .github/ci.Dockerfile pins the same versions; ci.yml
# fails if they drift. Neither tool's rule set is stable across versions.
ARG RUFF_VERSION=0.16.1
ARG LLVM_VERSION=18
RUN apt-get update && apt-get install -y \
        clangd-${LLVM_VERSION} clang-format-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${LLVM_VERSION} 100 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 100 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${LLVM_VERSION} 100 && \
    update-alternatives --install /usr/bin/run-clang-tidy run-clang-tidy /usr/bin/run-clang-tidy-${LLVM_VERSION} 100 && \
    (python3 -m pip install --no-cache-dir --break-system-packages ruff==${RUFF_VERSION} || \
     python3 -m pip install --no-cache-dir ruff==${RUFF_VERSION}) && \
    rm -rf /var/lib/apt/lists/*

# ============================================================================
# PROJECT-SPECIFIC CUSTOMIZATION
# ============================================================================

# ros2_control plus unitree_sdk2/mujoco build deps. Mesa GL is for headless Xvfb.
RUN apt-get update && apt-get install -y \
        libboost-all-dev \
        libeigen3-dev \
        libfmt-dev \
        libgl1-mesa-dri \
        libglfw3-dev \
        libglx-mesa0 \
        libspdlog-dev \
        libyaml-cpp-dev \
        ros-${ROS_DISTRO}-launch-testing \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-realtime-tools \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-ros2launch \
        ros-${ROS_DISTRO}-xacro \
        xvfb \
    && rm -rf /var/lib/apt/lists/*

# Compilation parallelism for every source build below. NOT $(nproc): heavy C++ here runs
# 0.5-1 GB per cc1plus, so one job per core OOM-kills a 32-core/30 GB machine mid-build --
# observed, twice, taking the desktop session with it. One job per ~2 GB of RAM is the rule.
# Raise on a bigger box with --build-arg BUILD_JOBS=N.
ARG BUILD_JOBS=8

# --- unitree_sdk2 -----------------------------------------------------------
# Source tree discarded after install. Pinned with UNITREE_MUJOCO_SHA as a matched pair.
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
    make -j"${BUILD_JOBS}" && \
    make install && \
    rm -rf /opt/unitree_robotics/unitree_sdk2

# --- MuJoCo ------------------------------------------------------------------
# The full tarball, not the wheel: unitree_mujoco compiles the viewer sources directly.
ARG MUJOCO_VERSION=3.3.6
RUN mkdir -p /opt/unitree_robotics/mujoco-${MUJOCO_VERSION} && \
    curl -fsSL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
      | tar -xz --strip-components=1 -C /opt/unitree_robotics/mujoco-${MUJOCO_VERSION}


# --- CycloneDDS / ROS domain (unconditional -- required by Unitree SDK) -----
# Pin the SDK's DDS discovery to loopback so the hardware component and unitree_mujoco find
# each other. ROS itself is on fastrtps and ignores this file.
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

# Second profile, for the robot. One difference: the interface. CycloneDDS expands
# ${VAR:-default} itself. Loopback keeps the simulator's rt/lowcmd off the LAN.
RUN cat > /etc/cyclonedds/cyclonedds.hardware.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="${GROVE_G1_ROBOT_NIC:-eth0}" priority="default" multicast="default" />
      </Interfaces>
    </General>
    <Discovery>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# ROS runs fastrtps and never loads a CycloneDDS RMW: unitree_sdk2 links its own CycloneDDS
# inside the hardware component, and two libddsc.so.0 of different ABIs corrupt the heap.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Still set: the SDK and unitree_mujoco read it directly, which is what pins them to loopback.
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
# Sim-only domain; the robot's is set through GROVE_G1_ROS_DOMAIN_ID below.
ENV ROS_DOMAIN_ID=1

# Puts back the ROS_DOMAIN_ID that the base image's 10-ros-env.sh overwrites with 0. Read from
# the GROVE_G1_-prefixed copy: the unprefixed name would read the 0 just written.
RUN cat > /etc/profile.d/99-grove-g1-env.sh <<'EOF'
export ROS_DOMAIN_ID="${GROVE_G1_ROS_DOMAIN_ID:-1}"
EOF

# rosidl needs empy 3.3.4 (apt python3-empy); pip's `em` and `empy` 4.x both shadow it from
# /usr/local, and 4.x's rewritten API throws TransientParseError while expanding templates.
RUN python3 -m pip uninstall -y em empy

# --- Milestone 2: LocoClient deps -------------------------------------------

# nlohmann-json: used for the LocoClient wire format (matches vendor's client).
RUN apt-get update && apt-get install -y nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Milestone 3: walking-policy deps ---------------------------------------

# ONNX Runtime CPU tarball. ldconfig (not RPATH) avoids LD_LIBRARY_PATH shadowing
# by ROS's /opt/ros/${ROS_DISTRO}/lib.
ARG ONNXRUNTIME_VERSION=1.20.1
RUN mkdir -p /opt/onnxruntime && \
    curl -fsSL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz" \
      | tar -xz --strip-components=1 -C /opt/onnxruntime && \
    echo /opt/onnxruntime/lib > /etc/ld.so.conf.d/onnxruntime.conf && \
    ldconfig

# teleop_twist_keyboard for manual velocity testing via ~/cmd_vel.
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# --- Milestone 4: perception sim track ---------------------------------------

# mujoco_ros2_control, the second simulation track (sensors + a simplified mobility body).
# Pinned to main, not a release: only main has the 3D PointCloud2 lidar the Mid360 needs.
ARG MUJOCO_ROS2_CONTROL_SHA=ae0b235affe8792cf8584cb7074e10038f26f57b
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-mujoco-vendor \
        ros-${ROS_DISTRO}-ros2-control-cmake \
        ros-${ROS_DISTRO}-backward-ros \
        ros-${ROS_DISTRO}-control-toolbox \
        ros-${ROS_DISTRO}-transmission-interface \
        ros-${ROS_DISTRO}-tinyxml2-vendor \
        ros-${ROS_DISTRO}-velocity-controllers \
    && rm -rf /var/lib/apt/lists/*

# demos/_tests skipped: extra deps and example robots we do not use.
RUN mkdir -p /opt/mujoco_ros2_control_ws/src && \
    git init -q /opt/mujoco_ros2_control_ws/src/mujoco_ros2_control && \
    cd /opt/mujoco_ros2_control_ws/src/mujoco_ros2_control && \
    git remote add origin https://github.com/ros-controls/mujoco_ros2_control.git && \
    git fetch --depth 1 origin ${MUJOCO_ROS2_CONTROL_SHA} && \
    git checkout -q FETCH_HEAD && \
    rm -rf .git && \
    cd /opt/mujoco_ros2_control_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    MAKEFLAGS=-j${BUILD_JOBS} colcon build --merge-install --parallel-workers 1 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-skip mujoco_ros2_control_demos mujoco_ros2_control_tests && \
    rm -rf build log src

# Overlay it for every shell. 99- already sorts after the base image's 10-ros-env.sh.
RUN printf '. /opt/mujoco_ros2_control_ws/install/setup.sh\n' \
      >> /etc/profile.d/99-grove-g1-env.sh

# --- Milestone 6: SLAM + Nav2 -------------------------------------------------

# Three of these four already ship in the base image; listed anyway, because the base tag
# floats and a silent removal would surface as a missing launch file. pointcloud-to-laserscan
# is the one genuinely absent -- slam_toolbox consumes LaserScan only.
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# --- Milestone 7: MoveIt 2 (structured manipulation) --------------------------

# Humble's 2.5.9. Each extra past the metapackage is deliberate: pick-ik replaces KDL for the
# 7-DoF arms, moveit-ros-perception carries the octomap updater plugins the metapackage omits,
# realsense2-description supplies the D435's real geometry. moveit-servo is deliberately absent.
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-configs-utils \
        ros-${ROS_DISTRO}-moveit-setup-assistant \
        ros-${ROS_DISTRO}-pick-ik \
        ros-${ROS_DISTRO}-moveit-ros-perception \
        ros-${ROS_DISTRO}-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# --- Milestone 9: orchestration + manipulation skills -------------------------

# behaviortree-cpp is v4 and coexists with the v3 Nav2 pulls in: disjoint files, disjoint
# include roots, and nothing links both. vision-msgs carries the Detection3DArray that
# g1_manipulation publishes. twist-mux is deliberately absent -- its Humble build is broken.
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-behaviortree-cpp \
        ros-${ROS_DISTRO}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# behaviortree_cpp 4.9.1 installs under lib/<triplet>/ while its own CMake export looks in
# lib/, so dependants fail to configure. Guarded so a fixed release keeps its real file.
RUN test -e /opt/ros/${ROS_DISTRO}/lib/libbehaviortree_cpp.so || \
    ln -s "$(dpkg-architecture -qDEB_HOST_MULTIARCH)/libbehaviortree_cpp.so" \
          /opt/ros/${ROS_DISTRO}/lib/libbehaviortree_cpp.so

# --- Milestone 10: LiDAR-inertial odometry ------------------------------------

# Livox-SDK2, the Mid360's transport library. livox_ros_driver2 does find_library(... REQUIRED)
# for it, and that driver defines the CustomMsg both FAST-LIO and the sim bridge speak -- so it
# is needed even on the sim track, where no LiDAR is ever opened.
ARG LIVOX_SDK2_SHA=08f523c930b2f0ba1e98a6afaa8d7476bf479908
RUN git init -q /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    git remote add origin https://github.com/Livox-SDK/Livox-SDK2.git && \
    git fetch --depth 1 origin ${LIVOX_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j"${BUILD_JOBS}" && \
    make install && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# --- unitree_mujoco (the `simulate` app) ------------------------------------
# LAST on purpose: the only block consuming COPYed host files, and cache invalidation flows
# forward, so a one-line sensor edit here must not re-run every apt block above it.
#
# The source tree stays in the image -- the binary resolves assets relative to its own
# location. Only the `unitree_mujoco` target is built; jstest fails against the current SDK.
COPY workspace/patches/unitree_mujoco/ /tmp/unitree_mujoco_patches/
COPY workspace/vendor/unitree_mujoco/ /tmp/unitree_mujoco_src/
ARG UNITREE_MUJOCO_SHA=ae6a8403e272733e9996ef59990880330496177f
RUN git init -q /opt/unitree_robotics/unitree_mujoco && \
    cd /opt/unitree_robotics/unitree_mujoco && \
    git remote add origin https://github.com/unitreerobotics/unitree_mujoco.git && \
    git fetch --depth 1 origin ${UNITREE_MUJOCO_SHA} && \
    git checkout -q FETCH_HEAD && \
    for p in /tmp/unitree_mujoco_patches/*.patch; do \
        [ -e "$p" ] || continue; \
        echo "Applying $(basename "$p")"; \
        git apply --check "$p" || { echo "PATCH DOES NOT APPLY: $p"; exit 1; }; \
        git apply "$p"; \
    done && \
    cp /tmp/unitree_mujoco_src/*.h /tmp/unitree_mujoco_src/*.cc simulate/src/ && \
    rm -rf .git && \
    rm -rf simulate/mujoco && \
    mv /opt/unitree_robotics/mujoco-${MUJOCO_VERSION} simulate/mujoco && \
    mkdir simulate/build && cd simulate/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_PREFIX_PATH=/opt/unitree_robotics/lib/cmake && \
    cmake --build . --target unitree_mujoco --parallel "${BUILD_JOBS}" && \
    rm -rf CMakeFiles && \
    rm -rf ../mujoco/sample ../mujoco/bin ../mujoco/doc ../mujoco/model
# include/ and simulate/ are kept deliberately: we compile our own sensor source into this app
# and CMakeLists globs simulate/*.cc, so stripping them costs a full rebuild per edit. The bulk
# (model/bin/doc/sample) is still removed. Binary at simulate/build/unitree_mujoco.

# --- editor configs -----------------------------------------------------------
# At /root, NOT /root/workspace: the ./workspace bind mount replaces that directory wholesale
# at runtime. These tools all search ancestor directories, so they are found from anywhere
# under it. Symlinks dangle at build time -- /root/.host-repo is a runtime mount.
RUN ln -sf /root/.host-repo/.clangd       /root/.clangd && \
    ln -sf /root/.host-repo/.clang-format /root/.clang-format && \
    ln -sf /root/.host-repo/.clang-tidy   /root/.clang-tidy && \
    ln -sf /root/.host-repo/ruff.toml     /root/ruff.toml

# Ensure default workspace exists (mounted from host: ./workspace -> /root/workspace)
RUN mkdir -p /root/workspace/src
RUN mkdir -p /root/data/

# Default working directory inside the container
WORKDIR /root/workspace
