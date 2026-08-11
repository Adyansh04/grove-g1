# Build-and-test image for CI. NOT the development environment.
#
# .devcontainer/Dockerfile is what you develop in and what the simulator needs; it derives from
# an 18 GB ROS desktop base and adds MuJoCo, CUDA tooling, RViz and a full GUI stack. A GitHub
# runner cannot pull that, and none of it is needed to answer the only question CI asks: does
# the workspace still compile, and do the tests that need no simulator still pass?
#
# So this starts from ros:humble-ros-base and adds only what the packages link against. What is
# deliberately absent: MuJoCo, ONNX's GPU providers, RViz, Xvfb, clangd, and the whole
# unitree_mujoco vendor tree. Anything under `ctest -L simulator` cannot run here, by design --
# see g1_bringup/CMakeLists.txt for why those stay off shared runners.
#
# THE SHAs BELOW MUST MATCH .devcontainer/Dockerfile. They are the versions the workspace is
# actually built against; a CI image on different ones would test a configuration nobody runs.
# .github/workflows/ci.yml checks that they agree and fails if they drift.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Toolchain, plus the lint binaries the workspace's own ctest lint targets look for. Version 14
# is pinned to match the dev image: clang-format's output is not stable across major versions,
# so an unpinned one would fail files that are correctly formatted locally.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ccache \
        clang-format-14 \
        cmake \
        curl \
        git \
        python3-colcon-common-extensions \
        python3-pip \
        python3-vcstool \
    && ln -sf /usr/bin/clang-format-14 /usr/bin/clang-format \
    && rm -rf /var/lib/apt/lists/*

# Must match .devcontainer/Dockerfile's pin: ruff's rule set changes between minor versions, so
# a different one here fails the workspace's own ruff_check_* targets on rules the developer's
# container never raised. ci.yml enforces that the two agree.
ARG RUFF_VERSION=0.16.1
RUN python3 -m pip install --no-cache-dir ruff==${RUFF_VERSION}

# Library dependencies, mirroring the dev image's list minus everything GUI. libboost and
# libeigen are unitree_sdk2's; PCL is FAST-LIO's.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libboost-all-dev \
        libeigen3-dev \
        libfmt-dev \
        libpcl-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS packages the workspace builds against. ros-base carries none of these.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ament-cmake-gmock \
        ros-humble-ament-cmake-pytest \
        ros-humble-ament-lint-auto \
        ros-humble-ament-lint-common \
        ros-humble-behaviortree-cpp \
        ros-humble-launch-testing \
        ros-humble-launch-testing-ament-cmake \
        ros-humble-moveit \
        ros-humble-moveit-configs-utils \
        ros-humble-moveit-ros-perception \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-pcl-conversions \
        ros-humble-pcl-ros \
        ros-humble-pick-ik \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-realsense2-description \
        ros-humble-realtime-tools \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-slam-toolbox \
        ros-humble-vision-msgs \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Same multiarch workaround the dev image carries: behaviortree_cpp 4.9.1 installs its library
# under lib/<triplet>/ while its own CMake export still looks in lib/, so every package that
# find_package()s it fails to configure. Guarded, so a fixed release keeps its real file.
RUN test -e /opt/ros/humble/lib/libbehaviortree_cpp.so || \
    ln -s "$(dpkg-architecture -qDEB_HOST_MULTIARCH)/libbehaviortree_cpp.so" \
          /opt/ros/humble/lib/libbehaviortree_cpp.so

# --- unitree_sdk2 -------------------------------------------------------------------------
ARG UNITREE_SDK2_SHA=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git init -q /tmp/unitree_sdk2 && \
    cd /tmp/unitree_sdk2 && \
    git remote add origin https://github.com/unitreerobotics/unitree_sdk2.git && \
    git fetch --depth 1 origin ${UNITREE_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && \
    make -j"$(nproc)" install && \
    ldconfig && \
    rm -rf /tmp/unitree_sdk2

# --- Livox-SDK2 ---------------------------------------------------------------------------
# livox_ros_driver2's CMakeLists does find_library(... /usr/local/lib REQUIRED), so the driver
# does not configure without it -- and the driver is what defines CustomMsg, which both
# FAST-LIO and g1_sensor_relay speak.
ARG LIVOX_SDK2_SHA=08f523c930b2f0ba1e98a6afaa8d7476bf479908
RUN git init -q /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    git remote add origin https://github.com/Livox-SDK/Livox-SDK2.git && \
    git fetch --depth 1 origin ${LIVOX_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j"$(nproc)" install && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# --- ONNX Runtime -------------------------------------------------------------------------
# g1_motion_service_sim links it for the walking policy. CPU build only; nothing in CI infers.
ARG ONNXRUNTIME_VERSION=1.20.1
RUN mkdir -p /opt/onnxruntime && \
    curl -fsSL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz" \
      | tar -xz --strip-components=1 -C /opt/onnxruntime && \
    echo /opt/onnxruntime/lib > /etc/ld.so.conf.d/onnxruntime.conf && \
    ldconfig

# Loopback DDS, as the dev image does. CI never talks to a robot, and a runner binding its real
# NIC would put a test's rt/lowcmd on whatever network GitHub gave it.
RUN mkdir -p /etc/cyclonedds && \
    printf '%s\n' \
      '<?xml version="1.0" encoding="UTF-8" ?>' \
      '<CycloneDDS xmlns="https://cdds.io/config">' \
      '  <Domain id="any"><General><Interfaces>' \
      '    <NetworkInterface name="lo" priority="default" multicast="default" />' \
      '  </Interfaces></General></Domain>' \
      '</CycloneDDS>' > /etc/cyclonedds/cyclonedds.xml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
ENV ROS_DOMAIN_ID=1
ENV CMAKE_C_COMPILER_LAUNCHER=ccache
ENV CMAKE_CXX_COMPILER_LAUNCHER=ccache

WORKDIR /workspace
