# Build-and-test image for CI. NOT the development environment -- .devcontainer/Dockerfile is,
# and at 19 GB a runner cannot pull it. This carries only what the packages link against.
#
# Absent by design: MuJoCo, CUDA, Xvfb, the unitree_mujoco tree. Anything under
# `ctest -L simulator` cannot run here. RViz arrives anyway, as a dependency of the moveit and
# navigation2 metapackages.
#
# The version ARGs below must match .devcontainer/Dockerfile; ci.yml fails the build if they
# drift, because CI on different versions tests a configuration nobody ships.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# clang-format's output is not stable across major versions, so an unpinned one rejects
# correctly formatted files. The same pin covers clang-tidy, whose check set and fix-its also
# move between releases -- and it has to match the dev container's 14, or the clang_tidy_check_*
# tests disagree with what a developer reproduces locally. liburdfdom-tools is check_urdf, which
# g1_description's xacro test shells out to.
ARG LLVM_VERSION=14
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ccache \
        clang-format-${LLVM_VERSION} \
        clang-tidy-${LLVM_VERSION} \
        cmake \
        curl \
        gcovr \
        git \
        liburdfdom-tools \
        python3-colcon-common-extensions \
        python3-pip \
        python3-vcstool \
    && ln -sf /usr/bin/clang-format-${LLVM_VERSION} /usr/bin/clang-format \
    && ln -sf /usr/bin/clang-tidy-${LLVM_VERSION} /usr/bin/clang-tidy \
    && rm -rf /var/lib/apt/lists/*

# Pinned: ruff's rule set changes between minor versions.
ARG RUFF_VERSION=0.16.1
RUN python3 -m pip install --no-cache-dir ruff==${RUFF_VERSION}

# libboost/libeigen are unitree_sdk2's, PCL is FAST-LIO's.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libboost-all-dev \
        libeigen3-dev \
        libfmt-dev \
        libpcl-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS packages the workspace builds against; ros-base carries none of them.
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

# behaviortree_cpp 4.9.1 installs under lib/<triplet>/ while its own CMake export looks in
# lib/, so dependants fail to configure. Guarded so a fixed release keeps its real file.
RUN test -e /opt/ros/humble/lib/libbehaviortree_cpp.so || \
    ln -s "$(dpkg-architecture -qDEB_HOST_MULTIARCH)/libbehaviortree_cpp.so" \
          /opt/ros/humble/lib/libbehaviortree_cpp.so

# Compilation parallelism for the three source builds below. Sized for the GitHub runner that
# builds this image, not for a workstation: heavy C++ here runs 0.5-1 GB per cc1plus, and the
# rule that keeps it off the OOM killer is one job per ~2 GB of RAM. Raise with
# --build-arg BUILD_JOBS=N on a bigger box.
ARG BUILD_JOBS=3

# --- unitree_sdk2 -------------------------------------------------------------------------
ARG UNITREE_SDK2_SHA=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git init -q /tmp/unitree_sdk2 && \
    cd /tmp/unitree_sdk2 && \
    git remote add origin https://github.com/unitreerobotics/unitree_sdk2.git && \
    git fetch --depth 1 origin ${UNITREE_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && \
    make -j"${BUILD_JOBS}" install && \
    ldconfig && \
    rm -rf /tmp/unitree_sdk2

# --- Livox-SDK2 ---------------------------------------------------------------------------
# livox_ros_driver2 does find_library(... REQUIRED) for it, and that driver defines the
# CustomMsg both FAST-LIO and g1_sensor_relay speak.
ARG LIVOX_SDK2_SHA=08f523c930b2f0ba1e98a6afaa8d7476bf479908
RUN git init -q /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    git remote add origin https://github.com/Livox-SDK/Livox-SDK2.git && \
    git fetch --depth 1 origin ${LIVOX_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j"${BUILD_JOBS}" install && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# --- ONNX Runtime -------------------------------------------------------------------------
# g1_motion_service_sim links it for the walking policy. CPU build only.
ARG ONNXRUNTIME_VERSION=1.20.1
RUN mkdir -p /opt/onnxruntime && \
    curl -fsSL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz" \
      | tar -xz --strip-components=1 -C /opt/onnxruntime && \
    echo /opt/onnxruntime/lib > /etc/ld.so.conf.d/onnxruntime.conf && \
    ldconfig

# Loopback DDS: a runner binding its real NIC would put a test's rt/lowcmd on GitHub's network.
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
