# The runtime for the G1's Orin (arm64): ros2_control with the balance policy, the lidar and
# camera drivers, and the Zenoh router the operator's machine links to. It holds dependencies
# only; the robot-side packages are synced onto the Orin and built there, in this image, by
# scripts/robot.sh. Built on the PC under QEMU, since the Orin has no internet:
#
#   ./scripts/robot.sh image
FROM ros:jazzy-ros-base

ARG ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# ros2_control and the control packages, the unitree_sdk2 build deps, the lidar stack's PCL and
# APR, and the RealSense driver with the image codecs that compress for the link.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        libapr1-dev \
        libboost-all-dev \
        libeigen3-dev \
        libfmt-dev \
        libpcl-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        nlohmann-json3-dev \
        python3-colcon-common-extensions \
        python3-yaml \
        ros-${ROS_DISTRO}-control-toolbox \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-pointcloud-to-laserscan \
        ros-${ROS_DISTRO}-realsense2-camera \
        ros-${ROS_DISTRO}-realsense2-description \
        ros-${ROS_DISTRO}-realtime-tools \
        ros-${ROS_DISTRO}-rmw-zenoh-cpp \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Emulated compilers are slow and memory-hungry; the PC has the cores for this many.
ARG BUILD_JOBS=8

# Same pin as the dev image: the SDK ships its aarch64 build of the library and its CycloneDDS.
ARG UNITREE_SDK2_SHA=21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b
RUN git init -q /tmp/unitree_sdk2 && \
    cd /tmp/unitree_sdk2 && \
    git remote add origin https://github.com/unitreerobotics/unitree_sdk2.git && \
    git fetch --depth 1 origin ${UNITREE_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics -DBUILD_EXAMPLES=OFF && \
    cmake --build build -j"${BUILD_JOBS}" && \
    cmake --install build && \
    rm -rf /tmp/unitree_sdk2

# The walking policy's runtime, the dev image's version in its aarch64 build.
ARG ONNXRUNTIME_VERSION=1.20.1
RUN mkdir -p /opt/onnxruntime && \
    curl -fsSL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz" \
      | tar -xz --strip-components=1 -C /opt/onnxruntime && \
    echo /opt/onnxruntime/lib > /etc/ld.so.conf.d/onnxruntime.conf && \
    ldconfig

# Same pin as the dev image: livox_ros_driver2 links it.
ARG LIVOX_SDK2_SHA=08f523c930b2f0ba1e98a6afaa8d7476bf479908
RUN git init -q /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    git remote add origin https://github.com/Livox-SDK/Livox-SDK2.git && \
    git fetch --depth 1 origin ${LIVOX_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j"${BUILD_JOBS}" && \
    cmake --install build && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# ROS runs on Zenoh, as in the dev image: unitree_sdk2 brings its own CycloneDDS, and two of
# different ABIs in one process corrupt the heap.
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=1

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /root/workspace/install/setup.bash ] && source /root/workspace/install/setup.bash' \
      >> /root/.bashrc

WORKDIR /root/workspace
