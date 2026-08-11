# Extends your pre-built image (tag is provided by docker-compose.yml via BASE_IMAGE)

ARG BASE_IMAGE=adyansh04/ros-dev:humble
FROM ${BASE_IMAGE}

# Allow pip installs in distro-managed Python images (required on Ubuntu 24.04+/ROS Jazzy).
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# ============================================================================
# EDITOR TOOLING
# These are pre-configured tools for C++ and Python development.
# ============================================================================
# ruff is PINNED, and .github/ci.Dockerfile pins the same version. Unpinned, every rebuild of
# either image picks up whatever ruff released that week, and the two then disagree about which
# rules exist -- 0.16 raises B905 on code 0.13 accepts. CI is what would fail, on a rule the
# developer's own colcon test never mentioned. ci.yml checks the two pins still match.
ARG RUFF_VERSION=0.16.1
RUN apt-get update && apt-get install -y clangd-14 clang-format-14 clang-tidy-14 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-14 100 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-14 100 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-14 100 && \
    (python3 -m pip install --no-cache-dir --break-system-packages ruff==${RUFF_VERSION} || \
     python3 -m pip install --no-cache-dir ruff==${RUFF_VERSION}) && \
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

# Second profile, for the robot. Same file, one difference: the interface. Loopback is what
# keeps the simulator's rt/lowcmd and rt/arm_sdk off the LAN; on the robot they have to reach the
# onboard controller, so the NIC is named instead. CycloneDDS expands ${VAR:-default} itself.
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

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
# Sim-only domain; the robot's is set through GROVE_G1_ROS_DOMAIN_ID below.
ENV ROS_DOMAIN_ID=1

# Puts back what the base image's /etc/profile.d/10-ros-env.sh has just overwritten: it exports
# ROS_DOMAIN_ID=0 unconditionally and wins by ordering. Only that one variable, and read from the
# GROVE_G1_-prefixed copy compose also sets -- reading the unprefixed name here would read the 0
# that was just written. RMW_IMPLEMENTATION and CYCLONEDDS_URI are not touched by that script, so
# the container environment stands.
RUN cat > /etc/profile.d/99-grove-g1-env.sh <<'EOF'
export ROS_DOMAIN_ID="${GROVE_G1_ROS_DOMAIN_ID:-1}"
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

# --- Milestone 6: SLAM + Nav2 -------------------------------------------------

# navigation2, nav2-bringup and slam-toolbox already ship in the base image, so three of
# these four are no-ops today. They are listed anyway: an undocumented base-image inclusion
# is a dependency we neither declare nor control, and BASE_IMAGE_TAG_SUFFIX floats, so a
# future base rebuild could drop them and the failure would surface as a missing launch file
# rather than a missing package.
#
# pointcloud-to-laserscan is the one that is genuinely absent. slam_toolbox is a 2D scan
# matcher and consumes LaserScan only, so flattening /livox/lidar is mandatory for mapping --
# the Nav2 costmaps take the PointCloud2 directly and do not go through it.
RUN apt-get update && apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# --- Milestone 7: MoveIt 2 (structured manipulation) --------------------------

# Humble's stable line, 2.5.9. The metapackage already covers core, OMPL, the simple
# controller manager and the RViz MotionPlanning display; the two extras below are named
# for specific reasons rather than habit. moveit-configs-utils otherwise arrives only as a
# transitive dependency of moveit-ros-benchmarks, which is not a chain to hang every launch
# file off. moveit-setup-assistant ships the collisions_updater binary, kept for
# regenerating g1_moveit_config's self-collision matrix -- though note that tool does not
# finish on this model at any trial count (g1_moveit_config/README.md), so the shipped matrix
# came from a different route. Upstream has also been reshuffling that package across releases.
#
# pick-ik replaces the default KDL solver because the arms are 7-DoF against a 6-DoF pose;
# see g1_moveit_config/config/kinematics.yaml. moveit-servo is deliberately absent: later
# milestone, and nothing streams to it yet.
#
# moveit-ros-perception carries BOTH octomap updater plugins, and the moveit metapackage does
# NOT pull it in -- only the occupancy-map-monitor framework arrives, which is why move_group
# warns about octomap resolution while having no plugin to load. Without this the sensor config
# fails as a pluginlib load error rather than anything that names the missing package.
#
# realsense2-description is Intel's own D435 model. The vendored G1 URDF leaves d435_link and
# mid360_link as empty links, so both sensors are invisible; this supplies the camera's real
# geometry instead of a guessed box. Visual only -- see g1_arm_sdk.urdf.xacro for why nothing
# here gets collision geometry.
RUN apt-get update && apt-get install -y \
        ros-humble-moveit \
        ros-humble-moveit-configs-utils \
        ros-humble-moveit-setup-assistant \
        ros-humble-pick-ik \
        ros-humble-moveit-ros-perception \
        ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# --- Milestone 9: orchestration + manipulation skills -------------------------

# behaviortree-cpp is v4 (4.9.0 on Humble). Nav2 depends on the SEPARATE
# behaviortree-cpp-v3 package and keeps it: the two install side by side, with disjoint files
# (bt4_* vs bt3_* binaries, libbehaviortree_cpp.so vs libbehaviortree_cpp_v3.so) and disjoint
# include roots. Verified here rather than assumed, because v4's deb did once collide with v3's
# over /opt/ros/humble/bin/bt3_log_cat (BehaviorTree.CPP issue #734) -- that is long fixed, and
# the two now coexist by design.
#
# Nothing links both: bt_navigator links v3 inside its own process, g1_orchestration links v4
# inside its own, and they only meet over the NavigateToPose action.
#
# btcpp-ros2 (BehaviorTree.ROS2) is NOT released for Humble, which is why g1_orchestration
# hand-rolls its one action-client node base rather than vendoring that repo for four leaves.
#
# vision-msgs carries Detection3DArray, the object-pose interface g1_manipulation publishes so
# real perception can later stand behind it unchanged.
#
# twist-mux is deliberately NOT here, though /cmd_vel does now have two writers and a mux is the
# standard answer. ros-humble-twist-mux 4.3.0 links libdiagnostic_updater.so, and the only
# ros-humble-diagnostic-updater this apt snapshot carries (4.0.6) is Python-only and ships no
# such library, so the binary dies at load with exit 127 and there is no second version to pin.
# g1_gait_shaper arbitrates its own two inputs instead; see its node docstring.
RUN apt-get update && apt-get install -y \
        ros-humble-behaviortree-cpp \
        ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# behaviortree_cpp 4.9.1 installs its library under lib/x86_64-linux-gnu/ while its own
# ament_cmake_export_libraries-extras.cmake still does find_library(... PATHS
# "${behaviortree_cpp_DIR}/../../../lib" NO_DEFAULT_PATH). Any package that find_package()s it
# then fails to CONFIGURE with "exports the library 'behaviortree_cpp' which couldn't be found".
# 4.9.0 installed to lib/ and did not have this. Guarded rather than unconditional so a fixed
# release does not get a symlink over the top of its real file.
RUN test -e /opt/ros/humble/lib/libbehaviortree_cpp.so || \
    ln -s "$(dpkg-architecture -qDEB_HOST_MULTIARCH)/libbehaviortree_cpp.so" \
          /opt/ros/humble/lib/libbehaviortree_cpp.so

# --- Milestone 10: LiDAR-inertial odometry ------------------------------------

# Livox-SDK2, the Mid360's transport library. livox_ros_driver2's CMakeLists does a
# find_library(... /usr/local/lib REQUIRED) for it, so the driver does not configure without
# it -- and the driver is what defines CustomMsg, which both FAST-LIO and the simulator
# bridge speak. So this is needed even on the sim track, where no LiDAR is ever opened.
#
# Same source-build-then-discard shape as unitree_sdk2 above. Installs to /usr/local, which
# is where the driver looks and is not searched by anything ROS ships.
# Pinned 2026-08-09 -- current tip at authoring time.
ARG LIVOX_SDK2_SHA=08f523c930b2f0ba1e98a6afaa8d7476bf479908
RUN git init -q /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    git remote add origin https://github.com/Livox-SDK/Livox-SDK2.git && \
    git fetch --depth 1 origin ${LIVOX_SDK2_SHA} && \
    git checkout -q FETCH_HEAD && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# Placed last on purpose. This is the only block that consumes COPYed host files
# (workspace/patches, workspace/vendor), and cache invalidation flows forward: sitting
# earlier, a one-line patch or sensor-source edit re-ran every apt block, the ONNX
# Runtime download and the mujoco_ros2_control colcon build below it. Its own two
# prerequisites -- unitree_sdk2 (CMAKE_PREFIX_PATH) and the MuJoCo tarball (moved into
# simulate/mujoco) -- stay above. Trade-off: bumping MUJOCO_ROS2_CONTROL_SHA or the nav2
# apt list now rebuilds this layer too, which is the rarer edit.
# --- unitree_mujoco (the `simulate` app) ------------------------------------
# unitree_mujoco — DDS-based sim binary. Source tree stays in image (binary resolves
# assets relative to its own location). Only the `unitree_mujoco` target is built
# (jstest fails against current unitree_sdk2 tip — unrelated upstream rename).
# Pinned 2026-07-17 — current tip of unitree_mujoco at authoring time,
# resolved together with UNITREE_SDK2_SHA above.
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
    cmake --build . --target unitree_mujoco --parallel "$(nproc)" && \
    rm -rf CMakeFiles && \
    rm -rf ../mujoco/sample ../mujoco/bin ../mujoco/doc ../mujoco/model
# include/ and simulate/ are deliberately kept (they used to be stripped here). Milestone 5
# compiles our own sensor source into this app, and keeping them is what makes an
# in-container rebuild possible -- CMakeLists globs mujoco/simulate/*.cc, so without them
# every one-line change costs a full image rebuild. They are small; the bulk was
# model/bin/doc/sample, still removed.
# Binary at simulate/build/unitree_mujoco; g1_bringup overrides config via CLI flags.

# --- editor configs -----------------------------------------------------------
# These live at /root, NOT /root/workspace: the ./workspace bind mount replaces that
# directory wholesale at runtime, so anything baked inside it is invisible (same reason
# the two mkdirs below are no-ops under compose). clangd, clang-format, clang-tidy and
# ruff all search ANCESTOR directories, so a tool run anywhere under /root/workspace
# walks up to /root and finds these.
#
# They replace four single-file bind mounts that targeted paths inside /root/workspace.
# Those pinned the host inode, so an editor's write-temp-then-rename left the container
# reading a detached file until a recreate; and because the targets were nested inside
# another mount, Docker materialised empty root-owned placeholders on the host inside
# ./workspace/, which then shadowed the real configs for host-side tools.
#
# Dangling at build time is expected -- /root/.host-repo is the runtime repo-root mount.
RUN ln -sf /root/.host-repo/.clangd       /root/.clangd && \
    ln -sf /root/.host-repo/.clang-format /root/.clang-format && \
    ln -sf /root/.host-repo/.clang-tidy   /root/.clang-tidy && \
    ln -sf /root/.host-repo/ruff.toml     /root/ruff.toml

# Ensure default workspace exists (mounted from host: ./workspace -> /root/workspace)
RUN mkdir -p /root/workspace/src
RUN mkdir -p /root/data/

# Default working directory inside the container
WORKDIR /root/workspace
