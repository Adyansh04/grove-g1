#!/usr/bin/env bash
#
# The G1's on-robot runtime: builds its arm64 image on this machine, ships the image and the
# robot-side packages to the Orin over the robot's network, builds them there and runs them. The
# Orin needs no internet. Nothing here takes the motors: that is control.launch.py hardware:=true,
# started by hand with `exec`.
#
# Usage: ./scripts/robot.sh <command> [args]
#   image          build robot/Dockerfile for arm64 (QEMU) and load it into the Orin's Docker
#   load           load the already built image into the Orin's Docker
#   sync           copy the robot-side packages to ~/grove-g1/src on the Orin
#   build [pkg..]  colcon build them on the Orin, in the image (everything synced by default)
#   up | down      start or remove the idle runtime container on the Orin
#   exec [cmd]     a shell, or cmd, in that container, with a terminal so Ctrl-C reaches it
#   ssh            a shell on the Orin
#
# Environment: G1_ROBOT (default unitree@192.168.123.164), G1_ROBOT_KEY (default
# ~/.ssh/id_ed25519_robot).

set -euo pipefail

readonly ROOT="$(cd "$(dirname "$0")/.." && pwd)"
readonly ROBOT="${G1_ROBOT:-unitree@192.168.123.164}"
readonly KEY="${G1_ROBOT_KEY:-$HOME/.ssh/id_ed25519_robot}"
readonly IMAGE="grove-g1-robot:local"
readonly CONTAINER="g1-robot"
# The workspace on the Orin, mounted at /root/workspace in the container.
readonly REMOTE_WS="grove-g1"
# What runs on the robot, as paths under workspace/src: the hardware component, the controllers
# and their launch, and the lidar odometry with its driver. FAST-LIO's repo also holds a
# localizer we do not use, so only its FAST_LIO package goes.
readonly SOURCES=(
    g1_description g1_hardware_interface g1_controllers g1_bringup
    g1_state_estimation g1_sensor_relay livox_ros_driver2 fast_lio_humanoid/FAST_LIO
)
readonly SSH_OPTS=(-o IdentitiesOnly=yes -i "$KEY" -o BatchMode=yes -o ConnectTimeout=5)

usage() {
    sed -n '/^# Usage/,/^# Environment/p' "$0" | sed 's/^# \{0,1\}//' | head -n -1
}

on_robot() {
    ssh "${SSH_OPTS[@]}" "$ROBOT" "$@"
}

build_image() {
    if ! ls /proc/sys/fs/binfmt_misc/ 2>/dev/null | grep -q aarch64; then
        echo "No arm64 emulation registered: run" >&2
        echo "  sudo apt install qemu-user-static" >&2
        exit 1
    fi
    # The build context is robot/ alone, so the repo's data and bags never go to the builder.
    docker buildx build --platform linux/arm64 -t "$IMAGE" --load "$ROOT/robot"
    load_image
}

load_image() {
    echo "Loading $IMAGE into the Orin's Docker..."
    docker save "$IMAGE" | gzip -1 | on_robot 'gunzip | docker load'
}

sync_packages() {
    # The packages' shared CMake settings, which they include from ../../cmake.
    on_robot "mkdir -p $REMOTE_WS"
    rsync -a --delete -e "ssh ${SSH_OPTS[*]}" "$ROOT/workspace/cmake" "$ROBOT:$REMOTE_WS/"
    local source
    for source in "${SOURCES[@]}"; do
        on_robot "mkdir -p $REMOTE_WS/src/$(dirname "$source")"
        rsync -a --delete --exclude __pycache__ -e "ssh ${SSH_OPTS[*]}" \
            "$ROOT/workspace/src/$source" "$ROBOT:$REMOTE_WS/src/$(dirname "$source")/"
    done
}

build_packages() {
    local select=""
    (( $# )) && select="--packages-select $*"
    # Four compilers per package and two packages at once fit the Orin's 16 GB.
    on_robot docker run --rm --network host -e MAKEFLAGS=-j4 \
        -v "\$HOME/$REMOTE_WS:/root/workspace" "$IMAGE" \
        bash -c "'source /opt/ros/jazzy/setup.bash && colcon build --parallel-workers 2 \
            $select --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF'"
}

container_up() {
    # Idles until a command is exec'd into it; privileged for real-time scheduling and USB.
    on_robot "docker ps -a --format '{{.Names}}' | grep -qx $CONTAINER || \
        docker run -d --name $CONTAINER --restart unless-stopped --privileged --ulimit rtprio=99 \
            --network host --ipc host -v /dev:/dev -v \$HOME/$REMOTE_WS:/root/workspace \
            $IMAGE sleep infinity"
    on_robot "docker start $CONTAINER >/dev/null && docker ps --filter name=$CONTAINER --format '{{.Names}}: {{.Status}}'"
}

container_exec() {
    local command="${*:-bash}"
    ssh -t "${SSH_OPTS[@]}" "$ROBOT" docker exec -it "$CONTAINER" bash -ic "'$command'"
}

main() {
    local command="${1:-help}"
    (( $# )) && shift
    case "$command" in
        image) build_image ;;
        load) load_image ;;
        sync) sync_packages ;;
        build) build_packages "$@" ;;
        up) container_up ;;
        down) on_robot docker rm -f "$CONTAINER" ;;
        exec) container_exec "$@" ;;
        ssh) ssh -t "${SSH_OPTS[@]}" "$ROBOT" ;;
        help | -h | --help) usage ;;
        *) usage >&2; exit 1 ;;
    esac
}

main "$@"
