#!/usr/bin/env bash
#
# The G1's on-robot runtime: builds its arm64 image on this machine, ships it and the robot-side
# packages to the Orin over the robot's network, builds them there and runs them detached. The
# Orin needs no internet. Robot work has its own ROS domain, so the simulator's graph on this
# machine never meets the robot's, and Fast DDS on both sides is pinned to the robot's network.
#
# Usage: ./scripts/robot.sh <command> [args]
#   image            build robot/Dockerfile for arm64 (QEMU) here, then `load`
#   load             load the robot image into the Orin's Docker
#   sync             copy the robot-side packages and the DDS profile to ~/grove-g1 on the Orin
#   build [pkg..]    colcon build them on the Orin, in the image (everything synced by default)
#   up | down        start or remove the idle runtime container on the Orin
#   start <stack> [arg:=value..]  run a stack detached on the Orin, with launch arguments:
#                    `control` takes the motors, `sensors` starts the camera and lidar
#   stop <stack>     stop it with SIGINT, as Ctrl-C would; control releases the motors first
#   log <stack>      follow its log
#   operator [cmd]   a shell, or cmd, in the dev container on the robot's domain and network
#   exec [cmd]       a shell, or cmd, in the robot container, with a terminal
#   ssh              a shell on the Orin
#
# `up` needs real-time budget for containers on the Orin, once:
# "cpu-rt-runtime": 950000 in its /etc/docker/daemon.json, then a Docker restart.
#
# Environment: G1_ROBOT (default unitree@192.168.123.164), G1_ROBOT_KEY (default
# ~/.ssh/id_ed25519_robot).

set -euo pipefail

readonly ROOT="$(cd "$(dirname "$0")/.." && pwd)"
readonly ROBOT="${G1_ROBOT:-unitree@192.168.123.164}"
readonly ROBOT_ADDRESS="${ROBOT#*@}"
readonly KEY="${G1_ROBOT_KEY:-$HOME/.ssh/id_ed25519_robot}"
readonly IMAGE="grove-g1-robot:local"
readonly CONTAINER="g1-robot"
readonly DEV_CONTAINER="ros_dev_jazzy"
# This machine's address on the robot's network (the g1 interface).
readonly OPERATOR_ADDRESS="${G1_OPERATOR_ADDRESS:-192.168.123.222}"
# The ROS domain of robot work. The simulator and dev work use 1, so neither can reach the robot.
readonly ROBOT_ROS_DOMAIN=7
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

stack_launch() {
    case "$1" in
        control) echo "ros2 launch g1_bringup control.launch.py hardware:=true" \
            "robot_address:=$ROBOT_ADDRESS hands:=false" ;;
        sensors) echo "ros2 launch g1_bringup robot_sensors.launch.py" ;;
        *) echo "Unknown stack '$1': control or sensors." >&2; exit 1 ;;
    esac
}

# The stack's launch process, as a pgrep/pkill pattern quoted for the Orin's shell.
stack_pattern() {
    printf '%q' "^/usr/bin/python3 /opt/ros/jazzy/bin/$(stack_launch "$1")"
}

build_image() {
    if ! ls /proc/sys/fs/binfmt_misc/ 2>/dev/null | grep -q aarch64; then
        echo "No arm64 emulation registered: run" >&2
        echo "  sudo apt install qemu-user-static" >&2
        exit 1
    fi
    # The build context is robot/ alone, so the repo's data and bags never go to the builder.
    docker buildx build --platform linux/arm64 -t "$IMAGE" --load "$ROOT/robot"
    load_images
}

load_images() {
    echo "Loading $IMAGE into the Orin's Docker..."
    docker save "$IMAGE" | gzip -1 | on_robot 'gunzip | docker load'
}

sync_packages() {
    on_robot "mkdir -p $REMOTE_WS"
    # The packages' shared CMake settings, which they include from ../../cmake, and the DDS profile.
    rsync -a --delete -e "ssh ${SSH_OPTS[*]}" "$ROOT/workspace/cmake" "$ROOT/robot/fastdds" \
        "$ROBOT:$REMOTE_WS/"
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
    # Idles until a stack starts in it. Privileged for USB; the real-time budget lets the control
    # loop run SCHED_FIFO. ROS finds the operator by unicast, which also works over WiFi.
    on_robot "docker ps -a --format '{{.Names}}' | grep -qx $CONTAINER || \
        docker run -d --name $CONTAINER --restart unless-stopped --privileged --ulimit rtprio=99 \
            --cpu-rt-runtime=900000 -e ROS_DOMAIN_ID=$ROBOT_ROS_DOMAIN \
            -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/workspace/fastdds/robot.xml \
            -e ROS_STATIC_PEERS=$OPERATOR_ADDRESS \
            --network host --ipc host -v /dev:/dev -v \$HOME/$REMOTE_WS:/root/workspace \
            $IMAGE sleep infinity"
    on_robot "docker start $CONTAINER >/dev/null && docker ps --filter name=$CONTAINER --format '{{.Names}}: {{.Status}}'"
}

stack_start() {
    local launch pattern
    launch="$(stack_launch "$1") ${*:2}"
    pattern="$(stack_pattern "$1")"
    if on_robot "docker exec $CONTAINER pgrep -f $pattern" >/dev/null; then
        echo "$1 is already running." >&2
        exit 1
    fi
    on_robot "mkdir -p $REMOTE_WS/log && docker exec -d $CONTAINER bash -c \
        'source /opt/ros/jazzy/setup.bash && source /root/workspace/install/setup.bash && \
         exec $launch > /root/workspace/log/$1.log 2>&1'"
    echo "$1 started; ./scripts/robot.sh log $1 follows it."
}

stack_stop() {
    local pattern
    pattern="$(stack_pattern "$1")"
    on_robot "docker exec $CONTAINER pkill -INT -f $pattern" ||
        { echo "$1 is not running." >&2; exit 1; }
    # The control stack ramps the motors out over 0.5 s before it exits.
    local waited=0
    while on_robot "docker exec $CONTAINER pgrep -f $pattern" >/dev/null; do
        (( waited++ < 30 )) || { echo "$1 did not stop within 15 s." >&2; exit 1; }
        sleep 0.5
    done
    on_robot "grep -h 'motors released' $REMOTE_WS/log/$1.log" || true
    echo "$1 stopped."
}

main() {
    local command="${1:-help}"
    (( $# )) && shift
    case "$command" in
        image) build_image ;;
        load) load_images ;;
        sync) sync_packages ;;
        build) build_packages "$@" ;;
        up) container_up ;;
        down) on_robot docker rm -f "$CONTAINER" ;;
        start) stack_start "${1:?start control|sensors}" "${@:2}" ;;
        stop) stack_stop "${1:?stop control|sensors}" ;;
        log) ssh -t "${SSH_OPTS[@]}" "$ROBOT" tail -n 40 -f "$REMOTE_WS/log/${1:?log control|sensors}.log" ;;
        operator)
            # The dev container's login profile takes the domain from GROVE_G1_ROS_DOMAIN_ID.
            docker exec -it -e GROVE_G1_ROS_DOMAIN_ID="$ROBOT_ROS_DOMAIN" -e ROS_DOMAIN_ID="$ROBOT_ROS_DOMAIN" \
                -e ROS_STATIC_PEERS="$ROBOT_ADDRESS" \
                -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/.host-repo/robot/fastdds/operator.xml \
                "$DEV_CONTAINER" bash -lc "cd /root/workspace && source install/setup.bash && ${*:-bash}" ;;
        exec) ssh -t "${SSH_OPTS[@]}" "$ROBOT" docker exec -it "$CONTAINER" bash -ic "'${*:-bash}'" ;;
        ssh) ssh -t "${SSH_OPTS[@]}" "$ROBOT" ;;
        help | -h | --help) usage ;;
        *) usage >&2; exit 1 ;;
    esac
}

main "$@"
