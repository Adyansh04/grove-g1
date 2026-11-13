#!/usr/bin/env bash
#
# The G1's on-robot runtime: builds its arm64 image on this machine, ships it and the robot-side
# packages to the Orin over the robot's network, builds them there and runs them detached. The
# Orin needs no internet. Robot work has its own ROS domain and its own Zenoh router on this
# machine, linked to the router of the Orin's container, so the simulator's graph never meets the
# robot's.
#
# Usage: ./scripts/robot.sh <command> [args]
#   image            build robot/Dockerfile for arm64 (QEMU) here, then `load`
#   load             load the robot image into the Orin's Docker
#   sync             copy the robot-side packages to ~/grove-g1 on the Orin
#   build [pkg..]    colcon build them on the Orin, in the image (everything synced by default)
#   up | down        start or remove the runtime container on the Orin, which runs its router
#   start <stack> [arg:=value..]  run a stack detached on the Orin, with launch arguments:
#                    `control` takes the motors, `sensors` starts the camera and lidar
#   stop <stack>     stop it with SIGINT, as Ctrl-C would; control releases the motors first
#   log <stack>      follow its log
#   stand            the legs from their freeze to the policy, blended in over 4 s: the robot in
#                    its stand, feet on the ground
#   arm acquire|release  the arms between their freeze and MoveIt's trajectory controller
#   release [--yes]  arms and legs back to their freezes, then stop sensors and control; asks
#                    first, since a robot that stands when control stops falls
#   operator [cmd]   a shell, or cmd, in the dev container on the robot's domain and router
#   exec [cmd]       a shell, or cmd, in the robot container, with a terminal
#   ssh              a shell on the Orin
#
# `up` needs real-time budget for containers on the Orin, once:
# "cpu-rt-runtime": 950000 in its /etc/docker/daemon.json, then a Docker restart.
#
# Environment: G1_ROBOT (default unitree@192.168.123.164), G1_ROBOT_KEY (default
# ~/.ssh/id_ed25519_robot), G1_HAND_MODEL (default inspire, the hands this G1 wears).

set -euo pipefail

readonly ROOT="$(cd "$(dirname "$0")/.." && pwd)"
readonly ROBOT="${G1_ROBOT:-unitree@192.168.123.164}"
readonly ROBOT_ADDRESS="${ROBOT#*@}"
readonly KEY="${G1_ROBOT_KEY:-$HOME/.ssh/id_ed25519_robot}"
readonly IMAGE="grove-g1-robot:local"
readonly CONTAINER="g1-robot"
readonly DEV_CONTAINER="ros_dev_jazzy"
# The ROS domain of robot work. The simulator and dev work use 1, so neither can reach the robot.
readonly ROBOT_ROS_DOMAIN=7
# The hands this G1 wears. Every launch on the robot and in the operator's shells takes its
# hand_model default from G1_HAND_MODEL, so MoveIt and the robot model carry the real hands.
readonly HAND_MODEL="${G1_HAND_MODEL:-inspire}"
# The Orin's router listens on rmw_zenoh's default port. The operator's router, in the dev
# container beside the simulator's (7447), takes this machine's loopback only.
readonly ROBOT_ROUTER="tcp/$ROBOT_ADDRESS:7447"
readonly OPERATOR_ROUTER_PORT=7448
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
        # No hand components until each hand's RS485 line is checked on the robot.
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
    # The packages' shared CMake settings, which they include from ../../cmake.
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
    # Its main process is the Zenoh router: the stacks started in it find each other through it,
    # and the operator's router links to it over Ethernet or WiFi. Privileged for USB; the
    # real-time budget lets the control loop run SCHED_FIFO.
    on_robot "docker ps -a --format '{{.Names}}' | grep -qx $CONTAINER || \
        docker run -d --name $CONTAINER --restart unless-stopped --init --privileged \
            --ulimit rtprio=99 --cpu-rt-runtime=900000 -e ROS_DOMAIN_ID=$ROBOT_ROS_DOMAIN \
            --network host --ipc host -v /dev:/dev -v \$HOME/$REMOTE_WS:/root/workspace \
            $IMAGE ros2 run rmw_zenoh_cpp rmw_zenohd"
    on_robot "docker start $CONTAINER >/dev/null && docker ps --filter name=$CONTAINER --format '{{.Names}}: {{.Status}}'"
}

stack_running() {
    on_robot "docker exec $CONTAINER pgrep -f $(stack_pattern "$1")" >/dev/null
}

stack_start() {
    local launch
    launch="$(stack_launch "$1") ${*:2}"
    if stack_running "$1"; then
        echo "$1 is already running." >&2
        exit 1
    fi
    on_robot "mkdir -p $REMOTE_WS/log && docker exec -d -e G1_HAND_MODEL=$HAND_MODEL $CONTAINER bash -c \
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

router_listening() {
    (exec 3<>"/dev/tcp/127.0.0.1/$OPERATOR_ROUTER_PORT") 2>/dev/null
}

# The operator's nodes connect to the operator's router alone, which links to the Orin's: one
# copy of each robot topic crosses the link, and the simulator's graph stays off it.
operator_shell() {
    local operator_router="tcp/127.0.0.1:$OPERATOR_ROUTER_PORT"
    if ! router_listening; then
        local listen="listen/endpoints=[\"$operator_router\"]"
        local connect="connect/endpoints=[\"$ROBOT_ROUTER\"]"
        docker exec -d -e ZENOH_CONFIG_OVERRIDE="$listen;$connect" "$DEV_CONTAINER" \
            bash -c 'source /opt/ros/jazzy/setup.bash && exec ros2 run rmw_zenoh_cpp rmw_zenohd'
        local waited=0
        until router_listening; do
            (( waited++ < 50 )) || { echo "The operator's router did not start." >&2; exit 1; }
            sleep 0.1
        done
    fi
    # A terminal only when there is one, so scripts can run operator commands too.
    local tty=(-i)
    [[ -t 0 && -t 1 ]] && tty=(-it)
    # The dev container's login profile takes the domain from GROVE_G1_ROS_DOMAIN_ID.
    docker exec "${tty[@]}" -e GROVE_G1_ROS_DOMAIN_ID="$ROBOT_ROS_DOMAIN" \
        -e ROS_DOMAIN_ID="$ROBOT_ROS_DOMAIN" -e G1_HAND_MODEL="$HAND_MODEL" \
        -e ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"$operator_router\"]" \
        "$DEV_CONTAINER" bash -lc "cd /root/workspace && source install/setup.bash && ${*:-bash}"
}

# ros2 CLI calls on the robot's domain, cut off with SIGINT: on rmw_zenoh one can ignore SIGTERM.
readonly ROS2="timeout -s INT -k 5 30 ros2"
# A line of `ros2 control list_controllers` for a controller in the active state; \< keeps
# "inactive" out.
active() {
    printf "grep -qE '^%s .*\\<active\\>' <<<\"\$listed\"" "$1"
}

stand_up() {
    # A spawner can hang on the Zenoh link at start and leave the policy's controllers unloaded:
    # load whatever is missing, inactive, the safety controller first since the policy chains into
    # it. The blend goes to 0 before the switch, so the policy takes over from the held pose even
    # when an earlier stand left it at 1.
    operator_shell "set -e
listed=\$($ROS2 control list_controllers)
for c in locomotion_safety_controller agile_controller; do
    grep -q \"^\$c \" <<<\"\$listed\" || $ROS2 control load_controller --set-state inactive \$c
done
$(active locomotion_freeze_controller) || { echo 'The legs are not frozen: start control first, or the robot already stands.' >&2; exit 1; }
$ROS2 param set /locomotion_safety_controller blend_ratio 0.0
$ROS2 control switch_controllers --deactivate locomotion_freeze_controller \\
    --activate locomotion_safety_controller agile_controller --strict
$ROS2 param set /locomotion_safety_controller blend_ratio 1.0
echo 'The policy is blending in over 4 s.'"
}

arm_switch() {
    local from to
    case "${1:-}" in
        acquire) from=arm_freeze_controller to=arm_trajectory_controller ;;
        release) from=arm_trajectory_controller to=arm_freeze_controller ;;
        *) echo "Usage: robot.sh arm acquire|release" >&2; exit 1 ;;
    esac
    operator_shell "$ROS2 control switch_controllers --deactivate $from --activate $to --strict"
}

release_robot() {
    if [[ "${1:-}" != --yes ]]; then
        local answer
        read -r -p "Does the robot hang in its stand? Stopping control drops a standing robot. [y/N] " \
            answer </dev/tty
        [[ "$answer" == [yY]* ]] || { echo "Nothing released." >&2; exit 1; }
    fi
    if stack_running control; then
        # Arms, then legs, each back to its freeze; one already there stays.
        operator_shell "listed=\$($ROS2 control list_controllers)
if $(active arm_trajectory_controller); then
    $ROS2 control switch_controllers --deactivate arm_trajectory_controller \\
        --activate arm_freeze_controller --strict
fi
if $(active locomotion_safety_controller); then
    $ROS2 control switch_controllers --deactivate locomotion_safety_controller agile_controller \\
        --activate locomotion_freeze_controller --strict
fi"
    fi
    if stack_running sensors; then stack_stop sensors; fi
    if stack_running control; then stack_stop control; fi
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
        stand) stand_up ;;
        arm) arm_switch "$@" ;;
        release) release_robot "$@" ;;
        operator) operator_shell "$@" ;;
        exec) ssh -t "${SSH_OPTS[@]}" "$ROBOT" docker exec -it -e G1_HAND_MODEL="$HAND_MODEL" \
            "$CONTAINER" bash -ic "'${*:-bash}'" ;;
        ssh) ssh -t "${SSH_OPTS[@]}" "$ROBOT" ;;
        help | -h | --help) usage ;;
        *) usage >&2; exit 1 ;;
    esac
}

main "$@"
