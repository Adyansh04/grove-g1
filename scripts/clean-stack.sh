#!/usr/bin/env bash
# Tears the G1 stack down and verifies the ROS graph is empty, rather than trusting the process
# table. Run it between launches. Exits non-zero if anything is left, so it can gate a test run.
#
#   ./scripts/clean-stack.sh
#
# Runs from the host or inside the container. Leftover nodes cause phantom bugs here, such as
# several simulators writing rt/lowcmd at once or a second controller_manager aborting a launch.
#
# Matches full command lines: `pgrep -x` reads /proc/PID/comm, truncated to 15 characters, so
# names like ros2_control_node never match. Loops until the graph is empty, since launch respawns
# children and DDS reports dead nodes for seconds. Skips its own ancestry, whose command lines
# contain the same patterns.
set -u

CONTAINER=${CONTAINER:-ros_dev_jazzy}

# From the host, re-run inside the container. A host sweep would ask the host's own ROS for a
# node list and report clean while the container is still full.
if [ ! -d /root/workspace ]; then
    if ! command -v docker >/dev/null 2>&1; then
        echo "not in the container and docker is not on PATH" >&2
        exit 1
    fi
    # Copied to a file, not piped to `bash -s`: the first command that reads stdin would swallow
    # the rest of the script.
    exec docker exec -i "$CONTAINER" bash -c \
        'cat > /tmp/g1-clean-stack.sh && bash /tmp/g1-clean-stack.sh' < "$0"
fi

# NAMED is the stack's own binaries. ANY_ROS catches anything else running from a ROS install or
# with --ros-args, including composed nodes such as Nav2's, which never appear by name.
NAMED='unitree_mujoco|ros2_control_node|move_group|g1_manipulation|g1_object_pose|g1_sensor_relay|g1_base_approach|g1_odometry|robot_state_publisher|rviz2|controller_manager|spawner|Xvfb|bt_executor|slam_toolbox|amcl|map_server|planner_server|controller_server|behavior_server|bt_navigator|lifecycle_manager|pointcloud_to_laserscan|planning_scene|transform_listener|activate_arm|deactivate_arm|nav_soak'
ANY_ROS='component_container|rclcpp_components|--ros-args|/opt/ros/[a-z]+/lib/|ros2 run |ros2 launch |ros2 daemon'

protected=" $$ $PPID "
walk=$PPID
while [ -n "$walk" ] && [ "$walk" -gt 1 ] 2>/dev/null; do
    walk=$(awk '{print $4}' "/proc/$walk/stat" 2>/dev/null)
    [ -n "$walk" ] && protected="$protected$walk "
done

sweep() {
    local pattern=$1 killed=0 pid cmd
    for proc in /proc/[0-9]*; do
        pid=${proc#/proc/}
        case "$protected" in *" $pid "*) continue ;; esac
        cmd=$(tr '\0' ' ' < "$proc/cmdline" 2>/dev/null) || continue
        [ -z "$cmd" ] && continue
        if printf '%s' "$cmd" | grep -qE "$pattern"; then
            kill -9 "$pid" 2>/dev/null && killed=$((killed + 1))
        fi
    done
    echo "$killed"
}

graph_nodes() { timeout 25 ros2 node list 2>/dev/null | grep -v '^$'; }

# The named sweep first, so ordinary runs report something recognisable, then the broad one.
total=0
for pass in 1 2 3 4; do
    n=$(sweep "$NAMED")
    m=$(sweep "$ANY_ROS")
    total=$((total + n + m))
    echo "pass $pass: killed $n by name, $m by ROS signature"
    [ "$((n + m))" -eq 0 ] && [ "$pass" -gt 1 ] && break
    sleep 4
done

# A crashed simulator leaves these behind, and the next launch dies on them: Xvfb reports
# "Fatal server error" on a stale lock, and the relay cannot bind an existing socket path.
rm -f /tmp/.X133-lock /tmp/.X11-unix/X133 /tmp/g1_sensors.sock

# ROS's setup scripts reference unbound variables, which would silently abort under `set -u`.
set +u
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash 2>/dev/null || true
[ -f /root/workspace/install/setup.bash ] && source /root/workspace/install/setup.bash
set -u
# The daemon keeps reporting nodes that are gone; restart it so the check below is real.
ros2 daemon stop >/dev/null 2>&1
sleep 3

# Anything still on the graph escaped the patterns: one more broad sweep, then report it.
if [ -n "$(graph_nodes)" ]; then
    echo "graph not empty after the sweeps; trying once more"
    total=$((total + $(sweep "$ANY_ROS")))
    sleep 4
    ros2 daemon stop >/dev/null 2>&1
    sleep 3
fi

echo "killed $total process(es) in total, inside the container"

# kill -9 leaves Fast DDS shared-memory segments behind; stale ones break delivery on later runs
# without any error.
shm=$(ls /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "cleared $shm stale DDS shared-memory segment(s)"

nodes=$(graph_nodes)
topics=$(timeout 25 ros2 topic list 2>/dev/null | grep -vE '^/parameter_events$|^/rosout$')

echo "--- ros2 node list (want: empty) ---"
if [ -z "$nodes" ]; then echo "  (empty)"; else echo "$nodes" | sed 's/^/  LEFTOVER: /'; fi
echo "--- ros2 topic list (want: only /parameter_events and /rosout) ---"
if [ -z "$topics" ]; then echo "  (clean)"; else echo "$topics" | sed 's/^/  LEFTOVER: /'; fi

[ -z "$nodes" ] && [ -z "$topics" ]
