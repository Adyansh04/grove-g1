#!/usr/bin/env bash
# Tears the G1 stack down and PROVES the ROS graph is empty, rather than trusting the process
# table. Run it between launches, and any time a run misbehaves for no reason you can see.
#
#   ./scripts/clean-stack.sh
#
# Runs from the host (like manage.sh) or from inside the container; it works out which and
# re-enters if needed. Exits non-zero if anything is still on the graph afterwards, so it can
# gate a test run rather than just being run hopefully.
#
# WHY THIS EXISTS. Leftover nodes are the most productive source of phantom bugs in this stack.
# Seen in one session: four motion_service_sim instances writing /lowcmd at once, a second
# controller_manager aborting on startup and taking the whole launch down with it, a pinned
# robot settling somewhere different every launch, and a ROS daemon cheerfully answering for
# nodes that had already exited. Every one of those looked like a bug somewhere else first.
#
# TWO THINGS THIS GETS RIGHT that the obvious version does not.
#
# 1. It matches the FULL COMMAND LINE, not the executable name. `pgrep -x` matches
#    /proc/PID/comm, which the kernel truncates to 15 characters, so `ros2_control_node` (17),
#    `motion_service_sim` (18) and `robot_state_publisher` (21) never match and are silently
#    left running. Most of this stack is `python3` or a `ros2 run` wrapper anyway, whose comm
#    says nothing useful at all.
#
# 2. It LOOPS until the graph is empty. One pass is never enough: launch supervisors respawn
#    children, and DDS keeps reporting a node for several seconds after it dies.
#
# It also skips its own process ancestry. A cleanup whose own command line contains the
# patterns it greps for will kill the shell running it, which is a mistake already made here
# (docs/notes/lingering-processes.md).
set -u

CONTAINER=${CONTAINER:-ros_dev_humble}

# Re-enter the container when run from the host. The worker is piped in rather than mounted:
# only ./workspace is bind-mounted, so this file is not visible inside.
if [ ! -d /root/workspace ]; then
    if ! command -v docker >/dev/null 2>&1; then
        echo "not in the container and docker is not on PATH" >&2
        exit 1
    fi
    # Written to a file and then run, NOT piped to `bash -s`. With `-s` the script arrives on
    # stdin, and the first command inside that reads stdin swallows the rest of it -- which
    # silently truncated this script right before its own verification step.
    exec docker exec -i "$CONTAINER" bash -c \
        'cat > /tmp/g1-clean-stack.sh && bash /tmp/g1-clean-stack.sh' < "$0"
fi

# Broad on purpose, matched against the whole command line. Everything this stack launches is
# one of these binaries, a python script, or a `ros2 run` wrapper around one.
PATTERNS='unitree_mujoco|ros2_control_node|move_group|g1_manipulation|g1_object_pose|g1_sensor_relay|motion_service|g1_loco|g1_gait|g1_odometry|robot_state_publisher|rviz2|controller_manager|spawner|Xvfb|bt_executor|slam_toolbox|amcl|map_server|planner_server|controller_server|behavior_server|bt_navigator|lifecycle_manager|pointcloud_to_laserscan|planning_scene|transform_listener|activate_arm|deactivate_arm|nav_soak'

protected=" $$ $PPID "
walk=$PPID
while [ -n "$walk" ] && [ "$walk" -gt 1 ] 2>/dev/null; do
    walk=$(awk '{print $4}' "/proc/$walk/stat" 2>/dev/null)
    [ -n "$walk" ] && protected="$protected$walk "
done

sweep() {
    local killed=0 pid cmd
    for proc in /proc/[0-9]*; do
        pid=${proc#/proc/}
        case "$protected" in *" $pid "*) continue ;; esac
        cmd=$(tr '\0' ' ' < "$proc/cmdline" 2>/dev/null) || continue
        [ -z "$cmd" ] && continue
        if printf '%s' "$cmd" | grep -qE "$PATTERNS"; then
            kill -9 "$pid" 2>/dev/null && killed=$((killed + 1))
        fi
    done
    echo "$killed"
}

total=0
for pass in 1 2 3; do
    n=$(sweep)
    total=$((total + n))
    echo "pass $pass: killed $n"
    [ "$n" -eq 0 ] && [ "$pass" -gt 1 ] && break
    sleep 4
done

# A crashed simulator leaves these behind, and the next launch dies on them: Xvfb reports
# "Fatal server error" on a stale lock, and the relay cannot bind an existing socket path.
rm -f /tmp/.X133-lock /tmp/.X11-unix/X133 /tmp/g1_sensors.sock

# The daemon caches the graph and keeps reporting nodes that are gone. Restarting it is what
# makes the check below mean anything at all.
# set +u around these: ROS's setup scripts reference unbound variables, and under `set -u` that
# aborts the script silently right here -- which looked exactly like the cleanup working and
# then skipping its own verification.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f /root/workspace/install/setup.bash ] && source /root/workspace/install/setup.bash
set -u
ros2 daemon stop >/dev/null 2>&1
sleep 3

echo "killed $total process(es) in total"

nodes=$(timeout 25 ros2 node list 2>/dev/null | grep -v '^$')
topics=$(timeout 25 ros2 topic list 2>/dev/null | grep -vE '^/parameter_events$|^/rosout$')

echo "--- ros2 node list (want: empty) ---"
if [ -z "$nodes" ]; then echo "  (empty)"; else echo "$nodes" | sed 's/^/  LEFTOVER: /'; fi
echo "--- ros2 topic list (want: only /parameter_events and /rosout) ---"
if [ -z "$topics" ]; then echo "  (clean)"; else echo "$topics" | sed 's/^/  LEFTOVER: /'; fi

[ -z "$nodes" ] && [ -z "$topics" ]
