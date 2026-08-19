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
# Seen in one session: four simulator instances writing rt/lowcmd at once, a second
# controller_manager aborting on startup and taking the whole launch down with it, a pinned
# robot settling somewhere different every launch, and a ROS daemon cheerfully answering for
# nodes that had already exited. Every one of those looked like a bug somewhere else first.
#
# TWO THINGS THIS GETS RIGHT that the obvious version does not.
#
# 1. It matches the FULL COMMAND LINE, not the executable name. `pgrep -x` matches
#    /proc/PID/comm, which the kernel truncates to 15 characters, so `ros2_control_node` (17),
#    `g1_base_approach` (16) and `robot_state_publisher` (21) never match and are silently
#    left running. Most of this stack is `python3` or a `ros2 run` wrapper anyway, whose comm
#    says nothing useful at all.
#
# 2. It LOOPS until the graph is empty. One pass is never enough: launch supervisors respawn
#    children, and DDS keeps reporting a node for several seconds after it dies.
#
# It also skips its own process ancestry. A cleanup whose own command line contains the
# patterns it greps for will kill the shell running it, which is a mistake already made here
#.
set -u

CONTAINER=${CONTAINER:-ros_dev_jazzy}

# Re-enter the container when run from the host. The worker is piped in rather than mounted:
# only ./workspace is bind-mounted, so this file is not visible inside.
#
# This is not cosmetic. The host may have its own ROS on PATH (a Jazzy install, for one), and a
# version of this script that swept the host would find no stack to kill, ask that ROS for a
# node list, get an empty one, and report "clean" while the container was still full. Verifying
# the wrong graph is worse than not verifying at all.
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

# Two rounds of matching, because a name list alone is not enough and that is not obvious.
#
# NAMED is the stack's own binaries. ANY_ROS is the safety net: it matches anything running out
# of a ROS install or carrying --ros-args at all. That second one is what catches composed
# nodes -- Nav2 runs amcl, map_server, pointcloud_to_laserscan and the lifecycle manager INSIDE
# one `nav2_container` process, so none of their names appear anywhere in the process table and
# a name-based sweep reports "killed 0" while `ros2 node list` still shows all of them. That
# exact combination is what motivated this rewrite.
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

# The daemon caches the graph and keeps reporting nodes that are gone. Restarting it is what
# makes the check below mean anything at all.
# set +u around these: ROS's setup scripts reference unbound variables, and under `set -u` that
# aborts the script silently right here -- which looked exactly like the cleanup working and
# then skipping its own verification.
set +u
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash 2>/dev/null || true
[ -f /root/workspace/install/setup.bash ] && source /root/workspace/install/setup.bash
set -u
ros2 daemon stop >/dev/null 2>&1
sleep 3

# One more look with the daemon restarted, and a last sweep if anything survived: a node that
# is still on the graph after all of the above is a process the patterns did not reach, and
# saying so is more useful than a clean-looking exit.
if [ -n "$(graph_nodes)" ]; then
    echo "graph not empty after the sweeps; trying once more"
    total=$((total + $(sweep "$ANY_ROS")))
    sleep 4
    ros2 daemon stop >/dev/null 2>&1
    sleep 3
fi

echo "killed $total process(es) in total, inside the container"

# Killing the processes does not reclaim their shared memory. Enough runs and the leftover
# segments break discovery: sensor topics come up but a subscriber never matches, which reads
# as "FAST-LIO is not running" rather than as a stale-transport problem.
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
