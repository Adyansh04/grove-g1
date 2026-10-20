#!/usr/bin/env bash
# Read-only ROS diagnostics. Never source environments or manage the running stack.
set -u
set -o pipefail
export LC_ALL=C PYTHONUNBUFFERED=1
LIMIT=${BODYGUARD_CHECK_SECONDS:-5}
RATE_LIMIT=${BODYGUARD_RATE_SECONDS:-8}
if [[ ! $LIMIT =~ ^[1-9][0-9]?$ || ! $RATE_LIMIT =~ ^[1-9][0-9]?$ ]]; then
    printf 'FAIL: timeout settings must be integer seconds in 1..99\n' >&2
    exit 2
fi
pass=0 fail=0 skip=0
report() {
    local status=$1; shift
    printf '%s: %s\n' "$status" "$*"
    case $status in PASS) ((pass+=1));; FAIL) ((fail+=1));; SKIP) ((skip+=1));; esac
}
finish() {
    printf '\nSummary: PASS=%d FAIL=%d SKIP=%d\n' "$pass" "$fail" "$skip"
    printf 'PASS describes only the stated observation, not safe motion or end-to-end navigation.\n'
    if ((fail)); then return 1; fi
    if ((skip)); then return 2; fi
    return 0
}
# Every external diagnostic is bounded, including CLI discovery and service waits.
run() { timeout --signal=TERM --kill-after=2s "${LIMIT}s" "$@"; }
unsupported() { [[ $1 == *'invalid choice'* || $1 == *'unrecognized arguments'* || $1 == *'No executable found'* ]]; }
query() {
    local label=$1 cause=$2 out rc; shift 2
    out=$(run "$@" 2>&1); rc=$?
    printf '\n--- %s ---\n%s\n' "$label" "$out"
    if unsupported "$out"; then report SKIP "$label: installed CLI lacks this command/options."
    elif ((rc)); then report FAIL "$label (exit $rc): $cause"
    else report PASS "$label query completed (inspect listed entries)."; fi
}
if ! command -v timeout >/dev/null; then report SKIP 'GNU timeout unavailable; no ROS commands attempted.'; finish; exit $?; fi
if [[ ${ROS_DISTRO:-} == jazzy ]]; then report PASS 'ROS_DISTRO=jazzy';
else report FAIL "ROS_DISTRO=${ROS_DISTRO:-unset}; use the sourced Jazzy development container."; fi
printf 'ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s ROS_LOCALHOST_ONLY=%s\n' \
    "${ROS_DOMAIN_ID:-default}" "${RMW_IMPLEMENTATION:-default}" "${ROS_LOCALHOST_ONLY:-unset}"
if ! command -v ros2 >/dev/null; then report SKIP 'ros2 unavailable; source the existing installation and workspace overlay.'; finish; exit $?; fi
for pkg in g1_bringup g1_navigation g1_state_estimation g1_sensor_relay nav2_msgs nav2_bt_navigator controller_manager tf2_ros; do
    out=$(run ros2 pkg prefix "$pkg" 2>&1); rc=$?
    if ((rc == 0)); then report PASS "package $pkg: $out";
    else report FAIL "package $pkg unavailable: overlay not sourced, package not built/installed, or query timed out. $out"; fi
done
query 'nodes' 'Check DDS domain/RMW/discovery and whether the intended stack is already running.' ros2 node list
query 'topics and types' 'Check DDS discovery and sourced message packages.' ros2 topic list -t
query 'actions and types' 'Nav2 may not be running or discovery may be incomplete.' ros2 action list -t
query 'ros2_control controllers' 'Controller manager may be absent, namespaced, or unresponsive; no controller changes attempted.' \
    ros2 control list_controllers -c "${BODYGUARD_CONTROLLER_MANAGER:-/controller_manager}"

check_topic() {
    local topic=$1 expected=$2 frame=$3 types info header rate rc observed_frame line
    types=$(run ros2 topic type "$topic" 2>&1); rc=$?
    if ((rc)) || [[ $types != "$expected" ]]; then
        report FAIL "$topic: expected exactly $expected, got '$types'; missing topic, remapping, conflicting types, or discovery failure."
        report SKIP "$topic header/rate: required type not confirmed."
        return
    fi
    report PASS "$topic type=$expected (not evidence of data delivery)."
    info=$(run ros2 topic info "$topic" 2>&1); rc=$?
    if ((rc == 0)) && [[ $info =~ Publisher\ count:\ ([1-9][0-9]*) ]]; then
        report PASS "$topic has advertised publisher(s)."
    else report FAIL "$topic: no confirmed publisher; topic may exist only because of subscribers. $info"; fi
    header=$(run ros2 topic echo "$topic" "$expected" --once --field header --qos-reliability best_effort --qos-durability volatile 2>&1); rc=$?
    observed_frame=''
    while IFS= read -r line; do
        [[ $line == frame_id:* ]] && observed_frame=${line#frame_id: }
    done <<< "$header"
    if unsupported "$header"; then report SKIP "$topic header: echo options unavailable."
    elif ((rc == 0)) && [[ $observed_frame == "$frame" ]]; then
        report PASS "$topic received a header in $frame: ${header//$'\n'/ }"
    else report FAIL "$topic: no received header in expected frame $frame (exit $rc). Check sensor enablement, QoS, relay socket, frame overrides, and stale publishers. $header"; fi
    rate=$(timeout --signal=TERM --kill-after=2s "${RATE_LIMIT}s" ros2 topic hz "$topic" --window 20 2>&1); rc=$?
    printf '%s rate observation:\n%s\n' "$topic" "$rate"
    if unsupported "$rate"; then report SKIP "$topic rate: hz options unavailable."
    elif [[ $rate =~ average\ rate:\ ([0-9]+([.][0-9]+)?) ]] && [[ ${BASH_REMATCH[1]} =~ [1-9] ]] && ((rc == 0 || rc == 124)); then
        report PASS "$topic delivered multiple samples during ${RATE_LIMIT}s; no performance threshold asserted."
    else report FAIL "$topic: no positive rate measured (exit $rc); paused source, QoS incompatibility, slow stream, or insufficient observation window."; fi
}
check_topic /g1_odometry_publisher/odom nav_msgs/msg/Odometry odom
for prefix in camera chest_camera; do
    for stream in color aligned_depth_to_color; do
        frame=${prefix}_color_optical_frame
        [[ $stream == aligned_depth_to_color ]] && frame=${prefix}_depth_optical_frame
        check_topic "/$prefix/$stream/image_raw" sensor_msgs/msg/Image "$frame"
        # CameraInfo is checked too: images alone do not establish usable camera intrinsics.
        check_topic "/$prefix/$stream/camera_info" sensor_msgs/msg/CameraInfo "$frame"
    done
done
query 'odometry source (ground truth versus estimate)' 'Cannot classify odometry provenance; do not infer it from the odom topic name.' \
    ros2 param get /g1_odometry_publisher odometry_source
printf 'ground_truth = exact MuJoCo state; fast_lio = LiDAR-inertial estimate. Relay object_poses are simulation ground truth, not perception detections.\n'
actions=$(run ros2 action list -t 2>&1); rc=$?
if ((rc == 0)) && [[ $actions == *'/navigate_to_pose [nav2_msgs/action/NavigateToPose]'* ]]; then
    report PASS 'NavigateToPose advertised with expected type.'
    info=$(run ros2 action info /navigate_to_pose 2>&1); rc=$?
    if ((rc == 0)) && [[ $info =~ Action\ servers:\ ([1-9][0-9]*) ]]; then report PASS 'NavigateToPose has an advertised action server; no goal sent.';
    else report FAIL "NavigateToPose has no confirmed server; Nav2 may be absent or not ready. $info"; fi
else report FAIL 'NavigateToPose missing or wrong type; check Nav2 enablement, namespace, and DDS discovery.'; fi
for node in g1_odometry_publisher controller_server planner_server behavior_server bt_navigator collision_monitor map_server amcl; do
    state=$(run ros2 lifecycle get "/$node" 2>&1); rc=$?
    if unsupported "$state"; then report SKIP "$node lifecycle CLI unavailable."
    elif ((rc == 0)) && [[ $state == 'active [3]' ]]; then report PASS "$node lifecycle active."
    else report FAIL "$node lifecycle not confirmed active: $state. Check startup/configuration; map_server and amcl are expected for fixed-goal localization, not mapping mode."; fi
done
for target in odom base_footprint pelvis camera_color_optical_frame camera_depth_optical_frame chest_camera_color_optical_frame chest_camera_depth_optical_frame; do
    tf=$(run ros2 run tf2_ros tf2_echo map "$target" 2>&1); rc=$?
    if unsupported "$tf"; then report SKIP "map -> $target: tf2_echo unavailable."
    elif [[ $tf == *'Translation: ['* && $tf == *'Rotation: in Quaternion'* ]] && ((rc == 0 || rc == 124)); then
        report PASS "map -> $target lookup observed; latest available transform only, freshness/accuracy not established."
    else report FAIL "map -> $target disconnected/unobserved (exit $rc). Check localization map->odom, active odometry, joint states and robot_state_publisher. $tf"; fi
done
finish
