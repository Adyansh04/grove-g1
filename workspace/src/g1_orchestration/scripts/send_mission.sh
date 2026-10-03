#!/usr/bin/env bash
# Sends a hand-written mission to the executor with the hash it needs, and follows it.
#
#   ros2 run g1_orchestration send_mission.sh <mission.xml> [mode] [max_duration_s]
#
# mode is execute (the default), validate, dry-run, or check, which calls the ValidateMission
# service instead of sending a goal. The hash is the SHA-256 of the file as read here with its
# trailing newlines removed, which is exactly what is sent: the executor hashes the bytes it gets.
set -euo pipefail

file=${1:?usage: send_mission.sh <mission.xml> [execute|validate|dry-run|check] [max_duration_s]}
mode_name=${2:-execute}
max_duration_s=${3:-0}
case "$mode_name" in
    execute) mode=0 ;;
    validate) mode=1 ;;
    dry-run) mode=2 ;;
    check) mode=0 ;;
    *) echo "mode must be execute, validate, dry-run or check" >&2; exit 2 ;;
esac

xml=$(cat "$file")
hash=$(printf '%s' "$xml" | sha256sum | cut -d' ' -f1)
# A literal block, with its indentation stated so the mission's own leading spaces survive.
block=$(printf '%s\n' "$xml" | sed 's/^/  /')

if [[ $mode_name == check ]]; then
    exec ros2 service call /nervros_executor/validate_mission \
        nervros_interfaces/srv/ValidateMission "$(printf 'tree_xml: |2-\n%s\n' "$block")"
fi

id="hand-$(date +%H%M%S)"
goal=$(printf 'mission_id: %s\ntree_sha256: %s\nmax_duration_s: %s\nmode: %s\ntree_xml: |2-\n%s\n' \
    "$id" "$hash" "$max_duration_s" "$mode" "$block")
exec ros2 action send_goal --feedback /nervros_executor/execute_mission \
    nervros_interfaces/action/ExecuteMission "$goal"
