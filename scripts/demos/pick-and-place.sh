#!/usr/bin/env bash
# Pick and place: docs/guides/pick-and-place.md.
#
#   ./scripts/demos/pick-and-place.sh <variant> [name:=value...] [--print]
#
#   in-place   pick the block at arm's length, hold it at carry, set it down beside where it was
#   mission    drive to the ball in the facility, pick it, carry it across, drop it in the box
#   sort       pick the red block from beside the blue one on the tabletop, into the box
#   stop       end whichever demo is running
#
# The tree starts once the arm is acquired. name:=value arguments go to the bringup launch, such
# as detector:=vision for the mission. --print lists the commands instead of opening them.
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

BRINGUP="ros2 launch g1_bringup bringup.launch.py"
AT_ARMS_LENGTH="moveit:=true manipulation:=true pin_pelvis:=true odometry:=ground_truth activate_arm:=true activate_arm_delay_s:=40.0"
TREE="ros2 launch g1_orchestration mission.launch.py tree:="

case "$VARIANT" in
    in-place)
        container bringup "$BRINGUP world:=manipulation $AT_ARMS_LENGTH$LAUNCH_ARGS"
        container tree "wait_for_arm && ${TREE}pick_and_place_in_place.xml"
        ;;
    mission)
        container bringup "$BRINGUP mode:=localization nav:=true moveit:=true manipulation:=true perception:=true world:=navigation rviz:=true activate_arm:=true activate_arm_delay_s:=40.0 headless:=false$LAUNCH_ARGS"
        container tree "wait_for_nav2 && wait_for_arm && ${TREE}pick_and_place.xml"
        ;;
    sort)
        container bringup "$BRINGUP world:=tabletop perception:=true $AT_ARMS_LENGTH$LAUNCH_ARGS"
        container tree "wait_for_arm && ${TREE}sort_into_box.xml"
        ;;
    *) usage ;;
esac
open_panes "Pick and place: $VARIANT"
