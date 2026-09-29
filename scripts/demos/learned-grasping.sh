#!/usr/bin/env bash
# Learned grasping: docs/guides/learned-grasping.md.
#
#   ./scripts/demos/learned-grasping.sh <variant> [name:=value...] [--print]
#
#   mock    the mock engine behind the planning-scene gate; no GPU, no downloads
#   groot   the GR00T policy, served on the host (./scripts/setup-groot.sh first)
#   servo   the mock engine, streamed through MoveIt Servo instead of trajectories
#   stop    end whichever demo is running
#
# The tree starts once the arm is acquired, and with groot once the policy server is up.
# name:=value arguments go to the bringup launch. --print lists the commands instead of opening them.
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

BRINGUP="ros2 launch g1_bringup bringup.launch.py moveit:=true manipulation:=true vla:=true world:=manipulation pin_pelvis:=true odometry:=ground_truth activate_arm:=true activate_arm_delay_s:=40.0"
TREE="ros2 launch g1_orchestration mission.launch.py tree:=vla_grasp_in_place.xml"

case "$VARIANT" in
    mock)
        container bringup "$BRINGUP vla_engine:=mock$LAUNCH_ARGS"
        container tree "wait_for_arm && $TREE"
        ;;
    groot)
        container bringup "$BRINGUP vla_engine:=groot$LAUNCH_ARGS"
        host policy "./scripts/serve.sh groot"
        container tree "wait_for_arm && wait_for_port 'the GR00T server' 5555 && $TREE"
        ;;
    servo)
        container bringup "$BRINGUP vla_engine:=mock vla_execution_mode:=servo$LAUNCH_ARGS"
        container tree "wait_for_arm && $TREE"
        ;;
    *) usage ;;
esac
open_panes "Learned grasping: $VARIANT"
