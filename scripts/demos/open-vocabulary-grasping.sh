#!/usr/bin/env bash
# Open-vocabulary perception: docs/guides/open-vocabulary-grasping.md.
#
#   ./scripts/demos/open-vocabulary-grasping.sh <variant> [name:=value...] [--print]
#
#   mock       masks cut from the simulator's truth; no GPU, no server
#   vision     the real detector, served on the host (./scripts/setup-vision.sh first)
#   graspgen   grasps from GraspGenX on the host, and a pick to send (./scripts/setup-graspgen.sh)
#   grounding  an instruction turned into phrases by a VLM beside the detector
#   stop       end whichever demo is running
#
# RViz shows the detections. name:=value arguments go to the bringup launch, such as
# phrases:="red block,white cup". --print lists the commands instead of opening them.
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

BRINGUP="ros2 launch g1_bringup bringup.launch.py world:=tabletop pin_pelvis:=true odometry:=ground_truth moveit:=true manipulation:=true perception:=true rviz:=true"
OBJECTS="ros2 topic echo --once /objects"

case "$VARIANT" in
    mock)
        container bringup "$BRINGUP detector:=mock$LAUNCH_ARGS"
        staged objects "" "$OBJECTS"
        ;;
    vision)
        container bringup "wait_for_port 'the vision server' 5560 && $BRINGUP detector:=vision phrases:='red block,white cup'$LAUNCH_ARGS"
        host detector "./scripts/serve.sh vision"
        staged objects "" "$OBJECTS"
        ;;
    graspgen)
        container bringup "wait_for_port GraspGenX 5556 && $BRINGUP detector:=mock grasp_engine:=graspgen grasp_source:=generated activate_arm:=true activate_arm_delay_s:=40.0$LAUNCH_ARGS"
        host generator "./scripts/serve.sh graspgen"
        staged grasps "wait_for_service /g1_grasp_engine/generate_grasps" "ros2 service call /g1_grasp_engine/generate_grasps g1_msgs/srv/GenerateGrasps '{object_id: red_block_0, hand: right}'"
        staged pick wait_for_arm "ros2 action send_goal /g1_manipulation_server/pick g1_msgs/action/Pick '{object_id: red_block, arm: right}' --feedback"
        ;;
    grounding)
        container bringup "wait_for_port 'the vision server' 5560 && $BRINGUP detector:=vision grounding:=true$LAUNCH_ARGS"
        host detector "./scripts/serve.sh vision --vlm Qwen/Qwen3-VL-2B-Instruct"
        staged ground "wait_for_service /ground_instruction" "ros2 service call /ground_instruction g1_msgs/srv/GroundInstruction \"{instruction: 'pick up the white cup next to the green cylinder'}\""
        staged objects "" "$OBJECTS"
        ;;
    *) usage ;;
esac
open_panes "Open-vocabulary perception: $VARIANT"
