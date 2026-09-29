#!/usr/bin/env bash
# Navigation and arm planning: docs/guides/navigation-and-moveit.md.
#
#   ./scripts/demos/navigation-and-moveit.sh <variant> [name:=value...] [--print]
#
#   sim        the simulator on its own, with its viewer
#   mapping    SLAM on the facility, driven from the keyboard in the teleop pane
#   navigate   AMCL and Nav2 on the committed map, and a goal to send
#   arm        MoveIt against the LiDAR octomap, the arm acquired, and its release to send
#   both       Nav2 and MoveIt at once, and a goal to send
#   stop       end whichever demo is running
#
# name:=value arguments go to the bringup launch, such as odometry:=ground_truth. --print lists
# the commands instead of opening them.
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

BRINGUP="ros2 launch g1_bringup bringup.launch.py"
GOAL="ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}'"

case "$VARIANT" in
    sim)
        container sim "$BRINGUP headless:=false$LAUNCH_ARGS"
        ;;
    mapping)
        container bringup "$BRINGUP mode:=mapping rviz:=true$LAUNCH_ARGS"
        container teleop "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
        ;;
    navigate)
        container bringup "$BRINGUP mode:=localization nav:=true rviz:=true$LAUNCH_ARGS"
        staged goal wait_for_nav2 "$GOAL"
        ;;
    arm)
        container bringup "$BRINGUP moveit:=true sensors:=true rviz:=true$LAUNCH_ARGS"
        container acquire "wait_for_controller arm_freeze_controller && ros2 launch g1_bringup activate_arm.launch.py"
        staged release "" "ros2 launch g1_bringup deactivate_arm.launch.py"
        ;;
    both)
        container bringup "$BRINGUP mode:=localization nav:=true moveit:=true rviz:=true activate_arm:=true activate_arm_delay_s:=40.0 headless:=false$LAUNCH_ARGS"
        staged goal wait_for_nav2 "$GOAL"
        ;;
    *) usage ;;
esac
open_panes "Navigation and MoveIt: $VARIANT"
