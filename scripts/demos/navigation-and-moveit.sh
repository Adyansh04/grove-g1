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
#   robot-mapping   the real G1: its side on the Orin, a pane that stands it up on Enter, SLAM,
#                   RViz, teleop, and a pane that saves the map into data/maps on Enter
#   robot-navigate  the real G1 on a saved map (map:=/root/data/maps/<name>.yaml): its side, the
#                   stand-up pane, MoveIt, AMCL and Nav2, both RViz windows, and a pane that hands
#                   the arms to MoveIt on Enter
#   stop       end whichever demo is running; on the robot, ./scripts/robot.sh release then
#              stops its side, with the robot in its stand
#
# name:=value arguments go to the bringup launch, such as odometry:=ground_truth, or for the robot
# to nav_stack.launch.py. --print lists the commands instead of opening them.
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

BRINGUP="ros2 launch g1_bringup bringup.launch.py"
GOAL="ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}'"
# The robot's side: its container, control (which straightens the waist as it starts), then the
# sensors, and control's log. A stack already running is left as it is.
ROBOT_SIDE="./scripts/robot.sh up && { ./scripts/robot.sh start control; ./scripts/robot.sh start sensors; ./scripts/robot.sh log control; }"
NAV_RVIZ="rviz2 -d /root/workspace/install/g1_navigation/share/g1_navigation/config/g1_navigation.rviz"
# On the robot the collision monitor's high band reads MoveIt's filtered cloud: the raw one holds
# the robot's own head.
ROBOT_NAV="ros2 launch g1_navigation nav_stack.launch.py mode:=localization nav:=true collision_high_cloud:=/filtered_cloud"

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
    robot-mapping)
        host robot "$ROBOT_SIDE"
        host_staged stand "" "./scripts/robot.sh stand"
        robot mapping "ros2 launch g1_navigation nav_stack.launch.py mode:=mapping$LAUNCH_ARGS"
        robot rviz "$NAV_RVIZ"
        robot teleop "wait_for_controller agile_controller && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
        robot_staged "save map" "" "ros2 run nav2_map_server map_saver_cli -f /root/data/maps/map_\$(date +%Y-%m-%d_%H%M)"
        ;;
    robot-navigate)
        if [[ "$LAUNCH_ARGS" != *map:=* ]]; then
            echo "robot-navigate needs the saved map: map:=/root/data/maps/<name>.yaml" >&2
            exit 1
        fi
        host robot "$ROBOT_SIDE"
        host_staged stand "" "./scripts/robot.sh stand"
        robot moveit "ros2 launch g1_moveit_config move_group.launch.py"
        robot nav2 "$ROBOT_NAV$LAUNCH_ARGS"
        robot rviz "$NAV_RVIZ"
        robot "moveit rviz" "ros2 launch g1_moveit_config moveit_rviz.launch.py"
        host_staged arm "" "./scripts/robot.sh arm acquire"
        ;;
    *) usage ;;
esac
open_panes "Navigation and MoveIt: $VARIANT"
