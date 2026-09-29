#!/usr/bin/env bash
# Exploring a building and asking what is where: docs/guides/world-model.md.
#
#   ./scripts/demos/world-model.sh <variant> [name:=value...] [--print]
#
#   mock     explore the apartment with masks cut from the simulator's truth; no GPU
#   real     the same with the real models, served on the host (canopy's servers/setup.sh first)
#   editor   check the saved world by hand in canopy's map editor
#   stop     end whichever demo is running
#
# Both explorations read the head and chest cameras, as the acceptance test does.
# name:=value arguments go to both launches, such as rviz:=false. --print lists the commands
# instead of opening them. Fetch the apartment's assets once first: ./scripts/setup-world-assets.sh
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

WORLD=/root/data/worlds/apartment
CAMERAS="cameras:=head,chest"
PHRASES="\"phrases:=['sofa','dustbin','bed','desk','chair','dining table','bookshelf','mug']\""
BRINGUP="ros2 launch g1_bringup bringup.launch.py mode:=mapping nav:=true world:=apartment headless:=true arms_at_sides:=true $CAMERAS$LAUNCH_ARGS"
EXPLORE="wait_for_nav2 && wait_for_service /canopy/next_viewpoint && ros2 run g1_orchestration g1_bt_executor --ros-args -p tree_file:=\$(ros2 pkg prefix g1_orchestration)/share/g1_orchestration/trees/explore.xml"
ASK="ros2 service call /canopy/find_objects canopy_msgs/srv/FindObjects '{query: dustbin, room: office}'"

case "$VARIANT" in
    mock)
        container bringup "$BRINGUP"
        container head-detector "ros2 run canopy_perception mock_detector --ros-args -r __node:=detector_head -p $PHRASES -r object_poses:=/g1_sensor_relay/object_poses -r depth/image_raw:=/camera/aligned_depth_to_color/image_raw -r camera_info:=/camera/color/camera_info"
        container chest-detector "ros2 run canopy_perception mock_detector --ros-args -r __node:=detector_chest -p $PHRASES -r object_poses:=/g1_sensor_relay/chest/object_poses -r depth/image_raw:=/chest_camera/aligned_depth_to_color/image_raw -r camera_info:=/chest_camera/color/camera_info"
        container canopy "ros2 launch g1_bringup world_model.launch.py world_dir:=$WORLD rviz:=true $CAMERAS$LAUNCH_ARGS"
        container explore "$EXPLORE"
        staged ask "wait_for_service /canopy/find_objects" "$ASK"
        ;;
    real)
        container bringup "$BRINGUP"
        host server "./scripts/serve.sh canopy"
        container canopy "wait_for_port 'the semantic server' 5561 && ros2 launch g1_bringup world_model.launch.py world_dir:=$WORLD rviz:=true detector:=true describe:=true $CAMERAS$LAUNCH_ARGS"
        container explore "$EXPLORE"
        staged ask "wait_for_service /canopy/find_objects" "$ASK"
        ;;
    editor)
        host editor "python3 workspace/src/canopy/editor/canopy_editor.py data/worlds/apartment"
        ;;
    *) usage ;;
esac
open_panes "World model: $VARIANT"
