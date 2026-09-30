#!/usr/bin/env bash
# Talking to the robot with NervROS: docs/guides/nervros.md.
#
#   ./scripts/demos/nervros.sh <variant> [name:=value...] [--print]
#
#   app       the apartment with canopy's semantic map (rooms, named objects), the mission
#             executor and the NervROS app: look, find things, walk to rooms and objects
#   chat      the same, with NervROS in the terminal instead of the app
#   explore   the apartment with an empty world model: ask the robot to explore and watch the map
#             fill in, in the app's World tab and viewer
#   facility  pick and place: the facility world's workbench and storage bench, and the app
#   stop      end whichever demo is running
#
# NervROS runs on the host (workspace/src/nervros) and answers with a local model, which
# local-llm.sh starts in its own container. The first run builds the app, which takes about
# twenty minutes. name:=value arguments go to the bringup launch. --print lists the commands
# instead of opening them.
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

# canopy writes into its world_dir, so it gets a copy of the committed semantic map, never the
# map itself. The saved world resumes only on the map it was built on, so that map is served.
SAVED=/root/data/worlds/nervros-apartment
EMPTY=/root/data/worlds/nervros-explore
APARTMENT="ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true world:=apartment map:=$SAVED/map.yaml headless:=true rviz:=false arms_at_sides:=true cameras:=head$LAUNCH_ARGS"
CANOPY="ros2 launch g1_bringup world_model.launch.py rviz:=false detector:=mock cameras:=head world_dir:="
# Both stacks take the arms themselves at start with empty hands, so the executor may trust them.
EXECUTOR="wait_for_nav2 && ros2 launch g1_orchestration nervros_executor.launch.py hands_empty_on_attach:=true"
PROFILE="$ROOT/workspace/src/g1_bringup/config/nervros/nervros.toml"
# The pick-and-place mission's stack; the detector looks for the two objects from the start.
FACILITY="ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true moveit:=true manipulation:=true world:=navigation perception:=true detector:=mock headless:=true rviz:=false activate_arm:=true activate_arm_delay_s:=8.0 phrases:='red_block=bright red plastic ball,brown_box=brown box container'$LAUNCH_ARGS"
# The graph runs as root in the container: host shared memory would fail, so UDP only. The build
# goes outside the colcon workspace, which would otherwise crawl its tens of gigabytes. The viewer
# finds the G1's meshes in the colcon build: install/ only links to container paths.
AGENT_ENV="cd workspace/src/nervros && ./scripts/build-overlay.sh ../g1_msgs ../canopy/canopy_msgs && export NERVROS_UDP_ONLY=1 NERVROS_EXTRA_IDL_PACKAGES='g1_msgs;canopy_msgs' ROS_DOMAIN_ID=1 ROS_PACKAGE_PATH=$ROOT/workspace/build CARGO_TARGET_DIR=\$HOME/.cache/nervros/target && source scripts/ros-env.sh"

# The copy is made once and then kept, with whatever the robot adds to it; delete
# data/worlds/nervros-apartment to start again from the committed map. The explore world starts
# empty every time.
prepare_worlds() {
    $PRINT && return
    local saved="$ROOT/data/worlds/nervros-apartment"
    [[ -d "$saved" ]] || cp -r "$ROOT/workspace/src/canopy/canopy/doc/apartment" "$saved"
    if [[ "$VARIANT" == explore ]]; then
        rm -rf "$ROOT/data/worlds/nervros-explore"
        mkdir -p "$ROOT/data/worlds/nervros-explore"
    fi
}

case "$VARIANT" in
    app | chat | explore)
        prepare_worlds
        container bringup "$APARTMENT"
        if [[ "$VARIANT" == explore ]]; then
            container canopy "$CANOPY$EMPTY"
        else
            container canopy "$CANOPY$SAVED"
        fi
        container executor "$EXECUTOR"
        host model "workspace/src/nervros/scripts/local-llm.sh start"
        if [[ "$VARIANT" == chat ]]; then
            host nervros "$AGENT_ENV && cargo run -p nervros-cli -- --profile $PROFILE chat"
        else
            host nervros "$AGENT_ENV && cargo run -p nervros-gui -- --profile $PROFILE"
        fi
        ;;
    facility)
        container bringup "$FACILITY"
        container executor "wait_for_nav2 && wait_for_arm && ros2 launch g1_orchestration nervros_executor.launch.py hands_empty_on_attach:=true"
        host model "workspace/src/nervros/scripts/local-llm.sh start"
        host nervros "$AGENT_ENV && cargo run -p nervros-gui -- --profile $ROOT/workspace/src/g1_bringup/config/nervros/facility.toml"
        ;;
    *) usage ;;
esac
open_panes "NervROS: $VARIANT"
