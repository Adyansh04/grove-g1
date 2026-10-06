#!/usr/bin/env bash
# Talking to the robot with NervROS: docs/guides/nervros.md.
#
#   ./scripts/demos/nervros.sh <variant> [name:=value...] [--print]
#
#   app       the apartment with canopy's semantic map (rooms, named objects), the mission
#             executor and the NervROS app: look, find things, walk to rooms and objects, and
#             carry the mug from the dining table to the tray on the office desk
#   chat      the same, with NervROS in the terminal instead of the app
#   explore   the apartment with an empty world model: ask the robot to explore and watch the map
#             fill in, in the app's World tab and viewer
#   map       map a world from nothing: SLAM instead of a saved map, and a new world model that
#             the robot explores and saves as data/worlds/<name> (name:=, default mapped-<world>;
#             world:= picks the simulated world, apartment by default)
#   stop      end whichever demo is running
#
# NervROS runs on the host (workspace/src/nervros) and answers with a local model, which
# local-llm.sh starts in its own container. The first run builds the app, which takes about
# twenty minutes. name:=value arguments go to the bringup launch, but saved:= (app and chat: the
# saved world under data/worlds to run on, nervros-apartment by default) and name:= (map).
# --print lists the commands instead of opening them.
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

# A name:=value this script reads itself, taken out of what goes to the bringup launch.
take_arg() {
    local name=$1 value=$2 kept="" arg
    for arg in $LAUNCH_ARGS; do
        if [[ "$arg" == "$name":=* ]]; then
            value=${arg#*:=}
        else
            kept+=" $arg"
        fi
    done
    LAUNCH_ARGS=$kept
    printf '%s' "$value"
}
SAVED_NAME=$(take_arg saved nervros-apartment)
SIM_WORLD=$(take_arg world apartment)
MAP_NAME=$(take_arg name "mapped-$SIM_WORLD")
[[ "$SAVED_NAME$MAP_NAME" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "world names are letters, digits, - and _" >&2; exit 1; }

# canopy writes into its world_dir, so it gets a copy of the committed semantic map, never the
# map itself. The saved world resumes only on the map it was built on, so that map is served.
SAVED=/root/data/worlds/$SAVED_NAME
EMPTY=/root/data/worlds/nervros-explore
# With the arm stack: the mock detector finds the mug and the tray by body name for the pick skills.
ARMS="moveit:=true manipulation:=true perception:=true detector:=mock activate_arm:=true activate_arm_delay_s:=8.0 phrases:='mug_4=red mug,tray_1=wooden tray'"
APARTMENT="ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true world:=$SIM_WORLD map:=$SAVED/map.yaml headless:=true rviz:=false cameras:=head,chest $ARMS$LAUNCH_ARGS"
# Exploring needs no arms: at the sides, they stay out of both cameras' views.
EXPLORING="ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true world:=$SIM_WORLD map:=$SAVED/map.yaml headless:=true rviz:=false arms_at_sides:=true cameras:=head,chest$LAUNCH_ARGS"
# SLAM builds the map as the robot explores; canopy saves its floor plan with the world.
MAPPING="ros2 launch g1_bringup bringup.launch.py mode:=mapping nav:=true world:=$SIM_WORLD headless:=true rviz:=false arms_at_sides:=true cameras:=head,chest$LAUNCH_ARGS"
MAPPED=/root/data/worlds/$MAP_NAME
CANOPY="ros2 launch g1_bringup world_model.launch.py rviz:=false detector:=mock cameras:=head,chest segmenter:=true world_dir:="
# Both stacks take the arms themselves at start with empty hands, so the executor may trust them.
EXECUTOR="wait_for_nav2 && ros2 launch g1_orchestration nervros_executor.launch.py hands_empty_on_attach:=true"
# The mock detector knows the mug and the tray by body name only, so the plans may name only those.
ARM_EXECUTOR="wait_for_nav2 && wait_for_arm && ros2 launch g1_orchestration nervros_executor.launch.py hands_empty_on_attach:=true arg_choices:='PickObject.object_id=mug_4;PlaceInto.container_id=tray_1'"
PROFILE="$ROOT/workspace/src/g1_bringup/config/nervros/nervros.toml"
# The graph runs as root in the container: host shared memory would fail, so UDP only. The build
# goes outside the colcon workspace, which would otherwise crawl its tens of gigabytes. The viewer
# finds the G1's meshes in the colcon build: install/ only links to container paths.
AGENT_ENV="cd workspace/src/nervros && ./scripts/build-overlay.sh ../g1_msgs ../canopy/canopy_msgs && export NERVROS_UDP_ONLY=1 NERVROS_EXTRA_IDL_PACKAGES='g1_msgs;canopy_msgs' ROS_DOMAIN_ID=1 ROS_PACKAGE_PATH=$ROOT/workspace/build CARGO_TARGET_DIR=\$HOME/.cache/nervros/target && source scripts/ros-env.sh"

# The copy is made once and then kept, with whatever the robot adds to it; delete
# data/worlds/nervros-apartment to start again from the committed map. The explore world starts
# empty every time.
prepare_worlds() {
    $PRINT && return
    local saved="$ROOT/data/worlds/$SAVED_NAME"
    if [[ "$SAVED_NAME" == nervros-apartment ]]; then
        [[ -d "$saved" ]] || cp -r "$ROOT/workspace/src/canopy/canopy/doc/apartment" "$saved"
    elif [[ ! -f "$saved/map.yaml" ]]; then
        echo "no saved world in data/worlds/$SAVED_NAME; map one with: $0 map name:=$SAVED_NAME" >&2
        exit 1
    fi
    if [[ "$VARIANT" == map ]]; then
        # A world already mapped is kept: pick another name, or delete it to map it again.
        if [[ -f "$ROOT/data/worlds/$MAP_NAME/world.yaml" ]]; then
            echo "data/worlds/$MAP_NAME holds a mapped world; run app saved:=$MAP_NAME, or pick another name:=" >&2
            exit 1
        fi
        mkdir -p "$ROOT/data/worlds/$MAP_NAME"
    fi
    if [[ "$VARIANT" == explore ]]; then
        rm -rf "$ROOT/data/worlds/nervros-explore"
        mkdir -p "$ROOT/data/worlds/nervros-explore"
    fi
}

case "$VARIANT" in
    app | chat | explore)
        prepare_worlds
        if [[ "$VARIANT" == explore ]]; then
            container bringup "$EXPLORING"
            container canopy "$CANOPY$EMPTY"
            container executor "$EXECUTOR"
        else
            container bringup "$APARTMENT"
            container canopy "$CANOPY$SAVED"
            container executor "$ARM_EXECUTOR"
        fi
        host model "workspace/src/nervros/scripts/local-llm.sh start"
        # The saved world, for the agent's world edits and the app's Edit world; the explore world
        # has none until its first save.
        [[ "$VARIANT" == explore ]] || host editor "python3 workspace/src/canopy/editor/canopy_editor.py data/worlds/$SAVED_NAME"
        if [[ "$VARIANT" == chat ]]; then
            host nervros "$AGENT_ENV && cargo run -p nervros-cli -- --profile $PROFILE chat"
        else
            host nervros "$AGENT_ENV && cargo run -p nervros-gui -- --profile $PROFILE"
        fi
        ;;
    map)
        prepare_worlds
        container bringup "$MAPPING"
        container canopy "$CANOPY$MAPPED"
        container executor "$EXECUTOR"
        host model "workspace/src/nervros/scripts/local-llm.sh start"
        # canopy saves a minute in; the editor serves the world from then on.
        host editor "until [ -f data/worlds/$MAP_NAME/map.yaml ]; do sleep 5; done; python3 workspace/src/canopy/editor/canopy_editor.py data/worlds/$MAP_NAME"
        host nervros "$AGENT_ENV && cargo run -p nervros-gui -- --profile $PROFILE"
        ;;
    *) usage ;;
esac
open_panes "NervROS: $VARIANT"
