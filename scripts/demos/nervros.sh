#!/usr/bin/env bash
# Talking to the robot with NervROS: docs/guides/nervros.md.
#
#   ./scripts/demos/nervros.sh <variant> [name:=value...] [--print]
#
#   app       the apartment with the world model and the mission executor, and the NervROS app:
#             look, find things, walk to rooms and objects
#   chat      the same, with NervROS in the terminal instead of the app
#   facility  pick and place: the facility world's workbench and storage bench, and the app
#   stop      end whichever demo is running
#
# NervROS runs on the host (workspace/src/nervros) and answers with a local model, which
# local-llm.sh starts in its own container. The first run builds the app, which takes about
# twenty minutes. name:=value arguments go to the bringup launch. --print lists the commands
# instead of opening them.
source "$(dirname "$0")/../lib/panes.sh"
parse_args "$@"

WORLD=/root/data/worlds/nervros
BRINGUP="ros2 launch g1_bringup bringup.launch.py mode:=mapping nav:=true world:=apartment headless:=true rviz:=false arms_at_sides:=true cameras:=head$LAUNCH_ARGS"
CANOPY="ros2 launch g1_bringup world_model.launch.py world_dir:=$WORLD rviz:=false detector:=mock cameras:=head"
# Both stacks take the arms themselves at start with empty hands, so the executor may trust them.
EXECUTOR="wait_for_nav2 && ros2 launch g1_orchestration nervros_executor.launch.py hands_empty_on_attach:=true"
PROFILE="$ROOT/workspace/src/g1_bringup/config/nervros/nervros.toml"
# The pick-and-place mission's stack; the detector looks for the two objects from the start.
FACILITY="ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true moveit:=true manipulation:=true world:=navigation perception:=true detector:=mock headless:=true rviz:=false activate_arm:=true activate_arm_delay_s:=8.0 phrases:='red_block=bright red plastic ball,brown_box=brown box container'$LAUNCH_ARGS"
# The graph runs as root in the container: host shared memory would fail, so UDP only. The build
# goes outside the colcon workspace, which would otherwise crawl its tens of gigabytes.
AGENT_ENV="cd workspace/src/nervros && ./scripts/build-overlay.sh ../g1_msgs ../canopy/canopy_msgs && export NERVROS_UDP_ONLY=1 NERVROS_EXTRA_IDL_PACKAGES='g1_msgs;canopy_msgs' ROS_DOMAIN_ID=1 CARGO_TARGET_DIR=\$HOME/.cache/nervros/target && source scripts/ros-env.sh"

case "$VARIANT" in
    app | chat)
        container bringup "$BRINGUP"
        container canopy "$CANOPY"
        container executor "$EXECUTOR"
        host model "workspace/src/nervros/scripts/local-llm.sh start"
        if [[ "$VARIANT" == app ]]; then
            host nervros "$AGENT_ENV && cargo run -p nervros-gui -- --profile $PROFILE"
        else
            host nervros "$AGENT_ENV && cargo run -p nervros-cli -- --profile $PROFILE chat"
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
