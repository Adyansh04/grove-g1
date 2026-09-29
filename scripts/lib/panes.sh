# Opens a demo's commands at once in the default terminal: split panes in one tilix window, and a
# window per command in anything else. Sourced by the scripts in scripts/demos:
#
#   source "$(dirname "$0")/../lib/panes.sh"
#   parse_args "$@"
#   case "$VARIANT" in
#       mapping)
#           container bringup "ros2 launch g1_bringup bringup.launch.py mode:=mapping$LAUNCH_ARGS"
#           container teleop "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
#           ;;
#       *) usage ;;
#   esac
#   open_panes
#
# `container` runs a command in the dev container, in /root/workspace with the workspace sourced,
# and it may start with the wait_for_* helpers below. `host` runs one on the host, from the
# repository root. `staged` is for a one-shot such as a goal: it waits, then sends the command
# each time Enter is pressed. A pane stays open in a shell once its command ends or is stopped.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

VARIANT=""
# Appended to a demo's main launch; a repeated name:=value takes the later value.
LAUNCH_ARGS=""
PRINT=false

PANE_WHERE=()
PANE_TITLES=()
PANE_COMMANDS=()
# What the pane shows it runs: a staged pane's command rather than its helpers.
PANE_SHOWN=()

# A demo script's header comment is its usage.
usage() {
    sed -n '2,/^[^#]/{/^#/s/^# \{0,1\}//p}' "$0"
    exit "${1:-1}"
}

# <variant> [name:=value...] [--print], in any order after the variant. `stop` needs no demo.
parse_args() {
    local arg
    for arg in "$@"; do
        case "$arg" in
            --print) PRINT=true ;;
            -h | --help | help) usage 0 ;;
            *:=*) LAUNCH_ARGS+=" $(printf '%q' "$arg")" ;;
            *)
                [[ -z "$VARIANT" ]] || usage
                VARIANT=$arg
                ;;
        esac
    done
    [[ -n "$VARIANT" ]] || usage
    if [[ "$VARIANT" == stop ]]; then
        exec "$ROOT/scripts/clean-stack.sh"
    fi
}

# add_pane <container|host> <title> <command> [<what it shows>]
add_pane() {
    PANE_WHERE+=("$1")
    PANE_TITLES+=("$2")
    PANE_COMMANDS+=("$3")
    PANE_SHOWN+=("${4:-$3}")
}

container() {
    add_pane container "$1" "$2"
}

host() {
    add_pane host "$1" "$2"
}

# staged <title> <wait, or ""> <command>
staged() {
    add_pane container "$1" "${2:+$2 && }send_on_enter $(printf '%q' "$3")" \
        "$3${2:+   # once $2 returns; Enter sends it}"
}

# --- Run inside the container panes -------------------------------------------------------------

# Polls a shell condition every two seconds, saying what it waits for.
wait_for() {
    local what=$1
    shift
    printf 'waiting for %s' "$what"
    until eval "$*" >/dev/null 2>&1; do
        printf .
        sleep 2
    done
    printf ' ready\n'
}

# Active, not merely up: an inactive bt_navigator rejects the first goal.
wait_for_nav2() {
    wait_for Nav2 "ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger | grep -q success=True"
}

wait_for_service() {
    wait_for "$1" "ros2 service list --no-daemon | grep -qx '$1'"
}

wait_for_controller() {
    wait_for "$1" "ros2 control list_controllers | grep -qE '^$1 .*\\<active\\>'"
}

# A host server, by the port it binds once its model has loaded.
wait_for_port() {
    wait_for "$1 on port $2" "(exec 3<>/dev/tcp/127.0.0.1/$2)"
}

# The arm acquired by bringup's activate_arm and swung clear: the controller active and the
# one-shot node gone. A tree started sooner would race the swing.
wait_for_arm() {
    wait_for_controller arm_trajectory_controller
    wait_for "activate_arm to finish" "! ros2 node list --no-daemon | grep -q g1_bringup_activate_arm"
}

send_on_enter() {
    while read -r -p "Enter sends it, Ctrl-D leaves a shell: "; do
        eval "$1"
    done
    echo
}

# --- Opening the terminal ------------------------------------------------------------------------

# What a pane runs. Ctrl-C ends the command, not the pane: `trap :` is reset for the command
# itself, which still gets the signal.
pane_script() {
    local i=$1 banner body
    banner=$(printf 'printf %q %q %q\n' '\n\e[1m%s\e[0m\n$ %s\n\n' "${PANE_TITLES[i]}" "${PANE_SHOWN[i]}")
    printf '#!/usr/bin/env bash\nprintf %q %q\n' '\e]0;%s\a' "${PANE_TITLES[i]}"
    if [[ "${PANE_WHERE[i]}" == host ]]; then
        printf 'cd %q\ntrap : INT\n%s\n%s\nexec "${SHELL:-bash}"\n' "$ROOT" "$banner" "${PANE_COMMANDS[i]}"
    else
        body=$(printf 'trap : INT\n%s\n%s\n%s\nexec bash\n' \
            "$(declare -f wait_for wait_for_nav2 wait_for_service wait_for_controller wait_for_port \
                wait_for_arm send_on_enter)" "$banner" "${PANE_COMMANDS[i]}")
        printf 'exec %q exec bash -c %q\n' "$ROOT/scripts/manage.sh" "$body"
    fi
}

tilix_terminal() {
    printf '{"type": "Terminal", "uuid": "%s", "profile": "%s", "directory": "%s", "overrideCommand": "%s", "readOnly": false, "synchronizedInput": false, "width": 800, "height": 450}' \
        "$(cat /proc/sys/kernel/random/uuid)" "$TILIX_PROFILE" "$ROOT" "${PANE_FILES[$1]}"
}

# Panes $1 to the last, top to bottom, each the same height. tilix wants a split's position in
# pixels as well as its ratio, and without it loads nothing.
tilix_stack() {
    local first=$1 last=$((${#PANE_FILES[@]} - 1)) share
    if ((first == last)); then
        tilix_terminal "$first"
        return
    fi
    share=$((last - first + 1))
    printf '{"type": "Paned", "orientation": 1, "position": %d, "ratio": %s, "child1": %s, "child2": %s}' \
        $((900 / share)) "$(awk -v n=$share 'BEGIN { printf "%.4f", 1 / n }')" \
        "$(tilix_terminal "$first")" "$(tilix_stack $((first + 1)))"
}

# The first pane, the launch that prints the most, on the left half and the rest on the right.
tilix_session() {
    local child
    if ((${#PANE_FILES[@]} == 1)); then
        child=$(tilix_terminal 0)
    else
        child=$(printf '{"type": "Paned", "orientation": 0, "position": 800, "ratio": 0.5, "child1": %s, "child2": %s}' \
            "$(tilix_terminal 0)" "$(tilix_stack 1)")
    fi
    printf '{"type": "Session", "name": "%s", "version": "1.0", "uuid": "%s", "synchronizedInput": false, "width": 1600, "height": 900, "child": %s}\n' \
        "$1" "$(cat /proc/sys/kernel/random/uuid)" "$child"
}

# Whether a pane opens RViz or the simulator's viewer, going by the last value it gives each.
needs_display() {
    local command rviz headless
    for command in "${PANE_COMMANDS[@]}"; do
        rviz=$(grep -o 'rviz:=[a-z]*' <<<"$command" | tail -n 1 || true)
        headless=$(grep -o 'headless:=[a-z]*' <<<"$command" | tail -n 1 || true)
        if [[ "$rviz" == rviz:=true || "$headless" == headless:=false ]]; then
            return 0
        fi
    done
    return 1
}

print_panes() {
    local i where
    for i in "${!PANE_COMMANDS[@]}"; do
        where="on the host, from the repository root"
        [[ "${PANE_WHERE[i]}" == host ]] || where="in the container: ./scripts/manage.sh exec"
        printf '# %s, %s\n%s\n\n' "${PANE_TITLES[i]}" "$where" "${PANE_SHOWN[i]}"
    done
    echo "# The wait_for_* and send_on_enter helpers are in scripts/lib/panes.sh."
}

# open_panes <window title>
open_panes() {
    if $PRINT; then
        print_panes
        return
    fi
    if [[ " ${PANE_WHERE[*]} " == *" container "* ]]; then
        if ! "$ROOT/scripts/manage.sh" exec true 2>/dev/null; then
            echo "The dev container is not running; start it with ./scripts/manage.sh start." >&2
            exit 1
        fi
        # RViz and the viewer draw on this display from inside the container, which can open it
        # only after `xhost +local:docker` (manage.sh start runs it). Without it they die at once
        # and take the whole launch down with them.
        if needs_display && ! "$ROOT/scripts/manage.sh" exec python3 -c \
            'import ctypes, sys; sys.exit(not ctypes.CDLL("libX11.so.6").XOpenDisplay(None))' 2>/dev/null; then
            echo "The container cannot open your display, which RViz and the viewer need." >&2
            echo "Allow it with: xhost +local:docker" >&2
            exit 1
        fi
        # A stack left from an earlier run shares the graph and fails this one at random.
        "$ROOT/scripts/clean-stack.sh"
    fi

    # In the per-user runtime directory, which is emptied at logout.
    local dir i
    dir=$(mktemp -d "${XDG_RUNTIME_DIR:-/tmp}/grove-demo.XXXXXX")
    PANE_FILES=()
    for i in "${!PANE_COMMANDS[@]}"; do
        PANE_FILES+=("$dir/$i-${PANE_TITLES[i]// /-}.sh")
        pane_script "$i" >"${PANE_FILES[i]}"
        chmod +x "${PANE_FILES[i]}"
    done

    # $TERMINAL wins, then Debian's default, x-terminal-emulator; tilix's is tilix.wrapper.
    local terminal
    if ! terminal=$(command -v "${TERMINAL:-x-terminal-emulator}"); then
        echo "No terminal found; set TERMINAL to one." >&2
        exit 1
    fi
    terminal=$(basename "$(readlink -f "$terminal")" .wrapper)
    if [[ "$terminal" == tilix ]] &&
        TILIX_PROFILE=$(gsettings get com.gexperts.Tilix.ProfilesList default 2>/dev/null); then
        TILIX_PROFILE=${TILIX_PROFILE//\'/}
        tilix_session "$1" >"$dir/session.json"
        # On a session it cannot load, tilix opens a bare default window and only logs why.
        python3 -m json.tool "$dir/session.json" >/dev/null
        tilix --maximize --session "$dir/session.json" >"$dir/tilix.log" 2>&1 &
        sleep 2
        if grep -q "Session could not be created" "$dir/tilix.log"; then
            echo "tilix could not load $dir/session.json:" >&2
            grep -m 1 "Session could not be created" "$dir/tilix.log" >&2
            exit 1
        fi
    else
        for i in "${!PANE_FILES[@]}"; do
            "${TERMINAL:-x-terminal-emulator}" -e "${PANE_FILES[i]}" >/dev/null 2>&1 &
        done
    fi
    disown -a
}
