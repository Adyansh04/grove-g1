#!/bin/bash
#
# Manages the ROS development container and runs commands inside it.
#
# Usage: ./scripts/manage.sh <command> [args]; ./scripts/manage.sh help lists the commands.
#

set -euo pipefail

# --- Configuration ---
SERVICE_NAME="ros-dev"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENV_FILE="$ROOT/.env"

# Compose looks for docker-compose.yml in the working directory, whatever the caller's.
cd "$ROOT"

# Load .env first so it wins over any inherited shell values.
if [[ -f "$ENV_FILE" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$ENV_FILE"
    set +a
else
    echo "Warning: no .env file found at $ENV_FILE; falling back to shell environment and Compose defaults." >&2
fi

# Our packages. The vendored ones' lint targets fail by design.
OUR_PACKAGES='^(g1_|canopy)'

# Run before every command in the container: ROS, then the workspace once it has been built.
IN_WORKSPACE='source "/opt/ros/$ROS_DISTRO/setup.bash" && cd /root/workspace && if [[ -f install/setup.bash ]]; then source install/setup.bash; fi'

# What format and lint cover: our C++ and Python, the host servers, and canopy's editor and
# servers, which are not ROS packages. Extensionless scripts are picked by their shebang, since
# ruff reads such a file only when it is named. ruff takes each file's nearest ruff.toml, so
# canopy's files are held to canopy's own settings.
STYLE_SOURCES='
    shopt -s globstar nullglob
    cpp=(src/g1_*/include/**/*.hpp src/g1_*/src/**/*.cpp src/g1_*/test/**/*.cpp)
    python=(src/g1_*/{launch,scripts,test,tools} src/canopy/{editor,servers} /root/.host-repo/scripts)
    for file in src/g1_*/scripts/*; do
        if [[ -f $file && $file != *.* ]] && head -c 32 "$file" | grep -q "^#!.*python"; then
            python+=("$file")
        fi
    done
'

# --- Helper Functions ---
# The image builds the simulator from the unitree_mujoco checkout: refuse one that is not the
# commit this repo records ('+' in git submodule status), or was never checked out ('-').
function check_simulator_checkout() {
    local status
    status=$(git -C "$ROOT" submodule status workspace/vendor/unitree_mujoco 2>/dev/null || true)
    if [[ "$status" == [+-]* ]]; then
        echo "workspace/vendor/unitree_mujoco is not at the commit this repo records:" >&2
        echo "  git submodule update --init workspace/vendor/unitree_mujoco" >&2
        exit 1
    fi
}

# Runs a command in the container, in /root/workspace with ROS and the workspace sourced.
function in_container() {
    local tty=()
    [[ -t 0 && -t 1 ]] || tty=(-T)
    docker compose exec "${tty[@]}" "${SERVICE_NAME}" bash -c "$IN_WORKSPACE && exec \"\$@\"" bash "$@"
}

# The named packages, or the whole workspace. Capped: uncapped, the compilers run the machine out
# of memory and the OOM killer takes the desktop session with them.
function build() {
    local select=()
    (( $# )) && select=(--packages-select "$@")
    in_container env MAKEFLAGS="${MAKEFLAGS:--j4}" colcon build --symlink-install --parallel-workers 4 \
        "${select[@]}" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

# Our packages' tests, or the named packages', without the simulator suites. With --sim, only
# those, after clean-stack.sh: a stack left on the graph fails them at random.
function run_tests() {
    local labels=(-LE simulator)
    if [[ "${1:-}" == --sim ]]; then
        shift
        labels=(-L simulator)
        "$ROOT/scripts/clean-stack.sh"
    fi
    local select=(--packages-select-regex "$OUR_PACKAGES")
    (( $# )) && select=(--packages-select "$@")
    # One package at a time, as CI runs them. Results from earlier runs are cleared first, or
    # they would be reported as this run's.
    in_container bash -c '
        labels=("$1" "$2")
        shift 2
        mapfile -t packages < <(colcon list --names-only "$@")
        for package in "${packages[@]}"; do rm -rf "build/$package/test_results"; done
        status=0
        colcon test --packages-select "${packages[@]}" --executor sequential \
            --return-code-on-test-failure --ctest-args "${labels[@]}" || status=$?
        for package in "${packages[@]}"; do
            [[ -d "build/$package/test_results" ]] || continue
            summary=$(colcon test-result --test-result-base "build/$package" | tail -n 1)
            printf "%-24s %s\n" "$package" "$summary"
            if [[ "$summary" != *" 0 errors, 0 failures"* ]]; then
                colcon test-result --test-result-base "build/$package" --verbose | sed "\$d"
            fi
        done
        exit "$status"
    ' bash "${labels[@]}" "${select[@]}"
}

# clang-format writes a new file, so the C++ comes back owned by root: give it back to the user
# who owns the checkout. ruff rewrites in place and keeps the owner.
function format_sources() {
    in_container bash -c "$STYLE_SOURCES"'
        clang-format --style=file:.clang-format -i "${cpp[@]}"
        chown "$1" "${cpp[@]}"
        ruff format --quiet --no-cache "${python[@]}"
    ' bash "$(id -u):$(id -g)"
}

function lint_sources() {
    in_container bash -c "$STYLE_SOURCES"'
        status=0
        clang-format --style=file:.clang-format --dry-run --Werror "${cpp[@]}" || status=1
        ruff format --check --no-cache "${python[@]}" || status=1
        ruff check --no-cache "${python[@]}" || status=1
        exit "$status"
    '
}

function print_usage() {
    cat <<EOF
Usage: $0 <command> [args]
.env is loaded first when present; otherwise the shell environment and Compose defaults apply.

  start                 Build and start the container in detached mode.
  stop                  Stop the running container.
  restart               Restart the container.
  recreate              Stop, remove, and rebuild the container from scratch.
  logs                  Follow the container's log output.
  exec [cmd...]         A bash shell in /root/workspace with the workspace sourced, or run cmd there.
  build [pkg...]        colcon build the named packages, or the whole workspace.
  test [--sim] [pkg...] Our packages' tests without a simulator, or with --sim only the simulator suites.
  format                clang-format our C++ and ruff format our Python, in place.
  lint                  Check both, and run ruff check, without a full colcon test.
EOF
}

# --- Main Logic ---
ACTION=${1:-"help"}

case "$ACTION" in
    start)
    echo "Starting ROS development container for ROS_DISTRO='${ROS_DISTRO:-<from defaults>}'..."
    echo "Granting GUI access (X11)..."
    xhost +local:docker 2>/dev/null || true
    check_simulator_checkout
    docker compose up -d --build "${SERVICE_NAME}"
    echo
    echo "Active services:"
    docker compose ps
    echo
    echo "Service '${SERVICE_NAME}' started. Use './scripts/manage.sh exec' to open a shell."
        ;;
    stop)
        echo "Stopping container..."
        docker compose stop
        echo "Stopped."
        ;;
    restart)
        echo "Restarting container..."
        docker compose restart
        echo "Restarted."
        ;;
    recreate)
    echo "Recreating container for ROS_DISTRO='${ROS_DISTRO:-<from defaults>}'..."
    echo "Code and data on host are preserved: ./workspace, ./data"
    echo "Granting GUI access (X11)..."
    xhost +local:docker 2>/dev/null || true
    check_simulator_checkout
    docker compose down
    docker compose up -d --build "${SERVICE_NAME}"
    echo
    echo "Active services:"
    docker compose ps
    echo "Recreated."
        ;;
    logs)
        echo "Following container logs (Ctrl+C to exit)..."
        docker compose logs -f
        ;;
    exec)
        shift
        if (( $# == 0 )); then
            echo "Attaching bash to Compose service '${SERVICE_NAME}'..."
            set -- bash
        fi
        in_container "$@"
        ;;
    build)
        shift
        build "$@"
        ;;
    test)
        shift
        run_tests "$@"
        ;;
    format)
        format_sources
        ;;
    lint)
        lint_sources
        ;;
    *)
        print_usage
        exit 1
        ;;
esac
