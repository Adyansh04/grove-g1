#!/bin/bash
#
# Unified script for managing the ROS development Docker container lifecycle.
#
# Usage: ROS_DISTRO=jazzy ./scripts/manage.sh [start|stop|restart|recreate|logs|exec|exec-as-me]
#

set -euo pipefail

# --- Configuration ---
SERVICE_NAME="ros-dev"
ENV_FILE="$(cd "$(dirname "$0")/.." && pwd)/.env"

# Load .env first so it wins over any inherited shell values.
if [[ -f "$ENV_FILE" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$ENV_FILE"
    set +a
else
    echo "Warning: no .env file found at $ENV_FILE; falling back to shell environment and Compose defaults." >&2
fi

# --- Helper Functions ---
function print_usage() {
    echo "Usage: ROS_DISTRO=<humble|jazzy|...> $0 [start|stop|restart|recreate|logs|exec|exec-as-me]"
    echo "Note: .env is loaded first when present; otherwise shell environment and Compose defaults are used."
    echo "  start     - Build and start the container in detached mode."
    echo "  stop      - Stop the running container."
    echo "  restart   - Restart the container."
    echo "  recreate  - Stop, remove, and rebuild the container from scratch."
    echo "  logs      - Follow the container's log output."
    echo "  exec      - Attach a bash shell to the running container (root)."
    echo "  exec-as-me - Same, as your host UID/GID. Use for clang-tidy --fix / clang-format -i."
}

# --- Main Logic ---
ACTION=${1:-"help"}

case "$ACTION" in
    start)
    echo "Starting ROS development container for ROS_DISTRO='${ROS_DISTRO:-<from defaults>}'..."
    echo "Granting GUI access (X11)..."
    xhost +local:docker 2>/dev/null || true
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
        echo "Attaching bash to Compose service '${SERVICE_NAME}'..."
        docker compose exec "${SERVICE_NAME}" bash
        ;;
    exec-as-me)
        # Same shell, but as the host's UID/GID instead of root. Use this for anything that
        # REWRITES sources in place -- clang-tidy --fix, clang-format -i -- because a
        # root-run tool flips the file's owner on the host and blocks the next host edit.
        # Everything else should keep using `exec`: root is what makes /root/workspace,
        # /root/.ssh and unrestricted /dev work as documented.
        echo "Attaching bash as $(id -u):$(id -g) to '${SERVICE_NAME}'..."
        docker compose exec --user "$(id -u):$(id -g)" "${SERVICE_NAME}" bash
        ;;
    *)
        print_usage
        exit 1
        ;;
esac
