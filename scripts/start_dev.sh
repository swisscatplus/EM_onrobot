#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/docker/compose.yaml"
COMPOSE_UBUNTU_FILE="$REPO_ROOT/docker/compose.ubuntu.yaml"

PROFILE="${1:-work_ubuntu}"
ACTION="${2:-up}"

case "$PROFILE" in
  work_ubuntu)
    docker compose -f "$COMPOSE_FILE" -f "$COMPOSE_UBUNTU_FILE" --profile work_ubuntu "$ACTION" em_robot_work_ubuntu
    ;;
  home_windows)
    docker compose -f "$COMPOSE_FILE" --profile home_windows "$ACTION" em_robot_home_windows
    ;;
  *)
    echo "Unsupported profile: $PROFILE"
    echo "Use one of: work_ubuntu, home_windows"
    exit 1
    ;;
esac
