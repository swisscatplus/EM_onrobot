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
  work_ubuntu_aruco_test)
    docker compose -f "$COMPOSE_FILE" -f "$COMPOSE_UBUNTU_FILE" --profile work_ubuntu_aruco_test "$ACTION" em_robot_work_ubuntu_aruco_test
    ;;
  work_ubuntu_localization_test)
    docker compose -f "$COMPOSE_FILE" -f "$COMPOSE_UBUNTU_FILE" --profile work_ubuntu_localization_test "$ACTION" em_robot_work_ubuntu_localization_test
    ;;
  home_windows)
    docker compose -f "$COMPOSE_FILE" --profile home_windows "$ACTION" em_robot_home_windows
    ;;
  desktop_replay)
    docker compose -f "$COMPOSE_FILE" --profile desktop_replay "$ACTION" em_robot_desktop_replay
    ;;
  *)
    echo "Unsupported profile: $PROFILE"
    echo "Use one of: work_ubuntu, work_ubuntu_aruco_test, work_ubuntu_localization_test, home_windows, desktop_replay"
    exit 1
    ;;
esac
