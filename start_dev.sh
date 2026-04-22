#!/bin/bash
set -e

PROFILE="${1:-work_ubuntu}"
ACTION="${2:-up}"

case "$PROFILE" in
  work_ubuntu)
    docker compose -f compose.yaml -f compose.ubuntu.yaml --profile work_ubuntu "$ACTION" em_robot_work_ubuntu
    ;;
  home_windows)
    docker compose -f compose.yaml --profile home_windows "$ACTION" em_robot_home_windows
    ;;
  *)
    echo "Unsupported profile: $PROFILE"
    echo "Use one of: work_ubuntu, home_windows"
    exit 1
    ;;
esac
