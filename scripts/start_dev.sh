#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/docker/compose.yaml"
COMPOSE_UBUNTU_FILE="$REPO_ROOT/docker/compose.ubuntu.yaml"

PROFILE="${1:-work_ubuntu_localization_test}"
ACTION="${2:-up}"

case "$PROFILE" in
  work_ubuntu_localization_test)
    docker compose -f "$COMPOSE_FILE" -f "$COMPOSE_UBUNTU_FILE" --profile work_ubuntu_localization_test "$ACTION" em_robot_work_ubuntu_localization_test
    ;;
  *)
    echo "Unsupported profile: $PROFILE"
    echo "Use: work_ubuntu_localization_test"
    exit 1
    ;;
esac
