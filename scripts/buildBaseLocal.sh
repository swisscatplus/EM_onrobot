#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LOCAL_BASE_IMAGE="${LOCAL_BASE_IMAGE:-em_robot_base:local}"

echo "Building local base image: $LOCAL_BASE_IMAGE"
docker build -f "$REPO_ROOT/docker/Dockerfile.base" -t "$LOCAL_BASE_IMAGE" "$REPO_ROOT"
echo "Done."
