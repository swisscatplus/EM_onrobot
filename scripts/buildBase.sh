#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

docker buildx build \
  -f "$REPO_ROOT/docker/Dockerfile.base" \
  -t ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest \
  --platform linux/arm64 \
  --push \
  "$REPO_ROOT"
