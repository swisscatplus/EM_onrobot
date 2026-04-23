#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CONTAINER_NAME="em_robot"
IMAGE_NAME="ghcr.io/swisscatplus/em_onrobot/em_robot:latest"

echo "Build and push image..."
docker build -f "$REPO_ROOT/docker/Dockerfile" -t $IMAGE_NAME "$REPO_ROOT"
docker push $IMAGE_NAME

