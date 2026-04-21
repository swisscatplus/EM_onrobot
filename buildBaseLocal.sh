#!/bin/bash
set -e

LOCAL_BASE_IMAGE="${LOCAL_BASE_IMAGE:-em_robot_base:local}"

echo "Building local base image: $LOCAL_BASE_IMAGE"
docker build -f Dockerfile.base -t "$LOCAL_BASE_IMAGE" .
echo "Done."
