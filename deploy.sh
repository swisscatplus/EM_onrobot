#!/bin/bash

CONTAINER_NAME="em_robot"
IMAGE_NAME="ghcr.io/swisscatplus/em_onrobot/em_robot:latest"

git fetch origin
git reset --hard origin/main

docker pull ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest

echo "Stopping old container..."
docker stop $CONTAINER_NAME 2>/dev/null || true

echo "Removing old container..."
docker rm $CONTAINER_NAME 2>/dev/null || true

echo "Build image..."
docker build -f Dockerfile -t $IMAGE_NAME .

echo "Starting new container..."

docker run -d \
  --restart unless-stopped \
  --network host \
  --name "$CONTAINER_NAME" \
  -v /run/udev:/run/udev:ro \
  -v /dev/:/dev/ \
  --cap-add SYS_ADMIN \
  --cap-add NET_ADMIN \
  --privileged \
  --device /dev/video0 \
  --device /dev/i2c-1:/dev/i2c-1 \
  --device /dev/video1 \
  --device /dev/media0 \
  --device /dev/media1 \
  --device /dev/dri/card0 \
  --group-add video \
  "$IMAGE_NAME"

