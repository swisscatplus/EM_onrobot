#!/bin/bash

CONTAINER_NAME="em_robot"
IMAGE_NAME="ghcr.io/swisscatplus/em_onrobot/em_robot:latest"

git fetch origin
git reset --hard origin/main

echo "Stopping old container..."
docker stop $CONTAINER_NAME 2>/dev/null || true

echo "Removing old container..."
docker rm $CONTAINER_NAME 2>/dev/null || true

echo "Build image..."
docker build -t $IMAGE_NAME .

echo "Starting new container..."
docker run -d \
  --network host \
  --name $CONTAINER_NAME \
  --device /dev/input \
  --device /dev/uinput \
  --device /dev/hidraw0 \
  -v /run/udev:/run/udev:ro \
  --cap-add SYS_ADMIN \
  --cap-add NET_ADMIN \
  --privileged \
  $IMAGE_NAME


