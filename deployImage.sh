#!/bin/bash

CONTAINER_NAME="em_robot"
IMAGE_NAME="ghcr.io/swisscatplus/em_onrobot/em_robot:latest"

echo "Build and push image..."
docker build -t $IMAGE_NAME .
docker push $IMAGE_NAME


