#!/bin/bash

# Check if an argument is provided
if [ -z "$1" ]; then
  echo "Usage: ./run.sh namespace, running without namespace"
  docker run -it --network host --privileged -v /dev/:/dev/ -v /run/udev:/run/udev  yanniscod/robot:og
else
  echo "Running with namespace $1"
  docker run -it --network host --privileged -v /dev/:/dev/ -v /run/udev:/run/udev -e namespace=$1 yanniscod/robot:og
fi
