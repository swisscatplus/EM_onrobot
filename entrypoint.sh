#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash
# Source le workspace local
source /ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=10

# Exécuter la commande passée en arguments (CMD)
exec "$@"
