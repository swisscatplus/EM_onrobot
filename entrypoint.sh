#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash
# Source local workspace
source /ros2_ws/install/setup.bash

# Set ROS Domain ID
export ROS_DOMAIN_ID=10

# Ensure Fast DDS profile path is active (optional but safe)
export FASTRTPS_DEFAULT_PROFILES_FILE=$/root/.ros/fastdds.xml

# Execute passed command
exec "$@"
