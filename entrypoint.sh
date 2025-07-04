#!/bin/bash
set -e

# Ensure Fast DDS profile path is active (optional but safe)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash
# Source local workspace
source /ros2_ws/install/setup.bash

# Set ROS Domain ID
export ROS_DOMAIN_ID=10

# Execute passed command
exec "$@"
