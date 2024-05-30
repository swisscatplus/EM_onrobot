#!/bin/bash
# Source the ROS2 setup script
source /opt/ros/humble/setup.bash
source /home/SwissCat-on_robot/install/setup.bash

# Run the ROS2 node
exec "$@"

