#!/bin/bash
# Source the ROS2 setup script
source /opt/ros/humble/setup.bash
source /home/SwissCat-on_robot/install/setup.bash

# Run the ROS2 node
if [ -z "$namespace" ]; then
  exec ros2 launch rpi_pkg rpi.launch.py
else
  exec ros2 launch rpi_pkg rpi.launch.py namespace:=$namespace
fi

exec "$@"
