FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-tf-transformations

# Build your ROS application
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select em_robot

RUN pip install rpi-libcamera

# Add the directory where libpisp.so.1 is installed to LD_LIBRARY_PATH.
ENV LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

# Copy the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
