FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

RUN pip3 install transforms3d

# Build your ROS application
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select em_robot

# Copy the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
