FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages/

# Build your ROS application
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select em_robot

# Install robot_localization package
RUN apt-get update && apt-get install ros-${ROS_DISTRO}-robot-localization

# Copy the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
#CMD ["tail", "-f", "/dev/null"]
