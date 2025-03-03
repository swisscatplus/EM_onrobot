FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages/

# Ensure entire ros2_ws is copied
WORKDIR /ros2_ws
COPY --from=ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest /ros2_ws /ros2_ws

# Copy only your specific package
COPY src/em_robot src/em_robot

# Source ROS and build everything
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Copy the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
