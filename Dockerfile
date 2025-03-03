FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages/

# Set up your workspace directory
WORKDIR /ros2_ws

# Copy your application code
COPY src/em_robot src/em_robot

# Clone the robot_localization repository into your workspace (choose the ros2 branch)
RUN git clone -b ros2 https://github.com/cra-ros-pkg/robot_localization.git src/robot_localization

# Build the entire workspace (your app + robot_localization)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

# Copy and set permissions for the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
