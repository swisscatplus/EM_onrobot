FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]

# --- Per-robot Fast DDS configuration (customize IP per robot) ---
RUN mkdir -p /root/.ros
COPY config/fastdds.xml /root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# --- Build your ROS workspace (changes often during development) ---
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select em_robot bno055 em_robot_srv

# --- Default command ---
CMD ["ros2", "launch", "em_robot", "bringup.launch.py"]
