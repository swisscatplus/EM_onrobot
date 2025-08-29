# Derived image built FROM your Jazzy base
FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest

SHELL ["/bin/bash", "-c"]

# Inherit Jazzy from base; keep it explicit so scripts can rely on it
ENV ROS_DISTRO=jazzy

# ---- Runtime deps you need in this layer ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-smbus i2c-tools python3-dev python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Use the venv pip from the base (/opt/camvenv/bin is on PATH there)
RUN python -m pip install --no-cache-dir smbus2

# ---- Fast-DDS profile (if you’re using it) ----
RUN mkdir -p /root/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
# Set BOTH env vars so either Fast-DDS name is picked up
ENV FASTDDS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# ---- Build your workspace ----
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055

# Source the right ROS (jazzy) and build selected packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --packages-select em_robot bno055 em_robot_srv

# ---- Entrypoint ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
