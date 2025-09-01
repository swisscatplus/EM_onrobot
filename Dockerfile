# --- Derived app image, built on your Jazzy base (with cam venv) ---
FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]

# Use Jazzy and put the venv first on PATH
ENV ROS_DISTRO=jazzy
ENV PATH="/opt/camvenv/bin:${PATH}"

# Runtime deps + colcon in the venv (so build runs under venv Python)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-smbus i2c-tools python3-dev python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
 && python -m pip install --no-cache-dir smbus2 \
 && python -m pip install --no-cache-dir colcon-core colcon-common-extensions

# Fast-DDS profile (facultatif)
RUN mkdir -p /root/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
ENV FASTDDS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ---- Build your workspace under the venv Python ----
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055

# Build with the venv colcon to ensure shebangs -> /opt/camvenv/bin/python
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && which python && python -V \
 && /opt/camvenv/bin/colcon build --packages-select em_robot bno055 em_robot_srv

# ---- Entrypoint: source ROS + venv, then run ros2 via venv python ----
RUN cat > /entrypoint.sh << 'SH'
#!/usr/bin/env bash
set -euo pipefail
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"



# ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# venv where cv2/picamera2 live
source /opt/camvenv/bin/activate
# overlay workspace
source /ros2_ws/install/setup.bash

# Launch via venv Python (no ambiguity)
exec python -m ros2 launch em_robot em_robot.launch.py
SH
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
