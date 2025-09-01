# --- Derived app image, built on your Jazzy base (with cam venv) ---
FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]

# Put the camera venv first on PATH; this is the Python you'll run at runtime
ENV ROS_DISTRO=jazzy
ENV PATH="/opt/camvenv/bin:${PATH}"
ENV PYTHONUNBUFFERED=1

# System deps you need at runtime + build helpers
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-smbus i2c-tools python3-dev python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# --- All Python deps go into the /opt/camvenv venv explicitly ---
RUN /opt/camvenv/bin/python -m pip install --upgrade --no-cache-dir pip setuptools wheel \
 && /opt/camvenv/bin/python -m pip install --no-cache-dir \
      smbus2 \
      "numpy<2.0" \
      "opencv-python-headless==4.10.*" \
      colcon-core colcon-common-extensions \
 && /opt/camvenv/bin/python - <<'PY'
import sys, cv2
print("Python:", sys.executable)
print("OpenCV:", cv2.__version__)
PY

# Fast-DDS profile (optional)
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

# Build with the venv's colcon so generated shebangs point to /opt/camvenv/bin/python
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && which python && python -V \
 && /opt/camvenv/bin/colcon build --packages-select em_robot bno055 em_robot_srv

# ---- Entrypoint: source ROS + venv, then launch via the venv python ----
RUN cat > /entrypoint.sh << 'SH'
#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# ROS env
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Activate the venv where cv2 (and any camera libs) live
source /opt/camvenv/bin/activate

# Overlay workspace (whose console_scripts were generated for this venv)
source /ros2_ws/install/setup.bash

# Launch using the venv interpreter to avoid ambiguity
exec python -m ros2 launch em_robot em_robot.launch.py
SH
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
