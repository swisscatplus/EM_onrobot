# --- App image au-dessus de la base (libcamera+picamera2 déjà dans /opt/camvenv) ---
FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=jazzy
ENV PATH="/opt/camvenv/bin:${PATH}"
ENV PYTHONUNBUFFERED=1
ENV LD_LIBRARY_PATH="/usr/local/lib/aarch64-linux-gnu:/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Colcon via apt (on l'exécute avec le Python du venv)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-dev python3-smbus i2c-tools build-essential git \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Déps Python app supplémentaires (si besoin)
RUN /opt/camvenv/bin/python -m pip install --no-cache-dir smbus2

# ---------- Code ----------
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055

# ---------- Build ----------
# Build via le Python du venv => shebangs -> /opt/camvenv/bin/python
RUN test -f "/opt/ros/${ROS_DISTRO}/setup.sh" || (echo "Missing /opt/ros/${ROS_DISTRO}/setup.sh" && exit 2) \
 && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
 && /opt/camvenv/bin/python -m colcon version-check || true \
 && /opt/camvenv/bin/python -m colcon build --packages-select em_robot bno055 em_robot_srv

# ---- Entrypoint ----
RUN cat > /entrypoint.sh << 'SH'
#!/usr/bin/env bash
set -eo pipefail
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/root/.ros/fastdds.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

# 1) ROS -> 2) venv -> 3) overlay
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/camvenv/bin/activate
source /ros2_ws/install/setup.bash

exec ros2 launch em_robot em_robot.launch.py
SH
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
