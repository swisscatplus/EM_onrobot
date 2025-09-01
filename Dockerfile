# --- App image built on your Jazzy base (with /opt/camvenv present) ---
FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=jazzy
ENV PATH="/opt/camvenv/bin:${PATH}"
ENV PYTHONUNBUFFERED=1

# Runtime deps (colcon from apt; we'll force venv python via env var)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-dev python3-smbus i2c-tools build-essential git \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Python deps INTO the venv (only what the app needs on top of base)
RUN /opt/camvenv/bin/python -m pip install --no-cache-dir smbus2

# Fast-DDS profile (optional)
RUN mkdir -p /root/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
ENV FASTDDS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ---------- Source ----------
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055

# ---------- Build ----------
# Use system colcon but FORCE the venv interpreter for ament_python
RUN test -f "/opt/ros/${ROS_DISTRO}/setup.sh" || (echo "Missing /opt/ros/${ROS_DISTRO}/setup.sh" && exit 2) \
 && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
 && COLCON_PYTHON_EXECUTABLE=/opt/camvenv/bin/python colcon build --packages-select em_robot bno055 em_robot_srv

# ---- Entrypoint ----
RUN cat > /entrypoint.sh << 'SH'
#!/usr/bin/env bash
set -eo pipefail
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# Source envs (ROS -> venv -> overlay)
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/camvenv/bin/activate
source /ros2_ws/install/setup.bash

# Extra safety: ensure libcamera (installed system-wide) is visible to the venv
pyver="$(python - <<'PY'
import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")
PY
)"
export PYTHONPATH="/usr/local/lib/python${pyver}/site-packages:/usr/local/lib/python${pyver}/dist-packages:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

# Launch
exec ros2 launch em_robot em_robot.launch.py
SH
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
