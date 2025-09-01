# --- App image built on your Jazzy base (with /opt/camvenv present) ---
FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=jazzy
# Make the venv tools first on PATH (nice-to-have); we still call them explicitly
ENV PATH="/opt/camvenv/bin:${PATH}"
ENV PYTHONUNBUFFERED=1


# System deps used at runtime and for building
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-dev python3-smbus i2c-tools build-essential git \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ---------- Python deps (ALL into /opt/camvenv) ----------
# Put deps in a separate layer to keep cache when only src/ changes
COPY requirements.txt /tmp/requirements.txt
RUN /opt/camvenv/bin/python -m pip install --upgrade --no-cache-dir pip setuptools wheel \
 && /opt/camvenv/bin/python -m pip install --no-cache-dir -r /tmp/requirements.txt \
 && /opt/camvenv/bin/python - <<'PY'
import sys, importlib.util
print("Using Python:", sys.executable)
print("Has cv2? ", importlib.util.find_spec("cv2") is not None)
print("Has colcon_core? ", importlib.util.find_spec("colcon_core") is not None)
PY

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
# Use module form to avoid relying on the console script shebang
RUN test -f "/opt/ros/${ROS_DISTRO}/setup.sh" || (echo "Missing /opt/ros/${ROS_DISTRO}/setup.sh" && exit 2) \
 && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
 && /opt/camvenv/bin/python -V \
 && /opt/camvenv/bin/python -m colcon version-check || true \
 && /opt/camvenv/bin/python -m colcon build --packages-select em_robot bno055 em_robot_srv

# ---- Entrypoint ----
RUN cat > /entrypoint.sh << 'SH'
#!/usr/bin/env bash
set -eo pipefail                      # <- no -u
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# Make sure these are defined (harmless if already set)
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export COLCON_CURRENT_PREFIX=${COLCON_CURRENT_PREFIX:-}

# Source environments
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/camvenv/bin/activate
source /ros2_ws/install/setup.bash

# Launch
exec python -m ros2 launch em_robot em_robot.launch.py
SH
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
