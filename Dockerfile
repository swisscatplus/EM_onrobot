# --- App image built on your Jazzy base (with /opt/camvenv present) ---
FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=jazzy
ENV PATH="/opt/camvenv/bin:${PATH}"
ENV PYTHONUNBUFFERED=1

# Dépendances utiles (colcon depuis apt OK, on l'exécutera via le python du venv)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-dev python3-smbus i2c-tools build-essential git \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# --- Python deps DANS le venv (garantit cv2 à l'exécution) ---
RUN /opt/camvenv/bin/python -m pip install --no-cache-dir \
      "numpy<2.0" \
      "opencv-python-headless==4.10.*" \
      smbus2 \
 && /opt/camvenv/bin/python - <<'PY'
import sys, cv2
print("Python:", sys.executable)
print("OpenCV:", cv2.__version__)
PY

# Fast-DDS profile (optionnel)
RUN mkdir -p /root/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
ENV FASTDDS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ---------- Code source ----------
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055

# ---------- Build ----------
# IMPORTANT : on exécute colcon avec le Python du venv,
# ainsi les scripts installés auront un shebang -> /opt/camvenv/bin/python
RUN test -f "/opt/ros/${ROS_DISTRO}/setup.sh" || (echo "Missing /opt/ros/${ROS_DISTRO}/setup.sh" && exit 2) \
 && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
 && /opt/camvenv/bin/python -m colcon version-check || true \
 && /opt/camvenv/bin/python -m colcon build --packages-select em_robot bno055 em_robot_srv

# ---- Entrypoint ----
RUN cat > /entrypoint.sh << 'SH'
#!/usr/bin/env bash
set -eo pipefail
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# 1) ROS, 2) venv (cv2/picamera2), 3) overlay
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/camvenv/bin/activate
source /ros2_ws/install/setup.bash

# Sécurité : si libcamera est sous /usr/local, expose-le au venv
pyver="$(python - <<'PY'
import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")
PY
)"
export PYTHONPATH="/usr/local/lib/python${pyver}/site-packages:/usr/local/lib/python${pyver}/dist-packages:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

# Lancement via le binaire ros2 (pas python -m)
exec ros2 launch em_robot em_robot.launch.py
SH
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
