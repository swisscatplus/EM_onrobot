FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=jazzy PYTHONUNBUFFERED=1 LD_LIBRARY_PATH="/usr/local/lib/aarch64-linux-gnu:/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

RUN apt-get update && apt-get install -y --no-install-recommends \
  python3-dev python3-smbus i2c-tools build-essential git \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir smbus2 pyserial dynamixel_sdk

WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && colcon build --packages-select em_robot bno055 em_robot_srv

RUN printf '%s\n' '#!/usr/bin/env bash' 'set -eo pipefail' \
 'export ROS_DISTRO="${ROS_DISTRO:-jazzy}"' \
 'source "/opt/ros/$ROS_DISTRO/setup.bash"' \
 'source /ros2_ws/install/setup.bash' \
 'exec ros2 launch em_robot em_robot.launch.py' > /entrypoint.sh \
 && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
