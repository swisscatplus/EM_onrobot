FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

RUN apt update && apt install -y python3-smbus i2c-tools
RUN pip install smbus2

ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages/

# === ADD FASTDDS CONFIG FILE ===
RUN mkdir -p /root/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# === BUILD ROS WORKSPACE ===
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/bno055 src/bno055
COPY src/em_robot_srv src/em_robot_srv
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select em_robot bno055

# === ENTRYPOINT ===
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
