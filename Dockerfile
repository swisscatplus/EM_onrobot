FROM ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
SHELL ["/bin/bash", "-c"]

# make sure the venv is the default python
ENV PATH="/opt/camvenv/bin:${PATH}"
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-smbus i2c-tools python3-dev python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --no-cache-dir smbus2

RUN mkdir -p /root/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
ENV FASTDDS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055

# Use the venv python while building, so entry points use /opt/camvenv/bin/python
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && which python && python -V \
 && colcon build --packages-select em_robot bno055 em_robot_srv

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
