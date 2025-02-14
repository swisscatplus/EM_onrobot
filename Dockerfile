# ==================================
# Stage 1: build l'application
# ==================================
FROM ros:humble-ros-core-jammy AS builder
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

# (1) Installer Python 3.10 et colcon
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10 python3-pip python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-rclpy \
    && rm -rf /var/lib/apt/lists/*

# (2) Installer d’éventuelles libs Python (exemple : pyserial)
#RUN python3.10 -m pip install --no-cache-dir pyserial

# (3) Copier et compiler ton package
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select em_robot


# =====================================
# Stage 2: image finale (runtime)
# =====================================
FROM ros:humble-ros-core-jammy AS runtime
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

COPY --from=builder /ros2_ws/install /ros2_ws/install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
