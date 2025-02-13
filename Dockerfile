# ==================================
# ===== Stage 1: build l'application
# ==================================
FROM ros:humble-ros-core-jammy AS builder
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y --no-install-recommends \
    python3.12 python3.12-distutils python3.12-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-example-interfaces \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3.12 get-pip.py && \
    rm get-pip.py

WORKDIR /ros2_ws
COPY src/EM_OnRobot src/rpi_pkg

RUN source /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select rpi_pkg


# =====================================
# ===== Stage 2: image finale (runtime)
# =====================================
FROM ros:humble-ros-core-jammy AS runtime
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

COPY --from=builder /ros2_ws/install /ros2_ws/install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rpi_pkg", "rpi.launch.py"]
