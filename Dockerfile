# ==================================
# Stage 1: build the application
# ==================================
FROM ros:galactic-ros-core-focal AS builder
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=galactic

# (1) Install Python 3.8 and colcon (default for Focal) along with necessary ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.8 python3-pip python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-rclpy \
    && rm -rf /var/lib/apt/lists/*

RUN pip install dynamixel_sdk

# (3) Copy and build your package
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select em_robot


# =====================================
# Stage 2: final image (runtime)
# =====================================
FROM ros:galactic-ros-core-focal AS runtime
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=galactic

# Install Python 3.8, pip3, and then upgrade pip and install dynamixel_sdk
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.8 python3-pip \
    && rm -rf /var/lib/apt/lists/*

### libcamera installation procedure:
RUN sudo apt-get update && sudo apt install -y libboost-dev \
    libgnutls28-dev openssl libtiff5-dev pybind11-dev \
    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
    meson cmake \
    python3-yaml python3-ply \
    libglib2.0-dev libgstreamer-plugins-base1.0-dev


# Upgrade pip and install dynamixel_sdk using pip3
RUN pip3 install --upgrade pip && pip3 install dynamixel_sdk

# Copy the installed workspace from the builder stage
COPY --from=builder /ros2_ws/install /ros2_ws/install

# Copy the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint to source the environments and run the launch command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]

