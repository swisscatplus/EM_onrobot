# syntax=docker/dockerfile:1.7

FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# --- System & ROS deps (NO 'meson' here) ---
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential ninja-build pkg-config cmake git \
      python3 python3-pip python3-dev python3-setuptools python3-wheel \
      python3-colcon-common-extensions python3-jinja2 python3-yaml python3-ply \
      ros-${ROS_DISTRO}-tf-transformations \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-example-interfaces ros-${ROS_DISTRO}-rclpy \
      libdrm-dev libexpat1-dev libjpeg-dev libpng-dev libtiff5-dev \
      libgnutls28-dev openssl libcap-dev libv4l-dev \
      libavcodec-dev libavformat-dev libswscale-dev \
      libglib2.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base \
      qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
      libfmt-dev python3-prctl \
      python3-smbus i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# --- Python deps (Meson from pip >=1.0) ---
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --upgrade pip && \
    pip3 install --no-cache-dir "meson>=1.4" dynamixel_sdk opencv-python smbus2 "numpy<1.24.0"

# --- libcamera (from source) ---
WORKDIR /packages
# Optionally pin: ARG LIBCAMERA_REF=v0.3.2 and add --branch ${LIBCAMERA_REF}
RUN git clone --depth 1 https://github.com/raspberrypi/libcamera.git
WORKDIR /packages/libcamera
RUN python3 -m mesonbuild.mesonmain setup build --buildtype=release \
      -Dpipelines=rpi/vc4,rpi/pisp \
      -Dipas=rpi/vc4,rpi/pisp \
      -Dv4l2=true \
      -Dgstreamer=enabled \
      -Dtest=false \
      -Dlc-compliance=disabled \
      -Dcam=disabled \
      -Dqcam=disabled \
      -Ddocumentation=disabled \
      -Dpycamera=enabled && \
    ninja -C build && ninja -C build install && ldconfig

# --- kmsxx (optional) ---
WORKDIR /packages
RUN git clone --depth 1 https://github.com/tomba/kmsxx.git
WORKDIR /packages/kmsxx
RUN git submodule update --init && \
    python3 -m mesonbuild.mesonmain setup build -Dpykms=enabled && \
    ninja -C build && ninja -C build install && ldconfig

# --- Picamera2 (from source; matches libcamera) ---
WORKDIR /packages
# Optionally pin: ARG PICAMERA2_REF=<tag>; add --branch ${PICAMERA2_REF}
RUN git clone --depth 1 https://github.com/raspberrypi/picamera2.git
WORKDIR /packages/picamera2
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --no-cache-dir .

# Debug logging if needed
ENV LIBCAMERA_LOG_LEVELS="*:INFO"

# --- Fast DDS config ---
RUN mkdir -p /root/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# --- Build your ROS workspace (last, changes often) ---
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select em_robot bno055 em_robot_srv

# --- Entrypoint ---
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
