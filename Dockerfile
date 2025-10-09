# ========= Base =========
FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# (optional) silence the locale warning you were seeing
RUN apt-get update && apt-get install -y locales && \
    sed -i 's/^# *en_US.UTF-8/en_US.UTF-8/' /etc/locale.gen && \
    locale-gen
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ========= System & ROS deps =========
RUN apt-get update && apt-get install -y --no-install-recommends \
    # build toolchain
    build-essential ninja-build pkg-config cmake git pybind11-dev \
    # python
    python3 python3-pip python3-dev python3-setuptools python3-wheel \
    python3-colcon-common-extensions python3-jinja2 python3-yaml python3-ply \
    # ROS pkgs
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-example-interfaces ros-${ROS_DISTRO}-rclpy \
    # libcamera deps
    libdrm-dev libexpat1-dev libjpeg-dev libpng-dev libtiff5-dev \
    libgnutls28-dev openssl libcap-dev libv4l-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libglib2.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base \
    # (optional preview libs)
    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
    # misc / useful
    python3-prctl ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# ========= Python tooling =========
RUN python3 -m pip install --upgrade pip && \
    pip3 install --no-cache-dir meson dynamixel_sdk opencv-python

# ========= libcamera (from source) =========
WORKDIR /packages
RUN git clone https://github.com/raspberrypi/libcamera.git
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
    -Dpycamera=enabled
RUN ninja -C build && ninja -C build install && ldconfig

# ========= kmsxx Python binding via PyPI (rpi-kms) =========
# This builds and installs the 'pykms' (aka 'kms') module for Python on Ubuntu 22.04.
# Official package & instructions: https://pypi.org/project/rpi-kms/
RUN pip3 install --no-cache-dir --upgrade "rpi-kms>=0.1a1"

# Sanity check: ensure pykms/kms is importable now
RUN python3 - <<'PY'
import importlib.util as iu
print("libcamera:", iu.find_spec("libcamera"))
print("picamera2 (expected None until installed):", iu.find_spec("picamera2"))
print("kms:", iu.find_spec("kms"))
print("pykms:", iu.find_spec("pykms"))
assert iu.find_spec("kms") or iu.find_spec("pykms"), "Missing kms/pykms (kmsxx Python binding)"
PY

# ========= Picamera2 (from source; matches built libcamera) =========
WORKDIR /packages
RUN git clone https://github.com/raspberrypi/picamera2.git
WORKDIR /packages/picamera2
RUN pip3 install --no-cache-dir .

# Optional sanity check
RUN python3 - <<'PY'
import importlib.util as iu
print("picamera2:", iu.find_spec("picamera2"))
assert iu.find_spec("picamera2"), "Picamera2 not importable"
PY

# ========= Your extra Python bits =========
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-smbus i2c-tools \
 && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir smbus2 'numpy<1.24.0'

# (Optional) helpful logs while debugging libcamera
ENV LIBCAMERA_LOG_LEVELS="*:INFO"

# ========= Fast DDS config =========
RUN mkdir -p /root/.ros
COPY fastdds.xml /root/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# ========= Build your ROS workspace =========
WORKDIR /ros2_ws
COPY src/em_robot src/em_robot
COPY src/em_robot_srv src/em_robot_srv
COPY src/bno055 src/bno055
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select em_robot bno055 em_robot_srv

# ========= Entrypoint =========
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "em_robot", "em_robot.launch.py"]
