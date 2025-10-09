# ========= Base =========
FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# (optional) clean locale warning in logs
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
    # (optional, only if you want Qt preview tools available)
    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
    # kmsxx deps
    libfmt-dev \
    # Picamera2-on-Ubuntu nicety
    python3-prctl \
    # misc
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Python deps (global) — use pip Meson (new enough)
RUN python3 -m pip install --upgrade pip && \
    pip3 install --no-cache-dir meson dynamixel_sdk opencv-python

# ========= libcamera (from source) =========
WORKDIR /packages
RUN git clone https://github.com/raspberrypi/libcamera.git
WORKDIR /packages/libcamera
# Use pip Meson explicitly to avoid old system meson confusion
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

# Make sure Python can find libcamera (& later kmsxx)
ENV PYTHONPATH="/usr/local/lib/python3.10/site-packages:/usr/local/lib/python3.10/dist-packages:/usr/local/lib/python3/dist-packages:${PYTHONPATH}"

# ========= kmsxx (with Python bindings) =========
WORKDIR /packages
RUN git clone https://github.com/tomba/kmsxx.git
WORKDIR /packages/kmsxx
RUN git submodule update --init
# IMPORTANT: use boolean 'true' here; 'enabled' may not create the target
RUN python3 -m mesonbuild.mesonmain setup build -Dpykms=true
RUN ninja -C build && ninja -C build install && ldconfig

# Fail fast if kms/pykms isn't importable; also print candidates path
RUN python3 - <<'PY'
import importlib.util as iu, sys, glob
print("sys.path:"); [print("  ", p) for p in sys.path]
print("libcamera:", iu.find_spec("libcamera"))
print("kms      :", iu.find_spec("kms"))
print("pykms    :", iu.find_spec("pykms"))
print("candidates:", glob.glob("/usr/local/lib/python*/site-packages/*kms*.so"))
assert iu.find_spec("kms") or iu.find_spec("pykms"), "Missing kms/pykms (kmsxx Python binding)"
PY

# ========= Picamera2 (from source; matches built libcamera) =========
WORKDIR /packages
RUN git clone https://github.com/raspberrypi/picamera2.git
WORKDIR /packages/picamera2
RUN pip3 install --no-cache-dir .

# Optional sanity check for picamera2
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

# Helpful logs while debugging libcamera
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
