FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-tf-transformations

##############################
# 1. Install system dependencies and build tools
##############################
RUN apt-get install -y --no-install-recommends \
    build-essential \
    ninja-build \
    pkg-config \
    cmake \
    ros-${ROS_DISTRO}-robot-localization \
    python3 python3-pip python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-example-interfaces ros-${ROS_DISTRO}-rclpy \
    python3-yaml python3-ply \
    git python3-jinja2 \
    libboost-dev libgnutls28-dev openssl libtiff5-dev pybind11-dev \
    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
    libglib2.0-dev libgstreamer-plugins-base1.0-dev \
    libdrm-dev libexpat1-dev libjpeg-dev libpng-dev libcap-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    gstreamer1.0-plugins-base && \
    rm -rf /var/lib/apt/lists/*

##############################
# 2. Upgrade pip and install Python dependencies
##############################
RUN pip3 install --upgrade pip && \
    pip3 install dynamixel_sdk meson

##############################
# 3. Clone and build libcamera with Python support
##############################
RUN mkdir /packages
WORKDIR /packages

RUN git clone https://github.com/raspberrypi/libcamera.git
WORKDIR /packages/libcamera
RUN meson setup build --buildtype=release \
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
RUN ninja -C build
RUN ninja -C build install

##############################
# 4. Install kmsxx dependencies and build with Python bindings
##############################
WORKDIR /packages
RUN apt-get install -y --no-install-recommends \
    libfmt-dev libdrm-dev libcap-dev && \
    rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/tomba/kmsxx.git
WORKDIR /packages/kmsxx
RUN git submodule update --init
RUN meson setup build -Dpykms=enabled
RUN ninja -C build && ninja -C build install

##############################
# 5. Install additional Python packages (picamera2 and opencv-python)
##############################
RUN pip3 install picamera2 opencv-python

# (Optionally, if needed, you can set PYTHONPATH or LD_LIBRARY_PATH here)
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages/
ENV LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

RUN apt update && apt install -y python3-smbus i2c-tools python3-dev
RUN pip install smbus2
RUN pip install 'numpy<1.24.0'

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
