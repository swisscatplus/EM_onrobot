# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN sudo apt update && sudo apt install -y python3-pip git python3-jinja2

### libcamera installation procedure:
RUN sudo apt-get update && sudo apt install -y libboost-dev \
    libgnutls28-dev openssl libtiff5-dev pybind11-dev \
    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
    meson cmake \
    python3-yaml python3-ply \
    libglib2.0-dev libgstreamer-plugins-base1.0-dev

RUN mkdir /packages
WORKDIR /packages

RUN git clone https://github.com/raspberrypi/libcamera.git

WORKDIR /packages/libcamera

RUN meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled

RUN ninja -C build

RUN sudo ninja -C build install
###

WORKDIR /packages

### install libkms dependencies
RUN sudo apt update && sudo apt install -y --no-install-recommends \
     libfmt-dev libdrm-dev libfmt-dev \
     libcap-dev

# install kmsxx
RUN git clone https://github.com/tomba/kmsxx.git

WORKDIR /packages/kmsxx

RUN git submodule update --init

RUN meson build -Dpykms=enabled

RUN sudo ninja -C build install

RUN pip install opencv-contrib-python \
    picamera2 \
    transforms3d \
    pyserial \
    smbus

RUN sudo apt update && sudo apt install -y --no-install-recommends \
     nano \
     ros-humble-tf-transformations

ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages/
ENV ROS_DOMAIN_ID=10

