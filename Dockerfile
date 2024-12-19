# Use the official ROS 2 Humble image as the base
FROM ros:humble-ros-core-jammy

# Set environment variables for ROS 2
ENV ROS_DISTRO=humble

# Update and install required tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

RUN pip install pyserial

# Set the working directory inside the container
WORKDIR /ros2_ws

# Copy your ROS 2 package to the workspace
COPY src/rpi_pkg src/rpi_pkg

# Build the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# Source the ROS 2 setup and workspace setup when launching the container
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Automatically source ROS 2 setup and launch the specified launch file
CMD ["bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch rpi_pkg rpi.launch.py"]
