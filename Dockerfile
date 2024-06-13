FROM jcswisscat/em_rpi:latest

WORKDIR /home

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    curl \
    qemu-user-static \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set up QEMU to enable running executables compiled for different architectures
COPY qemu-aarch64-static /usr/bin/
#RUN git fetch && gewit pull
# Remove the existing directory if it exists
RUN rm -rf SwissCat-on_robot
# clone using id and token, token may be not reusable and need to be updated
RUN git clone https://Yanniscod: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@github.com/swisscatplus/SwissCat-on_robot.git

#RUN pip install serial
 RUN pip install pyserial
WORKDIR /home/SwissCat-on_robot/

# Install dependencies and build the workspace
RUN colcon build

# Copy the entrypoint script into the Docker image
COPY ros_entrypoint.sh /home/SwissCat-on_robot/ros_entrypoint.sh

# Set the entrypoint script as the container's entrypoint
ENTRYPOINT ["/home/SwissCat-on_robot/ros_entrypoint.sh"]

# Default command to run ROS2 node
#CMD ["ros2", "run", "rpi_pkg","rpi_com_motors"]
CMD ["ros2", "launch", "rpi_pkg", "rpi.launch.py"]
