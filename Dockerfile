FROM yanniscod/rpi_vision:latest

WORKDIR /home/SwissCat-on_robot

# Install dependencies and build the workspace
RUN colcon build

# Copy the entrypoint script into the Docker image
COPY ros_entrypoint.sh /home/SwissCat-on_robot/ros_entrypoint.sh

# Set the entrypoint script as the container's entrypoint
ENTRYPOINT ["/home/SwissCat-on_robot/ros_entrypoint.sh"]

# Default command to run ROS2 node
CMD ["ros2", "run", "rpi_pkg", "rpi_cam"]

