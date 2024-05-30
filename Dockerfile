FROM yanniscod/vision:v1

WORKDIR /home

# clone using id and token, token may be not reusable and need to be updated
RUN git clone https://Yanniscod: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@github.com/Yanniscod/SwissCat-on_robot.git

RUN cd SwissCat-on_robot/

# Install dependencies and build the workspace
RUN colcon build

# Copy the entrypoint script into the Docker image
COPY ros_entrypoint.sh /home/SwissCat-on_robot/ros_entrypoint.sh

# Set the entrypoint script as the container's entrypoint
ENTRYPOINT ["/home/SwissCat-on_robot/ros_entrypoint.sh"]

# Default command to run ROS2 node
CMD ["ros2", "run", "rpi_pkg", "rpi_cam"]

