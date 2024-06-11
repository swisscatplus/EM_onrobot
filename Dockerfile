FROM yanniscod/vision:v1

WORKDIR /home

# clone using id and token, token may be not reusable and need to be updated
RUN git clone https://Yanniscod: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@github.com/swisscatplus/SwissCat-on_robot>

RUN pip install pyserial

WORKDIR /home/SwissCat-on_robot/

# Install dependencies and build the workspace
RUN colcon build

# Copy the entrypoint script into the Docker image
COPY ros_entrypoint.sh /home/SwissCat-on_robot/ros_entrypoint.sh

# Set the entrypoint script as the container's entrypoint
ENTRYPOINT ["/home/SwissCat-on_robot/ros_entrypoint.sh"]

# Default command to run ROS2 node
CMD ["ros2", "launch", "rpi_pkg", "rpi.launch.py"]
