FROM yanniscod/vision:latest

WORKDIR /home

# clone using id and token, token may be not reusable and need to be updated
<<<<<<< HEAD
RUN git clone https://Yanniscod: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@github.com/swisscatplus/SwissCat-on_robot.git
=======
RUN git clone https://Yanniscod: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@github.com/swisscatplus/SwissCat-on_robot>
>>>>>>> d8ef911e52a252ee224596b83d4dc8914f92c405

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
