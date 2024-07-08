FROM jcswisscat/em_onrobot:base

# Please adapt the git config to your own, here https is used
ARG GIT_USER=Yanniscod
ARG GIT_TOKEN=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

ENV namespace=

WORKDIR /home

# clone using id and token, token may be not reusable and need to be updated
RUN git clone --recursive https://$GIT_USER:$GIT_TOKEN@github.com/swisscatplus/SwissCat-on_robot.git

WORKDIR /home/SwissCat-on_robot

# Install dependencies and build the workspace
RUN colcon build

# Copy the entrypoint script into the Docker image
COPY ros_entrypoint.sh /home/SwissCat-on_robot/ros_entrypoint.sh

# Set the entrypoint script as the container's entrypoint
ENTRYPOINT ["/home/SwissCat-on_robot/ros_entrypoint.sh"]

# Default command to run ROS2 node
CMD ["ros2", "launch", "rpi_pkg", "rpi.launch.py", "namespace:={namespace}"]
#CMD ["bash"] # use this if you want to access the terminal, same as -it flag. Don't forget to comment the if statement of the ros_entrypoint.sh to make it work.
