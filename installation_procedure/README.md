# Installation procedure

## Straight-forward solution: 
Pull directly the docker image built, which enables running ROS2 and using the camera on the Raspberry Pi 5:
`
docker pull yanniscod/rpi_vision:v1
`

## Long solution but details how the docker image was built, provides Dockerfile
This detailed method installs docker on the Raspberry Pi 5, builds the image needed to use ros2 and libcamera and provides shell commands to run the required node inside the container properly.

Do the following commands in the installation_procedure directory, which you downloaded from GitHub (you can git clone, the ros2 workspace is supposed to be run on the RPI)

1) 'sh 1_setup_docker_apt.sh'
2) 'sh 2_install_docker.sh'
	-> you should get a "Hello from Docker! ..." message, showing that the installation worked
3) 'sh 3_build_docker.sh' 
	-> must be executed where the Dockerfile is # maybe use DockerHub, so only a pull would be required instead of fully building the image
4) 'sh 4_run_docker.sh'
	-> starts an interactive session # maybe -it not required when deploying overall

Then, in the container you should be inside the git folder created. 

5) 'sh run_vision_cam.sh'
	-> sources, builds and runs the ros2 application

