<br />
<div align="center">
  <a href="https://github.com/swisscatplus/task-scheduler">
    <img src="https://images.squarespace-cdn.com/content/v1/6012a0a1f4c67c587a8eff67/d7731755-2fa3-4548-bf1e-5a25182d67ae/Combined+Logo+CAT-ETH-EPFL+%282%29.png?format=1500w" alt="Logo" height="80">
  </a>

  <h1 align="center"> SwissCat-on_robot - Localization of a mobile robot using Python and ROS2 </h1>

</div>

This repo provides the code needed for obtaining an accurate positioning of a mobile wheeled robot and controlling it with a /cmd_vel topic. For sensors, it uses the odometry of the encoder ticks, a [bno055 IMU](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) and a [RaspberryPi Module 3 Wide camera](https://www.pi-shop.ch/raspberry-pi-camera-3-wide).

## Table of Contents
1. [Description](#description)<br>
2. [Getting started](#getting-started)<br>
3. [Deployment of a new robot](#deployment-of-a-new-robot)<br>
4. [Packages and topics](#packages-and-topics)<br>
5. [To improve](#to-improve)
   
## Description

Simply put, a RaspberryPi 5 embedded in the robot, named Edison, is connected to the camera, the IMU and the motors and publishes the corresponding topics /edi/cam, /bno055/imu and /left_ticks_counts - /right_ticks_counts. These are converted in the server to proper types and covariances before being fused in an Extended Kalman Filter node and outputting a reliable odometry topic. The functioning of these topics are the following:

### - Vision
For determining the localisation using the camera, ArUco codes were installed on the circuit, with their positions circuit-wise defined in a [config file](https://github.com/swisscatplus/SwissCat-on_robot/blob/main/src/rpi_pkg/config/cam.yaml). A [python script](https://github.com/swisscatplus/SwissCat-on_robot/blob/main/src/rpi_pkg/rpi_pkg/submodules/detect_aruco.py) is in charge of transforming the pixel position of the code from the camera frame to the circuit frame, to output the position of the centre of the robot.

### - IMU
The BNO055 IMU was used because of its high accuracy and easy deployment on the robot, since it has its [own open-source ROS2 driver](https://github.com/flynneva/bno055/tree/45e1ff16936101711260c9fda63fbad99376ce3b) available.

### - Odometry of the wheels
To compute the encoder ticks, an Arduino is used to retrieve and transfer them to the RPi, using serial communication. The speed of both motors was empirically tuned to try and achieve the most reliable odometry. These left and right ticks are then directly published.

## Getting Started
As the goal is to easily deploy this project on the mobile robots, it was decided to create a Docker image of this code. The reasons are to have a much lighter memory usage of the robot, but also to make the camera and ros2 compatible as the Raspberry Pi 5 cannot run Picamera2 while having an Ubuntu distribution. Therefore, all you need to do to push this code on your robot is to have Docker installed. 

### Docker

#### Installation
If you wish to see the full installation procedure of a docker image compatible for our project, it's described in the [installation folder](https://github.com/swisscatplus/SwissCat-on_robot/tree/config/installation_procedure). Be careful that the folder details the image built from this folder enabling running ROS2 and using the camera on the same platform, but you still need to import the git repository. 

You can also directly pull the image using:
```
docker pull jcswisscat/em_onrobot:base
```
#### Usage
A [Dockerfile](https://github.com/swisscatplus/SwissCat-on_robot/edit/main/Dockerfile) is proposed to build an image with the GitHub repository and automatically launch the code, build it using:
```
cd Swisscat-on_robot/
docker build . -t username/img_name:tag

# And then run it for your application:
docker run -it --network host --privileged -v /dev/:/dev/ -v /run/udev:/run/udev username/img_name:tag
# this will launch the program without any namespace, if you want to add one specify it using 
```
If you simply want to access the terminal within the Docker image and not run the ros2 launch file, use the Dockerfile command
``
CMD ["bash"]
``

The `ros_entrypoint.sh` is here used to source the ROS2 installation and the local packages, feel free to add whatever command you judge interesting.

### Custom SwissCat Set-up
To make it all work, the actual setup needs to be the following:
   - If we're still using the two batteries (5V and 12V), the 5V one needs to be connected to the RPi via its USB2 port. The other port provides less voltage to the RPi, resulting in the ill-functioning of the device. Also, make sure the battery is well-charged, strange behaviour can happen when it's under 30-40% charge.
   - Be careful about the network, to be able to communicate from the PC to the RPi via ROS2, you need to fulfil these requirements:
        - Match the ROS_DOMAIN_ID in the bashrc file: by default, the one from the RPi image is 10, you can always change it by adding the following line to the Dockerfile: `ENV ROS_DOMAIN_ID=##` or adapt it in your bashrc `export ROS_DOMAIN_ID=10`
        - Be sure that both the PC and RPi are on the same network, meaning they should be connected to TP-Link_03DC. From the open space the RPi has trouble connecting to it,  just put it in the lab near the NMR (where the router is).
    
### Recipes

TBD Yannis

1. [Deploy on a new robot + server update]<br>

2. [List of used packages and topics]<br>

3. [On EM Debug]<br>

5. [Fleet Manager Debug]<br>

6. [Add new modifications and deploy]<br>

## Deployment of a new robot
This section focuses on describing the steps to perform to deploy a new robot into the fleet.
1. First of all, you have to initialise the Raspberry with the Raspberry Pi OS using Imager. For more information, follow the guidelines [here](https://www.raspberrypi.com/documentation/computers/getting-started.html). Enable SSH and make sure you configured and wrote down the static IP of the RPi, again for more information see [this](https://phoenixnap.com/kb/raspberry-pi-static-ip). Make sure you configure on the same network on which you'll be running the main ROS application, with the same ROS_DOMAIN_ID as said in the above section.
2. Once the RPi is configured, you'll have to SSH to the RPi.
   ```
   # Let's suppose the static ip address of our RPi is 192.168.0.236
   ssh camera@192.168.0.236
   # or if you configured in the /etc/hosts file, 192.168.0.236 camera, you can run:
   ssh camera@camera
   # I suggest adding an alias in the bashrc, such as 'sshcam', then you only need to run this cmd
   # sshcam
   ```
3. Install docker by following the instructions described in the [installation folder](https://github.com/swisscatplus/SwissCat-on_robot/tree/config/installation_procedure).
4. Pull the image.
   ```
   docker pull jcswisscat/em_onrobot:base
   ```
5. If you don't have the repo, you should clone it. The branch [on_rpi](https://github.com/swisscatplus/SwissCat-on_robot/tree/on_rpi) contains the bare minimum for running the application.
   ```
   # we're cloning using the SSH command, thus you should have your own SSH key generated and linked to GitHub
   # for more info, follow the indications described at https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
   git clone -b on_rpi git@github.com:swisscatplus/SwissCat-on_robot.git
   ```
6. Upon adding new mobile robots, their cameras will need to be calibrated. This is done following the steps inside the [calibration folder](https://github.com/swisscatplus/SwissCat-on_robot/tree/config/calibration)
7. Run the shell scripts to build and run your image
   ```
   # enable permission to run
   chmod u+x build.sh run.sh
   
   # make sure your Docker daemon is running
   
   # build image, don't forget to change the name you want for it, by default it will be username/img_name:tag
   # make sure to specify the Git user and token ids inside the shell script, or as arguments to the shell command
   ./build.sh
   # run it and specify the namespace you want to give to the robot, if nothing is specified it will be ''
   ./run.sh namespace
   ```
   That's it, you have a running container publishing your topics. If you want to have access to the terminal after running the image, run in detached mode, this means modifying run.sh by     adding the -d flag.
  
7. As described above, the program will launch when running the Docker image. If you wish to test each node separately, build an image with `CMD ["bash"]`, once inside run the node you       want:

   ```
   # Individual commands to run the nodes, rpi.launch.py launches all three
   
   ros2 run rpi_pkg rpi_motors # will publish the left and right encoder ticks
   
   ros2 run rpi_pkg rpi_cam # will publish the absolute position
   
   ros2 run bno055 bno055 # will publish all sorts of imu-related topics, we use /bno055/imu.
   ```

## Packages and topics

### Dependencies
The package dependencies are seen in the [package.xml](https://github.com/swisscatplus/SwissCat-on_robot/blob/config/src/rpi_pkg/package.xml). For a complete list, they're shown inside the following bracket.
<details open>
  <summary>Show dependencies</summary>
    - python3-serial <br>
    - python3-smbus <br>
    - geometry_msgs <br>
    - rclpy <br>
    - std_msgs <br>
    - time <br>
</details>

### Topics
- Input topics: /cmd_vel
- Output topics: /left_tick_counts, /right_tick_counts, /edi/cam, /wheel/odom

## To improve
What should be further implemented is the namespace of each robot, so that the topics are published accordingly (/ns/topic). One option would be to put the namespace inside the config file and append it to each topic.
Furthermore, the node responsible for publishing the odometry should be replaced when the Dynamixel motors are implemented, as they provide their own ros2 driver.

