# SwissCat-on_robot - Localization of a mobile robot using Python and ROS2 

This repo provides the code needed for obtaining an accurate positioning of a mobile wheeled robot and controlling it with a /cmd_vel topic. For sensors, it uses the odometry of the encoder ticks, a [bno055 IMU](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) and a [RaspberryPi Module 3 Wide camera](https://www.pi-shop.ch/raspberry-pi-camera-3-wide).

## Table of Contents
1. [Description](#description)<br>
2. [Getting started](#getting-started)
## Description

Simply put, a RaspberryPi 5 embedded on the robot, named Edison, is connected to the camera, the IMU and the motors and publishes the corresponding topics /edi/cam, /bno055/imu and /left_ticks_counts - /right_ticks_counts. These are converted in the server to proper types and covariances before being fused in an Extended Kalman Filter node and outputing a reliable odometry topic. The functioning of these topics are the following:

### - Vision
For determining the localisation using the camera, ArUco codes were installed on the circuit, with their positions circuit-wise defined in a [config file](https://github.com/swisscatplus/SwissCat-on_robot/blob/main/src/rpi_pkg/config/cam.yaml). A [python script](https://github.com/swisscatplus/SwissCat-on_robot/blob/main/src/rpi_pkg/rpi_pkg/submodules/detect_aruco.py) is in charge of transforming the pixel position of the code from the camera frame to the circuit frame, to output the position of the center of the robot.

### - IMU
The BNO055 IMU was used because of its high accuracy and easy deployment on the robot, since it has its [own open-source ROS2 driver](https://github.com/flynneva/bno055/tree/45e1ff16936101711260c9fda63fbad99376ce3b) available.

### - Odometry of the wheels
To compute the encoder ticks, an arduino is used to retrieve and transfer them to the RPi, using serial communication. The speed of the two motors were empirically tuned to try and achieve the most reliable odometry. These left and right ticks are then directly published.

## Getting Started
As the goal is to easily deploy this project on the mobile robots, it was decided to create a Docker image of this code. The reasons are to have a much lighter memory usage of the robot, but also to make the camera and ros2 compatible as the RaspberryPi 5 cannot run Picamera2 while having an Ubuntu distribution. Therefore, all you need to do in order to push this code on your robot is to have Docker installed. 

### Docker

#### Installation
If you wish to see the full installation procedure of a docker image compatible for our project, it's described in the [installation folder](https://github.com/swisscatplus/SwissCat-on_robot/tree/config/installation_procedure). Be careful that the folder details the image built from this folder enables running ROS2 and using the camera on the same platform, but you still need to import the git repository. 

You can also directly pull the image using:
```
docker pull jcswisscat/em_onrobot:base
```
#### Usage
A [Dockerfile](https://github.com/swisscatplus/SwissCat-on_robot/edit/main/Dockerfile) is proposed to build an image with the github repository and automatically launch the code, build it using:
```
cd Swisscat-on_robot/
docker build . -t username/img_name:tag

# And then run it for your application:
docker run -it --network host --privileged -v /dev/:/dev/ -v /run/udev:/run/udev username/img_name:tag
```
If you simply want to access the terminal within the Docker image and not run the ros2 launch file, use the Dockerfile command
``
CMD ["bash"]
``

The `ros_entrypoint.sh` is here used to source the ROS2 installation and the local packages, feel free to add whatever command you judge interesting.

### Calibration
Upon adding new mobile robots, their cameras will need to be calibrated. This is done following the steps inside the [calibration folder]()

### Executing program
As described above, the program will launch itself when running the Docker image. If you wish to test each node separately, build an image wth `CMD ["bash"]`, once inside run the node you want:
```
# Individual commands to run the nodes, rpi.launch.py launches all three

ros2 run rpi_pkg rpi_motors # will publish the left and right encoder ticks

ros2 run rpi_pkg rpi_cam # will publish the absolute position

ros2 run bno055 bno055 # will publish all sorts of imu-related topics, we use /bno055/imu.
```

### Custom SwissCat Set-up

To make it all work, the actual setup needs to be the following:
   - If we're still using the two batteries (5V and 12V), the 5V one needs to be connected to the RPi via its USB2 port. The other port provides less voltage to the RPi, resulting in the ill-functioning of the device. Also, make sure the battery is well-charged, strange behavior can happen when it's under 30-40% charge.
   - Be careful about the network, to be able to communicate from the PC to the RPi via ROS2, you need to fulfil these requirements:
        - Match the ROS_DOMAIN_ID in the bashrc file: by default, the one from the RPi image is 10, you can always change it by adding the following line to the Dockerfile: `ENV ROS_DOMAIN_ID=##` or adapt it in your bashrc `export ROS_DOMAIN_ID=10`
        - Be sure that both the PC and RPi are on the same network, meaning they should be connected to TP-Link_03DC. From the open space the RPi has trouble connecting to it,  just put it in the lab near the NMR (where the router actually is).

### To continue
What should be further implemented is the namespace of each robot, so that the topics are published accordingly (/ns/topic). One option would be to put the namespace inside the config file and append it to each topic.
Furthermore, the node responsible for publishing the odometry should be replaced when the Dynamixel motors are implemented, as they provide their own ros2 driver.

## Authors

Contributors names and contact info

Yannis Coderey 
[@Yanniscod]

## Version History

* 0.1
    * Initial Release
