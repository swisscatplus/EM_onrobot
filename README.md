<a name="readme-top"></a>
<br />
<div align="center">
  <a href="https://github.com/swisscatplus/">
    <img src="./images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h1 align="center">EM_onrobot - Localization of a mobile robot using Python and ROS2</h1>

  <p align="center">
    EM_onrobot - Localization of a mobile robot using Python and ROS2
    <br />
    <a href="https://github.com/swisscatplus/EM_onrobot"><strong>Explore the docs »</strong></a>
    <br />
    <br />
  </p>
</div>

This repo provides the code needed for obtaining an accurate positioning of a mobile wheeled robot and controlling it with a /cmd_vel topic. For sensors, it uses the odometry of the encoder ticks, a [bno055 IMU](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) and a [RaspberryPi Module 3 Wide camera](https://www.pi-shop.ch/raspberry-pi-camera-3-wide). It is designed to work jointly with [this code](https://github.com/swisscatplus/EM_fleetmanager).

## Versions and software used
  - Ubuntu 22.04
  - ROS2 Humble
  - Docker

## Table of Contents
1. [Description](#description)<br>
2. [Deployment of a new robot](#deployment-of-a-new-robot)<be>
3. [How to run](how-to-run)<br>
4. [ROS2 interfaces](#ros2-interfaces)<br>
5. [To improve](#to-improve)
   
## Description

Simply put, a RaspberryPi 5 embedded in the robot, named Edison, is connected to the camera, the IMU and the motors and publishes the corresponding topics /edi/cam, /bno055/imu and /left_ticks_counts - /right_ticks_counts. These are converted in the server to proper types and covariances before being fused in an Extended Kalman Filter node and outputting a reliable odometry topic. The functioning of these topics are the following:

### - Vision
For determining the localisation using the camera, ArUco codes were installed on the circuit, with their positions circuit-wise defined in a [config file](https://github.com/swisscatplus/SwissCat-on_robot/blob/main/src/rpi_pkg/config/cam.yaml). A [python script](https://github.com/swisscatplus/SwissCat-on_robot/blob/main/src/rpi_pkg/rpi_pkg/submodules/detect_aruco.py) is in charge of transforming the pixel position of the code from the camera frame to the circuit frame, to output the position of the centre of the robot.

### - IMU
The BNO055 IMU was used because of its high accuracy and easy deployment on the robot, since it has its [own open-source ROS2 driver](https://github.com/flynneva/bno055/tree/45e1ff16936101711260c9fda63fbad99376ce3b) available.

### - Odometry of the wheels
To compute the encoder ticks, an Arduino is used to retrieve and transfer them to the RPi, using serial communication. The speed of both motors was empirically tuned to try and achieve the most reliable odometry. These left and right ticks are then directly published.

## Deployment of a new robot

### Before starting: custom SwissCat Set-up
Some specificities are required for the proper working of the code:
   - If we're still using the two batteries (5V and 12V) to power the RPi and the motors, the 5V battery needs to be connected to the RPi via its USB2 port. The other port provides less voltage to the RPi, resulting in the ill-functioning of the device. Also, make sure the battery is well-charged, strange behaviour can happen when it's under 30-40% charge.
   - Be careful about the network, to be able to communicate from the PC to the RPi via ROS2, you need to fulfil these requirements:
        - Match the ROS_DOMAIN_ID in the bashrc file: by default, the one from the RPi image is 10, you can always change it by adding the following line to the Dockerfile: `ENV ROS_DOMAIN_ID=##` or adapt it in your bashrc `export ROS_DOMAIN_ID=10`
        - Be sure that both the PC and RPi are on the same network, meaning they should be connected to TP-Link_03DC. From the open space the RPi has trouble connecting to it,  just put it in the lab near the NMR (where the router is).

### Installation procedure
This section focuses on describing the steps to perform to deploy a new robot into the fleet.
1. First of all, you have to initialise the Raspberry with the Raspberry Pi OS using Imager. I suggest adding the configuration for the network, username and password (needed for SSH) before writing on the SD card. For more information, follow the guidelines [here](https://www.raspberrypi.com/documentation/computers/getting-started.html). To configure a static IP address, one way is to connect directly to the RPi using cables and modify the `/etc/network/interfaces` file with the following structure, use the `ifconfig` and the wifi settings to know what values to put.
    ```
    # sudo nano /etc/network/interfaces                                                                   
    # interfaces(5) file used by ifup(8) and ifdown(8)
    # Include files from /etc/network/interfaces.d:
    source /etc/network/interfaces.d/*
  
    auto wlan0
    allow-hotplug wlan0
    
    iface wlan0 inet static
    # the static adress you want to assign
    address 192.168.0.96 
    netmask 255.255.255.0
    gateway 192.168.0.1
    network 192.168.0.1
    broadcast 192.168.0.255
    # the following dns line is not working
    dns-nameservers 192.168.0.1
    wpa-ssid "SSID"
    wpa-psk "PASSWORD"
    ```
    
    To properly configure the DNS: `sudo nano /etc/resolv.conf` and write:
    ```
    # Generated by NetworkManager
    nameserver 8.8.8.8
    nameserver 8.8.4.4
    ```
    
    Then reboot the RPi to assign the changes. Configure on the same network on which you'll be running the main ROS application.
   
3. Once the RPi is configured, SSH to the RPi.
   ```
   # Let's suppose the static ip address of our RPi is 192.168.0.21
   ssh swisscat@192.168.0.236
   # or if you configured in the `/etc/hosts` file: `192.168.0.21 robot1`, you can run:
   ssh swisscat@robot1
   # I suggest adding an alias in the bashrc, such as 'sshcam', then you only need to run this cmd
   # sshcam
   ```
4. Install docker and check installation:
    ```
    # enable apt
    sudo apt-get update
    sudo apt-get install ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    # Add the repository to Apt sources:
    echo \
      "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
      "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    
    # install docker
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    
    # check installation
    sudo docker run hello-world
    ```
    
    Remove the need for sudo when running docker:
    - Add the `docker` group if it doesn't already exist:
   
      ```
      sudo groupadd docker
      ```
   - Add the connected user "$USER" to the docker group. Change the user name to match your preferred user if you do not want to use your current user:
      ```
      sudo gpasswd -a $USER docker
      ```
   - Either do a `newgrp` docker or log out/in to activate the changes to groups.
   - You can use
      ```
      docker run hello-world
      ```
6. Pull the image.
   ```
   docker pull jcswisscat/em_onrobot:base
   ```
     If you want to know how this image was built, all the instructions are described in the [installation folder](https://github.com/swisscatplus/SwissCat-on_robot/tree/config/installation_procedure).
7. If you don't have the repo, you should clone it. The branch [on_rpi](https://github.com/swisscatplus/SwissCat-on_robot/tree/on_rpi) contains the bare minimum for running the application.
   ```
   # we're cloning using the SSH command, thus you should have your own SSH key generated and linked to GitHub
   # for more info, follow the indications described at https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
   git clone -b on_rpi git@github.com:swisscatplus/SwissCat-on_robot.git
   ```
8. Upon adding new mobile robots, their cameras will need to be calibrated. This is done following the steps inside the [calibration folder](https://github.com/swisscatplus/SwissCat-on_robot/tree/config/calibration)
9. Run the shell scripts to build and run your image
   ```
   cd ./EM_onrobot
   
   # enable permission to run
   chmod u+x build.sh run.sh
   
   # make sure your Docker daemon is running
   
   # build image, don't forget to change the name you want for it, by default it will be username/img_name:tag, + add correct git username and token inside the .sh
   # make sure to specify the Git user and token ids inside the shell script, or as arguments to the shell command
   ./build.sh
   # run it and specify the namespace you want to give to the robot, if nothing is specified it will be ''
   ./run.sh namespace
   ```
   
   That's it, you have a running container publishing your topics. If you want to have access to the terminal after running the image, run in detached mode, this means modifying run.sh by adding the -d flag.

## How to run
Once the whole installation is over, to run the docker container inside a Raspberry Pi is quite straight forward:
1. ssh to the robot you want to use
   ```
   # for example, here robot1 would be 192.168.0.21
   ssh swisscat@robot1
   ```
2. Run the container, add the namespace if you want:
   ```
   # without namespace
   ./run.sh
   
   # with namespace
   ./run.sh robot1
   ```
3. As described above, the program will launch when running the Docker image. If you wish to test each node separately, build an image with `CMD ["bash"]` and comment the if statement inside the `ros_entrypoint.sh`. Once inside run the command you want:

   ```
   # Individual commands to run the nodes, rpi.launch.py launches all three
   
   ros2 run rpi_pkg rpi_motors # will publish the left and right encoder ticks
   
   ros2 run rpi_pkg rpi_cam # will publish the absolute position
   
   ros2 run bno055 bno055 # will publish all sorts of imu-related topics, we use /bno055/imu.
   ```

## ROS2 interfaces

<center>

| Topic                | Type                                    | Description                                                      |
|----------------------|-----------------------------------------|------------------------------------------------------------------|
| `left_ticks_counts`   | std_msgs/Int16                          | Number of encoder ticks done by the left motor                   |
| `right_ticks_counts`  | std_msgs/Int16                          | Number of encoder ticks done by the right motor                  |
| `edi/cam`            | geometry_msgs/PoseWithCovarianceStamped | Position of the robot from PiCamera2 according to ArUco codes    |
| `bno055/imu`         | sensor_msgs/Imu                         | Fused IMU data from bno055 sensor                                |

| Parameter                | Type    | Description                                                                                                   |
|--------------------------|---------|---------------------------------------------------------------------------------------------------------------|
| `namespace`              | String  | Namespace of the robot, it'll be appended to the topics and nodes                                             |
| `config_file`            | String  | Path of the configuration file used by the camera node that describe ArUco positions and intrinsic parameters |

</center>

### Dependencies
The package dependencies are seen in the [package.xml](https://github.com/swisscatplus/SwissCat-on_robot/blob/config/src/rpi_pkg/package.xml). For a complete list, they're shown inside the following bracket. If some are missing, update the package.xml accordingly. 

<details open>
  <summary><strong>Show dependencies</strong></summary>
    - python3-serial <br>
    - python3-smbus <br>
    - geometry_msgs <br>
    - rclpy <br>
    - std_msgs <br>
    - time <br>
</details>

## To improve
The node responsible for publishing the odometry should be replaced when the Dynamixel motors are implemented, as they provide their own ros2 driver.

