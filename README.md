# SwissCat-on_robot - Localization of a mobile robot using Python and ROS2 

This repo provides the code needed for obtaining an accurate positioning of a mobile wheeled robot and controlling it with a /cmd_vel topic. For sensors, it uses the odometry of the encoder ticks, a [bno055 IMU](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) and a [RaspberryPi Module 3 Wide camera](https://www.pi-shop.ch/raspberry-pi-camera-3-wide).

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

### Calibration

### Dependencies

* Describe any prerequisites, libraries, OS version, etc., needed before installing program.
* ex. Windows 10

### Installing

* How/where to download your program
* Any modifications needed to be made to files/folders

### Executing program

* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Help

Any advise for common problems or issues.
```
command to run if program contains helper info
```

## Authors

Contributors names and contact info

ex. Yannis Coderey 
ex. [@Yanniscod]

## Version History

* 0.1
    * Initial Release

## License

--

## Acknowledgments

Inspiration, code snippets, etc.
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)