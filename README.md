<br />
<div align="center">
  <a href="https://github.com/swisscatplus/EM_onrobot">
    <img src="pictures/logo.png" alt="Logo" height="80">
  </a>

  <h1 align="center">EM_onrobot</h1>
</div>

- Onboard localization and control stack for the SWISSCAT mobile robot, 
developed in <b>Python</b> and <b>ROS 2 Humble</b>, running inside <b>Docker</b>.
- Designed for a <b>Raspberry Pi 5</b> equipped with <b>DYNAMIXEL XC430-W150</b> motors, 
a <b>BNO055 IMU</b>, and a <b>Raspberry Pi Camera Module 3 Wide</b>.
Integrates with the <a href="https://github.com/swisscatplus/EM_fleetmanager">EM_fleetmanager</a> for multi-robot coordination.

---

## Table of Contents

1. [Overview](#overview)  
2. [System Architecture](#system-architecture)  
3. [Deployment of a New Robot](#deployment-of-a-new-robot)  
4. [Docker Installation (recommended)](#docker-installation-recommended)  
5. [Local Installation (advanced)](#local-installation-advanced)  
6. [How to Run](#how-to-run)  
7. [ROS 2 Interfaces](#ros-2-interfaces)  
8. [Project Structure](#project-structure)  
9. [Improvements & Roadmap](#improvements--roadmap)  
10. [License](#license)  

---

## Overview

## Overview

The `EM_onrobot` repository contains the software and hardware resources for the SwissCat+ in-house mobile robot.

### Software
- Controls the two **DYNAMIXEL XC430-W150** differential-drive motors.  
- Reads **encoder odometry** directly through the Dynamixel SDK.  
- Collects **IMU data** from the Bosch **BNO055** sensor.  
- Runs an **Extended Kalman Filter (EKF)** to fuse IMU and wheel odometry.  
- Uses the **camera with ArUco markers** for drift correction in odometry (map correction).  
- Publishes **ROS 2 topics** for robot localization and subscribes to `/cmd_vel` for motion commands.  

### Hardware
- Includes the **3D-printable mechanical design** of the robot’s case (recommended material: PLA).  
- Provides the **electrical schematics** for wiring and integration.  

### Deployment
- Supports **containerized deployment with Docker** for easy setup and reproducibility.  


---

## Versions & Software

- **OS**: Ubuntu 22.04 (on Raspberry Pi OS 64-bit for Raspberry Pi 5)  
- **ROS 2**: Humble 
- **Docker image (recommended)**:  
  ```
  ghcr.io/swisscatplus/em_onrobot/em_robot:latest
  ```
- **Dependencies (local runs)**: see [Local Installation](#local-installation-advanced)

---

## System Architecture

- **Hardware**
  - Raspberry Pi 5  
  - 2x DYNAMIXEL Motors (XC430-W150)
  - DYNAMIXEL U2D2 Power Hub
  - DYNAMIXEL U2D2
  - 2x Pololu Scooter/Skate Wheel 70×25mm - Black (3272)
  - Bosch BNO055 IMU Adafruit Board (2472)
  - Raspberry Pi Camera Module 3 Wide  
  - Bosch Professional GBA 18V (5AH recommended)
  - Converter DC/DC to 5V (DTJ1524S05)
  - Converter DC/DC to 12V (DTJ1524S12)
  - 3D printed Mobile Robot Case


- **Software** 
  - OS installed on the RPi5 (recommended : Raspberry pi OS using Imager software) 
  - ROS 2 packages (all inside Docker):
    - `em_robot` → motor control and odometry  
    - `bno055` → IMU driver (The `src/bno055` package in this repository is a modified version of the original `flynneva/bno055` code, adapted to work with this application.)
    - `em_robot_srv` → service definitions

---

## Deployment of a New Robot

### Hardware Setup
- Mount and wire the robot according to the CAD and Electrical Schematics
- Insert the fully charged battery into the back holder
- Wait some time for the Raspberry Pi to initialize
- Place the RPi within Wi-Fi range.

### Networking
- Ensure **PC and RPi are on the same network**.  
- For that, configure a **static IP** on the RPi which should be the same as the one found in **fastdds.xml** (for example : 192.168.0.101)
- Match the **ROS_DOMAIN_ID** on both PC and RPi (default = `10`):  
  ```bash
  echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
  ```
  Or in the Dockerfile:  
  ```dockerfile
  ENV ROS_DOMAIN_ID=10
  ```
---

## Docker Installation (recommended)

1. **SSH into the RPi**:  
   ```bash
   ssh <rpi-name>@<robot-ip>
   ```

2. **Install Docker** on the RPi:  
   ```bash
   sudo apt-get update
   sudo apt-get install ca-certificates curl gnupg
   sudo install -m 0755 -d /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
   sudo chmod a+r /etc/apt/keyrings/docker.gpg

   echo      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" |      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

   sudo apt-get update
   sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   sudo groupadd docker
   sudo gpasswd -a $USER docker
   newgrp docker
   docker run hello-world
   ```

3. **Pull the container image**:  
   ```bash
   docker pull ghcr.io/swisscatplus/em_onrobot/em_robot:latest
   ```

4. **Run the container**:  
   ```bash
   ./deploy.sh
   ```

---

## Local Installation (advanced)

Running outside Docker is not the primary workflow, but it is possible.  
From the [Dockerfile](./Dockerfile), the required local dependencies are:

### System packages
```bash
sudo apt update
sudo apt install python3-smbus i2c-tools python3-dev
```

### Python packages
```bash
pip install smbus2
```

### ROS 2 packages
Make sure you have ROS 2 Humble installed with `colcon` and build tools.  
Then build the workspace:
```bash
cd ros2_ws
colcon build --packages-select em_robot em_robot_srv bno055
```

---

## How to Run

1. Connect to the RPi:
   ```bash
   ssh <rpi-name>>@<robot-ip>
   ```

2. Start the container:
   ```bash
   ./deploy.sh
   ```

3.  **Test movement:**

    **Important:** This is a **two-wheeled differential drive
    robot**. That means it can only move using:

    -   **Linear X** → forward/backward motion (both wheels turning in
        the same direction).
    -   **Angular Z (yaw)** → rotation on the spot or turning (wheels
        turning at different speeds).

    All other fields (`linear.y`, `linear.z`, `angular.x`, `angular.y`)
    are ignored.

    You have two options for testing:

    -   **Direct velocity commands (use with caution):**

        You can publish a velocity command directly, but be aware that
        the robot will **keep moving until you explicitly send another
        command with zero velocity**. For example:

        ``` bash
        ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
          x: 0.3
          y: 0.0
          z: 0.0
        angular:
          x: 0.0
          y: 0.0
          z: 0.5"
        ```

        To stop the robot, send the same command again but with all
        values set to `0.0`.

    -   **Safer option: use teleop:**

        The recommended way to test is to use the
        [teleop_twist_keyboard](https://index.ros.org/p/teleop_twist_keyboard/)
        package. This allows you to control the robot interactively from
        your PC with the keyboard:

        ``` bash
        ros2 run teleop_twist_keyboard teleop_twist_keyboard
        ```

        You can also connect a **gaming controller** and map the
        joystick to publish commands on `/cmd_vel` for smoother and
        safer manual control.

---

## ROS 2 Interfaces

### Topics
| Topic                | Type                                    | Description                                    |
|----------------------|-----------------------------------------|------------------------------------------------|
| `/left_ticks_counts` | `std_msgs/Int16`                        | Encoder ticks from left wheel                  |
| `/right_ticks_counts`| `std_msgs/Int16`                        | Encoder ticks from right wheel                 |
| `/bno055/imu`        | `sensor_msgs/Imu`                       | Orientation and acceleration from BNO055       |
| `/edi/cam`           | `geometry_msgs/PoseWithCovarianceStamped` | Pose correction from camera + ArUco markers   |
| `/cmd_vel`           | `geometry_msgs/Twist`                   | Input velocity command for differential drive  |

---

## Project Structure

```
EM_onrobot/
├── Dockerfile            # Build instructions for em_robot image
├── Dockerfile.base       # Base image with ROS 2 and dependencies
├── entrypoint.sh         # Container entrypoint script
├── fastdds.xml           # Fast DDS configuration
├── src/
│   ├── em_robot/         # Main robot package (motors, camera, odometry)
│   ├── em_robot_srv/     # ROS 2 service definitions
│   ├── bno055/           # External Bosch BNO055 driver
│   └── test.py           # Example script
├── images/logo.png
└── README.md
```

---

## Improvements & Roadmap

- Expand documentation with troubleshooting and advanced usage examples.

---

## License

This project is licensed under the [MIT License](./LICENSE).

It also includes code from [flynneva/bno055](https://github.com/flynneva/bno055),
which is licensed under the [BSD 3-Clause License](https://github.com/flynneva/bno055/blob/main/LICENSE).

