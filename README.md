<br />
<div align="center">
  <a href="https://github.com/swisscatplus/EM_onrobot">
    <img src="pictures/logo.png" alt="Logo" height="80">
  </a>
  <h1 align="center">EM_onrobot</h1>
</div>

ROS 2 robot-side stack for EM Robot.

This repository is responsible for:
- base control from `/cmd_vel`
- wheel odometry
- IMU integration
- camera-based localization with ArUco markers
- EKF fusion
- Dockerized development and deployment workflows

This repository is not the place for:
- path planning
- path tracking
- fleet management

Those live in other repositories.

## Project Scope

The goal of `EM_onrobot` is to run on the robot and make the robot know where it is.

The current stack combines:
- wheel odometry from the differential drive base
- IMU orientation from the BNO055
- camera-based map correction from detected ArUco markers
- `robot_localization` EKF fusion

The repo also includes desktop development workflows so the same codebase can be exercised on:
- Windows with fake base + fake IMU + Foxglove
- Ubuntu with fake base + fake IMU + OpenCV camera + RViz + Foxglove
- Raspberry Pi on the real robot with real motors, real IMU, and Pi camera

## Runtime Profiles

The main bringup entrypoint is:

```bash
ros2 launch em_robot bringup.launch.py profile:=<profile_name>
```

Available profiles:

| Profile | Intended system | Movement | IMU | Camera | Visual tools |
|---|---|---|---|---|---|
| `home_windows` | Windows desktop | fake | fake | disabled | Foxglove |
| `desktop_replay` | Windows or desktop replay | fake | fake | OpenCV video file | Foxglove |
| `work_ubuntu` | Ubuntu desktop | fake | fake | OpenCV device | RViz + Foxglove |
| `real_robot` | Raspberry Pi on robot | real | BNO055 | Picamera2 | none by default |

Profile files live in [`src/em_robot/config/profiles`](./src/em_robot/config/profiles).

## Supported Workflows

### Windows

Use this for logic development and visualization from Docker Desktop.

Behavior:
- fake base controller
- fake IMU that follows `/cmd_vel.angular.z`
- `cmd_vel` watchdog stop after `0.25 s` without new commands
- no localization camera
- Foxglove bridge on port `8765`
- diagnostics on `/diagnostics`

Start:

```powershell
.\scripts\start_dev.ps1
```

Stop:

```powershell
.\scripts\start_dev.ps1 -Profile home_windows -Action down
```

Logs:

```powershell
docker compose -f docker/compose.yaml logs -f em_robot_home_windows
```

Shell:

```powershell
docker compose -f docker/compose.yaml exec em_robot_home_windows bash
```

Keyboard teleop:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel -p repeat_rate:=30.0 -p key_timeout:=0.3
```

Foxglove:
- connect to `ws://localhost:8765`
- useful topics: `/odomWheel`, `/odometry/filtered`, `/bno055/imu`, `/tf`, `/cmd_vel`, `/diagnostics`

### Desktop Replay

Use this to test localization from a recorded video instead of a live camera.

Behavior:
- fake base controller
- fake IMU that follows `/cmd_vel.angular.z`
- localization enabled through an OpenCV video file
- marker map loaded from `src/em_robot/config/marker_map.yaml`
- Foxglove bridge on port `8765`

Start:

```powershell
.\scripts\start_dev.ps1 -Profile desktop_replay
```

Video source:
- default file: `data/replays/localization.mp4`
- profile: [`src/em_robot/config/profiles/desktop_replay.yaml`](./src/em_robot/config/profiles/desktop_replay.yaml)
- place your replay video at that path, or edit the profile to another file

### Ubuntu

Use this for desktop testing with fake motion and IMU, but with a non-robot camera routed through OpenCV.

Behavior:
- fake base controller
- fake IMU that follows `/cmd_vel.angular.z`
- `cmd_vel` watchdog stop after `0.25 s` without new commands
- localization enabled through an OpenCV camera source
- RViz enabled
- Foxglove bridge on port `8765`
- diagnostics on `/diagnostics`

Start:

```bash
./scripts/start_dev.sh work_ubuntu
```

Stop:

```bash
./scripts/start_dev.sh work_ubuntu down
```

Logs:

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml logs -f em_robot_work_ubuntu
```

Shell:

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml exec em_robot_work_ubuntu bash
```

If RViz cannot open:

```bash
xhost +local:docker
```

If you want another camera:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video1
./scripts/start_dev.sh work_ubuntu
```

Foxglove:
- connect to `ws://localhost:8765`

Note:
- this is not using the Raspberry Pi camera stack
- it expects a Linux video device such as `/dev/video0`

### Robot

Use this on the Raspberry Pi mounted on the robot.

Behavior:
- real Dynamixel base controller
- real BNO055 IMU
- localization enabled through `picamera2`
- EKF enabled

Start the on-robot development container:

```bash
EM_ROBOT_PROFILE_VALUE=real_robot ./scripts/deploy_dev.sh
```

The script:
- pulls `ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest`
- falls back to building `docker/Dockerfile.base` locally if the pull fails
- bind-mounts the repository into `/ros2_ws`
- builds the ROS workspace in the container
- launches `bringup.launch.py` with the `real_robot` profile

Logs:

```bash
docker logs -f em_robot_dev
```

Shell:

```bash
docker exec -it em_robot_dev bash
```

## Networking

For ROS 2 discovery to work correctly:
- the PC and robot must be on the same network
- `ROS_DOMAIN_ID` must match on both sides
- [`config/fastdds.xml`](./config/fastdds.xml) must be adapted to the correct network interface or robot IP setup

Example:

```bash
export ROS_DOMAIN_ID=10
```

Desktop workflows read environment variables from the shell and optionally from a local `.env` file.
The template is [`.env.example`](./.env.example).

Main variables:
- `ROS_DOMAIN_ID`
- `RMW_IMPLEMENTATION`
- `FOXGLOVE_PORT`
- `EM_ROBOT_CAMERA_DEVICE`

## Useful Commands Inside a Container

Source ROS first:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

Useful checks:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odomWheel
ros2 topic echo /bno055/imu
ros2 topic echo /diagnostics
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

## ROS Interfaces

### Topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command input for the base |
| `/odomWheel` | `nav_msgs/Odometry` | Wheel odometry from the real or fake base controller |
| `/bno055/imu` | `sensor_msgs/Imu` | IMU orientation and angular velocity |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Runtime health for command link, sensors, filter, and localization |

### Services

| Service | Type | Description |
|---|---|---|
| `/set_initial_pose` | `em_robot_srv/srv/SetInitialPose` | Set the initial `map -> odom` alignment |

## Main Folders

The repository is now organized around a few top-level folders:

- [`docker/`](./docker): Compose files, Dockerfiles, the shared container entrypoint, and legacy Docker experiments
- [`scripts/`](./scripts): Windows, Ubuntu, robot, and release helper scripts
- [`config/`](./config): non-ROS top-level runtime config such as Fast DDS
- [`src/`](./src): ROS 2 packages
- [`CAD/`](./CAD): mechanical assets
- [`Electrical Schematics/`](./Electrical%20Schematics): wiring and electronics assets

Important operational files:

- [`docker/compose.yaml`](./docker/compose.yaml): shared desktop Compose definition
- [`docker/compose.ubuntu.yaml`](./docker/compose.ubuntu.yaml): Ubuntu-only Compose overrides
- [`docker/Dockerfile.desktop`](./docker/Dockerfile.desktop): desktop development image
- [`docker/Dockerfile.base`](./docker/Dockerfile.base): Raspberry Pi base image
- [`docker/Dockerfile`](./docker/Dockerfile): runtime image
- [`scripts/start_dev.sh`](./scripts/start_dev.sh): Ubuntu launcher
- [`scripts/start_dev.ps1`](./scripts/start_dev.ps1): Windows launcher
- [`scripts/deploy_dev.sh`](./scripts/deploy_dev.sh): robot launcher
- [`scripts/deployImage.sh`](./scripts/deployImage.sh): runtime image build/push helper
- [`scripts/buildBase.sh`](./scripts/buildBase.sh): base image build/push helper
- [`scripts/buildBaseLocal.sh`](./scripts/buildBaseLocal.sh): local base image build helper
- [`config/fastdds.xml`](./config/fastdds.xml): Fast DDS transport config
- [`src/em_robot/config/marker_map.yaml`](./src/em_robot/config/marker_map.yaml): static ArUco marker map used by localization
- [`DEVELOPMENT_WORKFLOWS.md`](./DEVELOPMENT_WORKFLOWS.md): shorter day-to-day command reference

## Repository Layout

```text
EM_onrobot/
├── docker/
│   ├── compose.yaml
│   ├── compose.ubuntu.yaml
│   ├── Dockerfile
│   ├── Dockerfile.base
│   ├── Dockerfile.desktop
│   └── entrypoint.sh
├── scripts/
│   ├── deploy_dev.sh
│   ├── start_dev.sh
│   ├── start_dev.ps1
│   ├── deployImage.sh
│   ├── buildBase.sh
│   └── buildBaseLocal.sh
├── config/
│   └── fastdds.xml
├── src/
│   ├── em_robot/       # bringup, localization, base control, config, RViz
│   ├── em_robot_srv/   # ROS 2 service definitions
│   └── bno055/         # BNO055 driver package
├── CAD/
├── Electrical Schematics/
└── pictures/
```

## Notes

- [`DEVELOPMENT_WORKFLOWS.md`](./DEVELOPMENT_WORKFLOWS.md) is the short operational guide
- this README is the higher-level explanation of what the repo is and how each system is meant to use it

## License

This project is licensed under the [MIT License](./LICENSE).

It also includes code adapted from the original [`flynneva/bno055`](https://github.com/flynneva/bno055) package.
