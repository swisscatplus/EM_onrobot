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

The ArUco detector and pose-estimation helpers live in
[`src/em_robot/em_robot/aruco_utils.py`](./src/em_robot/em_robot/aruco_utils.py)
and are shared by both:
- [`aruco_camera_test_node.py`](./src/em_robot/em_robot/aruco_camera_test_node.py)
- [`localization_node.py`](./src/em_robot/em_robot/localization_node.py)

The repo also includes desktop development workflows so the same codebase can be exercised on:
- Windows with fake base + fake IMU + Foxglove
- Ubuntu with fake base + fake IMU + OpenCV camera + RViz2
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
| `work_ubuntu` | Ubuntu desktop | fake | fake | OpenCV device | RViz2 |
| `work_ubuntu_aruco_test` | Ubuntu laptop ArUco sanity check | none | none | OpenCV device | RViz2 |
| `work_ubuntu_localization_test` | Ubuntu laptop localization pipeline test | fake | fake | OpenCV device | RViz2 |
| `real_robot` | Raspberry Pi on robot | real | BNO055 | Picamera2 | none by default |

Profile files live in [`src/em_robot/config/profiles`](./src/em_robot/config/profiles).

## Shared ArUco Path

The laptop ArUco test is meaningful because it uses the same marker-detection and
pose-estimation code as the real robot localization path.

Shared between laptop test and robot:
- `create_aruco_detector()`
- `detect_aruco_markers()`
- `estimate_marker_pose()`
- marker-map loading from [`marker_map_loader.py`](./src/em_robot/em_robot/marker_map_loader.py)

Validated by the laptop ArUco test:
- USB camera access through the desktop Docker workflow
- ArUco detection and pose estimation
- marker-map lookup and map-relative camera pose output
- RViz2 visualization of TF and debug image topics

Not covered by the laptop ArUco test:
- `picamera2` camera backend used on the Raspberry Pi
- fake/real base integration
- IMU integration
- `map -> odom` fusion inside [`localization_node.py`](./src/em_robot/em_robot/localization_node.py)
- EKF behavior on the full robot stack

## Quick Launch Reference

| Profile | Command | Purpose |
|---|---|---|
| `home_windows` | `.\scripts\start_dev.ps1` | Fake base + fake IMU development on Windows with Foxglove |
| `desktop_replay` | `.\scripts\start_dev.ps1 -Profile desktop_replay` | Replay localization from a recorded video |
| `work_ubuntu` | `./scripts/start_dev.sh work_ubuntu` | Fake base + fake IMU + OpenCV camera on Ubuntu |
| `work_ubuntu_aruco_test` | `export EM_ROBOT_CAMERA_DEVICE=/dev/videoX && ./scripts/start_dev.sh work_ubuntu_aruco_test` | Laptop-only ArUco sanity check with host RViz2 |
| `work_ubuntu_localization_test` | `export EM_ROBOT_CAMERA_DEVICE=/dev/videoX && ./scripts/start_dev.sh work_ubuntu_localization_test` | Laptop test of the full localization node with fake base/IMU |
| `real_robot` | `EM_ROBOT_PROFILE_VALUE=real_robot ./scripts/deploy_dev.sh` | Full robot stack on the Raspberry Pi |

Use your actual Linux camera path for `EM_ROBOT_CAMERA_DEVICE`, for example `/dev/video4` for a Logitech C920 on many laptops.

## Validation Status

This section is meant to reflect the current branch state and should be updated as more manual tests are completed.

### Manually Verified

- `work_ubuntu_aruco_test` on Ubuntu with a USB webcam and host-side RViz2
- host-to-container ROS 2 discovery for the Ubuntu desktop workflow after the Fast DDS cleanup
- RViz2 display of `/aruco_test/debug_image` and ArUco TF/debug outputs

### Automated Tests Already In The Repo

- ArUco helper tests in [`test_aruco_utils.py`](./src/em_robot/test/test_aruco_utils.py)
  - detector output normalization
  - synthetic pose recovery from projected marker corners
- marker map parsing and duplicate-ID checks in [`test_marker_map_loader.py`](./src/em_robot/test/test_marker_map_loader.py)
- profile loading in [`test_profile_loader.py`](./src/em_robot/test/test_profile_loader.py)
- differential-drive math and startup motion gate logic in [`test_differential_drive.py`](./src/em_robot/test/test_differential_drive.py)
- robot state / LED mode logic in [`test_robot_state_modes.py`](./src/em_robot/test/test_robot_state_modes.py)
- transform helper compatibility in [`test_tf_transformations_compat.py`](./src/em_robot/test/test_tf_transformations_compat.py)
- LED helper utilities in [`test_led_utils.py`](./src/em_robot/test/test_led_utils.py)
- style/lint checks in [`test_flake8.py`](./src/em_robot/test/test_flake8.py), [`test_pep257.py`](./src/em_robot/test/test_pep257.py), and [`test_copyright.py`](./src/em_robot/test/test_copyright.py)

### Still Recommended To Test Manually

- `work_ubuntu_localization_test`
  - verify `base_link`, `camera_frame`, and `map -> odom` behavior in RViz2
  - verify marker-based corrections while fake odometry is running
- `desktop_replay`
  - verify the replay video still drives the localization pipeline correctly
- `home_windows`
  - verify the Windows Docker Desktop workflow and Foxglove connection still work
- `real_robot`
  - verify `picamera2`, real motors, BNO055, EKF, LEDs, and deployment on hardware

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
- host-side RViz2 supported
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

Open RViz on the Ubuntu host:

```bash
source /opt/ros/<your-distro>/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rviz2 -d /home/edy/Documents/GitHub/EM_onrobot/src/em_robot/rviz/localization_debug.rviz
```

If you want another camera:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video1
./scripts/start_dev.sh work_ubuntu
```

Note:
- this is not using the Raspberry Pi camera stack
- it expects a Linux video device such as `/dev/video0`

### Ubuntu ArUco Test

Use this on the laptop to validate USB camera access, ArUco detection, and single-marker map pose estimation before running the full localization stack.

Behavior:
- OpenCV USB camera input
- marker map loaded from [`src/em_robot/config/marker_map_laptop_test.yaml`](./src/em_robot/config/marker_map_laptop_test.yaml)
- raw detection outputs on `/aruco_test/*`
- annotated debug image on `/aruco_test/debug_image`
- host-side RViz2 with the ArUco test layout
- no fake base, IMU, or EKF

Start:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video0
./scripts/start_dev.sh work_ubuntu_aruco_test
```

Topics to inspect:
- `/aruco_test/debug_image`
- `/aruco_test/marker_poses`
- `/aruco_test/camera_pose`
- `/aruco_test/summary`
- `/diagnostics`

Open RViz on the Ubuntu host:

```bash
source /opt/ros/<your-distro>/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rviz2 -d /home/edy/Documents/GitHub/EM_onrobot/src/em_robot/rviz/aruco_test_debug.rviz
```

### Ubuntu Localization Test

Use this on the laptop to run the same localization node as the robot, but with a USB camera and fake base/IMU.

Behavior:
- fake base controller
- fake IMU
- OpenCV USB camera input
- localization enabled
- marker map loaded from [`src/em_robot/config/marker_map_laptop_test.yaml`](./src/em_robot/config/marker_map_laptop_test.yaml)
- calibration/config from [`src/em_robot/config/calibration_ubuntu_test.yaml`](./src/em_robot/config/calibration_ubuntu_test.yaml)
- host-side RViz2

Start:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video0
./scripts/start_dev.sh work_ubuntu_localization_test
```

Open RViz on the Ubuntu host:

```bash
source /opt/ros/<your-distro>/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rviz2 -d /home/edy/Documents/GitHub/EM_onrobot/src/em_robot/rviz/localization_debug.rviz
```

Notes:
- the laptop test marker map assumes marker `0` is at the map origin
- `calibration_ubuntu_test.yaml` is only a starter file; for accurate pose, recalibrate the Logitech camera
- if your printed marker is not `38 mm`, update `marker_size` in the calibration file

### Robot

Use this on the Raspberry Pi mounted on the robot.

Behavior:
- real Dynamixel base controller
- real BNO055 IMU
- localization enabled through `picamera2`
- EKF enabled
- two RGB status LEDs through Raspberry Pi GPIO

Start the on-robot development container:

```bash
EM_ROBOT_PROFILE_VALUE=real_robot ./scripts/deploy_dev.sh
```

If you want to rebuild the robot base image locally, for example after changing GPIO dependencies:

```bash
FORCE_LOCAL_BASE_BUILD=1 EM_ROBOT_PROFILE_VALUE=real_robot ./scripts/deploy_dev.sh
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

LED feedback in the `real_robot` profile:
- `rgb_led_controller` drives the two RGB LEDs from GPIO
- `robot_state_manager` publishes colors based on robot mobility and health
- front LED defaults to `GPIO23/GPIO24/GPIO25`
- rear LED defaults to `GPIO4/GPIO17/GPIO27`
- if the physical red/green/blue order is different, update `front_color_order` or `rear_color_order` in [`src/em_robot/config/profiles/real_robot.yaml`](./src/em_robot/config/profiles/real_robot.yaml)
- if the LED behaves inverted, set `active_low: true` in the same profile

### Robot Camera Calibration

Use this when you want to recalibrate a robot camera from the repository root.

Recommended runtime starting point for Raspberry Pi marker detection around `0.3-0.4 m`:
- `image_size: [1280, 720]`
- `lens_position: 8.0`
- `process_rate_hz: 5.0`
- `max_reprojection_error_px: 3.0`

The calibration helpers now read robot camera settings directly from
[`src/em_robot/config/calibration.yaml`](./src/em_robot/config/calibration.yaml), so the
capture resolution and lens position match runtime automatically.

One-time environment setup on each robot:

```bash
./scripts/setup_calibration_env.sh
```

Capture chessboard images from the repo root:

```bash
./scripts/capture_calibration_images.sh
```

Run calibration from the repo root:

```bash
./scripts/run_camera_calibration.sh
```

Images are stored in:
- [`src/em_robot/CameraCalibration/calibration_images`](./src/em_robot/CameraCalibration/calibration_images)

Notes:
- capture images at the real working distance and angles you care about
- keep the board sharp and well lit
- use the same focus setting for calibration and runtime
- the calibration script updates `camera_matrix` and `dist_coeff` in `calibration.yaml` and preserves the other robot settings

Quick calibration example:
- temporarily set `state_manager.enabled: false` in the `real_robot` profile while testing raw colors
- publish red, green, and blue one by one, then adjust `front_color_order` or `rear_color_order` until the physical LED matches
- once the mapping is correct, re-enable `state_manager`

```bash
ros2 topic pub --once /leds/front/color std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"
ros2 topic pub --once /leds/front/color std_msgs/msg/ColorRGBA "{r: 0.0, g: 1.0, b: 0.0, a: 1.0}"
ros2 topic pub --once /leds/front/color std_msgs/msg/ColorRGBA "{r: 0.0, g: 0.0, b: 1.0, a: 1.0}"
```

Default automatic modes:
- front LED: blinking white while booting, cyan when ready/idling, blue while moving, blinking amber on command timeout, blinking red when base odometry is stale
- rear LED: blinking white while booting, green when healthy, blinking amber when localization or diagnostics degrade, blinking red on faults

## Networking

For ROS 2 discovery to work correctly:
- the PC and robot must be on the same network
- `ROS_DOMAIN_ID` must match on both sides
- `RMW_IMPLEMENTATION` should match on host and container for desktop workflows

For the current desktop workflows, [`config/fastdds.xml`](./config/fastdds.xml) is now generic and should not require a machine-specific IP edit.
If discovery fails, check the ROS environment first before editing DDS transport config.

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
| `/leds/front/color` | `std_msgs/ColorRGBA` | Front RGB LED color command |
| `/leds/rear/color` | `std_msgs/ColorRGBA` | Rear RGB LED color command |
| `/robot_state/mobility` | `std_msgs/String` | Mobility state derived from `/cmd_vel` and wheel odometry |
| `/robot_state/health` | `std_msgs/String` | Health/localization state derived from IMU, EKF, and diagnostics |
| `/robot_state/overall` | `std_msgs/String` | High-level robot state summary |

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
