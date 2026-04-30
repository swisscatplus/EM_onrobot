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
- BNO055 IMU integration
- camera-based localization with ArUco markers
- `robot_localization` EKF fusion
- robot diagnostics, state, and LED control
- Dockerized robot deployment and one Ubuntu validation workflow

This repository is not the place for path planning, path tracking, fleet management, mechanical design source files, electronics design files, or recorded bags.

## Supported Target

The production target is the Raspberry Pi robot runtime on ROS 2 Humble.

The main launch command is:

```bash
ros2 launch em_robot bringup.launch.py profile:=real_robot
```

The default profile is `real_robot`.

## Runtime Profiles

Profile files live in [`src/em_robot/config/profiles`](./src/em_robot/config/profiles).

| Profile | Intended system | Movement | IMU | Camera | Purpose |
|---|---|---|---|---|---|
| `real_robot` | Raspberry Pi on robot | real Dynamixel base | BNO055 | Picamera2 | Production robot runtime |
| `work_ubuntu_localization_test` | Ubuntu workstation | disabled | disabled | OpenCV camera | Desktop validation of marker localization |

The runtime URDF used by `robot_state_publisher` is packaged with `em_robot` at [`src/em_robot/urdf/simple_box_robot.urdf`](./src/em_robot/urdf/simple_box_robot.urdf). Hardware design files are intentionally kept outside this software repository.

## Robot Deployment

Start or update the robot development container:

```bash
EM_ROBOT_PROFILE_VALUE=real_robot ./scripts/deploy_dev.sh
```

Useful follow-up commands:

```bash
docker logs -f em_robot_dev
docker exec -it em_robot_dev bash
```

The deployment script bind-mounts this workspace, builds `em_robot`, `em_robot_srv`, and `bno055`, then launches `bringup.launch.py`.

## Ubuntu Validation

Use this when validating localization with a USB camera on an Ubuntu workstation:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video0
./scripts/start_dev.sh work_ubuntu_localization_test
```

Stop it with:

```bash
./scripts/start_dev.sh work_ubuntu_localization_test down
```

The validation profile uses:

- calibration from [`src/em_robot/config/calibration_ubuntu_test.yaml`](./src/em_robot/config/calibration_ubuntu_test.yaml)
- marker map from [`src/em_robot/config/marker_map_laptop_test.yaml`](./src/em_robot/config/marker_map_laptop_test.yaml)
- RViz layout from [`src/em_robot/rviz/localization_debug.rviz`](./src/em_robot/rviz/localization_debug.rviz)

## Marker Survey

Run marker survey on the robot:

```bash
./scripts/run_marker_survey.sh real_robot single-shot
```

Run marker survey from the Ubuntu validation container:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video0
./scripts/run_marker_survey.sh work_ubuntu_localization_test single-shot
```

Manual mode keeps the node alive and lets you set the known robot pose before saving visible markers.

## Camera Calibration

Set up the local calibration environment:

```bash
./scripts/setup_calibration_env.sh
```

Capture images:

```bash
./scripts/capture_calibration_images.sh
```

Compute calibration:

```bash
./scripts/run_camera_calibration.sh
```

Validate ArUco distance:

```bash
./scripts/validate_aruco_distance.sh
```

Generated calibration images, validation outputs, and recorded bags are ignored by Git.

## Build And Test

Build the ROS workspace:

```bash
colcon build --symlink-install --packages-select em_robot em_robot_srv bno055
```

Run tests:

```bash
colcon test --packages-select em_robot em_robot_srv bno055
colcon test-result --verbose
```

For a quick source-only unit smoke test outside a fully built ROS workspace:

```bash
PYTHONPATH=src/em_robot python3 -m pytest src/em_robot/test -q \
  --ignore=src/em_robot/test/test_copyright.py \
  --ignore=src/em_robot/test/test_flake8.py \
  --ignore=src/em_robot/test/test_pep257.py
```

## Repository Layout

```text
EM_onrobot/
├── .github/workflows/ci.yml
├── config/
│   └── fastdds.xml
├── docker/
│   ├── compose.yaml
│   ├── compose.ubuntu.yaml
│   ├── Dockerfile
│   ├── Dockerfile.base
│   ├── Dockerfile.desktop
│   └── entrypoint.sh
├── scripts/
├── src/
│   ├── em_robot/
│   ├── em_robot_srv/
│   └── bno055/
└── pictures/
```

## Dependency Notes

`em_robot` is an `ament_python` package. ROS/system dependencies are declared in `package.xml` where rosdep keys exist. Picamera2 and libcamera are provided by the robot Docker base image because the Raspberry Pi camera stack is built there rather than resolved by rosdep.

The vendored `bno055` package keeps its upstream BSD license in [`src/bno055/LICENSE`](./src/bno055/LICENSE).

## License

MIT. See [`LICENSE`](./LICENSE).
