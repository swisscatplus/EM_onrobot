# EM_onrobot Development Workflows

This file is the short command reference for the software-only main branch.

## Official Runtime

The supported robot runtime is `real_robot` on ROS 2 Humble:

```bash
EM_ROBOT_PROFILE_VALUE=real_robot ./scripts/deploy_dev.sh
```

The deployed container name defaults to `em_robot_dev`.

Logs:

```bash
docker logs -f em_robot_dev
```

Shell:

```bash
docker exec -it em_robot_dev bash
```

## Ubuntu Localization Validation

The only desktop workflow kept in main is `work_ubuntu_localization_test`.

Start:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video0
./scripts/start_dev.sh work_ubuntu_localization_test
```

Stop:

```bash
./scripts/start_dev.sh work_ubuntu_localization_test down
```

Logs:

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml logs -f em_robot_work_ubuntu_localization_test
```

Shell:

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml exec em_robot_work_ubuntu_localization_test bash
```

Host RViz:

```bash
rviz2 -d src/em_robot/rviz/localization_debug.rviz
```

## Marker Survey

Robot:

```bash
./scripts/run_marker_survey.sh real_robot single-shot
```

Ubuntu validation:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video0
./scripts/run_marker_survey.sh work_ubuntu_localization_test single-shot
```

Manual mode:

```bash
./scripts/run_marker_survey.sh real_robot manual 0.0 0.0 0.0
```

## Calibration

```bash
./scripts/setup_calibration_env.sh
./scripts/capture_calibration_images.sh
./scripts/run_camera_calibration.sh
./scripts/validate_aruco_distance.sh
```

Generated calibration images and validation outputs are ignored by Git.

## Build And Test

```bash
colcon build --symlink-install --packages-select em_robot em_robot_srv bno055
colcon test --packages-select em_robot em_robot_srv bno055
colcon test-result --verbose
```

Quick unit smoke test:

```bash
PYTHONPATH=src/em_robot python3 -m pytest src/em_robot/test -q \
  --ignore=src/em_robot/test/test_copyright.py \
  --ignore=src/em_robot/test/test_flake8.py \
  --ignore=src/em_robot/test/test_pep257.py
```
