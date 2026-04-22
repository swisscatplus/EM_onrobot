# EM_onrobot Development Workflows

This file is a practical command reference for the different ways to run `EM_onrobot`.

It is intentionally short and task-oriented.

## 1. Main Modes

There are now two big families of workflows:

- `real robot`: run on the Raspberry Pi with the real motors, IMU, and Pi camera
- `desktop dev`: run on your PC with fake sensors and profile-based startup

The runtime behavior is selected with profiles:

- `real_robot`
- `work_ubuntu`
- `home_windows`
- `desktop_replay`

## 2. Windows Home Development

Use this when working from home on Windows with Docker Desktop.

Current behavior of `home_windows`:

- fake movement
- fake IMU that follows `/cmd_vel.angular.z`
- `cmd_vel` watchdog timeout after `0.25 s`
- no camera
- Foxglove bridge enabled on port `8765`
- no RViz by default
- diagnostics on `/diagnostics`

### Start

```powershell
.\scripts\start_dev.ps1
```

This is equivalent to:

```powershell
.\scripts\start_dev.ps1 -Profile home_windows
```

### Stop

```powershell
.\scripts\start_dev.ps1 -Profile home_windows -Action down
```

### See Logs

```powershell
docker compose -f docker/compose.yaml logs -f em_robot_home_windows
```

### Open a Shell in the Container

```powershell
docker compose -f docker/compose.yaml exec em_robot_home_windows bash
```

### Drive with Keyboard Teleop

1. Start the container in one terminal:

```powershell
.\scripts\start_dev.ps1
```

2. Open a second terminal and enter the running container:

```powershell
docker compose -f docker/compose.yaml exec em_robot_home_windows bash
```

3. Inside the container, source ROS and launch teleop:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel -p repeat_rate:=30.0 -p key_timeout:=0.3
```

Useful keys:

- `i` forward
- `,` backward
- `j` rotate left
- `l` rotate right
- `u`, `o`, `m`, `.` combined motion
- `k` stop

Terminal roles:

- terminal 1 running `start_dev.ps1`: starts the container and shows live logs
- terminal 2 running `exec ... bash`: gives you an interactive shell inside the same container
- optional extra terminal with `docker compose ... logs -f`: only watches logs; it does not start a second container

### Open the Visualizer

1. Start the container:

```powershell
.\scripts\start_dev.ps1
```

2. Open Foxglove in your browser or desktop app.
3. Create a new connection using:

```text
ws://localhost:8765
```

Recommended first panels:

- `3D`
- `Raw Messages`
- `Plot`
- `Topic Graph`
- `Publish`

### Rebuild the Container

```powershell
docker compose -f docker/compose.yaml build em_robot_home_windows
```

### Rebuild Without Cache

```powershell
docker compose -f docker/compose.yaml build --no-cache em_robot_home_windows
```

## 3. Desktop Replay

Use this when you want to test localization from a recorded video file.

Current behavior of `desktop_replay`:

- fake movement
- fake IMU that follows `/cmd_vel.angular.z`
- `cmd_vel` watchdog timeout after `0.25 s`
- localization from an OpenCV video file
- marker map loaded from `src/em_robot/config/marker_map.yaml`
- Foxglove bridge enabled on port `8765`
- diagnostics on `/diagnostics`

### Start

```powershell
.\scripts\start_dev.ps1 -Profile desktop_replay
```

### Stop

```powershell
.\scripts\start_dev.ps1 -Profile desktop_replay -Action down
```

### Replay File

- default path inside the repo: `data/replays/localization.mp4`
- profile config: `src/em_robot/config/profiles/desktop_replay.yaml`
- if the file name differs, edit the profile before starting

## 4. Ubuntu Desktop Development

Use this when working on Ubuntu at work with Docker.

Current behavior of `work_ubuntu`:

- fake movement
- fake IMU that follows `/cmd_vel.angular.z`
- `cmd_vel` watchdog timeout after `0.25 s`
- OpenCV camera backend
- RViz enabled
- Foxglove bridge enabled on port `8765`
- diagnostics on `/diagnostics`

### Start

```bash
./scripts/start_dev.sh work_ubuntu
```

### Stop

```bash
./scripts/start_dev.sh work_ubuntu down
```

### See Logs

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml logs -f em_robot_work_ubuntu
```

### Open a Shell in the Container

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml exec em_robot_work_ubuntu bash
```

### If RViz Cannot Open

Allow Docker to use your X server:

```bash
xhost +local:docker
```

### If You Want Another Camera

Set the camera device before starting:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video1
./scripts/start_dev.sh work_ubuntu
```

### Open Foxglove

Connect to:

```text
ws://localhost:8765
```

## 5. Real Robot Development on the Raspberry Pi

Use this on the robot itself.

### Start the Dev Container

```bash
EM_ROBOT_PROFILE_VALUE=real_robot ./scripts/deploy_dev.sh
```

### Default Start

If you do not set the profile explicitly, the current script also starts `real_robot` by default.

```bash
./scripts/deploy_dev.sh
```

### See Logs

```bash
docker logs -f em_robot_dev
```

### Open a Shell in the Container

```bash
docker exec -it em_robot_dev bash
```

## 6. Useful Commands Inside the Container

After opening a shell in a container, source ROS first:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

### See Running Nodes

```bash
ros2 node list
```

### See Topics

```bash
ros2 topic list
```

### Watch Fake Odometry

```bash
ros2 topic echo /odomWheel
```

### Send a Velocity Command

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### Watch IMU Messages

```bash
ros2 topic echo /bno055/imu
ros2 topic echo /diagnostics
```

## 7. What Windows Can Visualize Now

With the Foxglove bridge enabled in `home_windows`, Windows is now useful for visual debugging of:

- `/odomWheel`
- `/odometry/filtered`
- `/bno055/imu`
- `/tf`
- `/cmd_vel`
- `/diagnostics`

That means you can:

- publish `cmd_vel`
- watch odometry evolve
- inspect the TF tree
- plot values over time
- understand whether the fake stack is behaving correctly

## 8. Profiles

Profile files live here:

- [real_robot.yaml](src/em_robot/config/profiles/real_robot.yaml)
- [work_ubuntu.yaml](src/em_robot/config/profiles/work_ubuntu.yaml)
- [home_windows.yaml](src/em_robot/config/profiles/home_windows.yaml)
- [desktop_replay.yaml](src/em_robot/config/profiles/desktop_replay.yaml)

These files decide which backends are used:

- movement backend
- IMU backend
- camera backend
- RViz on/off
- EKF on/off
- watchdog and diagnostics settings
- marker map configuration

If you want to change behavior, start by editing the relevant profile.

## 9. Most Common Recovery Commands

### Stop Everything

Windows:

```powershell
docker compose -f docker/compose.yaml down
```

Ubuntu:

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml down
```

### Rebuild and Restart

Windows:

```powershell
docker compose -f docker/compose.yaml build em_robot_home_windows
.\scripts\start_dev.ps1
```

Ubuntu:

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml build em_robot_work_ubuntu
./scripts/start_dev.sh work_ubuntu
```

### Clean Rebuild

Windows:

```powershell
docker compose -f docker/compose.yaml build --no-cache em_robot_home_windows
```

Ubuntu:

```bash
docker compose -f docker/compose.yaml -f docker/compose.ubuntu.yaml build --no-cache em_robot_work_ubuntu
```

## 10. Mental Model

When in doubt:

- `scripts/start_dev.ps1` is for Windows desktop development
- `scripts/start_dev.sh work_ubuntu` is for Ubuntu desktop development
- `scripts/deploy_dev.sh` is for the real robot on the Raspberry Pi

If something behaves strangely, check:

1. which machine you are on
2. which profile you are running
3. whether the container logs show startup errors
