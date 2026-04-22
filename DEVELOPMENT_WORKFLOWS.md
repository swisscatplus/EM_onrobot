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

## 2. Windows Home Development

Use this when working from home on Windows with Docker Desktop.

Current behavior of `home_windows`:

- fake movement
- fake IMU
- no camera
- no RViz by default

### Start

```powershell
.\start_dev.ps1
```

This is equivalent to:

```powershell
.\start_dev.ps1 -Profile home_windows
```

### Stop

```powershell
.\start_dev.ps1 -Profile home_windows -Action down
```

### See Logs

```powershell
docker compose -f compose.yaml logs -f em_robot_home_windows
```

### Open a Shell in the Container

```powershell
docker compose -f compose.yaml exec em_robot_home_windows bash
```

### Rebuild the Container

```powershell
docker compose -f compose.yaml build em_robot_home_windows
```

### Rebuild Without Cache

```powershell
docker compose -f compose.yaml build --no-cache em_robot_home_windows
```

## 3. Ubuntu Desktop Development

Use this when working on Ubuntu at work with Docker.

Current behavior of `work_ubuntu`:

- fake movement
- fake IMU
- OpenCV camera backend
- RViz enabled

### Start

```bash
./start_dev.sh work_ubuntu
```

### Stop

```bash
./start_dev.sh work_ubuntu down
```

### See Logs

```bash
docker compose -f compose.yaml -f compose.ubuntu.yaml logs -f em_robot_work_ubuntu
```

### Open a Shell in the Container

```bash
docker compose -f compose.yaml -f compose.ubuntu.yaml exec em_robot_work_ubuntu bash
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
./start_dev.sh work_ubuntu
```

## 4. Real Robot Development on the Raspberry Pi

Use this on the robot itself.

### Start the Dev Container

```bash
EM_ROBOT_PROFILE_VALUE=real_robot ./deploy_dev.sh
```

### Default Start

If you do not set the profile explicitly, the current script also starts `real_robot` by default.

```bash
./deploy_dev.sh
```

### See Logs

```bash
docker logs -f em_robot_dev
```

### Open a Shell in the Container

```bash
docker exec -it em_robot_dev bash
```

## 5. Useful Commands Inside the Container

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
```

## 6. Profiles

Profile files live here:

- [real_robot.yaml](src/em_robot/config/profiles/real_robot.yaml)
- [work_ubuntu.yaml](src/em_robot/config/profiles/work_ubuntu.yaml)
- [home_windows.yaml](src/em_robot/config/profiles/home_windows.yaml)

These files decide which backends are used:

- movement backend
- IMU backend
- camera backend
- RViz on/off
- EKF on/off

If you want to change behavior, start by editing the relevant profile.

## 7. Most Common Recovery Commands

### Stop Everything

Windows:

```powershell
docker compose -f compose.yaml down
```

Ubuntu:

```bash
docker compose -f compose.yaml -f compose.ubuntu.yaml down
```

### Rebuild and Restart

Windows:

```powershell
docker compose -f compose.yaml build em_robot_home_windows
.\start_dev.ps1
```

Ubuntu:

```bash
docker compose -f compose.yaml -f compose.ubuntu.yaml build em_robot_work_ubuntu
./start_dev.sh work_ubuntu
```

### Clean Rebuild

Windows:

```powershell
docker compose -f compose.yaml build --no-cache em_robot_home_windows
```

Ubuntu:

```bash
docker compose -f compose.yaml -f compose.ubuntu.yaml build --no-cache em_robot_work_ubuntu
```

## 8. Mental Model

When in doubt:

- `start_dev.ps1` is for Windows desktop development
- `start_dev.sh work_ubuntu` is for Ubuntu desktop development
- `deploy_dev.sh` is for the real robot on the Raspberry Pi

If something behaves strangely, check:

1. which machine you are on
2. which profile you are running
3. whether the container logs show startup errors
