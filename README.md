<br />
<div align="center">
  <a href="https://github.com/swisscatplus/EM_onrobot">
    <img src="pictures/logo.png" alt="Logo" height="80">
  </a>
  <h1 align="center">EM_onrobot</h1>
</div>

This is the software that runs **on the mobile robot** (on its Raspberry Pi). It:

- drives the two wheels from velocity commands (`cmd_vel`)
- measures how far the wheels turned (wheel odometry)
- reads the BNO055 IMU (orientation and acceleration sensor)
- finds the robot's position on the lab map with the camera and ArUco markers
- merges all of that into one position estimate (EKF, `robot_localization`)
- reports the robot's health (diagnostics)

Path planning, path following and fleet management are **not** here: they live in [EM_fleetmanager](https://github.com/swisscatplus/EM_fleetmanager), which runs on a laptop and sends commands to the robot.

> [!WARNING]
> **Protect the SD card.** The Raspberry Pi runs from an SD card, and SD cards wear out quickly when they are written to all the time. Several robots have already lost their card this way. Please:
>
> - Run `sudo ./scripts/setup_pi_sd_protection.sh` once on every new robot (see [step 6](#6-protect-the-sd-card)).
> - **Do not** record ROS bags (`ros2 bag record`), images or other large files on the robot's SD card. Record them on your laptop, or on a USB stick plugged into the robot.
> - **Do not** add `colcon build` to the container's start command, and do not change `--restart unless-stopped` in `deploy_dev.sh`. Both are set up on purpose (see [How it starts](#how-the-robot-starts-by-itself)).
> - Do not add log messages that print many times per second. If a message runs inside a loop or timer, throttle it: `self.get_logger().warn("...", throttle_duration_sec=2.0)`.
> - When you can, shut down cleanly (`sudo shutdown now`) before cutting the battery. Cutting the power while the card is being written can corrupt it.
>
> Buy "High Endurance" SD cards (for example Samsung PRO Endurance or SanDisk High Endurance) for the robots.

## Contents

1. [Words you need to know](#words-you-need-to-know)
2. [How the robot starts by itself](#how-the-robot-starts-by-itself)
3. [First-time setup of a new robot](#first-time-setup-of-a-new-robot)
4. [Everyday use](#everyday-use)
5. [Updating the code on the robot](#updating-the-code-on-the-robot)
6. [Talking to the robot from your laptop](#talking-to-the-robot-from-your-laptop)
7. [Troubleshooting](#troubleshooting)
8. [Advanced topics](#advanced-topics)

## Words you need to know

| Word | What it means here |
|---|---|
| **ROS 2** | The framework the robot software is built with. We use the **Humble** version. |
| **Node** | One small program, for example `base_controller`, which drives the wheels. The robot runs about ten nodes at the same time. |
| **Topic** | A named channel that nodes use to send each other messages, for example `/robot1/cmd_vel` (speed commands) or `/robot1/odomWheel` (wheel odometry). |
| **Namespace** | A prefix in front of every topic, such as `robot1`, so that several robots can share the same network without mixing up their topics. |
| **ROS_DOMAIN_ID** | A number that decides which machines can see each other's topics. The robot uses **10**. Your laptop must use 10 too. |
| **Launch file** | A file that starts several nodes at once. Here: `bringup.launch.py`. |
| **Docker image** | A frozen, ready-to-use copy of an operating system with ROS and all libraries installed. |
| **Docker container** | A running instance of an image. The robot software runs inside a container called `em_robot_dev`. |
| **colcon build** | The command that compiles the ROS code. It creates the `build/` and `install/` folders. |

## How the robot starts by itself

```text
Robot powers on
  └─ Raspberry Pi boots
      └─ Docker starts
          └─ Docker restarts the container "em_robot_dev" (because of --restart unless-stopped)
              └─ the container runs: ros2 launch em_robot bringup.launch.py
                  └─ all the nodes start: wheels, IMU, camera, localization, EKF, diagnostics
```

You do **not** need to start anything by hand after a power cycle, for example after charging the battery. Wait about one minute after power-on, and the robot is ready.

The code is compiled **only** when you run `./scripts/deploy_dev.sh`, never at start-up. This keeps start-up fast and saves the SD card.

## First-time setup of a new robot

Do this once per robot. Every step ends with a check: do not go to the next step until the check passes.

You need:

- the robot with its Raspberry Pi, camera, BNO055 IMU, U2D2 adapter and the two Dynamixel motors
- a charged battery (the U2D2 does **not** power the motors: the motors need the battery)
- a laptop on the same Wi-Fi network as the robot
- an internet connection for the robot during setup (to download the Docker image)

### 1. Connect to the Raspberry Pi

From your laptop, connect over SSH. Replace `<user>` and `<robot-ip>` with the robot's values (ask your supervisor):

```bash
ssh <user>@<robot-ip>
```

All the commands below run **on the robot**, in this SSH session, unless a step says otherwise.

**Check:** your terminal prompt now shows the robot's name.

### 2. Install Docker

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
```

Log out (`exit`) and SSH in again so the group change applies.

**Check:** `docker run --rm hello-world` prints "Hello from Docker!".

### 3. Download this repository

```bash
cd ~
git clone https://github.com/swisscatplus/EM_onrobot.git
cd EM_onrobot
```

From now on, run every command from this `~/EM_onrobot` folder.

### 4. Give the robot its name

Every robot needs its own namespace (`robot1`, `robot2`, ...), otherwise two robots on the same network would mix their topics.

```bash
cp config/robot.local.env.example config/robot.local.env
nano config/robot.local.env
```

Set `ROBOT_NAMESPACE=robot1` (or `robot2`, ... if other robots already use `robot1`). Save with `Ctrl+O`, `Enter`, then quit with `Ctrl+X`.

This file is ignored by Git, so `git pull` never overwrites it.

**Check:** `cat config/robot.local.env` shows your namespace.

### 5. Set up the hardware access

**a. Motors: give the U2D2 a fixed name.** Linux names USB adapters `/dev/ttyUSB0`, `/dev/ttyUSB1`, ..., and the number can change at every reboot. We create a rule so the U2D2 is always called `/dev/dynamixel`.

First, plug in the U2D2 and find its IDs:

```bash
lsusb | grep -i 0403
```

You should see something like `ID 0403:6001 Future Technology Devices ...`. The part after `0403:` is the product ID (here `6001`). If yours is different, for example `6014`, use your value in `idProduct` below.

Create the rule:

```bash
sudo tee /etc/udev/rules.d/99-dynamixel.rules > /dev/null <<'EOF'
# Match the FTDI chip of the Dynamixel U2D2
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="dynamixel", MODE="0666", GROUP="dialout"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Check:** `ls -l /dev/dynamixel` shows `/dev/dynamixel -> ttyUSB0` (the number may differ).

If another FTDI adapter is plugged into the robot, the rule can pick the wrong one. In that case, add `ATTRS{serial}=="<serial>"` to the rule, using the U2D2 serial number shown by `udevadm info -a -n /dev/ttyUSB0 | grep serial`.

**b. IMU: enable I2C.** The BNO055 is connected over I2C.

```bash
sudo raspi-config
```

Go to **Interface Options → I2C → Yes**, then reboot (`sudo reboot`).

**Check:** `ls /dev/i2c-1` exists, and `sudo i2cdetect -y 1` shows `28` in the table (the IMU's address).

**c. Camera.** **Check:** `ls /dev/video0` exists.

### 6. Protect the SD card

```bash
sudo ./scripts/setup_pi_sd_protection.sh
sudo reboot
```

This script:

- stops Linux from writing on every file read (`noatime`)
- keeps the system logs in RAM instead of on the card (they are lost at each reboot)
- turns off swap on the SD card
- limits the size of Docker logs

It backs up every file it changes (`<file>.bak.<date>`). You can run it again safely.

**Check:** after the reboot, `swapon --show` prints nothing.

### 7. Allow the robot to download the Docker image

The robot's Docker image (`ghcr.io/swisscatplus/em_onrobot/em_robot_base`) is stored on GitHub. If it is private, log in once with a GitHub account that has access. Create a token at GitHub → Settings → Developer settings → Personal access tokens, with the `read:packages` permission, then run:

```bash
docker login ghcr.io -u <your-github-username>
```

Paste the token when it asks for a password.

### 8. Deploy

Make sure the battery is connected (the motors must be powered), then:

```bash
./scripts/deploy_dev.sh
```

The first time, this downloads a large image (several GB) and can take a while. The script:

1. compiles the code once (`colcon build`)
2. stops the old container, if there is one
3. starts the `em_robot_dev` container, which launches all the nodes

**Check:** follow [Is the robot running?](#is-the-robot-running) below.

Finally, reboot the robot (`sudo reboot`) and check again: this confirms that it starts by itself.

## Everyday use

### Power on

Connect the battery and wait about one minute. Everything starts by itself.

### Is the robot running?

On the robot (over SSH):

```bash
docker ps
```

You should see `em_robot_dev` with the status `Up ...`.

- `Restarting` means that a node crashes at start-up. Read the logs, see [Troubleshooting](#troubleshooting).
- If `em_robot_dev` is missing, run `./scripts/deploy_dev.sh`.

Read the logs (`Ctrl+C` to stop reading; this does not stop the robot):

```bash
docker logs -f --tail 100 em_robot_dev
```

List the topics from inside the container:

```bash
docker exec -it em_robot_dev bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic list
```

You should see topics such as `/robot1/cmd_vel`, `/robot1/odomWheel`, `/robot1/bno055/imu` and `/robot1/odometry/filtered`. Type `exit` to leave the container.

### Useful topics

(Replace `robot1` with your robot's namespace.)

| Topic | What it contains |
|---|---|
| `/robot1/cmd_vel` | Speed commands **to** the robot (linear speed in m/s, rotation speed in rad/s) |
| `/robot1/odomWheel` | Position estimated from the wheels only |
| `/robot1/bno055/imu` | IMU data |
| `/robot1/odometry/filtered` | Best position estimate (wheels + IMU, fused by the EKF) |
| `/tf` | Positions of all the frames (`map`, `robot1/odom`, `robot1/base_link`, ...) |

See how often a topic publishes: `ros2 topic hz /robot1/odomWheel`. Print its messages: `ros2 topic echo /robot1/odomWheel`.

### Make the robot move (test)

> [!CAUTION]
> The robot will move. Put it on the floor with free space around it, or lift it so the wheels do not touch anything.

Inside the container (see above), send a slow forward command 10 times per second:

```bash
ros2 topic pub -r 10 /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Press `Ctrl+C` to stop sending. The robot stops by itself about one second after the last command (safety watchdog).

To send the robot along a path, use [EM_fleetmanager](https://github.com/swisscatplus/EM_fleetmanager) from your laptop.

### Power off

When you can, shut down cleanly before cutting the battery:

```bash
sudo shutdown now
```

Wait until the Raspberry Pi's green LED stops blinking, then disconnect the battery.

## Updating the code on the robot

After new code was pushed to GitHub, on the robot:

```bash
cd ~/EM_onrobot
git pull
./scripts/deploy_dev.sh
```

Changed a file directly on the robot?

| What you changed | What to run |
|---|---|
| An existing Python file (`.py`) or config file (`.yaml`) | `docker restart em_robot_dev` is enough |
| Added a new file, node, message or service, or changed `setup.py` / `package.xml` | `./scripts/deploy_dev.sh` (recompiles) |
| Not sure | `./scripts/deploy_dev.sh`: it is always safe |

`docker restart` does **not** recompile. If the compilation fails, `deploy_dev.sh` stops and the robot keeps running the previous version.

## Talking to the robot from your laptop

Your laptop can see the robot's topics and send it commands, if:

1. **Both are on the same network**: the robot's Wi-Fi router.
2. **Your laptop has no other network active.** If your laptop is also plugged into another network (for example a network cable), ROS may send its discovery messages there and never find the robot. Unplug the cable, or run `nmcli dev disconnect eth0`.
3. **Your laptop uses the same domain**: `export ROS_DOMAIN_ID=10`. Put this line in your `~/.bashrc` so you do not forget it.
4. **Your laptop uses ROS 2 Humble**, like the robot. Other versions (for example Jazzy) can often list the topics, but may fail to read some messages. The simplest way to get Humble on any Linux laptop is the laptop Docker setup of EM_fleetmanager.

Then, on your laptop:

```bash
ros2 daemon stop    # forget old discovery results
ros2 topic list
```

**Check:** you see the `/robot1/...` topics.

If you do not see them:

- Check that your laptop can reach the robot: `ping <robot-ip>`.
- Check that ROS discovery can get through the network. Run `ros2 multicast receive` on your laptop, then `ros2 multicast send` on the robot, inside the container. If nothing arrives, the network or a firewall blocks discovery (`sudo ufw status`).

## Troubleshooting

First, always read the logs: `docker logs --tail 100 em_robot_dev`. Then find your error below.

| What you see | Most likely cause | What to do |
|---|---|---|
| `Failed to open Dynamixel port: /dev/dynamixel` | The U2D2 is not plugged in, or the udev rule is missing | Check `ls -l /dev/dynamixel`, and redo [step 5a](#5-set-up-the-hardware-access) |
| `Profile acceleration setup failed on motor ID=2` or `Torque enable failed on motor ID=...` | The robot can open the U2D2 but the motors do not answer. Usually the motors are not powered (battery flat or disconnected). It can also be a loose motor cable, or a motor whose ID or baud rate was changed. | Check the battery and the cables. At power-on, the LEDs on the motors should blink once. The motors must use IDs 1 (left) and 2 (right) at 57600 baud, which you can check with Dynamixel Wizard 2.0 on a laptop. |
| `Failed to read present position for motor ID=...` while running | The motor connection is lost | Check the battery level and the motor cables |
| IMU errors (`Receiving sensor data failed`, ...) | I2C is disabled or the IMU is badly connected | Redo [step 5b](#5-set-up-the-hardware-access). `sudo i2cdetect -y 1` must show `28`. |
| `docker ps` shows `Restarting` | A node crashes at start-up | Read the logs to find which one, then look for its error in this table |
| `Registry pull failed` or `unauthorized` | You are not logged in to GitHub's registry | Redo [step 7](#7-allow-the-robot-to-download-the-docker-image) |
| `lookup registry-1.docker.io ... server misbehaving` or `Temporary failure in name resolution` | The robot has no internet | Connect the robot to a network with internet access to download the image |
| The topics show on the robot but not on the laptop | Network or configuration problem | See [Talking to the robot from your laptop](#talking-to-the-robot-from-your-laptop) |
| The robot does not start after a reboot | The container was removed, or Docker does not start at boot | `sudo systemctl enable docker`, then `./scripts/deploy_dev.sh` |

Still stuck? Copy the last 100 lines of the logs (`docker logs --tail 100 em_robot_dev`) and send them to your supervisor.

## Advanced topics

The sections below are for maintainers. You do not need them to deploy and use the robot.

### Runtime profiles

A profile is a YAML file that decides which nodes start and with which settings. Profile files live in [`src/em_robot/config/profiles`](./src/em_robot/config/profiles).

| Profile | Intended system | Movement | IMU | Camera | Purpose |
|---|---|---|---|---|---|
| `real_robot` (default) | Raspberry Pi on robot | real Dynamixel base | BNO055 | Picamera2 | Production robot runtime |
| `work_ubuntu_localization_test` | Ubuntu workstation | disabled | disabled | OpenCV camera | Desktop validation of marker localization |

`deploy_dev.sh` uses `real_robot` unless you set `EM_ROBOT_PROFILE_VALUE` in `config/robot.local.env`. The container runs:

```bash
ros2 launch em_robot bringup.launch.py profile:=real_robot namespace:=robot1
```

The runtime URDF used by `robot_state_publisher` is packaged with `em_robot` at [`src/em_robot/urdf/simple_box_robot.urdf`](./src/em_robot/urdf/simple_box_robot.urdf). Hardware design files are intentionally kept outside this software repository.

### What `deploy_dev.sh` does

1. Loads `config/robot.local.env` (namespace, profile).
2. Picks the base image: the local `em_robot_base:local` if it exists, otherwise it pulls `ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest`. `ALLOW_LOCAL_BASE_BUILD=1` builds it on the robot if the pull fails (this takes a very long time on a Raspberry Pi).
3. Runs `colcon build --symlink-install` once, in a throwaway container. The workspace is bind-mounted, so `build/` and `install/` are written to the repository folder. colcon's own logs are discarded.
4. Replaces the `em_robot_dev` container. That container only runs `ros2 launch`, with:
   - `--restart unless-stopped`, which restarts it at boot. Do not use `on-failure`: Docker does not restart those containers at boot.
   - Docker logs capped at 2 × 10 MB
   - `/root/.ros/log` and `/tmp` in RAM (tmpfs)
   - host networking, so ROS discovery works on the robot's network
   - access to `/dev` (motors, camera, I2C)

### Ubuntu validation

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

### Marker survey

Run marker survey on the robot:

```bash
./scripts/run_marker_survey.sh real_robot single-shot
```

Run marker survey from the Ubuntu validation container:

```bash
export EM_ROBOT_CAMERA_DEVICE=/dev/video0
./scripts/run_marker_survey.sh work_ubuntu_localization_test single-shot
```

Manual mode keeps the node alive and lets you set the known robot pose before saving visible markers:

```bash
./scripts/run_marker_survey.sh real_robot manual 0.0 0.0 0.0
```

### Camera calibration

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

Generated calibration images, validation outputs, and recorded bags are ignored by Git. Calibration images are written to disk: on the robot, keep their number small (see the SD card warning at the top).

### Build and test

Build the ROS workspace (inside a ROS 2 Humble environment):

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

Run the tests on your laptop rather than on the robot, to spare the SD card.

### Repository layout

```text
EM_onrobot/
├── config/
│   ├── fastdds.xml                 # ROS network (DDS) settings
│   └── robot.local.env.example     # per-robot settings template (copy to robot.local.env)
├── docker/
│   ├── compose.yaml                # Ubuntu validation workflow
│   ├── compose.ubuntu.yaml
│   ├── Dockerfile
│   ├── Dockerfile.base             # robot base image (ROS Humble + libcamera + Dynamixel SDK)
│   ├── Dockerfile.desktop
│   └── entrypoint.sh
├── scripts/
│   ├── deploy_dev.sh               # build + (re)start the robot container
│   ├── setup_pi_sd_protection.sh   # one-time SD card protection for a new robot
│   └── ...                         # calibration, marker survey, image builds
├── src/
│   ├── em_robot/                   # main package: nodes, launch file, config, profiles
│   ├── em_robot_srv/               # custom ROS services
│   └── bno055/                     # IMU driver (vendored)
└── pictures/
```

### Dependency notes

`em_robot` is an `ament_python` package. ROS/system dependencies are declared in `package.xml` where rosdep keys exist. Picamera2 and libcamera are provided by the robot Docker base image because the Raspberry Pi camera stack is built there rather than resolved by rosdep.

The vendored `bno055` package keeps its upstream BSD license in [`src/bno055/LICENSE`](./src/bno055/LICENSE).

## License

MIT. See [`LICENSE`](./LICENSE).
