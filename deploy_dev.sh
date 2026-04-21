#!/bin/bash
set -e

CONTAINER_NAME="${CONTAINER_NAME:-em_robot_dev}"
BASE_IMAGE="${BASE_IMAGE:-ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest}"
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Pulling base image..."
docker pull "$BASE_IMAGE"

echo "Stopping old dev container..."
docker stop "$CONTAINER_NAME" 2>/dev/null || true

echo "Removing old dev container..."
docker rm "$CONTAINER_NAME" 2>/dev/null || true

echo "Starting dev container with bind-mounted workspace..."
docker run -d \
  --restart unless-stopped \
  --network host \
  --name "$CONTAINER_NAME" \
  --entrypoint /bin/bash \
  -v "$WORKSPACE_DIR:/ros2_ws" \
  -v /run/udev:/run/udev:ro \
  -v /dev/:/dev/ \
  --cap-add SYS_ADMIN \
  --cap-add NET_ADMIN \
  --privileged \
  --device /dev/video0 \
  --device /dev/i2c-1:/dev/i2c-1 \
  --device /dev/video1 \
  --device /dev/media0 \
  --device /dev/media1 \
  --device /dev/dri/card0 \
  --group-add video \
  "$BASE_IMAGE" \
  -lc "source /opt/ros/\$ROS_DISTRO/setup.bash && cd /ros2_ws && colcon build --symlink-install --packages-select em_robot bno055 em_robot_srv && source /ros2_ws/install/setup.bash && ros2 launch em_robot em_robot.launch.py"

echo "Dev container started."
echo "Use: docker logs -f $CONTAINER_NAME"
echo "Shell in with: docker exec -it $CONTAINER_NAME bash"
