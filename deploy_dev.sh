#!/bin/bash
set -e

CONTAINER_NAME="${CONTAINER_NAME:-em_robot_dev}"
BASE_IMAGE="${BASE_IMAGE:-ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest}"
LOCAL_BASE_IMAGE="${LOCAL_BASE_IMAGE:-em_robot_base:local}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-10}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION_VALUE:-rmw_fastrtps_cpp}"
FASTRTPS_PROFILE_PATH="${FASTRTPS_PROFILE_PATH:-/root/.ros/fastdds.xml}"
EM_ROBOT_PROFILE_VALUE="${EM_ROBOT_PROFILE_VALUE:-real_robot}"
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Pulling base image..."
if docker pull "$BASE_IMAGE"; then
  RUN_IMAGE="$BASE_IMAGE"
else
  echo "Registry pull failed for $BASE_IMAGE."
  echo "Falling back to a local base image build: $LOCAL_BASE_IMAGE"
  docker build -f Dockerfile.base -t "$LOCAL_BASE_IMAGE" .
  RUN_IMAGE="$LOCAL_BASE_IMAGE"
fi

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
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_VALUE" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_PROFILE_PATH" \
  -e EM_ROBOT_PROFILE="$EM_ROBOT_PROFILE_VALUE" \
  -v "$WORKSPACE_DIR:/ros2_ws" \
  -v "$WORKSPACE_DIR/fastdds.xml:$FASTRTPS_PROFILE_PATH:ro" \
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
  "$RUN_IMAGE" \
  -lc "source /opt/ros/\$ROS_DISTRO/setup.bash && cd /ros2_ws && colcon build --symlink-install --packages-select em_robot bno055 em_robot_srv && source /ros2_ws/install/setup.bash && ros2 launch em_robot em_robot.launch.py profile:=\$EM_ROBOT_PROFILE"

echo "Dev container started."
echo "Use: docker logs -f $CONTAINER_NAME"
echo "Shell in with: docker exec -it $CONTAINER_NAME bash"
