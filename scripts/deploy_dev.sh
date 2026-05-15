#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_DIR="$WORKSPACE_DIR/docker"
CONFIG_DIR="$WORKSPACE_DIR/config"
LOCAL_ROBOT_CONFIG="${LOCAL_ROBOT_CONFIG:-$CONFIG_DIR/robot.local.env}"

if [ -f "$LOCAL_ROBOT_CONFIG" ]; then
  # shellcheck source=/dev/null
  source "$LOCAL_ROBOT_CONFIG"
  echo "Loaded local robot config: $LOCAL_ROBOT_CONFIG"
else
  echo "No local robot config found at $LOCAL_ROBOT_CONFIG"
  echo "Create it from config/robot.local.env.example to set this robot's namespace."
fi

CONTAINER_NAME="${CONTAINER_NAME:-em_robot_dev}"
BASE_IMAGE="${BASE_IMAGE:-ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest}"
LOCAL_BASE_IMAGE="${LOCAL_BASE_IMAGE:-em_robot_base:local}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-10}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION_VALUE:-rmw_fastrtps_cpp}"
FASTRTPS_PROFILE_PATH="${FASTRTPS_PROFILE_PATH:-/root/.ros/fastdds.xml}"
EM_ROBOT_PROFILE_VALUE="${EM_ROBOT_PROFILE_VALUE:-real_robot}"
ROBOT_NAMESPACE_VALUE="${ROBOT_NAMESPACE_VALUE:-${ROBOT_NAMESPACE:-robot1}}"
FORCE_LOCAL_BASE_BUILD="${FORCE_LOCAL_BASE_BUILD:-0}"
ALLOW_LOCAL_BASE_BUILD="${ALLOW_LOCAL_BASE_BUILD:-0}"
PREFER_LOCAL_BASE_IMAGE="${PREFER_LOCAL_BASE_IMAGE:-1}"

build_local_base_image() {
  echo "Building local base image: $LOCAL_BASE_IMAGE"
  docker build -f "$DOCKER_DIR/Dockerfile.base" -t "$LOCAL_BASE_IMAGE" "$WORKSPACE_DIR"
}

if [ "$FORCE_LOCAL_BASE_BUILD" = "1" ]; then
  echo "FORCE_LOCAL_BASE_BUILD=1, building local base image: $LOCAL_BASE_IMAGE"
  build_local_base_image
  RUN_IMAGE="$LOCAL_BASE_IMAGE"
elif [ "$PREFER_LOCAL_BASE_IMAGE" = "1" ] && docker image inspect "$LOCAL_BASE_IMAGE" >/dev/null 2>&1; then
  echo "Using existing local base image: $LOCAL_BASE_IMAGE"
  RUN_IMAGE="$LOCAL_BASE_IMAGE"
else
  echo "Pulling base image..."
  if docker pull "$BASE_IMAGE"; then
    RUN_IMAGE="$BASE_IMAGE"
  elif docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
    echo "Registry pull failed, using cached base image: $BASE_IMAGE"
    RUN_IMAGE="$BASE_IMAGE"
  elif [ "$ALLOW_LOCAL_BASE_BUILD" = "1" ]; then
    echo "Registry pull failed for $BASE_IMAGE."
    echo "ALLOW_LOCAL_BASE_BUILD=1, falling back to a local base image build."
    build_local_base_image
    RUN_IMAGE="$LOCAL_BASE_IMAGE"
  else
    echo "Registry pull failed for $BASE_IMAGE."
    echo
    echo "The base image is probably private or your GHCR login expired."
    echo "Fix one of these, then run this script again:"
    echo "  1. docker login ghcr.io"
    echo "  2. Use an existing local image tagged as $LOCAL_BASE_IMAGE"
    echo "  3. Rebuild explicitly with ALLOW_LOCAL_BASE_BUILD=1 ./scripts/deploy_dev.sh"
    exit 1
  fi
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
  -e ROBOT_NAMESPACE="$ROBOT_NAMESPACE_VALUE" \
  -v "$WORKSPACE_DIR:/ros2_ws" \
  -v "$CONFIG_DIR/fastdds.xml:$FASTRTPS_PROFILE_PATH:ro" \
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
  -lc "source /opt/ros/\$ROS_DISTRO/setup.bash && cd /ros2_ws && colcon build --symlink-install --packages-select em_robot bno055 em_robot_srv && source /ros2_ws/install/setup.bash && ros2 launch em_robot bringup.launch.py profile:=\$EM_ROBOT_PROFILE namespace:=\$ROBOT_NAMESPACE"

echo "Dev container started."
echo "Robot namespace: $ROBOT_NAMESPACE_VALUE"
echo "Use: docker logs -f $CONTAINER_NAME"
echo "Shell in with: docker exec -it $CONTAINER_NAME bash"
