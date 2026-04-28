#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/docker/compose.yaml"
COMPOSE_UBUNTU_FILE="$REPO_ROOT/docker/compose.ubuntu.yaml"
DOCKER_DIR="$REPO_ROOT/docker"
CONFIG_DIR="$REPO_ROOT/config"

PROFILE="${1:-work_ubuntu}"
SURVEY_MODE="${2:-single-shot}"
KNOWN_X="${3:-0.0}"
KNOWN_Y="${4:-0.0}"
KNOWN_YAW="${5:-0.0}"
MARKER_MAP_FILE="${6:-/ros2_ws/src/em_robot/config/marker_map.yaml}"
CALIBRATION_FILE="${7:-}"
CONTAINER_NAME="${CONTAINER_NAME:-em_robot_dev}"
BASE_IMAGE="${BASE_IMAGE:-ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest}"
LOCAL_BASE_IMAGE="${LOCAL_BASE_IMAGE:-em_robot_base:local}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-10}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION_VALUE:-rmw_fastrtps_cpp}"
FASTRTPS_PROFILE_PATH="${FASTRTPS_PROFILE_PATH:-/root/.ros/fastdds.xml}"
FORCE_LOCAL_BASE_BUILD="${FORCE_LOCAL_BASE_BUILD:-0}"

case "$PROFILE" in
  work_ubuntu)
    SERVICE="em_robot_work_ubuntu"
    COMPOSE_ARGS=(-f "$COMPOSE_FILE" -f "$COMPOSE_UBUNTU_FILE" --profile work_ubuntu)
    CAMERA_BACKEND="picamera2"
    CAMERA_SOURCE="0"
    DEFAULT_CALIBRATION_FILE="/ros2_ws/src/em_robot/config/calibration.yaml"
    ;;
  work_ubuntu_aruco_test)
    SERVICE="em_robot_work_ubuntu_aruco_test"
    COMPOSE_ARGS=(-f "$COMPOSE_FILE" -f "$COMPOSE_UBUNTU_FILE" --profile work_ubuntu_aruco_test)
    CAMERA_BACKEND="opencv"
    CAMERA_SOURCE="${EM_ROBOT_CAMERA_DEVICE:-0}"
    DEFAULT_CALIBRATION_FILE="/ros2_ws/src/em_robot/config/calibration_ubuntu_test.yaml"
    ;;
  work_ubuntu_localization_test)
    SERVICE="em_robot_work_ubuntu_localization_test"
    COMPOSE_ARGS=(-f "$COMPOSE_FILE" -f "$COMPOSE_UBUNTU_FILE" --profile work_ubuntu_localization_test)
    CAMERA_BACKEND="opencv"
    CAMERA_SOURCE="${EM_ROBOT_CAMERA_DEVICE:-0}"
    DEFAULT_CALIBRATION_FILE="/ros2_ws/src/em_robot/config/calibration_ubuntu_test.yaml"
    ;;
  home_windows)
    SERVICE="em_robot_home_windows"
    COMPOSE_ARGS=(-f "$COMPOSE_FILE" --profile home_windows)
    CAMERA_BACKEND="opencv"
    CAMERA_SOURCE="0"
    DEFAULT_CALIBRATION_FILE="/ros2_ws/src/em_robot/config/calibration_ubuntu_test.yaml"
    ;;
  real_robot)
    SERVICE=""
    CAMERA_BACKEND="picamera2"
    CAMERA_SOURCE="0"
    DEFAULT_CALIBRATION_FILE="/ros2_ws/src/em_robot/config/calibration.yaml"
    ;;
  *)
    echo "Unsupported profile: $PROFILE"
    echo "Use one of: real_robot, work_ubuntu, work_ubuntu_aruco_test, work_ubuntu_localization_test, home_windows"
    exit 1
    ;;
esac

if [[ "$SURVEY_MODE" != "manual" && "$SURVEY_MODE" != "single-shot" ]]; then
  echo "Unsupported survey mode: $SURVEY_MODE"
  echo "Use either: manual, single-shot"
  exit 1
fi

if [[ -z "$CALIBRATION_FILE" ]]; then
  CALIBRATION_FILE="$DEFAULT_CALIBRATION_FILE"
fi

if [[ "$PROFILE" == "real_robot" ]]; then
  if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "The deployed robot container $CONTAINER_NAME is already running."
    echo "Stop it first so marker_survey can use the camera without competing with localization."
    echo
    echo "Suggested command:"
    echo "  docker stop $CONTAINER_NAME"
    exit 1
  fi
else
  if docker compose "${COMPOSE_ARGS[@]}" ps --status running "$SERVICE" | grep -q "$SERVICE"; then
    echo "The service $SERVICE is already running."
    echo "Stop it first so marker_survey can use the camera without competing with localization."
    echo
    echo "Suggested command:"
    echo "  ./scripts/start_dev.sh $PROFILE down"
    exit 1
  fi
fi

echo "Starting marker_survey in Docker for profile '$PROFILE'"
echo "Survey mode: $SURVEY_MODE"
echo "Known robot pose: x=$KNOWN_X y=$KNOWN_Y yaw=$KNOWN_YAW"
echo "Marker map file: $MARKER_MAP_FILE"
echo "Calibration file: $CALIBRATION_FILE"
echo "Camera backend: $CAMERA_BACKEND"
echo "Camera source: $CAMERA_SOURCE"
echo
if [[ "$SURVEY_MODE" == "manual" ]]; then
  echo "After the node starts, set or update the pose if needed with:"
  echo "  ros2 service call /set_known_robot_pose em_robot_srv/srv/SetInitialPose \"{x: $KNOWN_X, y: $KNOWN_Y, yaw: $KNOWN_YAW}\""
  echo "Then save stable markers with:"
  echo "  ros2 service call /save_visible_markers std_srvs/srv/Trigger \"{}\""
else
  echo "The node will save the first stable marker set it sees, then exit automatically."
fi
echo

if [[ "$PROFILE" == "real_robot" ]]; then
  if [[ "$FORCE_LOCAL_BASE_BUILD" == "1" ]]; then
    echo "FORCE_LOCAL_BASE_BUILD=1, building local base image: $LOCAL_BASE_IMAGE"
    docker build -f "$DOCKER_DIR/Dockerfile.base" -t "$LOCAL_BASE_IMAGE" "$REPO_ROOT"
    RUN_IMAGE="$LOCAL_BASE_IMAGE"
  else
    echo "Pulling base image..."
    if docker pull "$BASE_IMAGE"; then
      RUN_IMAGE="$BASE_IMAGE"
    else
      echo "Registry pull failed for $BASE_IMAGE."
      echo "Falling back to a local base image build: $LOCAL_BASE_IMAGE"
      docker build -f "$DOCKER_DIR/Dockerfile.base" -t "$LOCAL_BASE_IMAGE" "$REPO_ROOT"
      RUN_IMAGE="$LOCAL_BASE_IMAGE"
    fi
  fi

  docker run --rm \
    --network host \
    --name em_robot_marker_survey \
    --entrypoint /bin/bash \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" \
    -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_VALUE" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_PROFILE_PATH" \
    -v "$REPO_ROOT:/ros2_ws" \
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
    -lc "
      source /opt/ros/\$ROS_DISTRO/setup.bash &&
      cd /ros2_ws &&
      colcon build --symlink-install --packages-select em_robot em_robot_srv bno055 &&
      source install/setup.bash &&
      ros2 run em_robot marker_survey --ros-args \
        -p camera_backend:=$CAMERA_BACKEND \
        -p camera_source:=$CAMERA_SOURCE \
        -p config_file:=$CALIBRATION_FILE \
        -p marker_map_file:=$MARKER_MAP_FILE \
        -p survey_mode:=$SURVEY_MODE \
        -p known_robot_x:=$KNOWN_X \
        -p known_robot_y:=$KNOWN_Y \
        -p known_robot_yaw:=$KNOWN_YAW
    "
else
  docker compose "${COMPOSE_ARGS[@]}" run --rm "$SERVICE" bash -lc "
    source /opt/ros/\$ROS_DISTRO/setup.bash &&
    cd /ros2_ws &&
    colcon build --symlink-install --packages-select em_robot em_robot_srv bno055 &&
    source install/setup.bash &&
    ros2 run em_robot marker_survey --ros-args \
      -p camera_backend:=$CAMERA_BACKEND \
      -p camera_source:=$CAMERA_SOURCE \
      -p config_file:=$CALIBRATION_FILE \
      -p marker_map_file:=$MARKER_MAP_FILE \
      -p survey_mode:=$SURVEY_MODE \
      -p known_robot_x:=$KNOWN_X \
      -p known_robot_y:=$KNOWN_Y \
      -p known_robot_yaw:=$KNOWN_YAW
  "
fi
