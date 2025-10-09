docker buildx build -f Dockerfile.base -t ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest --platform linux/arm64 .
docker buildx push ghcr.io/swisscatplus/em_onrobot/em_robot_base:latest
