#!/bin/bash

# 현재 스크립트 위치(docker/)의 상위 폴더(PIKA_ROS2)를 로컬 경로로 설정
LOCAL_PATH=$(cd "$(dirname "$0")/.." && pwd)
CONTAINER_NAME="pika_dev"
IMAGE_NAME="pika_ros2:latest"
DOCKER_PATH="/pika_ros2"
MY_DOMAIN_ID=10

echo "=== Pika ROS 2 Workspace Linker ==="
echo "[Link] Local: $LOCAL_PATH"
echo "[Link] Docker: $DOCKER_PATH"

# GUI 권한 허용
xhost +local:docker > /dev/null

# 기존 컨테이너 삭제
docker rm -f $CONTAINER_NAME 2>/dev/null

# 실행 (모든 USB 장치 및 로컬 폴더 연결)
docker run -it \
  --name $CONTAINER_NAME \
  --privileged \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=$MY_DOMAIN_ID \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$LOCAL_PATH:$DOCKER_PATH" \
  -v /dev:/dev \
  -v /sys:/sys:ro \
  -v /run/udev:/run/udev:ro \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  $IMAGE_NAME bash