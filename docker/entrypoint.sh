#!/bin/bash
set -e

# 1. ROS 2 및 워크스페이스 환경 소싱
# 이 단계에서 기본 PYTHONPATH 등이 설정됩니다.
source "/opt/ros/jazzy/setup.bash"
if [ -f "/pika_ros2/install/setup.bash" ]; then
    source "/pika_ros2/install/setup.bash"
fi

# 2. libsurvive 및 커스텀 경로 추가
# Dockerfile의 경고 없이 안전하게 추가 가능합니다.
export LIBSURVIVE_HOME="/pika_ros2/src/pika_driver/pika_driver/libsurvive"
export PYTHONPATH="${PYTHONPATH:-}:${LIBSURVIVE_HOME}/bindings/python"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${LIBSURVIVE_HOME}/bin"

# 3. 추가적인 워크스페이스 소스 경로 (필요 시)
export PYTHONPATH="${PYTHONPATH}:/pika_ros2/src"

exec "$@"