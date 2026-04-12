FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

# 1. 필수 시스템 패키지 설치
# python3-dev를 추가하여 Python 바인딩 빌드 호환성을 높였습니다.
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --fix-missing \
    python3-pip \
    python3-setuptools \
    python3-dev \
    python3-yaml \
    build-essential \
    cmake \
    git \
    udev \
    libudev-dev \
    zlib1g-dev \
    libx11-dev \
    libusb-1.0-0-dev \
    freeglut3-dev \
    liblapacke-dev \
    libopenblas-dev \
    v4l-utils \     
    ros-jazzy-cv-bridge \  
    ros-jazzy-image-transport \
    libgl1-mesa-dri \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*


    # 1. 빌드에 필요한 의존성 설치
RUN apt-get update && apt-get install -y \
    libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. librealsense 소스 빌드
WORKDIR /opt
RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    # [중요] FORCE_RSUSB_BACKEND: 커널 패치 없이 USB 장치에 직접 접근하도록 설정
    # BUILD_GRAPHICAL_EXAMPLES: realsense-viewer를 빌드함
    cmake .. \
        -DCMAKE_BUILD_TYPE=release \
        -DFORCE_RSUSB_BACKEND=true \
        -DBUILD_PYTHON_BINDINGS:bool=true \
        -DBUILD_EXAMPLES:bool=true \
        -DBUILD_GRAPHICAL_EXAMPLES:bool=true && \
    make -j$(nproc) && \
    make install && \
    # 빌드 후 용량 최적화를 위해 소스 및 빌드 폴더 삭제 (선택 사항)
    cd /opt && rm -rf librealsense

# 3. udev rule 적용 (도커 실행 시 권한 문제를 줄여줌)
RUN cp /opt/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ || true

# 2. 작업 디렉토리 설정 및 소스 복사
WORKDIR /pika_ros2
COPY . .

# 3. Python 의존성 설치
RUN if [ -f requirements.txt ]; then \
    pip3 install --no-cache-dir -r requirements.txt --break-system-packages; \
    fi

# 4. libsurvive 빌드 및 환경 변수 설정
# 이전 빌드 잔재(bin/, CMakeCache.txt)를 제거하여 경로 불일치 에러를 방지합니다.
WORKDIR /pika_ros2/src/pika_driver/pika_driver/libsurvive
RUN rm -rf bin/ CMakeCache.txt CMakeFiles/ && \
    if [ -d .git ]; then git submodule update --init --recursive; fi && \
    make -j$(nproc)

# --- [중요] libsurvive를 위한 환경 변수 통합 ---
# PYTHONPATH: 파이썬이 pysurvive 모듈을 찾을 수 있게 함
# LD_LIBRARY_PATH: 시스템이 libsurvive.so 라이브러리를 찾을 수 있게 함
# ENV LIBSURVIVE_HOME=/pika_ros2/src/pika_driver/pika_driver/libsurvive
# ENV PYTHONPATH="${PYTHONPATH}:${LIBSURVIVE_HOME}/bindings/python"
# ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${LIBSURVIVE_HOME}/bin"

# 5. libsurvive 설정 디렉토리 생성 (Warning 방지)
RUN mkdir -p /root/.config/libsurvive

# 6. ROS 2 워크스페이스 빌드
WORKDIR /pika_ros2
RUN rm -rf build/ install/ log/
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# 7. 쉘 환경 설정 (.bashrc 업데이트)
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /pika_ros2/install/setup.bash" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:${LIBSURVIVE_HOME}/bindings/python" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${LIBSURVIVE_HOME}/bin" >> ~/.bashrc

# entrypoint 설정
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]