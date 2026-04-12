# PIKA Driver Setup Guide

PIKA 로봇 시스템을 위한 ROS 2 드라이버 패키지입니다. 이 저장소는 카메라, 모터, Vive Tracker, RealSense를 하나의 설정 파일과 고정 장치 경로로 관리하도록 구성되어 있습니다.

## Overview

- `pika_info.json`에서 장치 시리얼과 네임스페이스를 관리합니다.
- launch 파일로 단일/복수 센서, 단일/복수 그리퍼 구성을 선택적으로 실행합니다.

## Prerequisites

- ROS 2 jazzy 환경
- Docker

## Setup

### 1. 저장소 클론

서브모듈을 포함해서 저장소를 내려받습니다.

```bash
git clone --recursive https://github.com/JHyoonirl/pika_ros2.git
cd pika_ros2
```

### 2. Docker 이미지 빌드

개발 및 실행 환경을 맞추기 위해 Docker 이미지를 빌드합니다.

```bash
docker build -t pika_ros2:latest -f docker/Dockerfile .
```

### 3. 컨테이너 실행

GPU와 USB 접근 권한이 포함된 실행 스크립트를 사용합니다.

```bash
chmod +x run_docker.sh
bash docker/run_docker.sh
```

### 4. USB 물리 주소 확인

장치가 어떤 포트에 연결되었는지 물리 주소를 확인합니다.

```bash
bash scripts/find_addr.bash
```

출력된 값은 `scripts/setup_devices.bash`의 주소 변수에 반영합니다. 예를 들어 `1-4.1.1` 같은 값이 표시되면 해당 장치의 주소로 기록합니다.

### 5. 고정 장치 링크 생성

물리 주소를 바탕으로 `/dev/pika_*` 형태의 심볼릭 링크를 생성합니다.

```bash
bash scripts/setup_devices.bash
```

### 6. RealSense 시리얼 번호 설정

`realsense-viewer`로 각 카메라의 시리얼 번호를 확인한 뒤 `src/pika_driver/info/pika_info.json`에 반영합니다.

```bash
realsense-viewer
```

설정해야 하는 항목은 `sense_left`, `sense_right`, `gripper_left`, `gripper_right` 같은 섹션입니다.

### 7. 패키지 빌드

설정 변경 후 ROS 2 워크스페이스를 빌드하고 환경을 다시 불러옵니다.

```bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Commands

### 센서 구성

| Scenario | Command | Notes |
| --- | --- | --- |
| 단일 센서 | `ros2 launch pika_driver pika_single_sense.launch.py side:=left` | `side`는 `left` 또는 `right` |
| 양쪽 센서 | `ros2 launch pika_driver pika_multi_sense.launch.py` | `left`, `right`를 함께 실행 |

### 그리퍼 구성

| Scenario | Command | Notes |
| --- | --- | --- |
| 단일 그리퍼 | `ros2 launch pika_driver pika_single_gripper.launch.py side:=left` | `side`는 `left` 또는 `right` |
| 양쪽 그리퍼 | `ros2 launch pika_driver pika_multi_gripper.launch.py` | `left`, `right`를 함께 실행 |

### 센서 + 그리퍼 구성

| Scenario | Command | Notes |
| --- | --- | --- |
| 단일 조합 | `ros2 launch pika_driver pika_sense_gripper.launch.py side:=left` | 센서와 그리퍼를 같은 side 기준으로 실행 |
| 양쪽 조합 | `ros2 launch pika_driver pika_multi_sense_gripper.launch.py` | left/right 조합을 함께 실행 |

## Key Files

- `src/pika_driver/info/pika_info.json`: 하드웨어 시리얼, 물리 주소, 네임스페이스 설정
- `scripts/find_addr.bash`: 현재 연결된 USB 장치의 물리 주소 확인
- `scripts/setup_devices.bash`: `/dev/pika_*` 고정 링크 생성
- `src/pika_driver/launch/`: 실행용 launch 파일 모음