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
sudo chmod +x docker/run_docker.sh
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

## Vive Tracker 초기 사용

이 저장소의 Vive Tracker는 SteamVR/VRPN이 아니라 `libsurvive + pysurvive`를 통해 직접 추적합니다. 아래 명령은 `bash docker/run_docker.sh`로 실행한 컨테이너 안에서 수행합니다.

### 1. libsurvive 빌드

호스트 저장소를 `/pika_ros2`에 마운트하기 때문에 Docker 이미지에서 빌드된 `libsurvive`의 `bin/`이 가려질 수 있습니다. 컨테이너 안에서 한 번 빌드합니다.

```bash
cd /pika_ros2/src/pika_driver/pika_driver/libsurvive
make -j$(nproc)
ls -l bin/libsurvive.so*
```

`bin/`은 호스트 저장소에 마운트된 경로이므로 컨테이너를 삭제해도 빌드 결과가 남습니다.

컨테이너의 `entrypoint.sh`가 다음 환경변수를 설정하지만, 직접 실행한 셸이라면 명시적으로 설정합니다.

```bash
export LIBSURVIVE_HOME=/pika_ros2/src/pika_driver/pika_driver/libsurvive
export PYTHONPATH="$LIBSURVIVE_HOME/bindings/python:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$LIBSURVIVE_HOME/bin:${LD_LIBRARY_PATH:-}"
```

### 2. Tracker serial 확인

Tracker 전원을 켜고 Lighthouse가 보이는 위치에 둔 뒤 실행합니다.

```bash
cd /pika_ros2
python3 src/pika_driver/pika_driver/utils/tracker_serial_no_reader.py
```

다음과 같이 `LHR-...` serial이 출력되어야 합니다.

```text
name=... | serial=LHR-XXXXXXXX
```

`Detected LH` 또는 `Got OOTX packet`은 Lighthouse가 감지되었다는 뜻입니다. `WM0`만 보이고 `DEVICE SERIAL LIST`가 비어 있으면 실제 Tracker serial을 받은 것이 아니므로 Tracker 전원, 동글 페어링, Lighthouse 시야를 확인합니다.

필요하면 libsurvive 자체 실행 파일로도 확인할 수 있습니다.

```bash
cd /pika_ros2/src/pika_driver/pika_driver/libsurvive
./bin/survive-cli
```

### 3. Tracker 노드 실행

Tracker만 실행할 때는 `serial_numbers`에 위에서 확인한 값을 넣습니다.

```bash
ros2 run pika_driver vive_tracker_driver --ros-args \
  -p serial_numbers:="['<TRACKER_SERIAL>']" \
  -p tracker_ids:="['left']" \
  -p namespaces:="['pika_left_sense']" \
  -p publish_rate:=100.0 \
  -p frame_id:=world
```

양쪽 Tracker는 다음처럼 실행합니다.

```bash
ros2 run pika_driver vive_tracker_driver --ros-args \
  -p serial_numbers:="['<LEFT_SERIAL>','<RIGHT_SERIAL>']" \
  -p tracker_ids:="['left','right']" \
  -p namespaces:="['pika_left_sense','pika_right_sense']" \
  -p publish_rate:=100.0 \
  -p frame_id:=world
```

센서 구성 전체를 실행할 때는 `pika_info.json`의 Tracker serial을 사용하므로 다음 launch를 사용합니다.

```bash
ros2 launch pika_driver pika_single_sense.launch.py side:=left
ros2 launch pika_driver pika_multi_sense.launch.py
```

### 4. Pose topic 확인

```bash
ros2 topic echo /pika_left_sense/left/pose_raw
ros2 topic echo /pika_left_sense/left/pose
```

- `/pose_raw`: libsurvive가 계산한 Tracker 원본 pose
- `/pose`: 원본 pose에 PIKA의 회전 보정과 `(0.172, 0, -0.076)` m 오프셋을 적용한 pose
- 메시지 타입: `geometry_msgs/msg/PoseStamped`
- 기본 주기: 100 Hz

### 5. 좌표 원점

표준 HTC VIVE Tracker 3.0/2018의 물리적 Tracker 좌표계 원점은 1/4인치 장착 나사 중심축과 장착 링 상단면이 만나는 Datum B입니다. 이 원점은 첫 pose 수신 시 새로 설정되지 않고, Tracker device configuration이 libsurvive에 로드될 때 고정됩니다.

`libsurvive`의 `world` 원점은 프로그램 시작 시 Lighthouse 장면을 초기화·추정하면서 정해지는 전역 좌표계입니다. 따라서 로봇 base 원점과 자동으로 일치하지 않습니다.

실제 Tracker 원점을 사용하려면 `/pose_raw`를 사용해야 합니다. `/pose`는 Tracker 원점이 아니라 PIKA에서 보정한 작업점입니다.

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
