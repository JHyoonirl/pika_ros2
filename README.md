# pika_ros2

Surgical Assistant Robot using AI and Modular-Humanoid (SARAM-H) 프로젝트를 위한 ROS 2 기반 Pika 드라이버 및 데이터 수집 패키지입니다.

---

## 🚀 Installation Guide

이 가이드는 Ubuntu 환경에서 가상 환경(venv)을 사용하여 의존성을 관리하고, `libsurvive`를 통한 하드웨어 연결 설정을 포함합니다.

### 1. Repository Clone & Virtual Environment
레포지토리를 클론하고 Python 가상 환경을 생성하여 의존성 충돌을 방지합니다.

```bash
# 레포지토리 클론
git clone [https://github.com/JHyoonirl/pika_ros2.git](https://github.com/JHyoonirl/pika_ros2.git)
cd pika_ros2

# 가상 환경 생성 및 활성화
python3 -m venv .venv
source .venv/bin/activate

# 기본 빌드 도구 업그레이드 및 필수 패키지 설치
pip install --upgrade pip setuptools wheel
pip install PyYAML



pip install -r requirements.txt


cd src/pika_driver/pika_driver
git clone https://github.com/collabora/libsurvive.git
cd libsurvive

# Vive/Pika 하드웨어를 위한 udev 규칙 복사
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 시스템 패키지 업데이트 및 설치
sudo apt update
sudo apt install -y build-essential cmake zlib1g-dev libx11-dev \
libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev

# 멀티코어를 활용한 빌드
make -j"$(nproc)"

# pika_driver를 파이썬 패키지로 설치
pip install .
