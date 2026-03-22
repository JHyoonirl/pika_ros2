git clone https://github.com/your-name/pika_ros2.git
cd pika_ros2

python3 -m venv .venv
source .venv/bin/activate

pip install PyYAML
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

cd src/pika_driver/pika_driver
git clone https://github.com/collabora/libsurvive.git
cd libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  zlib1g-dev \
  libx11-dev \
  libusb-1.0-0-dev \
  freeglut3-dev \
  liblapacke-dev \
  libopenblas-dev

make -j"$(nproc)"

pip install .
