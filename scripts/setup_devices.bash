#!/bin/bash

# --- 색상 설정 ---
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Pika Fast Device Linker (No-Freeze Ver.) ===${NC}"
echo "물리 주소를 직접 추적하여 심볼릭 링크를 생성합니다."

# =========================================================
# 1. [설정] 물리 주소 입력 (find_addr.bash 결과값)
# =========================================================

# [Right Sensor]
R_SENSE_SERIAL_ADDR="1-3.4:1.0"
R_SENSE_VIDEO_ADDR="1-3.1"

# [Left Sensor]
L_SENSE_SERIAL_ADDR="1-1.4.1.4:1.0"
L_SENSE_VIDEO_ADDR="1-1.4.1.1"

# [Right Gripper]
R_GRIPPER_SERIAL_ADDR="1-1.3.1.4:1.0"
R_GRIPPER_VIDEO_ADDR="1-1.3.1.1"

# [Left Gripper]
L_GRIPPER_SERIAL_ADDR="1-5.2.1.4:1.0"
L_GRIPPER_VIDEO_ADDR="1-5.2.1.1"

# =========================================================
# 2. 핵심 함수: Sysfs 경로 직접 매칭 (udevadm 대체)
# =========================================================
function make_link() {
    local target_addr=$1    # 찾을 물리 주소 (예: 1-4.2.1)
    local type=$2           # tty 또는 video
    local link_name=$3      # 만들고 싶은 이름

    if [ -z "$target_addr" ]; then return; fi

    local found_dev=""

    if [ "$type" == "tty" ]; then
        for dev in /dev/ttyUSB*; do
            [ -e "$dev" ] || continue
            # /sys/class/tty/ttyUSBX/device의 부모 폴더명이 물리 주소임
            local sys_path=$(readlink -f /sys/class/tty/$(basename "$dev")/device)
            local current_addr=$(basename $(dirname "$sys_path"))
            
            if [ "$current_addr" == "$target_addr" ]; then
                found_dev=$dev
                break
            fi
        done
    elif [ "$type" == "video" ]; then
        for dev in /dev/video*; do
            [ -e "$dev" ] || continue
            local dev_name=$(basename "$dev")
            # 1bcf 제조사인지 먼저 가볍게 확인
            if [ -f "/sys/class/video4linux/$dev_name/device/../idVendor" ]; then
                local vendor=$(cat "/sys/class/video4linux/$dev_name/device/../idVendor")
                if [ "$vendor" == "1bcf" ]; then
                    local sys_path=$(readlink -f "/sys/class/video4linux/$dev_name/device")
                    local current_addr=$(basename $(dirname "$sys_path"))
                    
                    if [ "$current_addr" == "$target_addr" ]; then
                        found_dev=$dev
                        break
                    fi
                fi
            fi
        done
    fi

    # 링크 생성 로직
    if [ ! -z "$found_dev" ]; then
        echo -e "${CYAN}[연결]${NC} $link_name -> $found_dev"
        if [ -L "/dev/$link_name" ]; then rm "/dev/$link_name"; fi
        ln -sf "$found_dev" "/dev/$link_name"
        chmod 777 "$found_dev" 2>/dev/null
    else
        echo -e "${YELLOW}[실패]${NC} 주소($target_addr) 장치를 찾지 못했습니다."
    fi
}

# =========================================================
# 3. 설정 실행
# =========================================================

echo "------------------------------------------------"
make_link "$R_SENSE_SERIAL_ADDR" "tty"   "pika_sense_right_serial"
make_link "$R_SENSE_VIDEO_ADDR"  "video" "pika_sense_right_video"

make_link "$L_SENSE_SERIAL_ADDR" "tty"   "pika_sense_left_serial"
make_link "$L_SENSE_VIDEO_ADDR"  "video" "pika_sense_left_video"

make_link "$R_GRIPPER_SERIAL_ADDR" "tty"   "pika_gripper_right_serial"
make_link "$R_GRIPPER_VIDEO_ADDR"  "video" "pika_gripper_right_video"

make_link "$L_GRIPPER_SERIAL_ADDR" "tty"   "pika_gripper_left_serial"
make_link "$L_GRIPPER_VIDEO_ADDR"  "video" "pika_gripper_left_video"

echo "------------------------------------------------"
# echo -e "${GREEN}설정 완료! 현재 생성된 링크 목록:${NC}"
# ls -l /dev/pika* 2>/dev/null