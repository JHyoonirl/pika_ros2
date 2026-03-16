#!/bin/bash

# --- 색상 설정 ---
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Pika Device Linker ===${NC}"
echo "물리 주소를 기반으로 장치 이름표(Symlink)를 생성합니다."

# =========================================================
# 1. [설정] 물리 주소 입력 (find_real_addr.bash 결과값)
# =========================================================

# [Right Sensor] - (확인된 주소)
R_SENSE_SERIAL_ADDR="5-1.2.4"
R_SENSE_VIDEO_ADDR="5-1.2.1"

# [Left Sensor] - (필요시 입력)
L_SENSE_SERIAL_ADDR="1-4.1.1.4"
L_SENSE_VIDEO_ADDR="1-4.1.1.1"

# [Right Gripper]
R_GRIPPER_SERIAL_ADDR="1-5.2.1.4"
R_GRIPPER_VIDEO_ADDR="1-5.2.1.1"

# [Left Gripper]
L_GRIPPER_SERIAL_ADDR="5-1.1.4"
L_GRIPPER_VIDEO_ADDR="5-1.1.1"

# =========================================================
# 2. 핵심 함수: 주소로 장치 찾아서 이름 붙이기
# =========================================================
function make_link() {
    local target_addr=$1    # 찾을 물리 주소
    local type=$2           # tty 또는 video
    local link_name=$3      # 만들고 싶은 이름 (예: pika_sensor_right_serial)

    # 주소가 비어있으면 스킵
    if [ -z "$target_addr" ]; then return; fi

    local found_dev=""

    if [ "$type" == "tty" ]; then
        # ttyUSB 검색
        for dev in /dev/ttyUSB*; do
            if [ -e "$dev" ]; then
                if udevadm info -a -n $dev 2>/dev/null | grep -q "$target_addr"; then
                    found_dev=$dev
                    break
                fi
            fi
        done
    elif [ "$type" == "video" ]; then
        # Video 검색
        for dev in /dev/video*; do
            if [ -e "$dev" ]; then
                # 1bcf 제조사이면서 주소가 맞는지 확인
                if udevadm info -a -n $dev 2>/dev/null | grep -q "idVendor.*1bcf"; then
                    if udevadm info -a -n $dev 2>/dev/null | grep -q "$target_addr"; then
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
        
        # 기존 링크가 있으면 삭제
        if [ -L "/dev/$link_name" ]; then rm "/dev/$link_name"; fi
        
        # 심볼릭 링크 생성 (바로가기 만들기)
        ln -sf "$found_dev" "/dev/$link_name"
        
        # 권한 777 부여 (원본 장치에)
        chmod 777 "$found_dev"
    else
        echo -e "${YELLOW}[실패]${NC} 주소($target_addr)를 가진 장치를 찾을 수 없습니다."
    fi
}

# =========================================================
# 3. 설정 실행
# =========================================================

echo "------------------------------------------------" >&2
# Right Sensor
make_link "$R_SENSE_SERIAL_ADDR" "tty"   "pika_sense_right_serial"
make_link "$R_SENSE_VIDEO_ADDR"  "video" "pika_sense_right_video"

# Left Sensor
make_link "$L_SENSE_SERIAL_ADDR" "tty"   "pika_sense_left_serial"
make_link "$L_SENSE_VIDEO_ADDR"  "video" "pika_sense_left_video"

# Right Gripper
make_link "$R_GRIPPER_SERIAL_ADDR" "tty"   "pika_gripper_right_serial"
make_link "$R_GRIPPER_VIDEO_ADDR"  "video" "pika_gripper_right_video"

# Left Gripper
make_link "$L_GRIPPER_SERIAL_ADDR" "tty"   "pika_gripper_left_serial"
make_link "$L_GRIPPER_VIDEO_ADDR"  "video" "pika_gripper_left_video"

echo "------------------------------------------------" >&2
echo -e "${GREEN}설정 완료! 현재 생성된 링크 목록:${NC}" >&2
ls -l /dev/pika* 2>/dev/null >&2

echo "" >&2
echo -e "${GREEN}환경변수가 출력되었습니다. 다음 방법으로 적용하세요:${NC}" >&2
echo -e "${CYAN}  eval \$(sudo ./setup_devices.bash)${NC}" >&2
echo "" >&2
echo "  L_SENSE_DEPTH_SN_R  = $L_SENSE_DEPTH_SN_R"
echo "  R_SENSE_DEPTH_SN_R  = $R_SENSE_DEPTH_SN_R"
echo "  L_GRIPPER_DEPTH_SN_R = $L_GRIPPER_DEPTH_SN_R"
echo "  R_GRIPPER_DEPTH_SN_R = $R_GRIPPER_DEPTH_SN_R"
echo ""
echo -e "${CYAN}이제 start_irl_sensor_gripper.bash를 실행할 수 있습니다.${NC}"