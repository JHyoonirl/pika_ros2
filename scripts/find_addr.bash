#!/bin/bash

# --- 색상 설정 ---
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[0;33m'
NC='\033[0m'

echo -e "${GREEN}=== Pika Final Physical Address Scanner (No-Freeze) ===${NC}"
echo "------------------------------------------------"

# 1. 시리얼 포트 (ttyUSB) 스캔
echo -e "${CYAN}[1] 시리얼 포트 (ttyUSB)${NC}"
for dev in /dev/ttyUSB*; do
    [ -e "$dev" ] || continue # 장치가 없으면 패스
    
    # 1단계: 실제 물리 경로 획득
    # 예: /sys/devices/.../usb3/3-2/3-2.2/3-2.2.1/3-2.2.1.4/3-2.2.1.4:1.0/ttyUSB0
    real_path=$(readlink -f /sys/class/tty/$(basename "$dev")/device)
    
    # 2단계: 인터페이스(:1.0) 폴더의 부모 폴더가 바로 '물리 주소'입니다.
    # dirname으로 /3-2.2.1.4:1.0 부분을 날리고, 그 윗 단계의 이름을 가져옵니다.
    addr=$(basename $(dirname "$real_path"))
    
    echo -e "  장치: ${YELLOW}$dev${NC}"
    echo -e "  주소: ${GREEN}$addr${NC}"
    echo "------------------------------------------------"
done

# 2. 어안 카메라 (Fisheye) 스캔
echo -e "${CYAN}[2] 어안 카메라 (Fisheye)${NC}"
for dev in /sys/class/video4linux/video*; do
    [ -e "$dev" ] || continue
    vid_name=$(basename "$dev")
    
    # 1bcf (어안 카메라 제조사) 제품인지 확인
    if [ -f "$dev/device/../idVendor" ] && [ "$(cat "$dev/device/../idVendor")" == "1bcf" ]; then
        real_path=$(readlink -f "$dev/device")
        
        # 비디오 장치는 경로 구조가 다를 수 있어 조건문 처리
        if [[ "$real_path" == *":"* ]]; then
            addr=$(basename $(dirname "$real_path"))
        else
            addr=$(basename "$real_path")
        fi
        
        echo -e "  장치: ${YELLOW}/dev/$vid_name${NC}"
        echo -e "  주소: ${GREEN}$addr${NC}"
        echo "------------------------------------------------"
    fi
done

echo -e "${GREEN}스캔 완료.${NC}"