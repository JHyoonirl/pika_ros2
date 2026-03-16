#!/bin/bash

# --- 색상 설정 ---
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${GREEN}=== Pika Physical Address Scanner ===${NC}"
echo " [요령] 'ttyUSB0' 같은 이름 말고, '1-4.3.4' 처럼 숫자와 점으로 된 것을 복사하세요."
echo "------------------------------------------------"

# 1. 시리얼 장치(ttyUSB) 스캔
echo -e "${CYAN}[1] 시리얼 포트 (ttyUSB)${NC}"

# 장치가 있는지 확인
count=$(ls /dev/ttyUSB* 2>/dev/null | wc -l)
if [ "$count" == "0" ]; then
    echo "  (연결된 ttyUSB 장치가 없습니다)"
else
    for dev in /dev/ttyUSB*; do
        echo "  장치: $dev"
        echo "  [주소 후보]"
        # 상위 5개 KERNELS 중 'ttyUSB'가 포함되지 않은 줄만 보여줌
        udevadm info -a -n $dev | grep "KERNELS==" | grep -v "ttyUSB" | head -n 3
        echo "------------------------------------------------"
    done
fi

# 2. 어안 카메라(1bcf) 스캔
echo -e "${CYAN}[2] 어안 카메라 (Fisheye)${NC}"

# 장치가 있는지 확인
count_vid=$(ls /dev/video* 2>/dev/null | wc -l)
if [ "$count_vid" == "0" ]; then
    echo "  (연결된 비디오 장치가 없습니다)"
else
    for dev in /dev/video*; do
        # 1bcf 제조사인지 확인 (2>/dev/null로 에러 숨김)
        if udevadm info -a -n $dev 2>/dev/null | grep -q "idVendor.*1bcf"; then
            echo "  장치: $dev"
            echo "  [주소 후보]"
            # 상위 5개 KERNELS 중 'video'나 '1bcf'가 아닌, 숫자 패턴을 찾음
            udevadm info -a -n $dev | grep "KERNELS==" | grep -v "video" | head -n 3
            echo "------------------------------------------------"
        fi
    done
fi
echo "스캔 완료."