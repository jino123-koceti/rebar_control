#!/bin/bash
#===============================================================================
# 자동 실행 서비스 설정 스크립트
#===============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_step() {
    echo -e "${GREEN}[STEP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

echo "=========================================="
echo " 자동 실행 서비스 설정"
echo "=========================================="

# 서비스 파일 복사
print_step "서비스 파일 설치"
sudo cp /home/koceti/ros2_ws/robot-control.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/robot-control.service

# systemd 재로드
print_step "systemd 재로드"
sudo systemctl daemon-reload

# 서비스 활성화
print_step "서비스 활성화"
sudo systemctl enable robot-control.service

print_success "자동 실행 서비스 설정 완료!"
echo ""
echo "서비스 관리 명령어:"
echo "  시작:   ${YELLOW}sudo systemctl start robot-control.service${NC}"
echo "  중지:   ${YELLOW}sudo systemctl stop robot-control.service${NC}"
echo "  상태:   ${YELLOW}sudo systemctl status robot-control.service${NC}"
echo "  로그:   ${YELLOW}sudo journalctl -u robot-control.service -f${NC}"
echo "  비활성화: ${YELLOW}sudo systemctl disable robot-control.service${NC}"
echo ""
echo "재부팅 후 자동으로 시작됩니다."
echo ""








