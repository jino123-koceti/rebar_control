#!/bin/bash
#===============================================================================
# 자동 로그인 설정 스크립트 (부팅 시 패스워드 입력 없이 자동 로그인)
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

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

USERNAME="koceti"

echo "=========================================="
echo " 자동 로그인 설정 (부팅 시 패스워드 없이)"
echo "=========================================="
echo ""

# 1. GDM3 자동 로그인 설정
print_step "GDM3 자동 로그인 설정"
if [ -f /etc/gdm3/custom.conf ]; then
    # 백업
    sudo cp /etc/gdm3/custom.conf /etc/gdm3/custom.conf.backup.$(date +%Y%m%d_%H%M%S)
    
    # 자동 로그인 활성화
    sudo sed -i 's/#  AutomaticLoginEnable = true/AutomaticLoginEnable = true/' /etc/gdm3/custom.conf
    sudo sed -i 's/#  AutomaticLogin = user1/AutomaticLogin = koceti/' /etc/gdm3/custom.conf
    
    # 자동 로그인 설정이 없으면 추가
    if ! grep -q "AutomaticLoginEnable" /etc/gdm3/custom.conf; then
        sudo sed -i '/\[daemon\]/a AutomaticLoginEnable = true\nAutomaticLogin = koceti' /etc/gdm3/custom.conf
    fi
    
    print_success "GDM3 자동 로그인 설정 완료"
else
    print_warning "GDM3 설정 파일을 찾을 수 없습니다"
fi

# 2. TTY 자동 로그인 설정 (콘솔)
print_step "TTY 자동 로그인 설정"
TTY_OVERRIDE_DIR="/etc/systemd/system/getty@tty1.service.d"
sudo mkdir -p $TTY_OVERRIDE_DIR

sudo tee $TTY_OVERRIDE_DIR/override.conf > /dev/null <<EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $USERNAME --noclear %I \$TERM
EOF

print_success "TTY 자동 로그인 설정 완료"

# 3. sudo 패스워드 없이 실행 가능하도록 설정 (제어 스크립트용)
print_step "sudo NOPASSWD 설정 (제어 스크립트용)"
SUDOERS_ENTRY="$USERNAME ALL=(ALL) NOPASSWD: /home/$USERNAME/ros2_ws/integrated_control_debug.sh, /home/$USERNAME/ros2_ws/integrated_control.sh, /home/$USERNAME/ros2_ws/setup_can.sh, /sbin/ip, /sbin/ifconfig"

if ! sudo grep -q "$USERNAME ALL=(ALL) NOPASSWD:" /etc/sudoers.d/robot-control 2>/dev/null; then
    echo "$SUDOERS_ENTRY" | sudo tee /etc/sudoers.d/robot-control > /dev/null
    sudo chmod 440 /etc/sudoers.d/robot-control
    print_success "sudo NOPASSWD 설정 완료"
else
    print_warning "sudo NOPASSWD 설정이 이미 존재합니다"
fi

# 4. systemd 재로드
print_step "systemd 재로드"
sudo systemctl daemon-reload

print_success "자동 로그인 설정 완료!"
echo ""
echo "설정 내용:"
echo "  - GDM3 자동 로그인: 활성화 (사용자: $USERNAME)"
echo "  - TTY 자동 로그인: 활성화 (사용자: $USERNAME)"
echo "  - sudo NOPASSWD: 제어 스크립트용 설정"
echo ""
echo "⚠️  재부팅 후 자동 로그인이 적용됩니다."
echo "   재부팅: sudo reboot"
echo ""
echo "자동 로그인 비활성화 방법:"
echo "  1. GDM3: sudo nano /etc/gdm3/custom.conf"
echo "     AutomaticLoginEnable = true → # AutomaticLoginEnable = true"
echo "  2. TTY: sudo rm -rf $TTY_OVERRIDE_DIR"
echo "  3. sudo: sudo rm /etc/sudoers.d/robot-control"
echo ""





