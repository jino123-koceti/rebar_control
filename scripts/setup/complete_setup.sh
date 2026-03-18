#!/bin/bash
#===============================================================================
# 환경 구축 나머지 단계 완료 스크립트
#===============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_step() {
    echo -e "\n${GREEN}[STEP]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

#===============================================================================
# 1. PATH에 ~/.local/bin 추가
#===============================================================================
print_step "PATH에 ~/.local/bin 추가"

if ! grep -q "\$HOME/.local/bin" ~/.bashrc; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    export PATH="$HOME/.local/bin:$PATH"
    echo "PATH 업데이트 완료"
else
    echo "PATH는 이미 설정되어 있습니다"
fi

#===============================================================================
# 2. 시리얼 포트 권한 설정
#===============================================================================
print_step "시리얼 포트 권한 설정"
sudo usermod -aG dialout $USER

#===============================================================================
# 3. udev rules 설정
#===============================================================================
print_step "udev rules 설정"

# CAN 어댑터 udev rules
cat << 'EOF' | sudo tee /etc/udev/rules.d/50-usbcan.rules
# USB CAN 어댑터 규칙
SUBSYSTEM=="net", ACTION=="add", ATTR{type}=="280", NAME="can%n"
EOF

# Pololu SMC udev rules
cat << 'EOF' | sudo tee /etc/udev/rules.d/99-pololu.rules
# Pololu Simple Motor Controller
SUBSYSTEM=="usb", ATTRS{idVendor}=="1ffb", MODE="0666"
EOF

# 시리얼 포트 (그리퍼용) udev rules
cat << 'EOF' | sudo tee /etc/udev/rules.d/99-seengrip.rules
# Seengrip USB Serial
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", MODE="0666", GROUP="dialout"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

#===============================================================================
# 4. CAN 인터페이스 자동 설정 서비스
#===============================================================================
print_step "CAN 인터페이스 자동 설정 스크립트 생성"

cat << 'EOF' | sudo tee /usr/local/bin/setup_can.sh
#!/bin/bash
# CAN 인터페이스 설정 스크립트

# can2: 모터 제어 (1Mbps)
if ip link show can2 > /dev/null 2>&1; then
    sudo ip link set can2 down 2>/dev/null
    sudo ip link set can2 type can bitrate 1000000
    sudo ip link set can2 up
    echo "can2: 1Mbps 설정 완료"
fi

# can3: 리모콘 (250kbps)
if ip link show can3 > /dev/null 2>&1; then
    sudo ip link set can3 down 2>/dev/null
    sudo ip link set can3 type can bitrate 250000
    sudo ip link set can3 up
    echo "can3: 250kbps 설정 완료"
fi
EOF

sudo chmod +x /usr/local/bin/setup_can.sh

# systemd 서비스 생성
cat << 'EOF' | sudo tee /etc/systemd/system/can-setup.service
[Unit]
Description=CAN Interface Setup
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_can.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable can-setup.service

#===============================================================================
# 5. bashrc 설정
#===============================================================================
print_step "~/.bashrc ROS2 설정 추가"

# 중복 방지하여 추가
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    cat << 'EOF' >> ~/.bashrc

# ROS2 Humble 환경
source /opt/ros/humble/setup.bash

# ROS2 Workspace
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# ROS2 Domain ID (필요시 수정)
export ROS_DOMAIN_ID=0

# CAN 인터페이스 설정 alias
alias setup_can='sudo /usr/local/bin/setup_can.sh'

# 로그 디렉토리 생성
if [ ! -d /var/log/robot_control ]; then
    sudo mkdir -p /var/log/robot_control
    sudo chown $USER:$USER /var/log/robot_control
fi

# colcon 자동완성
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
EOF
fi

#===============================================================================
# 6. 로그 디렉토리 생성
#===============================================================================
print_step "로그 디렉토리 생성"
sudo mkdir -p /var/log/robot_control
sudo chown $USER:$USER /var/log/robot_control

#===============================================================================
# 완료
#===============================================================================
echo ""
echo "=========================================="
echo -e "${GREEN} 환경 구축 완료!${NC}"
echo "=========================================="
echo ""
echo "✅ Python 패키지 설치 완료"
echo "✅ CAN 도구 설치 완료"
echo "✅ udev rules 설정 완료"
echo "✅ CAN 자동 설정 서비스 등록 완료"
echo "✅ bashrc 설정 완료"
echo "✅ 로그 디렉토리 생성 완료"
echo ""
echo "다음 단계:"
echo "1. 터미널을 다시 열거나 'source ~/.bashrc' 실행"
echo "2. Workspace 빌드:"
echo "   cd ~/ros2_ws"
echo "   colcon build --symlink-install"
echo "   source install/setup.bash"
echo ""
echo "3. CAN 인터페이스 설정 (하드웨어 연결 후):"
echo "   setup_can"
echo ""











