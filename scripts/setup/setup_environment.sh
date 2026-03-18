#!/bin/bash
#===============================================================================
# 철근 결속 로봇 ROS2 개발 환경 구축 스크립트
# Jetson AGX Orin + JetPack 6.x + Ubuntu 22.04 + ROS2 Humble
#===============================================================================

set -e

echo "=========================================="
echo " 철근 결속 로봇 ROS2 환경 구축 스크립트"
echo "=========================================="

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_step() {
    echo -e "\n${GREEN}[STEP]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

#===============================================================================
# 1. 시스템 업데이트
#===============================================================================
print_step "시스템 패키지 업데이트"
sudo apt update && sudo apt upgrade -y

#===============================================================================
# 2. ROS2 Humble 설치
#===============================================================================
print_step "ROS2 Humble 설치 준비"

# locale 설정
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 필수 도구 설치
sudo apt install -y software-properties-common curl gnupg lsb-release

# ROS2 GPG 키 추가
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 저장소 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

print_step "ROS2 Humble Desktop 설치 (약 10-20분 소요)"
sudo apt install -y ros-humble-desktop

# 개발 도구 설치
print_step "ROS2 개발 도구 설치"
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-dev-tools

# rosdep 초기화
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    print_step "rosdep 초기화"
    sudo rosdep init || print_warning "rosdep이 이미 초기화되어 있습니다"
fi
rosdep update

#===============================================================================
# 3. ROS2 추가 패키지 설치
#===============================================================================
print_step "ROS2 추가 패키지 설치"
sudo apt install -y \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-message-filters \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2

#===============================================================================
# 4. Python 패키지 설치
#===============================================================================
print_step "Python 패키지 설치"

# pip3 설치 (없는 경우)
if ! command -v pip3 &> /dev/null; then
    print_step "pip3 설치 중..."
    sudo apt install -y python3-pip
fi

# pip 업그레이드
pip3 install --upgrade pip

# 필수 Python 패키지
pip3 install \
    python-can \
    pymodbus \
    pyserial \
    cantools \
    numpy \
    opencv-python

# PyQt5 (GUI용, 필요시)
sudo apt install -y python3-pyqt5

#===============================================================================
# 5. CAN 인터페이스 도구 설치
#===============================================================================
print_step "CAN 인터페이스 도구 설치"
sudo apt install -y \
    can-utils \
    iproute2

#===============================================================================
# 6. 시리얼 포트 권한 설정
#===============================================================================
print_step "시리얼 포트 권한 설정"
sudo usermod -aG dialout $USER

#===============================================================================
# 7. udev rules 설정
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
# 8. CAN 인터페이스 자동 설정 서비스
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
# 9. bashrc 설정
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
# 10. 로그 디렉토리 생성
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
echo "다음 단계:"
echo "1. 터미널을 다시 열거나 'source ~/.bashrc' 실행"
echo "2. CAN 인터페이스 설정: sudo /usr/local/bin/setup_can.sh"
echo "3. Workspace 빌드:"
echo "   cd ~/ros2_ws"
echo "   colcon build --symlink-install"
echo ""
echo "ZED 카메라를 사용하시면 별도로 ZED SDK를 설치해야 합니다:"
echo "   ./setup_zed_camera.sh"
echo ""
print_warning "시스템 재부팅을 권장합니다: sudo reboot"

