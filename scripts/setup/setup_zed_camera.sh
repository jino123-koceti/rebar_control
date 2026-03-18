#!/bin/bash
#===============================================================================
# ZED 카메라 SDK 및 ROS2 래퍼 설치 스크립트
# Jetson AGX Orin + JetPack 6.x
#===============================================================================

set -e

echo "=========================================="
echo " ZED 카메라 환경 구축 스크립트"
echo "=========================================="

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
# JetPack 버전 확인
#===============================================================================
print_step "JetPack 버전 확인"

if [ -f /etc/nv_tegra_release ]; then
    cat /etc/nv_tegra_release
    JETPACK_VERSION=$(cat /etc/nv_tegra_release | head -1 | grep -oP 'R\d+')
    echo "감지된 JetPack: $JETPACK_VERSION"
else
    print_warning "Jetson 환경을 감지할 수 없습니다"
fi

#===============================================================================
# ZED SDK 다운로드 및 설치
#===============================================================================
print_step "ZED SDK 설치"

# ZED SDK 4.2 for JetPack 6.0 (L4T 36.x)
# 최신 버전은 https://www.stereolabs.com/developers/release 에서 확인
ZED_SDK_URL="https://download.stereolabs.com/zedsdk/4.2/l4t36.4/jetsons"

cd /tmp

if [ ! -f ZED_SDK_Ubuntu22_cuda12.6_v4.2.3.run ]; then
    print_step "ZED SDK 다운로드 중..."
    wget -O ZED_SDK_installer.run "$ZED_SDK_URL"
else
    print_warning "이미 다운로드된 SDK를 사용합니다"
    mv ZED_SDK_Ubuntu22_cuda12.6_v4.2.3.run ZED_SDK_installer.run 2>/dev/null || true
fi

print_step "ZED SDK 설치 중..."
chmod +x ZED_SDK_installer.run
./ZED_SDK_installer.run -- silent skip_cuda skip_python

#===============================================================================
# CUDA 환경 변수 설정
#===============================================================================
print_step "CUDA 환경 변수 설정"

if ! grep -q "CUDA_HOME" ~/.bashrc; then
    cat << 'EOF' >> ~/.bashrc

# CUDA 환경 변수
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
EOF
fi

# 현재 세션에도 적용
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

#===============================================================================
# ZED ROS2 래퍼 의존성 설치
#===============================================================================
print_step "ZED ROS2 래퍼 의존성 설치"

source /opt/ros/humble/setup.bash

sudo apt install -y \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-pointcloud-to-laserscan

#===============================================================================
# zed-ros2-wrapper 빌드
#===============================================================================
print_step "zed-ros2-wrapper 빌드"

cd ~/ros2_ws

# 먼저 zed-ros2-wrapper만 빌드
colcon build --packages-select zed_components zed_wrapper zed_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release

#===============================================================================
# 완료
#===============================================================================
echo ""
echo "=========================================="
echo -e "${GREEN} ZED 카메라 환경 구축 완료!${NC}"
echo "=========================================="
echo ""
echo "ZED 카메라 테스트:"
echo "  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2"
echo ""
echo "ZED 노드 확인:"
echo "  ros2 topic list | grep zed"
echo ""











