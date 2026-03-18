#!/bin/bash
#===============================================================================
# PEAK CAN USB 어댑터 gs_usb 커널 모듈 빌드 및 설치
# Jetson AGX Orin + JetPack 6.x (L4T R36.4.4)
# Based on: https://github.com/lucianovk/jetson-gs_usb-kernel-builder
#===============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "\n${YELLOW}===== $1 =====${NC}"
}

print_step() {
    echo -e "${GREEN}[STEP]${NC} $1"
}

print_info() {
    echo -e "[INFO] $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

#===============================================================================
# 1. 시스템 확인
#===============================================================================
print_header "시스템 확인"

KERNEL_VERSION=$(uname -r)
ARCHITECTURE=$(uname -m)

print_info "커널 버전: $KERNEL_VERSION"
print_info "아키텍처: $ARCHITECTURE"

if [ "$ARCHITECTURE" != "aarch64" ]; then
    print_error "이 스크립트는 Jetson (ARM64)에서만 실행 가능합니다"
    exit 1
fi

# PEAK CAN USB 확인
print_step "PEAK CAN USB 어댑터 확인"
if lsusb | grep -q "0c72:000c"; then
    PEAK_COUNT=$(lsusb | grep -c "0c72:000c")
    print_success "PEAK CAN USB 어댑터 $PEAK_COUNT개 감지됨"
else
    print_error "PEAK CAN USB 어댑터를 찾을 수 없습니다"
    exit 1
fi

#===============================================================================
# 2. 의존성 설치
#===============================================================================
print_header "의존성 설치"

print_step "패키지 업데이트"
sudo apt-get update -qq

print_step "필수 패키지 설치"
sudo apt-get install -y \
    build-essential \
    bc \
    libssl-dev \
    flex \
    bison \
    wget \
    git \
    pv \
    kmod \
    ca-certificates \
    libelf-dev

# linux-headers는 선택사항 (커널 소스에서 빌드하므로)
print_step "linux-headers 확인 (선택사항)"
if sudo apt-get install -y linux-headers-${KERNEL_VERSION} 2>/dev/null; then
    print_success "linux-headers 설치 완료"
else
    print_warning "linux-headers 패키지를 찾을 수 없습니다. 커널 소스에서 빌드합니다."
fi

#===============================================================================
# 3. 커널 소스 다운로드 및 설정
#===============================================================================
print_header "커널 소스 준비"

BUILD_DIR="$HOME/gs_usb_build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# 커널 설정 파일 생성
if [ ! -f "config" ]; then
    print_step "커널 설정 파일 생성"
    if [ -f /proc/config.gz ]; then
        zcat /proc/config.gz > config
        print_success "커널 설정 파일 생성 완료"
    else
        print_error "/proc/config.gz를 찾을 수 없습니다"
        exit 1
    fi
fi

# L4T 버전 확인 (R36.4.4)
L4T_VERSION="36.4.4"
print_info "L4T 버전: R$L4T_VERSION"

#===============================================================================
# 4. 커널 소스 다운로드
#===============================================================================
print_header "커널 소스 다운로드"

if [ ! -f "public_sources.tbz2" ]; then
    print_step "NVIDIA L4T 소스 다운로드 중 (~1.2GB, 시간이 걸릴 수 있습니다)..."
    # R36.4.4 소스 URL (버전에 맞게 조정 필요)
    wget --show-progress https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.4/sources/public_sources.tbz2 || {
        print_warning "R36.4.4 소스를 찾을 수 없습니다. R36.4.3 소스를 시도합니다..."
        wget --show-progress https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/sources/public_sources.tbz2
    }
else
    print_info "소스 파일이 이미 존재합니다"
fi

#===============================================================================
# 5. 소스 압축 해제
#===============================================================================
print_header "소스 압축 해제"

if [ ! -d "Linux_for_Tegra/source" ]; then
    print_step "압축 해제 중..."
    pv public_sources.tbz2 | tar xjf - || tar xjf public_sources.tbz2
    print_success "압축 해제 완료"
else
    print_info "소스가 이미 압축 해제되어 있습니다"
fi

cd Linux_for_Tegra/source/ || exit 1

# 커널 소스 압축 해제
if [ ! -d "kernel/kernel-jammy-src" ]; then
    print_step "커널 소스 압축 해제 중..."
    tar xf kernel_src.tbz2
    print_success "커널 소스 압축 해제 완료"
fi

#===============================================================================
# 6. 빌드 설정
#===============================================================================
print_header "빌드 설정"

cd kernel/kernel-jammy-src || exit 1

OUT_PATH="$BUILD_DIR/kernel_out"
mkdir -p "$OUT_PATH"

print_step "빌드 설정 복사"
cp "$BUILD_DIR/config" "$OUT_PATH/.config"

# gs_usb 모듈 설정 추가
if ! grep -q "CONFIG_CAN_GS_USB=m" "$OUT_PATH/.config"; then
    print_step "gs_usb 모듈 설정 추가"
    echo "CONFIG_CAN_GS_USB=m" >> "$OUT_PATH/.config"
    print_success "모듈 설정 추가 완료"
fi

#===============================================================================
# 7. 모듈 빌드
#===============================================================================
print_header "모듈 빌드"

print_step "빌드 환경 준비"
make O="$OUT_PATH" modules_prepare 2>&1 | tee "$OUT_PATH/prepare.log" || {
    print_error "빌드 환경 준비 실패"
    exit 1
}

print_step "gs_usb 모듈 컴파일 중..."
make O="$OUT_PATH" M=drivers/net/can/usb/ modules 2>&1 | tee "$OUT_PATH/build.log" || {
    print_error "모듈 컴파일 실패"
    exit 1
}

print_success "컴파일 완료!"

#===============================================================================
# 8. 모듈 설치
#===============================================================================
print_header "모듈 설치"

MODULE_SOURCE="$OUT_PATH/drivers/net/can/usb/gs_usb.ko"
MODULE_INSTALL_PATH="/lib/modules/${KERNEL_VERSION}/kernel/net/can/usb"

if [ ! -f "$MODULE_SOURCE" ]; then
    print_error "컴파일된 모듈을 찾을 수 없습니다: $MODULE_SOURCE"
    exit 1
fi

print_step "모듈 설치"
sudo mkdir -p "$MODULE_INSTALL_PATH"
sudo cp -v "$MODULE_SOURCE" "$MODULE_INSTALL_PATH/"

print_step "모듈 의존성 업데이트"
sudo depmod -a

print_step "자동 로드 설정"
if ! grep -q "^gs_usb" /etc/modules; then
    echo "gs_usb" | sudo tee -a /etc/modules
    print_success "자동 로드 설정 완료"
else
    print_info "이미 자동 로드가 설정되어 있습니다"
fi

#===============================================================================
# 9. udev rules 설정 (can2, can3 매핑)
#===============================================================================
print_header "udev rules 설정"

print_step "PEAK CAN USB udev rules 생성"
print_info "can2: PEAK CAN USB -> 모터 제어 (1Mbps)"
print_info "can3: Iron-MD 리모콘 수신기 (250kbps)"

cat << 'EOF' | sudo tee /etc/udev/rules.d/99-peak-can-usb.rules
# PEAK CAN USB 어댑터 규칙
# can2: PEAK CAN USB -> 모터 제어 (1Mbps)
# 첫 번째 PEAK CAN USB 어댑터를 can2로 매핑
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="0c72", ATTRS{idProduct}=="000c", ATTR{type}=="280", KERNELS=="*:1.1", NAME="can2"

# can3: Iron-MD 리모콘 수신기 (250kbps)
# 두 번째 PEAK CAN USB 어댑터를 can3로 매핑 (또는 Iron-MD용)
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="0c72", ATTRS{idProduct}=="000c", ATTR{type}=="280", KERNELS=="*:1.2", NAME="can3"
EOF

print_warning "참고: Iron-MD 리모콘이 별도 CAN 인터페이스를 사용하는 경우, can3는 수동으로 설정해야 할 수 있습니다."

sudo udevadm control --reload-rules
sudo udevadm trigger

#===============================================================================
# 10. 모듈 로드 테스트
#===============================================================================
print_header "모듈 로드 테스트"

print_step "gs_usb 모듈 로드"
sudo modprobe gs_usb || {
    print_warning "모듈 로드 실패. 재부팅 후 자동으로 로드됩니다"
}

sleep 2

# CAN 인터페이스 확인
print_step "CAN 인터페이스 확인"
if ip link show | grep -q "can"; then
    ip link show | grep "can"
    print_success "CAN 인터페이스가 생성되었습니다"
else
    print_warning "CAN 인터페이스가 아직 생성되지 않았습니다. 재부팅이 필요할 수 있습니다"
fi

#===============================================================================
# 완료
#===============================================================================
print_header "설치 완료"

echo ""
print_success "gs_usb 커널 모듈 설치가 완료되었습니다!"
echo ""
echo "다음 단계:"
echo "1. 시스템 재부팅: ${YELLOW}sudo reboot${NC}"
echo "2. 재부팅 후 CAN 인터페이스 확인:"
echo "   ${YELLOW}ip link show | grep can${NC}"
echo "3. CAN 인터페이스 설정:"
echo "   ${YELLOW}sudo ip link set can2 type can bitrate 1000000${NC}  # 모터 제어"
echo "   ${YELLOW}sudo ip link set can3 type can bitrate 250000${NC}   # Iron-MD 리모콘"
echo "   ${YELLOW}sudo ip link set can2 up${NC}"
echo "   ${YELLOW}sudo ip link set can3 up${NC}"
echo ""
echo "또는 기존 setup_can 스크립트 사용:"
echo "   ${YELLOW}setup_can${NC}"
echo ""
echo "설정 요약:"
echo "  - can2: PEAK CAN USB -> 모터 제어 (1Mbps)"
echo "  - can3: Iron-MD 리모콘 수신기 (250kbps)"
echo ""

