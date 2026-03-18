#!/bin/bash
#===============================================================================
# PEAK CAN USB 어댑터 peak_usb 커널 모듈 빌드 및 설치
# Jetson AGX Orin + JetPack 6.x (L4T R36.4.4)
# PEAK CAN USB (0c72:000c)는 peak_usb 드라이버가 필요함
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

# 기존 빌드 디렉토리 확인
if [ -d "$HOME/gs_usb_build" ]; then
    BUILD_DIR="$HOME/gs_usb_build"
    print_info "기존 빌드 디렉토리 사용: $BUILD_DIR"
else
    print_error "커널 소스가 없습니다. 먼저 setup_peak_can.sh를 실행하세요"
    exit 1
fi

#===============================================================================
# 2. 커널 소스 경로 확인
#===============================================================================
print_header "커널 소스 확인"

KERNEL_SRC="$BUILD_DIR/Linux_for_Tegra/source/kernel/kernel-jammy-src"
OUT_PATH="$BUILD_DIR/kernel_out"

if [ ! -d "$KERNEL_SRC" ]; then
    print_error "커널 소스를 찾을 수 없습니다: $KERNEL_SRC"
    exit 1
fi

print_info "커널 소스: $KERNEL_SRC"
print_info "출력 경로: $OUT_PATH"

#===============================================================================
# 3. peak_usb 모듈 빌드
#===============================================================================
print_header "peak_usb 모듈 빌드"

cd "$KERNEL_SRC" || exit 1

# .config에 peak_usb 추가
if ! grep -q "CONFIG_CAN_PEAK_USB=m" "$OUT_PATH/.config"; then
    print_step "peak_usb 모듈 설정 추가"
    echo "CONFIG_CAN_PEAK_USB=m" >> "$OUT_PATH/.config"
    print_success "모듈 설정 추가 완료"
else
    print_info "peak_usb 모듈이 이미 설정되어 있습니다"
fi

print_step "peak_usb 모듈 컴파일 중..."
make O="$OUT_PATH" M=drivers/net/can/usb/ modules 2>&1 | tee "$OUT_PATH/peak_usb_build.log" || {
    print_error "모듈 컴파일 실패"
    exit 1
}

print_success "컴파일 완료!"

#===============================================================================
# 4. 모듈 설치
#===============================================================================
print_header "모듈 설치"

# peak_usb 모듈은 peak_usb 디렉토리 안에 생성됨
MODULE_SOURCE="$OUT_PATH/drivers/net/can/usb/peak_usb/peak_usb.ko"
MODULE_INSTALL_PATH="/lib/modules/${KERNEL_VERSION}/kernel/net/can/usb"

# 빌드가 안 되어 있으면 다시 빌드
if [ ! -f "$MODULE_SOURCE" ]; then
    print_step "peak_usb 모듈 빌드 중..."
    cd "$KERNEL_SRC" || exit 1
    make O="$OUT_PATH" modules_prepare > /dev/null 2>&1
    make O="$OUT_PATH" M=drivers/net/can/usb modules 2>&1 | grep -E "(peak|CC|LD)" | tail -5 || true
fi

if [ ! -f "$MODULE_SOURCE" ]; then
    print_error "컴파일된 모듈을 찾을 수 없습니다: $MODULE_SOURCE"
    print_info "실제 생성된 파일:"
    find "$OUT_PATH/drivers/net/can/usb" -name "*peak*.ko" 2>/dev/null || true
    exit 1
fi

print_info "모듈 파일 발견: $MODULE_SOURCE"

print_step "모듈 설치"
sudo mkdir -p "$MODULE_INSTALL_PATH"
sudo cp -v "$MODULE_SOURCE" "$MODULE_INSTALL_PATH/"

print_step "모듈 의존성 업데이트"
sudo depmod -a

print_step "자동 로드 설정"
if ! grep -q "^peak_usb" /etc/modules; then
    echo "peak_usb" | sudo tee -a /etc/modules
    print_success "자동 로드 설정 완료"
else
    print_info "이미 자동 로드가 설정되어 있습니다"
fi

#===============================================================================
# 5. udev rules 설정
#===============================================================================
print_header "udev rules 설정"

print_step "PEAK CAN USB udev rules 업데이트"
print_info "can2: PEAK CAN USB -> 모터 제어 (1Mbps)"
print_info "can3: Iron-MD 리모콘 수신기 (250kbps)"

cat << 'EOF' | sudo tee /etc/udev/rules.d/99-peak-can-usb.rules
# PEAK CAN USB 어댑터 규칙
# can2: PEAK CAN USB -> 모터 제어 (1Mbps)
# peak_usb 드라이버로 생성된 첫 번째 인터페이스를 can2로 매핑
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="peak_usb", ATTR{type}=="280", NAME="can2"

# can3: Iron-MD 리모콘 수신기 (250kbps) 또는 두 번째 PEAK CAN USB
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="0c72", ATTRS{idProduct}=="000c", ATTR{type}=="280", NAME!="can2", NAME="can3"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

#===============================================================================
# 6. 모듈 로드 테스트
#===============================================================================
print_header "모듈 로드 테스트"

print_step "peak_usb 모듈 로드"
sudo modprobe peak_usb || {
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
print_success "peak_usb 커널 모듈 설치가 완료되었습니다!"
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

