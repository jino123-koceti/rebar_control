#!/bin/bash
# VSCode 설치 스크립트 for Ubuntu 22.04

set -e

echo "VSCode 설치를 시작합니다..."

# 방법 1: Microsoft 공식 repository를 통한 설치 (권장)
install_vscode_apt() {
    echo "Microsoft 공식 repository를 추가합니다..."
    
    # GPG 키 다운로드 및 설치
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
    sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
    rm -f packages.microsoft.gpg
    
    # Repository 추가
    sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
    
    # 패키지 목록 업데이트
    sudo apt update
    
    # VSCode 설치
    sudo apt install -y code
}

# 방법 2: Snap을 통한 설치 (대안)
install_vscode_snap() {
    echo "Snap을 통해 VSCode를 설치합니다..."
    sudo snap install --classic code
}

# 방법 3: 직접 .deb 패키지 다운로드 및 설치
install_vscode_direct() {
    echo "직접 .deb 패키지를 다운로드하여 설치합니다..."
    
    # ARM64 아키텍처 확인
    ARCH=$(dpkg --print-architecture)
    
    if [ "$ARCH" = "arm64" ]; then
        URL="https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-arm64"
    elif [ "$ARCH" = "amd64" ]; then
        URL="https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64"
    else
        echo "지원되지 않는 아키텍처: $ARCH"
        exit 1
    fi
    
    TEMP_FILE=$(mktemp /tmp/vscode_XXXXXX.deb)
    wget -O "$TEMP_FILE" "$URL"
    sudo dpkg -i "$TEMP_FILE"
    sudo apt-get install -f -y  # 의존성 해결
    rm -f "$TEMP_FILE"
}

# 설치 방법 선택
echo ""
echo "VSCode 설치 방법을 선택하세요:"
echo "1) Microsoft 공식 repository (권장)"
echo "2) Snap 패키지"
echo "3) 직접 .deb 패키지 다운로드"
echo ""
read -p "선택 (1-3, 기본값: 1): " choice
choice=${choice:-1}

case $choice in
    1)
        install_vscode_apt
        ;;
    2)
        install_vscode_snap
        ;;
    3)
        install_vscode_direct
        ;;
    *)
        echo "잘못된 선택입니다. 기본 방법(1)을 사용합니다."
        install_vscode_apt
        ;;
esac

# 설치 확인
if command -v code &> /dev/null; then
    echo ""
    echo "✓ VSCode가 성공적으로 설치되었습니다!"
    echo ""
    echo "설치된 버전:"
    code --version
    echo ""
    echo "VSCode를 실행하려면 다음 명령어를 사용하세요:"
    echo "  code"
    echo "  또는"
    echo "  code /home/koceti/ros2_ws"
else
    echo ""
    echo "✗ VSCode 설치에 실패했습니다. 수동으로 확인해주세요."
    exit 1
fi


