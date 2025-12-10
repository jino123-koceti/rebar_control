# ZED Link Quad 카메라 인식 실패 - 핵심 문제 분석

## 진단 요약

**상태**: ❌ 카메라 인식 실패
**원인**: **ZED Link 드라이버 버전과 L4T 버전 불일치**

---

## 시스템 정보

### 현재 환경
- **L4T 버전**: R36.4.4 (JetPack 6.2.1)
- **커널**: 5.15.148-tegra (OOT variant)
- **ZED SDK**: 4.2.5 (Build 97569)
- **설치된 드라이버**: `stereolabs-zedlink-quad 1.3.2-SL-MAX96712-all-L4T36.4.0`

### ⚠️ 핵심 문제
**드라이버가 L4T 36.4.0용으로 빌드되었으나, 시스템은 L4T 36.4.4를 사용 중입니다!**

---

## ZED_Diagnostic 오류 메시지

```
Errors occurred
A detailed report can be saved to be shared with support if needed.

Camera not detected
Make sure the camera is plugged in or try another USB 3.0 port.

GMSL driver incompatible with your L4T version.
If you updated L4T, you should update the GMSL driver too.
```

---

## 하드웨어 및 드라이버 상태

### ✅ 정상 작동 중
1. **하드웨어 감지**: 4개의 ZED 카메라 I2C 감지됨
   ```
   13-0028: zedx 1 (ZED X mini)
   13-0020: zedx 0 (ZED X mini)
   13-0038: zedx 3 (ZED X)
   13-0030: zedx 2 (ZED X)
   ```

2. **커널 모듈 로드됨**:
   ```bash
   $ lsmod | grep sl_
   sl_zedxone_uhd         36864  0
   sl_zedx                28672  1
   sl_zedxpro             20480  0
   sl_max9295             16384  3 sl_zedxpro,sl_zedxone_uhd,sl_zedx
   sl_max96712            24576  4 sl_zedxpro,sl_zedxone_uhd,sl_max9295,sl_zedx
   ```

3. **ZED X Daemon 실행 중**:
   ```bash
   $ systemctl status zed_x_daemon
   ● zed_x_daemon.service - ZED-X Daemon service
        Active: active (running)
   ```

4. **V4L2 디바이스 생성됨**:
   ```
   /dev/video0-7 존재
   ```

### ❌ 문제 발생
- **ZED_Diagnostic**: "GMSL driver incompatible with your L4T version"
- **ZED SDK**: "CAMERA NOT DETECTED" 반복 오류
- **ROS2 ZED wrapper**: Camera open 실패

---

## 버전 호환성 분석

### Stereolabs ZED Link Driver 변경 이력

#### v1.3.1 (July 16, 2025)
- ✅ **JetPack 6.2.1 (L4T 36.4.4) 지원 추가**
- ✅ 카메라 동기화 지원
- ✅ Real-Time (RT) 커널 지원

#### v1.3.2 (October 27, 2025)
- ✅ **L4T R36.4.7까지 지원**

### 현재 설치된 버전의 문제
```bash
$ dpkg -l | grep stereolabs-zedlink-quad
ii  stereolabs-zedlink-quad  1.3.2-SL-MAX96712-all-L4T36.4.0  arm64
```

**문제점**: 패키지 이름에 `L4T36.4.0`으로 표시됨
- 이는 L4T 36.4.0용으로 빌드된 바이너리
- L4T 36.4.4와 커널 API 호환성 문제 발생 가능

---

## 해결 방법

### 🔧 방법 1: 올바른 버전의 드라이버 재설치 (권장)

1. **최신 드라이버 다운로드**
   - 방문: https://www.stereolabs.com/developers/drivers
   - 다운로드: `stereolabs-zedlink-quad_1.3.2-SL-MAX96712-L4T36.4.4_arm64.deb` 또는 최신 버전

2. **기존 드라이버 제거**
   ```bash
   sudo apt remove stereolabs-zedlink-quad
   sudo apt autoremove
   ```

3. **새 드라이버 설치**
   ```bash
   sudo dpkg -i stereolabs-zedlink-quad_*_L4T36.4.*_arm64.deb
   sudo apt install -f  # 의존성 해결
   ```

4. **시스템 재부팅**
   ```bash
   sudo reboot
   ```

5. **설치 확인**
   ```bash
   # 재부팅 후
   dpkg -l | grep stereolabs-zedlink-quad
   lsmod | grep sl_
   ZED_Diagnostic
   ```

---

### 🔧 방법 2: ZED SDK 업그레이드

현재 ZED SDK 4.2.5는 구버전입니다. SDK 5.1로 업그레이드:

1. **최신 SDK 다운로드**
   - https://www.stereolabs.com/developers/release/
   - JetPack 6.2 (L4T 36.4)용 ZED SDK **5.1** 다운로드

2. **설치**
   ```bash
   chmod +x ZED_SDK_Jetson_*_v5.1.*.run
   sudo ./ZED_SDK_Jetson_*_v5.1.*.run -- skip_cuda=yes
   ```

3. **확인**
   ```bash
   cat /usr/local/zed/zed-config.cmake | grep VERSION
   ZED_Diagnostic
   ```

---

### 🔧 방법 3: 수동 커널 모듈 재빌드 (고급)

드라이버 소스가 있다면 현재 커널에 맞게 재컴파일:

```bash
# 드라이버 소스 확인
ls -la /usr/src/ | grep zed

# DKMS를 통한 재빌드 (가능한 경우)
sudo dkms status
sudo dkms remove stereolabs-zedlink-quad -k $(uname -r)
sudo dkms install stereolabs-zedlink-quad -k $(uname -r)
```

---

## 검증 단계

드라이버 재설치 후 다음 단계로 검증:

### 1. 커널 모듈 확인
```bash
lsmod | grep -E "sl_zedx|sl_max96712"
dmesg | grep -i "zed\|gmsl" | tail -30
```

### 2. ZED_Diagnostic 실행
```bash
ZED_Diagnostic
# "GMSL driver incompatible" 오류가 사라져야 함
```

### 3. ROS2 테스트
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch zed_wrapper single_zedxmini_test.launch.py
# "CAMERA NOT DETECTED" 오류가 사라지고 정상 초기화되어야 함
```

---

## 추가 조사 사항

### Stereolabs 포럼 확인
관련 이슈:
- [ZED Link Drivers v1.3.0 is missing support for JP6.2.1 (l4t 36.4.4)](https://community.stereolabs.com/t/zed-link-drivers-v1-3-0-is-missing-support-for-jp6-2-1-l4t-36-4-4/9212)

### 패키지 이름 확인
정확한 패키지 이름은 다음과 같아야 합니다:
- ❌ `stereolabs-zedlink-quad_1.3.2-SL-MAX96712-all-L4T36.4.0_arm64.deb`
- ✅ `stereolabs-zedlink-quad_1.3.2-SL-MAX96712-L4T36.4.4_arm64.deb`

또는

- ✅ `stereolabs-zedlink-quad_1.3.2-SL-MAX96712-all-L4T36.4.7_arm64.deb`

---

## 참고 자료

- Stereolabs Drivers: https://www.stereolabs.com/developers/drivers
- ZED SDK Release: https://www.stereolabs.com/developers/release
- Community Forum: https://community.stereolabs.com/
- Install Guide: https://www.stereolabs.com/docs/embedded/zed-link/install-the-drivers

---

## 다음 작업

1. ✅ 문제 진단 완료
2. ⏳ **Stereolabs 드라이버 페이지에서 L4T 36.4.4 호환 패키지 다운로드**
3. ⏳ **드라이버 재설치**
4. ⏳ 재부팅 후 ZED_Diagnostic 재실행
5. ⏳ ROS2 ZED wrapper 테스트
6. ⏳ 4개 카메라 동시 구동 확인
