# ZED Camera Troubleshooting Guide

## 문제: "CAMERA STREAM FAILED TO START" 오류

### 원인
카메라 상태가 "NOT AVAILABLE"로 표시되는 경우, 다음과 같은 원인이 있을 수 있습니다:
1. ZEDX_Daemon이 카메라를 점유하고 있음
2. 이전 프로세스가 카메라를 정상적으로 해제하지 못함
3. 전원 공급 문제
4. USB 대역폭 부족

### 해결 방법

#### 방법 1: ZEDX_Daemon 재시작 (권장)

```bash
# Daemon 상태 확인
sudo systemctl status zedx_daemon

# Daemon 재시작
sudo systemctl restart zedx_daemon

# 또는 Daemon 중지 (ROS2 wrapper만 사용하는 경우)
sudo systemctl stop zedx_daemon

# 카메라 상태 다시 확인
python3 -c "import pyzed.sl as sl; zed = sl.Camera(); devices = zed.get_device_list(); [print(f'Camera {i}: SN={d.serial_number}, State={d.camera_state}') for i, d in enumerate(devices)]"
```

#### 방법 2: 시스템 재부팅

가장 확실한 방법은 시스템을 재부팅하는 것입니다:
```bash
sudo reboot
```

재부팅 후:
```bash
cd /home/test/ros2_ws
source install/setup.bash
ros2 launch zed_wrapper two_zedxmini.launch.py
```

#### 방법 3: USB 케이블 재연결

물리적으로 카메라를 재연결:
1. ZED X Mini 카메라들의 USB/GMSL 케이블을 분리
2. 10초 대기
3. 다시 연결
4. 카메라 상태 확인:
```bash
python3 -c "import pyzed.sl as sl; zed = sl.Camera(); devices = zed.get_device_list(); [print(f'Camera {i}: SN={d.serial_number}, State={d.camera_state}') for i, d in enumerate(devices)]"
```

#### 방법 4: 카메라 개별 테스트

한 번에 한 대씩 테스트하여 문제 카메라 식별:

**ZED X Mini 1만 실행:**
```bash
ros2 run zed_wrapper zed_wrapper --ros-args \
  -p general.camera_model:=zedxm \
  -p general.serial_number:=56755054 \
  -p general.camera_name:=zedxmini1
```

**ZED X Mini 2만 실행:**
```bash
ros2 run zed_wrapper zed_wrapper --ros-args \
  -p general.camera_model:=zedxm \
  -p general.serial_number:=54946194 \
  -p general.camera_name:=zedxmini2
```

#### 방법 5: Launch 파일에서 순차 실행

두 카메라를 동시에 시작하지 않고 순차적으로 시작하도록 지연 추가.

#### 방법 6: 해상도 및 프레임레이트 낮추기

카메라 설정 파일 수정:
```bash
# 설정 파일 열기
nano /home/test/ros2_ws/install/zed_wrapper/share/zed_wrapper/config/zedxm.yaml

# 다음 값들을 낮추기:
# grab_resolution: 'HD720'  # HD1200 대신
# grab_frame_rate: 15        # 30 대신
```

#### 방법 7: ZED Diagnostic 실행

```bash
# 카메라 진단 도구 실행
ZED_Diagnostic

# 또는 간단한 테스트
ZED_Explorer
```

### 권장 순서

1. **먼저 시도**: ZEDX_Daemon 재시작 (방법 1)
2. **그래도 안되면**: 시스템 재부팅 (방법 2)
3. **문제 지속시**: 카메라 개별 테스트 (방법 4)

### 현재 상황 확인 명령어

```bash
# 카메라 상태
python3 -c "import pyzed.sl as sl; zed = sl.Camera(); devices = zed.get_device_list(); [print(f'Camera {i}: SN={d.serial_number}, Model={d.camera_model}, State={d.camera_state}') for i, d in enumerate(devices)]"

# ZED 프로세스 확인
ps aux | grep -i zed

# Video 장치 확인
ls -l /dev/video*

# I2C 장치 확인
ls -l /dev/i2c-*

# Daemon 상태
sudo systemctl status zedx_daemon
```

### 추가 정보

- ZED X Mini는 ZED Link Duo 카드를 통해 GMSL 연결을 사용합니다
- 여러 카메라를 동시에 사용할 때는 충분한 USB 대역폭과 전원이 필요합니다
- ZEDX_Daemon은 백그라운드에서 카메라를 관리하지만, ROS2 wrapper와 충돌할 수 있습니다
