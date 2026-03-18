# integrated_control_debug.sh 프로그램 개발 내용 분석

## 📋 목차
1. [스크립트 개요](#스크립트-개요)
2. [실행되는 노드 분석](#실행되는-노드-분석)
3. [시스템 아키텍처](#시스템-아키텍처)
4. [제어 흐름](#제어-흐름)
5. [로그 시스템](#로그-시스템)
6. [프로세스 모니터링](#프로세스-모니터링)
7. [개발 내용 요약](#개발-내용-요약)

---

## 스크립트 개요

### 파일 위치
- **경로**: `/home/koceti/ros2_ws/integrated_control_debug.sh`
- **용도**: 철근 결속 로봇 통합 제어 시스템 (부팅 자동 실행용)
- **환경**: 모니터 없는 헤드리스 환경 - 로그 중심 실행

### 주요 기능
1. **4개 ROS2 노드 순차 실행**
   - RMD 모터 제어 노드 (can2)
   - EZI-IO 리미트 센서 노드 (192.168.0.3)
   - Seengrip 그리퍼 노드 (/dev/ttyUSB0)
   - Iron-MD 텔레옵 노드 (can3)

2. **로그 관리**
   - 타임스탬프 기반 로그 파일 생성
   - 최신 로그 심볼릭 링크 유지
   - 모든 출력을 로그 파일로 리다이렉션

3. **프로세스 모니터링**
   - 5분마다 프로세스 상태 확인
   - 노드 종료 시 자동 로깅

4. **안전 종료**
   - SIGINT/SIGTERM 트랩 처리
   - 모든 노드 순차 종료

---

## 실행되는 노드 분석

### 1. RMD 모터 제어 노드 (position_control_node)

#### 실행 명령
```bash
ros2 run rmd_robot_control position_control_node >> $LOG_FILE 2>&1 &
```

#### 소스 파일
- **경로**: `/home/koceti/ros2_ws/src/rmd_robot_control/rmd_robot_control/position_control_node.py`
- **크기**: 약 1,140줄

#### 주요 기능
- **7개 RMD 모터 통합 제어**
  - `0x141, 0x142`: 속도 제어 (주행 모터, cmd_vel 기반)
  - `0x143-0x147`: 위치 제어 (관절 모터)

- **cmd_vel 처리**
  - 차등 구동(differential drive) 계산
  - 좌우 바퀴 RPM 변환
  - 최대 속도 제한 (linear: 10.0 m/s, angular: 2.0 rad/s)

- **위치 제어**
  - 절대 위치 제어 (0xA4 프로토콜)
  - 위치 제한 (min: -36000°, max: 36000°)
  - 위치 허용 오차: 1.0°

- **브레이크 제어 서비스**
  - `/safe_brake_release`: 순차적 브레이크 해제
  - `/safe_brake_lock`: 순차적 브레이크 잠금

- **CAN 통신**
  - 인터페이스: `can2` (1 Mbps)
  - 프로토콜: RMD X4 Protocol V4.3
  - 이벤트 기반 피드백 (0x92 멀티턴 각도 읽기)

#### 발행 토픽
- `/joint_states` (sensor_msgs/JointState): 전체 관절 상태
- `/motor_status` (std_msgs/Float64MultiArray): 모터별 상태 배열
- `/motor_0x141_rpm` (std_msgs/Float32): 좌측 바퀴 RPM
- `/motor_0x142_rpm` (std_msgs/Float32): 우측 바퀴 RPM

#### 구독 토픽
- `/cmd_vel` (geometry_msgs/Twist): 주행 제어
- `/joint_1/position` (std_msgs/Float64MultiArray): 0x146 리프팅
- `/joint_2/position` (std_msgs/Float64MultiArray): 0x144 X축
- `/joint_3/position` (std_msgs/Float64MultiArray): 0x145 Y축
- `/joint_4/position` (std_msgs/Float64MultiArray): 0x143 횡이동
- `/joint_5/position` (std_msgs/Float64MultiArray): 0x147 Yaw

---

### 2. EZI-IO 리미트 센서 노드 (ezi_io_node)

#### 실행 명령
```bash
ros2 run ezi_io_ros2 ezi_io_node --ros-args -p ip_address:=192.168.0.3 >> $LOG_FILE 2>&1 &
```

#### 소스 파일
- **경로**: `/home/koceti/ros2_ws/src/ezi_io_ros2/ezi_io_ros2/ezi_io_node.py`
- **크기**: 약 237줄

#### 주요 기능
- **FASTECH EZI-IO-EN-L16O16N-T I/O 모듈 제어**
  - Modbus TCP 통신
  - IP 주소: 192.168.0.3:502

- **리미트 센서 모니터링**
  - 6개 리미트 센서 읽기
    - X축: min (IN02), max (IN03)
    - Y축: min (IN01), max (IN00)
    - Z축: min (IN05), max (IN06)
    - Yaw: 원점 (IN04)

- **업데이트 주기**
  - 기본: 20Hz (0.05초마다)

#### 발행 토픽
- `/limit_sensors/x_min` (std_msgs/Bool)
- `/limit_sensors/x_max` (std_msgs/Bool)
- `/limit_sensors/y_min` (std_msgs/Bool)
- `/limit_sensors/y_max` (std_msgs/Bool)
- `/limit_sensors/z_min` (std_msgs/Bool)
- `/limit_sensors/z_max` (std_msgs/Bool)
- `/limit_sensors/yaw_min` (std_msgs/Bool)

#### 의존성
- FASTECH Plus-E 라이브러리
- 경로: `/home/koceti/python/PE/Library` (환경 변수로 설정 가능)

---

### 3. Seengrip 그리퍼 노드 (seengrip_node)

#### 실행 명령
```bash
ros2 run seengrip_ros2 seengrip_node --ros-args -p serial_port:=/dev/ttyUSB0 >> $LOG_FILE 2>&1 &
```

#### 소스 파일
- **경로**: `/home/koceti/ros2_ws/src/seengrip_ros2/seengrip_ros2/seengrip_node.py`
- **크기**: 약 469줄

#### 주요 기능
- **Seengrip Optimum Gripper 제어**
  - Modbus RTU 통신
  - 시리얼 포트: `/dev/ttyUSB0`
  - 보드레이트: 115200
  - Slave ID: 1

- **그리퍼 제어 명령**
  - `CMD_HOME` (1): 원점 복귀
  - `CMD_MOVE` (2): 위치 이동
  - `CMD_GRIP` (5): 파지
  - `CMD_SPEEDY_GRIP` (6): 빠른 파지

- **기본 설정**
  - 열림 위치: 0
  - 닫힘 위치: 2000
  - 기본 속도: 500 (0.1% 단위)

#### 구독 토픽
- `/gripper/position` (std_msgs/Float32): 목표 위치 (0-2000)
- `/gripper/command` (std_msgs/Int32): 명령 코드

#### 발행 토픽
- `/gripper/status` (sensor_msgs/JointState): 그리퍼 상태
- `/gripper/present_position` (std_msgs/Float32): 현재 위치
- `/gripper/present_speed` (std_msgs/Float32): 현재 속도
- `/gripper/present_temperature` (std_msgs/Float32): 온도
- `/gripper/present_voltage` (std_msgs/Float32): 전압
- `/gripper/fault` (std_msgs/Int32): 에러 코드

---

### 4. Iron-MD 텔레옵 노드 (iron_md_teleop)

#### 실행 명령
```bash
ros2 run rebar_control iron_md_teleop >> $LOG_FILE 2>&1 &
```

#### 소스 파일
- **경로**: `/home/koceti/ros2_ws/src/rebar_control/rebar_control/iron_md_teleop_node.py`
- **크기**: 약 1,255줄

#### 주요 기능
- **Iron-MD 무선 리모콘 CAN 통신**
  - CAN 인터페이스: `can3` (250 kbps)
  - CAN ID 파싱:
    - `0x1E4` (484): 조이스틱 아날로그 데이터 (50ms)
    - `0x2E4` (740): 스위치 및 상태 (50ms)
    - `0x764` (1892): Heartbeat (300ms)

- **조이스틱 매핑**
  - `AN3`: 전후진 → `/cmd_vel` linear.x (코드상 angular 변수 사용, 모터 180도 장착)
  - `AN4`: 좌우 회전 → `/cmd_vel` angular.z (코드상 linear 변수 사용, 모터 180도 장착)
  - `AN1`: X축 이동 → `/joint_2/position` (0x144)
  - `AN2`: Y축 이동 → `/joint_3/position` (0x145)

- **스위치 매핑**
  - `S13`: 브레이크 토글
  - `S14`: 위치 리셋
  - `S17/S18`: 횡이동 ±50mm (0x143)
  - `S23/S24`: Yaw ±30° (0x147)
  - `S21/S22`: 작업 시퀀스
  - `Emergency_Stop`: 비상 정지

- **제어 루프**
  - 주기: 20Hz (0.05초마다)
  - 조이스틱 데드존: 20
  - 조이스틱 중립값: 127

#### 발행 토픽
- `/cmd_vel` (geometry_msgs/Twist): 주행 제어
- `/joint_1/position` (std_msgs/Float64MultiArray): 0x146 리프팅
- `/joint_2/position` (std_msgs/Float64MultiArray): 0x144 X축
- `/joint_3/position` (std_msgs/Float64MultiArray): 0x145 Y축
- `/joint_4/position` (std_msgs/Float64MultiArray): 0x143 횡이동
- `/joint_5/position` (std_msgs/Float64MultiArray): 0x147 Yaw
- `/gripper/position` (std_msgs/Float32): 그리퍼 위치
- `/motor_0/vel` (std_msgs/Float32): Pololu 트리거 속도
- `/emergency_stop` (std_msgs/Bool): 비상 정지 플래그

---

## 시스템 아키텍처

### 하드웨어 구성

```
┌─────────────────────────────────────────────────────────────┐
│                    Iron-MD Wireless Remote                  │
│              (CAN3 @ 250kbps - 4 Joysticks)                │
└──────────────────────────┬──────────────────────────────────┘
                           │
                    ┌──────▼──────┐
                    │  Ubuntu PC  │
                    │ ROS2 Humble │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐      ┌──────▼──────┐    ┌─────▼─────┐
   │  CAN2   │      │  Modbus RTU │    │Modbus TCP │
   │ 1 Mbps  │      │/dev/ttyUSB0 │    │192.168.0.3│
   └────┬────┘      └──────┬──────┘    └─────┬─────┘
        │                  │                  │
   ┌────▼────────────┐ ┌───▼────┐      ┌─────▼─────┐
   │ 7 RMD Motors    │ │Seengrip│      │ EZI-IO    │
   │ 0x141 - 0x147   │ │Gripper │      │  Sensors  │
   └─────────────────┘ └────────┘      └───────────┘
```

### 모터 배치

| Motor ID | Function | Control Type | Notes |
|----------|----------|--------------|-------|
| 0x141 | 좌측 주행 모터 | 속도 제어 (0xA2) | Differential drive |
| 0x142 | 우측 주행 모터 | 속도 제어 (0xA2) | Differential drive |
| 0x143 | 횡이동 (Lateral) | 위치 제어 (0xA4) | ±360° rotation |
| 0x144 | X축 스테이지 | 속도 제어 (0xA2) | Linear motion |
| 0x145 | Y축 스테이지 | 속도 제어 (0xA2) | Linear motion |
| 0x146 | Z축 (상하) | 위치 제어 (0xA4) | Work sequence |
| 0x147 | Yaw (회전) | 위치 제어 (0xA4) | ±30° rotation |

---

## 제어 흐름

### 전체 제어 흐름도

```
Iron-MD 리모콘 (CAN3)
  ↓
iron_md_teleop_node
  ├─► /cmd_vel (Twist) → position_control_node → 0x141, 0x142 (주행)
  ├─► /joint_1/position → position_control_node → 0x146 (리프팅)
  ├─► /joint_2/position → position_control_node → 0x144 (X축)
  ├─► /joint_3/position → position_control_node → 0x145 (Y축)
  ├─► /joint_4/position → position_control_node → 0x143 (횡이동)
  ├─► /joint_5/position → position_control_node → 0x147 (Yaw)
  ├─► /gripper/position → seengrip_node → Seengrip 그리퍼
  └─► /motor_0/vel → pololu_node → Pololu 트리거

position_control_node (CAN2)
  ├─► 0x141, 0x142: 속도 제어 명령
  ├─► 0x143-0x147: 위치 제어 명령
  └─► 피드백: /joint_states, /motor_status

ezi_io_node (Modbus TCP)
  └─► /limit_sensors/*: 리미트 센서 상태
```

### 실행 순서

1. **RMD 모터 제어 노드** 시작 (3초 대기)
   - CAN2 인터페이스 연결
   - 7개 모터 초기화
   - ROS2 토픽 구독 준비

2. **EZI-IO 센서 노드** 시작 (2초 대기)
   - Modbus TCP 연결 (192.168.0.3)
   - 리미트 센서 모니터링 시작

3. **Seengrip 그리퍼 노드** 시작 (2초 대기)
   - Modbus RTU 연결 (/dev/ttyUSB0)
   - 그리퍼 초기화

4. **Iron-MD 텔레옵 노드** 시작 (2초 대기)
   - CAN3 인터페이스 연결
   - 리모콘 입력 수신 시작

---

## 로그 시스템

### 로그 디렉토리
- **경로**: `/var/log/robot_control/`
- **파일명 형식**: `control_YYYYMMDD_HHMMSS.log`
- **최신 로그**: `control_latest.log` (심볼릭 링크)

### 로그 함수
```bash
log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $LOG_FILE
}
```

### 로그 내용
- 시스템 시작/종료 시간
- 각 노드의 PID
- 프로세스 상태 확인 결과
- 노드 종료 에러 메시지

### 로그 확인 방법
```bash
# 실시간 로그 보기
tail -f /var/log/robot_control/control_latest.log

# 로그 뷰어 사용
./view_logs.sh -f  # 실시간 로그
./view_logs.sh -e  # 에러만 보기
./view_logs.sh -j  # 조이스틱 로그
```

---

## 프로세스 모니터링

### 모니터링 함수
```bash
monitor_processes() {
    while true; do
        sleep 300  # 5분
        # 프로세스 상태 확인
        if ! kill -0 $MOTOR_PID 2>/dev/null; then
            log_msg "⚠️ ERROR: 모터 제어 노드 종료됨 (PID $MOTOR_PID)"
        fi
        # ... 다른 노드들도 동일하게 확인
    done
}
```

### 모니터링 주기
- **5분마다** 프로세스 상태 확인
- 노드 종료 시 로그에 에러 기록

### 종료 처리
```bash
cleanup() {
    log_msg "========== 시스템 종료 중 =========="
    kill $MONITOR_PID 2>/dev/null
    kill $TELEOP_PID 2>/dev/null
    kill $GRIPPER_PID 2>/dev/null
    kill $EZIIO_PID 2>/dev/null
    kill $MOTOR_PID 2>/dev/null
    sleep 1
    log_msg "모든 노드 종료 완료"
}
```

---

## 개발 내용 요약

### 1. 통합 제어 스크립트 개발
- **목적**: 헤드리스 환경에서 자동 실행
- **기능**: 4개 노드 순차 실행, 로그 관리, 프로세스 모니터링
- **특징**: 모니터 없는 환경에 최적화

### 2. RMD 모터 제어 노드
- **7개 모터 통합 제어**
- **cmd_vel 기반 차등 구동**
- **위치/속도 제어 통합**
- **브레이크 제어 서비스**

### 3. EZI-IO 리미트 센서 노드
- **Modbus TCP 통신**
- **6개 리미트 센서 모니터링**
- **20Hz 업데이트 주기**

### 4. Seengrip 그리퍼 노드
- **Modbus RTU 통신**
- **그리퍼 위치/속도 제어**
- **에러 상태 모니터링**

### 5. Iron-MD 텔레옵 노드
- **CAN 통신 (can3)**
- **조이스틱/스위치 매핑**
- **작업 시퀀스 자동화**
- **비상 정지 기능**

### 6. 로그 시스템
- **타임스탬프 기반 로그 파일**
- **최신 로그 심볼릭 링크**
- **로그 뷰어 유틸리티**

### 7. 프로세스 모니터링
- **5분마다 상태 확인**
- **노드 종료 자동 감지**
- **에러 로깅**

### 8. 시스템 서비스 통합
- **systemd 서비스 설정**
- **부팅 시 자동 실행**
- **자동 재시작 기능**

---

## 관련 파일

### 스크립트 파일
- `integrated_control_debug.sh`: 메인 실행 스크립트
- `integrated_control.sh`: 일반 실행 스크립트 (로그 없음)
- `view_logs.sh`: 로그 뷰어
- `cleanup_logs.sh`: 로그 정리

### 설정 파일
- `robot-control.service`: systemd 서비스 설정
- `setup_autostart.sh`: 자동 시작 설정 스크립트

### 문서
- `README.md`: 프로젝트 개요
- `ROBOT_CONTROL_ARCHITECTURE.md`: 시스템 아키텍처 상세
- `HEADLESS_OPERATION.md`: 헤드리스 운영 가이드
- `MOTOR_CONTROL_GUIDE.md`: 모터 제어 가이드

---

## 결론

`integrated_control_debug.sh`는 철근 결속 로봇의 통합 제어 시스템을 헤드리스 환경에서 자동으로 실행하기 위한 스크립트입니다. 4개의 ROS2 노드를 순차적으로 실행하고, 로그를 관리하며, 프로세스를 모니터링하여 안정적인 운영을 보장합니다.

주요 특징:
- ✅ 모니터 없는 환경 최적화
- ✅ 자동 로그 관리
- ✅ 프로세스 모니터링
- ✅ 안전한 종료 처리
- ✅ 부팅 시 자동 실행

---

**작성일**: 2025-01-XX  
**분석 대상**: integrated_control_debug.sh 및 관련 노드들


