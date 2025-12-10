# Phase 2 시작 가이드

## 현재 상태
- **브랜치:** `refactoring/phase1-interfaces`
- **완료:** Phase 1 (메시지 인터페이스 정의)
- **다음:** Phase 2 (하드웨어 추상화 계층 구현)

## Phase 2 시작 전 체크리스트

### 1. 브랜치 확인
```bash
cd /home/koceti/ros2_ws
git status
# 현재 브랜치: refactoring/phase1-interfaces
```

### 2. Phase 2 브랜치 생성
```bash
git checkout -b refactoring/phase2-base-control
```

### 3. 환경 설정
```bash
source install/setup.bash
```

### 4. 의존성 설치
```bash
# python-statemachine 설치 (State Machine용)
pip3 install python-statemachine
```

## Phase 2 작업 순서

### Step 1: 패키지 구조 생성
```bash
mkdir -p src/rebar_base_control/rebar_base_control
mkdir -p src/rebar_base_control/launch
mkdir -p src/rebar_base_control/config
cd src/rebar_base_control
```

### Step 2: 구현할 노드 (우선순위 순)

#### 1. can_parser.py (가장 먼저)
- CAN 메시지 수신 → ROS2 메시지 변환
- 참조: `src/rmd_robot_control/rmd_robot_control/position_control_node.py`
- 발행: `/motor_feedback`, `/remote_control`

#### 2. can_sender.py
- ROS2 메시지 → CAN 메시지 전송
- 참조: `src/rmd_robot_control/rmd_robot_control/position_control_node.py`
- 구독: `/drive_control`

#### 3. drive_controller.py
- `/cmd_vel` → DriveControl 변환
- Differential drive kinematics
- 참조: `src/rmd_robot_control/rmd_robot_control/position_control_node.py` (속도 변환 부분)

#### 4. modbus_controller.py
- Seengrip + EZI-IO 통합
- 참조: `src/ezi_io_ros2/ezi_io_ros2/ezi_io_node.py`
- 참조: `src/seengrip_ros2/seengrip_ros2/seengrip_node.py`

#### 5. authority_controller.py
- S10/S20 모드 전환 관리
- 참조: `src/rebar_control/rebar_control/iron_md_teleop_node.py`

#### 6. navigator_base.py (마지막)
- State Machine 구현
- 참조: `src/tire_roller/tire_roller_basecontrol/navigator.py`

### Step 3: 설정 파일
- `config/can_devices.yaml`
- `config/modbus_devices.yaml`

### Step 4: Launch 파일
- `launch/base_system.launch.py`

## 참고 자료

### 기존 코드 위치
```
백업 (참조용):
- src/rmd_robot_control/rmd_robot_control/position_control_node.py
- src/rebar_control/rebar_control/iron_md_teleop_node.py
- src/ezi_io_ros2/ezi_io_ros2/ezi_io_node.py
- src/seengrip_ros2/seengrip_ros2/seengrip_node.py

Tire Roller 참조:
- src/tire_roller/tire_roller_basecontrol/authority_controller.py
- src/tire_roller/tire_roller_basecontrol/drive_controller.py
- src/tire_roller/tire_roller_basecontrol/navigator.py (State Machine)
```

### 하드웨어 스펙
```yaml
CAN:
  CAN2:
    bitrate: 1000000  # 1Mbps
    devices:
      - id: 0x141  # Left motor
      - id: 0x142  # Right motor (inverted)
  CAN3:
    bitrate: 250000   # 250kbps
    devices:
      - remote_control

Modbus:
  Seengrip:
    type: RTU
    port: /dev/ttyUSB0
    baudrate: 115200
  EZI_IO:
    type: TCP
    host: 192.168.1.100
```

### Differential Drive
- 왼쪽 모터 (0x141): 그대로
- 오른쪽 모터 (0x142): **반전 필요** (서로 마주보고 장착)

## 테스트 계획

### 단위 테스트 (각 노드별)
1. can_parser 테스트: CAN 메시지 파싱
2. can_sender 테스트: CAN 메시지 전송
3. drive_controller 테스트: 속도 변환

### 통합 테스트
1. 모든 노드 동시 실행
2. CAN 통신 확인
3. Modbus 통신 확인
4. State Machine 전이 확인

## 예상 일정
- Day 1: can_parser, can_sender
- Day 2: drive_controller, modbus_controller
- Day 3: authority_controller, navigator_base
- Day 4: 통합 테스트, 디버깅

## 문의 및 지원
- 전체 계획: `REBAR_REFACTORING_PLAN.md`
- Phase 1 요약: `PHASE1_SUMMARY.md`
