# Phase 2 완료 요약

**날짜:** 2025-12-17
**브랜치:** `refactoring/phase2-base-control`
**커밋:** `fa1b412 - feat(phase2): implement hardware abstraction layer (rebar_base_control)`

---

## 완료 내역

### 1. 패키지 생성
`rebar_base_control` - 하드웨어 추상화 계층

```
src/rebar_base_control/
├── rebar_base_control/
│   ├── __init__.py
│   ├── can_parser.py              # CAN 메시지 파싱
│   ├── can_sender.py              # CAN 메시지 전송
│   ├── drive_controller.py        # cmd_vel → DriveControl 변환
│   ├── modbus_controller.py       # Modbus 통신 (Seengrip + EZI-IO)
│   ├── authority_controller.py    # 제어권한 관리
│   └── navigator_base.py          # State Machine
├── launch/
│   └── base_system.launch.py      # 전체 시스템 Launch
├── config/
│   ├── can_devices.yaml           # CAN 설정
│   └── modbus_devices.yaml        # Modbus 설정
├── CMakeLists.txt
├── package.xml
└── setup.py
```

---

## 2. 구현된 노드 (6개)

### 2.1 can_parser.py
**역할:** CAN 메시지 수신 → ROS2 메시지 변환

**CAN 버스:**
- CAN2 (1Mbps): 모터 피드백 (0x241, 0x242)
- CAN3 (250kbps): 리모콘 (0x1E4, 0x2E4, 0x764)

**발행 토픽:**
- `/motor_feedback` (MotorFeedback)
- `/remote_control` (RemoteControl)

**주요 기능:**
- 모터 응답 파싱 (온도, 전류, 속도, 엔코더)
- 리모콘 조이스틱 데이터 파싱
- 리모콘 스위치 상태 파싱 (S10/S20, 비상정지 등)

---

### 2.2 can_sender.py
**역할:** ROS2 메시지 → CAN 메시지 전송

**구독 토픽:**
- `/drive_control` (DriveControl)

**CAN 명령:**
- 0x141 (왼쪽 모터): 속도 제어 (0xA2)
- 0x142 (오른쪽 모터): 속도 제어 (0xA2, 반전)

**주요 기능:**
- m/s → dps 변환
- RMD-X4 속도 제어 프로토콜 (0xA2)
- 오른쪽 모터 반전 처리

---

### 2.3 drive_controller.py
**역할:** Twist (cmd_vel) → DriveControl 변환

**구독 토픽:**
- `/cmd_vel` (Twist)

**발행 토픽:**
- `/drive_control` (DriveControl)

**주요 기능:**
- Differential drive kinematics
  ```
  v_left = v_linear - (ω * wheel_base / 2)
  v_right = v_linear + (ω * wheel_base / 2)
  ```
- 속도 제한 (max_linear_vel, max_angular_vel)

**파라미터:**
- `wheel_base`: 0.5 m
- `max_linear_vel`: 10.0 m/s
- `max_angular_vel`: 2.0 rad/s

---

### 2.4 modbus_controller.py
**역할:** Modbus 통신 통합 관리

**Modbus 장치:**
1. **Seengrip (Modbus RTU)**
   - Port: /dev/ttyUSB0
   - Baudrate: 115200
   - 기능: 그리퍼 제어

2. **EZI-IO (Modbus TCP)**
   - Host: 192.168.1.100
   - Port: 502
   - 기능: 센서 상태 읽기

**구독 토픽:**
- `/gripper_control` (GripperControl)

**발행 토픽:**
- `/io_status` (IOStatus) - 10Hz

**주요 기능:**
- 그리퍼 제어 (열기/닫기/정지)
- 디지털 입력 읽기 (리미트 스위치 등)
- 아날로그 입력 읽기 (배터리 전압 등)

---

### 2.5 authority_controller.py
**역할:** 제어권한 관리 (Manual/Auto 전환)

**구독 토픽:**
- `/remote_control` (RemoteControl)
- `/control_mode_request` (String)

**발행 토픽:**
- `/authority_status` (String) - 5Hz
- `/emergency_stop` (Bool)

**모드:**
- `idle`: 초기 상태 (중립)
- `manual`: S10 (S19) 활성화 - 리모콘 제어 허용
- `auto`: S20 활성화 - UI/Navigator 제어만 허용
- `emergency_stop`: 비상정지

**주요 기능:**
- S10/S20 스위치 감지
- 비상정지 감지 및 발행
- Auto 모드에서 리모콘 명령 차단

---

### 2.6 navigator_base.py
**역할:** State Machine 기반 상태 관리

**State Machine (python-statemachine):**

```
상태:
- idle (초기)
- manual (S10 모드)
- auto (S20 모드)
- navigating (정밀 주행 중)
- tying (결속 작업 중)
- emergency_stop (비상정지)

전이:
- idle → manual (S10 활성화)
- idle → auto (S20 활성화)
- manual ↔ auto (스위치 전환)
- auto → navigating (주행 명령)
- navigating → tying (결속 명령)
- * → emergency_stop (비상정지)
- emergency_stop → idle (해제)
```

**구독 토픽:**
- `/remote_control` (RemoteControl)
- `/authority_status` (String)
- `/emergency_stop` (Bool)
- `/rebar_motion_cmd` (String)

**발행 토픽:**
- `/control_mode` (String) - 5Hz
- `/state_machine_status` (String) - 5Hz

**주요 기능:**
- 상태 전이 관리
- 상위 제어 계층 명령 처리 (MOVE_, START_TYING 등)
- 상태별 진입/퇴출 콜백

---

## 3. 설정 파일

### 3.1 can_devices.yaml
```yaml
can_parser:
  motor_can_interface: 'can2'
  remote_can_interface: 'can3'
  motor_can_bitrate: 1000000
  remote_can_bitrate: 250000

can_sender:
  can_interface: 'can2'
  left_motor_id: 0x141
  right_motor_id: 0x142
  wheel_radius: 0.1

drive_controller:
  wheel_base: 0.5
  max_linear_vel: 10.0
  max_angular_vel: 2.0
```

### 3.2 modbus_devices.yaml
```yaml
modbus_controller:
  seengrip_port: '/dev/ttyUSB0'
  seengrip_baudrate: 115200
  ezi_io_host: '192.168.1.100'
  ezi_io_port: 502
```

---

## 4. Launch 파일

### base_system.launch.py
전체 하드웨어 추상화 계층 실행

**실행 노드:**
1. can_parser
2. can_sender
3. drive_controller
4. modbus_controller
5. authority_controller
6. navigator_base

**실행 방법:**
```bash
source install/setup.bash
ros2 launch rebar_base_control base_system.launch.py
```

---

## 5. 빌드 결과

```bash
$ colcon build --packages-select rebar_base_control --symlink-install

Starting >>> rebar_base_control
Finished <<< rebar_base_control [2.98s]

Summary: 1 package finished [3.54s]
```

**빌드 상태:** ✅ 성공

---

## 6. 토픽 구조

### 하드웨어 → ROS2 (발행)
```
/motor_feedback (MotorFeedback)          # CAN2 모터 피드백
/remote_control (RemoteControl)          # CAN3 리모콘
/io_status (IOStatus)                    # Modbus EZI-IO 센서
/authority_status (String)               # 현재 권한 상태
/control_mode (String)                   # 현재 State Machine 상태
/state_machine_status (String)           # 상세 상태
/emergency_stop (Bool)                   # 비상정지 신호
```

### ROS2 → 하드웨어 (구독)
```
/cmd_vel (Twist)                         # 속도 명령 (상위 계층)
/drive_control (DriveControl)            # 모터 속도 (내부)
/gripper_control (GripperControl)        # 그리퍼 제어
/rebar_motion_cmd (String)               # 모션 명령 (상위 계층)
/control_mode_request (String)           # 모드 요청 (상위 계층)
```

---

## 7. 주요 설계 결정

### 7.1 계층 분리
- **하드웨어 계층 (rebar_base_control)**: CAN/Modbus 직접 통신
- **제어 계층 (rebar_control)**: 상위 로직 (Phase 3에서 구현)

### 7.2 State Machine 도입
- `python-statemachine` 라이브러리 사용
- 명확한 상태 전이 규칙
- 상태별 진입/퇴출 콜백

### 7.3 권한 관리
- Manual 모드: 리모콘 제어
- Auto 모드: UI/Navigator 제어
- 비상정지: 모든 모드에서 우선

### 7.4 Differential Drive
- 표준 kinematic 변환
- 오른쪽 모터 반전은 can_sender에서 처리

---

## 8. 다음 단계 (Phase 3)

### 목표
`rebar_control` 패키지 리팩토링 - 상위 제어 계층

### 구현할 노드
1. **navigator.py**
   - 경로 계획 및 작업 시퀀스 관리
   - PrecisionNav Action Client
   - TyingSequence Action Client

2. **precision_nav_controller.py**
   - Action Server로 변경
   - PrecisionNav.action 사용
   - Goal/Feedback/Result 구조

3. **tying_sequence_controller.py** (Optional)
   - TyingSequence Action Server
   - 결속 시퀀스 로직

### 삭제 예정 파일
- `cmd_vel_relay.py` (불필요한 중계 노드)

### 예상 소요 시간
2-3일

---

## 9. Git 커밋 이력

```
fa1b412 - feat(phase2): implement hardware abstraction layer (rebar_base_control)
c52f245 - feat(phase1): implement message interfaces
054fdd3 - backup: current development state before refactoring
```

---

## 10. 참고 사항

### 기존 코드 활용
Phase 2에서 참조한 기존 파일:
- `position_control_node.py` - CAN 통신 로직
- `iron_md_teleop_node.py` - 리모콘 파싱 로직
- `ezi_io_node.py` - Modbus TCP 로직
- `seengrip_node.py` - Modbus RTU 로직

이 파일들은 Phase 4에서 백업 후 삭제 예정

### 하드웨어 특성
- **Differential Drive:** 0x142 (오른쪽) 모터만 반전 필요
- **CAN Bus:** CAN2 (1Mbps), CAN3 (250kbps)
- **Modbus:** Seengrip (RTU, 115200), EZI-IO (TCP, 192.168.1.100)

---

**리팩토링 전체 계획:** `REBAR_REFACTORING_PLAN.md` 참조
**Phase 1 요약:** `PHASE1_SUMMARY.md` 참조
**Phase 2 시작 가이드:** `README_PHASE2.md` 참조
