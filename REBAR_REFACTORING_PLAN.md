# Rebar Control System Refactoring Plan

## 목표
Tire Roller의 검증된 아키텍처를 참고하여 현재 개발 중인 Rebar 제어 시스템을 계층화된 구조로 리팩토링

---

## ✅ 진행 상황

### Phase 1: 메시지 인터페이스 정의 - **완료** (2024-12-10)

**브랜치:** `refactoring/phase1-interfaces`
**커밋:** `c52f245 - feat(phase1): implement message interfaces`

#### 완료된 작업
- [x] Git 저장소 초기화 및 현재 상태 백업 (커밋: 054fdd3)
- [x] Phase 1 브랜치 생성 (`refactoring/phase1-interfaces`)
- [x] `rebar_base_interfaces` 패키지 생성
  - [x] DriveControl.msg
  - [x] RemoteControl.msg
  - [x] MotorFeedback.msg
  - [x] IOStatus.msg
  - [x] GripperControl.msg
  - [x] CMakeLists.txt, package.xml
- [x] `rebar_msgs` → `rebar_interfaces` 리팩토링
  - [x] RebarStatus.msg
  - [x] PrecisionNav.action
  - [x] TyingSequence.action
  - [x] SetMode.srv
  - [x] CMakeLists.txt, package.xml 업데이트
- [x] 빌드 테스트 성공 (13.2초, 2개 패키지)
- [x] 인터페이스 검증 완료 (9개 인터페이스)

#### 생성된 인터페이스
```bash
# Base interfaces (5개)
rebar_base_interfaces/msg/DriveControl
rebar_base_interfaces/msg/GripperControl
rebar_base_interfaces/msg/IOStatus
rebar_base_interfaces/msg/MotorFeedback
rebar_base_interfaces/msg/RemoteControl

# Control interfaces (4개)
rebar_interfaces/msg/RebarStatus
rebar_interfaces/action/PrecisionNav
rebar_interfaces/action/TyingSequence
rebar_interfaces/srv/SetMode
```

---

### Phase 2: 하드웨어 추상화 계층 - **완료** (2025-12-17)

**브랜치:** `refactoring/phase2-base-control`
**커밋:** `fa1b412 - feat(phase2): implement hardware abstraction layer (rebar_base_control)`

#### 완료된 작업
- [x] `rebar_base_control` 패키지 생성
  - [x] can_parser.py (CAN 메시지 파싱)
  - [x] can_sender.py (CAN 메시지 전송)
  - [x] drive_controller.py (cmd_vel → DriveControl)
  - [x] modbus_controller.py (Seengrip + EZI-IO)
  - [x] authority_controller.py (권한 관리)
  - [x] navigator_base.py (State Machine)
- [x] python-statemachine 설치 및 State Machine 구현
- [x] 설정 파일 작성 (can_devices.yaml, modbus_devices.yaml)
- [x] Launch 파일 작성 (base_system.launch.py)
- [x] 빌드 테스트 성공 (3.54초, 1개 패키지)

#### State Machine
- 상태: idle, manual, auto, navigating, tying, emergency_stop
- 전이: S10/S20 스위치, 모션 명령, 비상정지
- 라이브러리: python-statemachine

#### 다음 단계
Phase 3: 상위 제어 계층 (`rebar_control`) 리팩토링 예정

---

## 현재 시스템 분석

### 기존 구조의 문제점
1. **단일 노드에 과도한 책임**
   - `position_control_node.py`: CAN 통신 + 속도 계산 + PID 제어 + 모터 제어
   - 약 1500+ 라인의 복잡한 단일 파일

2. **계층 분리 부재**
   - 하드웨어 추상화 계층과 제어 로직이 혼재
   - 재사용성과 테스트 용이성 저하

3. **상태 관리 부재**
   - S10/S20 모드 전환 로직이 분산
   - 체계적인 상태 머신 없음

4. **임시 해결책 존재**
   - `cmd_vel_relay.py`: 단순 토픽 중계 노드
   - 임시 브릿지 코드

5. **설정 분산**
   - 여러 yaml 파일에 설정이 분산
   - 일관성 없는 설정 관리

### 현재 파일 구조
```
src/
├── rebar_control/
│   ├── iron_md_teleop_node.py      # 리모콘 + AN 명령 처리
│   ├── precision_navigation_node.py # 정밀 주행 제어
│   ├── cmd_vel_relay.py            # 임시 중계 노드
│   └── safety_monitor.py
├── rmd_robot_control/
│   └── position_control_node.py    # 모터 CAN 통신 + 제어
├── ezi_io_ros2/
│   └── ezi_io_node.py              # Modbus TCP (센서)
├── seengrip_ros2/
│   └── seengrip_node.py            # Modbus RTU (그리퍼)
└── rebar_msgs/
    └── msg/
        ├── PrecisionNavGoal.msg
        ├── PrecisionNavFeedback.msg
        └── PrecisionNavStatus.msg
```

---

## Tire Roller 아키텍처 분석

### 핵심 설계 원칙

#### 1. 2-Layer Architecture
```
┌─────────────────────────────────────────┐
│   High-level Control Layer              │
│   (tire_roller_control)                 │
│   - navigator                           │
│   - base_controller (Action Server)     │
│   - roller_controller                   │
│   - Algorithm modules                   │
└─────────────────────────────────────────┘
              ↓ Abstract Messages
┌─────────────────────────────────────────┐
│   Low-level Hardware Layer              │
│   (tire_roller_basecontrol)             │
│   - can_parser (RX)                     │
│   - can_sender (TX)                     │
│   - authority_controller                │
│   - drive_controller                    │
│   - navigator_base (State Machine)      │
└─────────────────────────────────────────┘
              ↓ Hardware Protocol
┌─────────────────────────────────────────┐
│   Hardware                              │
│   - CAN Bus, Modbus, etc                │
└─────────────────────────────────────────┘
```

#### 2. State Machine 기반 제어
- `python-statemachine` 라이브러리 사용
- 명확한 상태 전이 규칙
- 상태별 진입/퇴출 콜백

#### 3. Action Server/Client 패턴
- 장시간 작업에 Action 인터페이스 사용
- Goal/Feedback/Result 구조
- 취소 가능한 비동기 작업

#### 4. 메시지 타입 분리
- `roller_base_interfaces`: 하드웨어 계층용 메시지
- `roller_interfaces`: 상위 제어용 메시지 (Action 포함)

---

## 새로운 Rebar 시스템 아키텍처

### Package 구조

```
src/
├── rebar_base_interfaces/          # NEW: 하드웨어 계층 메시지
│   ├── msg/
│   │   ├── DriveControl.msg
│   │   ├── RemoteControl.msg
│   │   ├── MotorFeedback.msg
│   │   ├── GripperControl.msg
│   │   └── IOStatus.msg
│   └── CMakeLists.txt
│
├── rebar_interfaces/               # REFACTOR: 상위 제어 메시지
│   ├── msg/
│   │   ├── RebarStatus.msg
│   │   └── NavigationFeedback.msg
│   ├── action/
│   │   ├── PrecisionNav.action
│   │   └── TyingSequence.action
│   ├── srv/
│   │   └── SetMode.srv
│   └── CMakeLists.txt
│
├── rebar_base_control/             # NEW: 하드웨어 추상화 계층
│   ├── rebar_base_control/
│   │   ├── can_parser.py
│   │   ├── can_sender.py
│   │   ├── modbus_controller.py
│   │   ├── authority_controller.py
│   │   ├── drive_controller.py
│   │   └── navigator_base.py       # State Machine
│   ├── launch/
│   │   └── base_system.launch.py
│   ├── config/
│   │   ├── can_devices.yaml
│   │   └── modbus_devices.yaml
│   └── setup.py
│
├── rebar_control/                  # REFACTOR: 상위 제어 계층
│   ├── rebar_control/
│   │   ├── navigator.py            # 경로 계획 + 상태 관리
│   │   ├── precision_nav_controller.py  # Action Server
│   │   ├── tying_sequence_controller.py
│   │   └── utils/
│   │       ├── pid_controller.py
│   │       └── quaternion_utils.py
│   ├── launch/
│   │   └── control_system.launch.py
│   ├── config/
│   │   ├── precision_nav.yaml
│   │   └── navigation_params.yaml
│   └── setup.py
│
├── rebar_vision/                   # FUTURE: 비전 처리
│   └── (ZED 기반 철근 인식)
│
└── zed-ros2-wrapper/               # EXISTING: ZED 카메라
    └── (기존 유지)
```

---

## 세부 설계

### 1. rebar_base_control Package

#### 1.1 can_parser.py
**역할:** CAN 메시지 수신 → ROS2 메시지 변환

**구독:**
- (없음 - CAN 하드웨어에서 직접 수신)

**발행:**
- `/motor_feedback` (MotorFeedback)
- `/remote_control` (RemoteControl)

**주요 기능:**
```python
class CANParser(Node):
    def __init__(self):
        # CAN2 (1Mbps): 모터 피드백
        self.bus_motor = can.Bus('can2', bustype='socketcan', bitrate=1000000)
        # CAN3 (250kbps): 리모콘
        self.bus_remote = can.Bus('can3', bustype='socketcan', bitrate=250000)

    def parse_motor_feedback(self, msg):
        # 0x141, 0x142 피드백 파싱
        pass

    def parse_remote_control(self, msg):
        # 리모콘 명령 파싱
        pass
```

**마이그레이션:**
- `position_control_node.py`의 CAN 수신 로직
- `iron_md_teleop_node.py`의 리모콘 파싱 로직

---

#### 1.2 can_sender.py
**역할:** ROS2 메시지 → CAN 메시지 전송

**구독:**
- `/drive_control` (DriveControl)
- `/gripper_control` (GripperControl)

**발행:**
- (없음 - CAN 하드웨어로 직접 전송)

**주요 기능:**
```python
class CANSender(Node):
    def drive_control_callback(self, msg):
        # DriveControl → CAN 메시지 (0x141, 0x142)
        left_speed = msg.left_speed
        right_speed = msg.right_speed
        self.send_motor_command(0x141, left_speed)
        self.send_motor_command(0x142, right_speed)
```

**마이그레이션:**
- `position_control_node.py`의 CAN 전송 로직

---

#### 1.3 modbus_controller.py
**역할:** Modbus 통신 통합 관리

**구독:**
- `/gripper_command` (GripperControl)

**발행:**
- `/io_status` (IOStatus)
- `/gripper_status` (GripperStatus)

**주요 기능:**
```python
class ModbusController(Node):
    def __init__(self):
        # Seengrip (Modbus RTU)
        self.seengrip = ModbusSerialClient(port='/dev/ttyUSB0', baudrate=115200)
        # EZI-IO (Modbus TCP)
        self.ezi_io = ModbusTcpClient(host='192.168.1.100')

    def update_io_sensors(self):
        # EZI-IO 센서 읽기
        pass

    def control_gripper(self, msg):
        # Seengrip 제어
        pass
```

**마이그레이션:**
- `ezi_io_node.py` 전체
- `seengrip_node.py` 전체

---

#### 1.4 authority_controller.py
**역할:** 제어권한 관리 (Manual/Auto 전환)

**구독:**
- `/remote_control` (RemoteControl)
- `/control_mode` (String) - navigator_base에서 발행

**발행:**
- `/authority_status` (String)

**주요 기능:**
```python
class AuthorityController(Node):
    def __init__(self):
        self.current_mode = 'idle'  # idle, manual, auto

    def process_mode_switch(self):
        # S10 (Manual), S20 (Auto) 스위치 처리
        # S20 모드에서는 리모콘 명령 완전 무시
        if self.switch_s20_active:
            # UI 명령만 수신, 리모콘 무시
            pass
```

**마이그레이션:**
- `iron_md_teleop_node.py`의 모드 전환 로직

---

#### 1.5 drive_controller.py
**역할:** 상위 속도 명령 → 모터 제어 명령 변환

**구독:**
- `/cmd_vel` (Twist)

**발행:**
- `/drive_control` (DriveControl)

**주요 기능:**
```python
class DriveController(Node):
    def cmd_vel_callback(self, msg):
        # Twist → Differential Drive 변환
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive kinematics
        left_speed = linear - angular * wheel_base / 2
        right_speed = linear + angular * wheel_base / 2

        # 오른쪽 모터만 반전 (하드웨어 특성)
        right_speed = -right_speed

        drive_msg = DriveControl()
        drive_msg.left_speed = left_speed
        drive_msg.right_speed = right_speed
        self.pub.publish(drive_msg)
```

**마이그레이션:**
- `position_control_node.py`의 속도 변환 로직만 분리

---

#### 1.6 navigator_base.py
**역할:** State Machine 기반 상태 관리

**구독:**
- `/remote_control` (RemoteControl)
- `/rebar_motion_cmd` (String) - UI에서 명령

**발행:**
- `/control_mode` (String)

**주요 기능:**
```python
from statemachine import State, StateMachine

class RebarStateMachine(StateMachine):
    idle = State(initial=True)
    manual = State()       # S10 모드
    auto = State()         # S20 모드
    navigating = State()
    tying = State()
    e_stop = State()

    to_manual = idle.to(manual) | auto.to(manual)
    to_auto = manual.to(auto)
    to_nav = auto.to(navigating)
    to_tying = navigating.to(tying)
    to_estop = (manual | auto | navigating | tying).to(e_stop)

    def on_enter_auto(self):
        self.navigator.get_logger().info('Entered AUTO mode (S20)')

    def on_enter_manual(self):
        self.navigator.get_logger().info('Entered MANUAL mode (S10)')

class NavigatorBase(Node):
    def __init__(self):
        self.sm = RebarStateMachine(self)

    def manage_state(self):
        # 리모콘 스위치 상태에 따라 상태 전이
        if self.remote.switch_s20:
            self.sm.to_auto()
        elif self.remote.switch_s10:
            self.sm.to_manual()
        elif self.remote.emergency_stop:
            self.sm.to_estop()
```

**마이그레이션:**
- 새로 구현 (tire_roller의 navigator.py 참고)

---

### 2. rebar_control Package

#### 2.1 navigator.py
**역할:** 경로 계획 및 작업 시퀀스 관리

**구독:**
- `/rebar_motion_cmd` (String) - UI에서 명령
- `/zed/zed_node/odom` (Odometry)
- `/control_mode` (String)

**발행:**
- `/rebar_motion_result` (String)

**Action Client:**
- `/precision_nav` (PrecisionNav.action)
- `/tying_sequence` (TyingSequence.action)

**주요 기능:**
```python
class Navigator(Node):
    def __init__(self):
        self.nav_client = ActionClient(self, PrecisionNav, 'precision_nav')
        self.state = 'idle'

    def receive_motion_cmd(self, msg):
        if msg.data == 'MOVE_FORWARD_600':
            self.send_nav_goal(distance=0.6, direction='forward')
        elif msg.data == 'MOVE_BACKWARD_600':
            self.send_nav_goal(distance=0.6, direction='backward')
        elif msg.data == 'START_TYING':
            self.send_tying_goal()

    def send_nav_goal(self, distance, direction):
        goal = PrecisionNav.Goal()
        goal.distance = distance
        goal.direction = direction
        self.nav_client.send_goal_async(goal)
```

**마이그레이션:**
- 새로 구현 (현재는 UI에서 직접 토픽 발행)

---

#### 2.2 precision_nav_controller.py
**역할:** 정밀 주행 제어 (Action Server)

**구독:**
- `/zed/zed_node/imu/data` (Imu)
- `/zed/zed_node/odom` (Odometry)

**발행:**
- `/cmd_vel` (Twist)

**Action Server:**
- `/precision_nav` (PrecisionNav.action)

**주요 기능:**
```python
class PrecisionNavController(Node):
    def __init__(self):
        self.action_server = ActionServer(
            self, PrecisionNav, 'precision_nav',
            execute_callback=self.execute_callback
        )
        self.heading_pid = PIDController(kp=0.5, ki=0.0, kd=0.1)
        self.distance_pid = PIDController(kp=0.003, ki=0.0, kd=0.0001)

    async def execute_callback(self, goal_handle):
        feedback = PrecisionNav.Feedback()

        while not self.reached_goal():
            # PID 제어
            heading_correction = self.heading_pid.update(self.heading_error)
            speed = self.distance_pid.update(self.distance_error)

            # cmd_vel 발행
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = heading_correction
            self.cmd_vel_pub.publish(twist)

            # Feedback 전송
            feedback.current_distance = self.traveled_distance
            feedback.remaining_distance = goal_handle.request.distance - self.traveled_distance
            goal_handle.publish_feedback(feedback)

            await asyncio.sleep(0.05)

        # 완료
        result = PrecisionNav.Result()
        result.success = True
        return result
```

**마이그레이션:**
- `precision_navigation_node.py` 전체를 Action Server로 변경
- `cmd_vel_relay.py` 삭제 (불필요)

---

#### 2.3 tying_sequence_controller.py
**역할:** 철근 결속 시퀀스 제어

**Action Server:**
- `/tying_sequence` (TyingSequence.action)

**Action Client:**
- `/precision_nav` (PrecisionNav.action)

**Service Client:**
- `/gripper_control` (GripperControl service)

**주요 기능:**
```python
class TyingSequenceController(Node):
    async def execute_tying_sequence(self, goal_handle):
        # 1. 위치 이동
        await self.move_to_position()

        # 2. 그리퍼 열기
        await self.open_gripper()

        # 3. 와이어 걸기
        await self.engage_wire()

        # 4. 결속
        await self.tie()

        # 5. 완료
        return TyingSequence.Result(success=True)
```

**마이그레이션:**
- 새로 구현 (미래 기능)

---

### 3. Message/Action 정의

#### 3.1 rebar_base_interfaces

**DriveControl.msg**
```
float32 left_speed   # m/s
float32 right_speed  # m/s
```

**RemoteControl.msg**
```
bool switch_s10      # Manual mode
bool switch_s20      # Auto mode
bool emergency_stop
uint8[] buttons
float32[] joysticks
```

**MotorFeedback.msg**
```
uint8 motor_id       # 0x141, 0x142
float32 current_speed
float32 current_position
uint8 error_code
```

**IOStatus.msg**
```
bool[] digital_inputs
float32[] analog_inputs
```

**GripperControl.msg**
```
uint8 command        # OPEN=1, CLOSE=2, STOP=0
uint8 speed
```

---

#### 3.2 rebar_interfaces

**PrecisionNav.action**
```
# Goal
float32 distance        # 이동 거리 (m)
string direction        # "forward" or "backward"
float32 max_velocity    # 최대 속도 (m/s)

---
# Result
bool success
float32 final_error     # 최종 위치 오차 (m)

---
# Feedback
float32 current_distance
float32 remaining_distance
float32 heading_error
```

**TyingSequence.action**
```
# Goal
uint8 mode              # SINGLE=1, CONTINUOUS=2

---
# Result
bool success
uint8 tied_count

---
# Feedback
string current_step
float32 progress        # 0.0 ~ 1.0
```

**RebarStatus.msg**
```
string control_mode     # "idle", "manual", "auto", "navigating", "tying"
float32 battery_voltage
bool gripper_ready
bool navigation_ready
geometry_msgs/Pose2D current_pose
```

---

### 4. Launch 파일

#### 4.1 base_system.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # CAN 통신
        Node(
            package='rebar_base_control',
            executable='can_parser',
            name='can_parser'
        ),
        Node(
            package='rebar_base_control',
            executable='can_sender',
            name='can_sender'
        ),

        # Modbus 통신
        Node(
            package='rebar_base_control',
            executable='modbus_controller',
            name='modbus_controller',
            parameters=[{'config': 'config/modbus_devices.yaml'}]
        ),

        # 제어권한 관리
        Node(
            package='rebar_base_control',
            executable='authority_controller',
            name='authority_controller'
        ),

        # 주행 제어
        Node(
            package='rebar_base_control',
            executable='drive_controller',
            name='drive_controller'
        ),

        # 상태 머신
        Node(
            package='rebar_base_control',
            executable='navigator_base',
            name='navigator_base'
        ),
    ])
```

#### 4.2 control_system.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Base system
        IncludeLaunchDescription('base_system.launch.py'),

        # Navigator
        Node(
            package='rebar_control',
            executable='navigator',
            name='navigator'
        ),

        # Precision navigation controller
        Node(
            package='rebar_control',
            executable='precision_nav_controller',
            name='precision_nav_controller',
            parameters=[{'config': 'config/precision_nav.yaml'}]
        ),

        # ZED Camera
        IncludeLaunchDescription('zed_camera.launch.py'),
    ])
```

---

## 마이그레이션 체크리스트

### Phase 1: 메시지 인터페이스 정의 ✅ **완료 (2024-12-10)**
- [x] `rebar_base_interfaces` 패키지 생성
  - [x] `DriveControl.msg`
  - [x] `RemoteControl.msg`
  - [x] `MotorFeedback.msg`
  - [x] `IOStatus.msg`
  - [x] `GripperControl.msg`
  - [x] CMakeLists.txt 작성
  - [x] package.xml 작성
  - [x] 빌드 테스트

- [x] `rebar_interfaces` 패키지 리팩토링
  - [x] `PrecisionNav.action` 작성
  - [x] `TyingSequence.action` 작성
  - [x] `RebarStatus.msg` 작성
  - [x] `SetMode.srv` 작성
  - [x] 기존 메시지 삭제 (PrecisionNavGoal.msg 등)
  - [x] 빌드 테스트

### Phase 2: 하드웨어 추상화 계층 (rebar_base_control) ✅ **완료 (2025-12-17)**
- [x] `rebar_base_control` 패키지 생성
  - [x] 디렉토리 구조 생성
  - [x] setup.py 작성
  - [x] package.xml 작성

- [x] `can_parser.py` 구현
  - [x] CAN2 모터 피드백 파싱 (0x241, 0x242)
  - [x] CAN3 리모콘 파싱 (0x1E4, 0x2E4, 0x764)
  - [x] MotorFeedback 메시지 발행
  - [x] RemoteControl 메시지 발행
  - [x] `position_control_node.py`, `iron_md_teleop_node.py` 참조

- [x] `can_sender.py` 구현
  - [x] DriveControl 구독
  - [x] CAN 메시지 전송 (0x141, 0x142)
  - [x] 속도 제어 명령 (0xA2)
  - [x] 오른쪽 모터 반전
  - [x] `position_control_node.py` 참조

- [x] `modbus_controller.py` 구현
  - [x] Seengrip Modbus RTU 통신
  - [x] EZI-IO Modbus TCP 통신
  - [x] GripperControl 구독
  - [x] IOStatus 발행 (10Hz)
  - [x] `ezi_io_node.py`, `seengrip_node.py` 참조

- [x] `authority_controller.py` 구현
  - [x] RemoteControl 구독
  - [x] S10/S20 모드 전환 로직
  - [x] 비상정지 처리
  - [x] authority_status 발행 (5Hz)
  - [x] `iron_md_teleop_node.py` 참조

- [x] `drive_controller.py` 구현
  - [x] `/cmd_vel` 구독
  - [x] Differential drive kinematics 변환
  - [x] DriveControl 발행
  - [x] 속도 제한
  - [x] `position_control_node.py` 참조

- [x] `navigator_base.py` 구현
  - [x] `python-statemachine` 설치
  - [x] RebarStateMachine 클래스 작성
  - [x] 6개 상태 정의 (idle, manual, auto, navigating, tying, emergency_stop)
  - [x] 상태 전이 로직
  - [x] control_mode 발행 (5Hz)

- [x] 설정 파일 작성
  - [x] `config/can_devices.yaml`
  - [x] `config/modbus_devices.yaml`

- [x] Launch 파일 작성
  - [x] `launch/base_system.launch.py` (6개 노드)

- [x] 빌드 테스트
  - [x] 빌드 성공 (3.54초)

### Phase 3: 상위 제어 계층 (rebar_control)
- [ ] `rebar_control` 리팩토링
  - [ ] 불필요한 파일 삭제 (`cmd_vel_relay.py`)

- [ ] `navigator.py` 구현
  - [ ] `/rebar_motion_cmd` 구독
  - [ ] `/rebar_motion_result` 발행
  - [ ] PrecisionNav Action Client
  - [ ] TyingSequence Action Client
  - [ ] 작업 시퀀스 관리 로직
  - [ ] 단위 테스트 작성

- [ ] `precision_nav_controller.py` 리팩토링
  - [ ] Action Server 구조로 변경
  - [ ] PrecisionNav.action 사용
  - [ ] Goal/Feedback/Result 처리
  - [ ] Heading PID 유지
  - [ ] Distance PID 유지
  - [ ] 취소 기능 구현
  - [ ] 단위 테스트 작성
  - [ ] `precision_navigation_node.py`에서 코드 마이그레이션

- [ ] `tying_sequence_controller.py` 구현 (Future)
  - [ ] TyingSequence Action Server
  - [ ] 시퀀스 로직 구현
  - [ ] 단위 테스트 작성

- [ ] 유틸리티 모듈 작성
  - [ ] `utils/pid_controller.py`
  - [ ] `utils/quaternion_utils.py`

- [ ] 설정 파일 작성
  - [ ] `config/precision_nav.yaml`
  - [ ] `config/navigation_params.yaml`

- [ ] Launch 파일 작성
  - [ ] `launch/control_system.launch.py`

- [ ] 통합 테스트
  - [ ] 정밀 주행 테스트 (600mm)
  - [ ] 위치 오차 측정
  - [ ] PID 튜닝

### Phase 4: 통합 및 검증
- [ ] 전체 시스템 Launch 파일
  - [ ] `launch/rebar_system.launch.py` (모든 노드 통합)

- [ ] 기존 스크립트 업데이트
  - [ ] `integrated_control_debug.sh` 수정
  - [ ] 새로운 launch 파일 사용

- [ ] 문서화
  - [ ] 아키텍처 다이어그램
  - [ ] 각 노드 README
  - [ ] 메시지/액션 인터페이스 문서
  - [ ] 설정 파라미터 가이드

- [ ] 성능 테스트
  - [ ] 600mm 전진 정확도 (목표: ±10mm)
  - [ ] 600mm 후진 정확도 (목표: ±10mm)
  - [ ] Heading 유지 성능
  - [ ] 응답 지연 시간 측정

- [ ] 기존 코드 정리
  - [ ] `position_control_node.py` 백업 후 삭제
  - [ ] `iron_md_teleop_node.py` 백업 후 삭제
  - [ ] `precision_navigation_node.py` 백업 후 삭제
  - [ ] `cmd_vel_relay.py` 삭제
  - [ ] `ezi_io_node.py` 백업 후 삭제
  - [ ] `seengrip_node.py` 백업 후 삭제

### Phase 5: 추가 기능 (Optional)
- [ ] RQT UI 개발
  - [ ] S20 모드 제어 패널
  - [ ] 상태 모니터링
  - [ ] 수동 명령 입력

- [ ] 로깅 및 모니터링
  - [ ] 주행 데이터 기록
  - [ ] 에러 로그 수집
  - [ ] 성능 메트릭 분석

- [ ] Vision 시스템 (rebar_vision)
  - [ ] ZED 기반 철근 인식
  - [ ] 위치 보정 알고리즘

---

## 예상 개발 일정

| Phase | 작업 내용 | 예상 공수 |
|-------|----------|----------|
| Phase 1 | 메시지 인터페이스 정의 | 1일 |
| Phase 2 | 하드웨어 추상화 계층 | 3-4일 |
| Phase 3 | 상위 제어 계층 | 2-3일 |
| Phase 4 | 통합 및 검증 | 2-3일 |
| Phase 5 | 추가 기능 (Optional) | 3-5일 |

**총 예상 개발 기간:** 약 2주

---

## 리팩토링 시 주의사항

### 1. 점진적 마이그레이션
- 한 번에 모든 코드를 변경하지 말고 단계적으로 진행
- 각 Phase 완료 후 반드시 통합 테스트 수행
- 기존 코드는 백업 후 삭제

### 2. 하드웨어 특성 유지
- **Differential Drive 모터 방향:**
  - 0x141 (왼쪽): 그대로
  - 0x142 (오른쪽): 반전 필요 (모터가 서로 마주보고 장착)
- **CAN 버스:**
  - CAN2 (1Mbps): 모터 통신
  - CAN3 (250kbps): 리모콘
- **Modbus:**
  - Seengrip: RTU, 115200 baud, /dev/ttyUSB0
  - EZI-IO: TCP, 192.168.1.100

### 3. 기존 PID 파라미터 유지
```yaml
# precision_nav.yaml
heading_pid:
  kp: 0.5
  ki: 0.0
  kd: 0.1

distance_pid:
  kp: 0.003
  ki: 0.0
  kd: 0.0001
```

### 4. 의존성 추가
```bash
# python-statemachine 설치
pip3 install python-statemachine
```

### 5. 테스트 우선
- 각 노드 구현 시 단위 테스트 작성
- 통합 전 개별 노드 동작 검증
- 실제 하드웨어 테스트 전 시뮬레이션 테스트

---

## 기대 효과

### 1. 코드 품질 향상
- 계층화된 구조로 유지보수성 향상
- 단일 책임 원칙 준수
- 테스트 용이성 증가

### 2. 재사용성 증가
- 하드웨어 계층과 제어 계층 분리
- 다른 로봇 프로젝트에도 활용 가능
- 모듈 단위 교체 용이

### 3. 확장성 향상
- 새로운 기능 추가 용이
- Vision 시스템 통합 준비
- RQT UI 개발 기반 마련

### 4. 안정성 향상
- State Machine 기반 안전한 상태 관리
- Action Server의 취소 기능
- 명확한 에러 처리

---

## 참고 자료

### Tire Roller 코드 위치
```
/home/koceti/ros2_ws/src/tire_roller/
├── tire_roller_basecontrol/
│   ├── authority_controller.py
│   ├── drive_controller.py
│   └── navigator.py (State Machine)
└── tire_roller_control/
    ├── navigator.py (High-level)
    └── base_controller.py
```

### 현재 Rebar 코드 위치
```
/home/koceti/ros2_ws/src/
├── rebar_control/
├── rmd_robot_control/
├── ezi_io_ros2/
└── seengrip_ros2/
```

### 핵심 파일
- **State Machine 예제:** `tire_roller_basecontrol/navigator.py`
- **Action Server 예제:** `tire_roller_control/base_controller.py`
- **CAN 통신 예제:** `tire_roller_basecontrol/authority_controller.py`

---

## 문의 및 지원

리팩토리 과정에서 문제 발생 시:
1. 각 Phase별 체크리스트 확인
2. Tire Roller 코드 참고
3. 단위 테스트로 문제 격리
4. 로그 분석 및 디버깅

**리팩토링 완료 목표:** 안정적이고 확장 가능한 Rebar 제어 시스템 구축
