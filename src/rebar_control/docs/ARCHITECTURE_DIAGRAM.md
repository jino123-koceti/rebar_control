# Rebar 제어 시스템 아키텍처

## 전체 시스템 구조

```mermaid
graph TB
    subgraph "UI Layer"
        UI[Laptop UI<br/>Zenoh]
    end

    subgraph "상위 제어 계층 (rebar_control)"
        ZC[zenoh_client]
        NAV[navigator]
        RC[rebar_controller]
        RP[rebar_publisher]

        subgraph "Pose Source"
            PM[pose_mux<br/>듀얼 ZED]
            O2P[odom_to_pose<br/>단일 ZED]
        end
    end

    subgraph "하드웨어 추상화 계층 (rebar_base_control)"
        DC[drive_controller]
        JC[joint_controller]
        CP[can_parser]
        CS[can_sender]
        MC[modbus_controller]
        AC[authority_controller]
        NB[navigator_base]
    end

    subgraph "Sensors"
        ZED1[ZED Front]
        ZED2[ZED Back]
        ZED_SINGLE[ZED X<br/>단일]
        REMOTE[Remote Control<br/>CAN3]
    end

    subgraph "Actuators"
        MOTOR_L[Left Wheel<br/>0x141]
        MOTOR_R[Right Wheel<br/>0x142]
        MOTOR_LAT[Lateral<br/>0x143]
        MOTOR_J[Joint Motors<br/>0x144-0x147]
        GRIPPER[Seengrip]
        IO[EZI-IO]
    end

    %% UI Connections
    UI -->|Zenoh| ZC
    ZC -->|/mission/command| NAV
    RP -->|/mission/status| ZC
    ZC -->|Zenoh| UI

    %% Control Flow (Auto Mode)
    NAV -->|/mission/target_pose| RC
    RC -->|/cmd_vel| DC

    %% Pose Source Selection (Dual ZED)
    ZED1 -->|/zed_front/odom| PM
    ZED2 -->|/zed_back/odom| PM
    PM -->|/robot_pose| RC
    PM -.->|/robot_pose| RP

    %% Pose Source Selection (Single ZED)
    ZED_SINGLE -->|/zed/odom| O2P
    O2P -->|/robot_pose| RC
    O2P -.->|/robot_pose| RP

    %% Remote Control (Manual Mode)
    REMOTE -->|CAN3| CP
    CP -->|/remote_control| DC

    %% Drive Control
    DC -->|/drive_control| CS
    CS -->|CAN2| MOTOR_L
    CS -->|CAN2| MOTOR_R
    CS -->|CAN2| MOTOR_LAT

    %% Joint Control
    NAV -->|/joint_control| JC
    JC -->|CAN2 0xA4| MOTOR_J

    %% Modbus Control
    NAV -->|/gripper_control| MC
    MC -->|Modbus RTU| GRIPPER
    MC -->|Modbus TCP| IO

    %% State Machine
    NB -->|/control_mode| NAV
    AC -->|/e_stop| NB

    %% Feedback
    CP -->|/motor_feedback| RP
    MC -->|/io_status| RP

    style PM fill:#e1f5ff
    style O2P fill:#ffe1f5
    style ZC fill:#fff4e1
    style NAV fill:#e1ffe1
    style RC fill:#ffe1e1
```

---

## Pose Source 선택 로직

### 듀얼 ZED 환경 (기본)

```mermaid
graph LR
    ZF[ZED Front<br/>/zed_front/odom]
    ZB[ZED Back<br/>/zed_back/odom]
    CV[/cmd_vel<br/>방향 신호]

    PM[pose_mux]
    OUT[/robot_pose]

    ZF --> PM
    ZB --> PM
    CV --> PM

    PM -->|선택된 Odometry| OUT

    style PM fill:#e1f5ff
```

**선택 기준:**
- `/cmd_vel.linear.x > 0` → Front ZED 사용
- `/cmd_vel.linear.x < 0` → Back ZED 사용
- 정지 시 → Front ZED 우선 (파라미터 설정 가능)

### 단일 ZED 환경

```mermaid
graph LR
    ZED[ZED X<br/>/zed/odom]
    O2P[odom_to_pose]
    OUT[/robot_pose]

    ZED -->|Odometry| O2P
    O2P -->|PoseStamped<br/>변환| OUT

    style O2P fill:#ffe1f5
```

**특징:**
- 단순 메시지 타입 변환 (Odometry → PoseStamped)
- 타임스탬프 유지
- 최소 지연 (< 1ms)

---

## 제어 모드별 데이터 플로우

### Manual 모드 (리모콘 제어)

```mermaid
sequenceDiagram
    participant Remote as 리모콘 (CAN3)
    participant CP as can_parser
    participant DC as drive_controller
    participant CS as can_sender
    participant Motor as 모터 (CAN2)

    Remote->>CP: CAN 조이스틱 메시지
    CP->>DC: /remote_control (RemoteControl)
    DC->>DC: Kinematics 계산<br/>(Diff Drive)
    DC->>CS: /drive_control (DriveControl)
    CS->>Motor: CAN 0x141, 0x142, 0x143
```

### Auto 모드 (자동 주행)

```mermaid
sequenceDiagram
    participant UI as Laptop UI
    participant ZC as zenoh_client
    participant NAV as navigator
    participant RC as rebar_controller
    participant ZED as ZED Camera
    participant O2P as odom_to_pose
    participant DC as drive_controller
    participant Motor as 모터

    UI->>ZC: Zenoh WAYPOINTS
    ZC->>NAV: /mission/command
    NAV->>NAV: 웨이포인트 로드
    NAV->>RC: /mission/target_pose

    loop 20Hz Control Loop
        ZED->>O2P: /zed/odom
        O2P->>RC: /robot_pose
        RC->>RC: PID 제어
        RC->>DC: /cmd_vel
        DC->>Motor: CAN 명령
    end

    RC->>NAV: /mission/waypoint_reached
    NAV->>NAV: 다음 웨이포인트
```

---

## 토픽 맵

### 상위 제어 계층 (rebar_control)

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/robot_pose` | PoseStamped | pose_mux 또는 odom_to_pose | rebar_controller, rebar_publisher, zenoh_client | 로봇 현재 위치 |
| `/mission/command` | String | zenoh_client | navigator | UI 명령 (E-STOP, START, WAYPOINTS 등) |
| `/mission/target_pose` | PoseStamped | navigator | rebar_controller | 현재 목표 위치 |
| `/mission/feedback` | String | navigator | rebar_publisher | 미션 진행 상황 |
| `/mission/waypoint_reached` | String | rebar_controller | navigator | 목표 도달 알림 |
| `/mission/status` | String (JSON) | rebar_publisher | zenoh_client | 통합 상태 정보 |
| `/cmd_vel` | Twist | rebar_controller | drive_controller | 속도 명령 |

### 하드웨어 추상화 계층 (rebar_base_control)

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/drive_control` | DriveControl | drive_controller | can_sender | 주행 모터 제어 |
| `/joint_control` | JointControl | (상위) | joint_controller | 관절 모터 제어 |
| `/remote_control` | RemoteControl | can_parser | drive_controller | 리모콘 입력 |
| `/motor_feedback` | MotorFeedback | can_parser | (상위) | 모터 피드백 |
| `/control_mode` | String | navigator_base | navigator | 제어 모드 (manual/auto/emergency) |

---

## Launch 파일 구조

### control_system.launch.py

```mermaid
graph TD
    START[control_system.launch.py]

    ARG1{use_dual_zed<br/>파라미터}
    ARG2[single_zed_odom_topic<br/>파라미터]

    PM[pose_mux 노드<br/>condition: use_dual_zed==true]
    O2P[odom_to_pose 노드<br/>condition: use_dual_zed==false]

    ZC[zenoh_client]
    NAV[navigator]
    RC[rebar_controller]
    RP[rebar_publisher]

    START --> ARG1
    START --> ARG2

    ARG1 -->|true| PM
    ARG1 -->|false| O2P

    START --> ZC
    START --> NAV
    START --> RC
    START --> RP

    style PM fill:#e1f5ff
    style O2P fill:#ffe1f5
```

**실행 예시:**

```bash
# 듀얼 ZED (기본)
ros2 launch rebar_control control_system.launch.py

# 단일 ZED
ros2 launch rebar_control control_system.launch.py use_dual_zed:=false

# 단일 ZED (커스텀 토픽)
ros2 launch rebar_control control_system.launch.py \
  use_dual_zed:=false \
  single_zed_odom_topic:=/custom/zed/odom
```

---

## State Machine 통합

### navigator_base (하드웨어 계층)

```mermaid
stateDiagram-v2
    [*] --> idle
    idle --> manual: 리모콘 입력
    idle --> auto: AUTO 명령

    manual --> idle: IDLE 명령
    auto --> navigating: 미션 시작

    navigating --> auto: 미션 완료
    auto --> idle: IDLE 명령

    idle --> emergency_stop: E-STOP
    manual --> emergency_stop: E-STOP
    auto --> emergency_stop: E-STOP
    navigating --> emergency_stop: E-STOP

    emergency_stop --> idle: RESET
```

### navigator (상위 계층)

```mermaid
stateDiagram-v2
    [*] --> idle
    idle --> planning: WAYPOINTS 수신
    planning --> navigating: 미션 시작

    navigating --> navigating: 웨이포인트 순회
    navigating --> mission_done: 마지막 웨이포인트 도달

    mission_done --> idle: RESET

    idle --> emergency_stop: E-STOP
    planning --> emergency_stop: E-STOP
    navigating --> emergency_stop: E-STOP

    emergency_stop --> idle: RESET
```

---

## 파일 구조

```
rebar_control/
├── rebar_control/
│   ├── zenoh_client.py          # UI 통신
│   ├── navigator.py              # 미션 관리
│   ├── rebar_controller.py       # 경로 추종
│   ├── rebar_publisher.py        # 상태 발행
│   ├── pose_mux.py               # 듀얼 ZED 처리
│   └── odom_to_pose.py          # 단일 ZED 처리 ⭐ NEW
│
├── launch/
│   ├── control_system.launch.py  # 상위 제어 계층 (수정됨)
│   └── full_system.launch.py     # 전체 시스템 (TODO)
│
├── config/
│   └── zenoh_config.yaml
│
├── docs/
│   ├── ODOM_TO_POSE_NODE.md     # odom_to_pose 문서 ⭐ NEW
│   └── ARCHITECTURE_DIAGRAM.md   # 아키텍처 다이어그램 ⭐ NEW
│
└── setup.py                      # odom_to_pose entry_point 추가됨
```

---

## 다음 단계

### Phase 3 완료를 위한 작업

1. **full_system.launch.py 작성**
   - `base_system.launch.py` include
   - `control_system.launch.py` include
   - 전체 시스템 한 번에 실행

2. **ZED X 실제 연동 테스트**
   - ZED 카메라 실행
   - odom_to_pose 통합 테스트
   - rebar_controller 동작 확인

3. **navigator PAUSE/RESUME 기능**
   - State Machine 확장
   - /mission/command 명령 추가

4. **성능 테스트**
   - 600mm 주행 정확도 측정
   - PID 파라미터 튜닝
   - 응답 지연 측정

---

**작성일**: 2025-12-23
**작성자**: Claude Code (Sonnet 4.5)
