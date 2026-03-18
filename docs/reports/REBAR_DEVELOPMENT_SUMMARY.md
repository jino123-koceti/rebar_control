# Rebar 제어 시스템 개발 현황 및 Claude 작업 가이드

> 작성일: 2025-12-23 (최종 업데이트: 2025-12-26)
> 프로젝트: 철근 결속 자동화 로봇 제어 시스템

---

## 1. 프로젝트 개요

### 1.1 목적
철근 결속 자동화 로봇의 제어 시스템을 Tire Roller의 검증된 아키텍처를 참고하여 **2계층 구조**로 리팩토링

### 1.2 아키텍처 설계
```
┌─────────────────────────────────────────────────────────────┐
│                     상위 제어 계층                             │
│                   (rebar_control)                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────────┐  ┌──────────┐ │
│  │ zenoh_   │  │navigator │  │   rebar_     │  │ rebar_   │ │
│  │ client   │  │          │  │  controller  │  │publisher │ │
│  └──────────┘  └──────────┘  └──────────────┘  └──────────┘ │
└─────────────────────────────────────────────────────────────┘
                              ▲ ▼
                    ROS2 Topics (cmd_vel, etc.)
                              ▲ ▼
┌─────────────────────────────────────────────────────────────┐
│                 하드웨어 추상화 계층                           │
│                 (rebar_base_control)                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐ │
│  │   can_   │  │  drive_  │  │  joint_  │  │   modbus_    │ │
│  │  parser  │  │controller│  │controller│  │  controller  │ │
│  └──────────┘  └──────────┘  └──────────┘  └──────────────┘ │
│  ┌──────────┐  ┌──────────────────────────────────────────┐ │
│  │authority │  │       navigator_base                     │ │
│  │controller│  │       (State Machine)                    │ │
│  └──────────┘  └──────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              ▲ ▼
                      하드웨어 (CAN, Modbus)
```

---

## 2. 패키지 구조

### 2.1 rebar_base_interfaces
**타입:** ROS2 Message 패키지 (CMake)
**역할:** 하드웨어 계층 메시지 정의

#### 메시지 정의
```
msg/
├── DriveControl.msg      # 주행 모터 제어 (left/right/lateral speeds)
├── JointControl.msg      # 관절 모터 제어 (위치 제어)
├── Waypoint.msg          # Enhanced 웨이포인트 (motion type 포함) ✅ NEW
├── MotorFeedback.msg     # 모터 피드백
├── RemoteControl.msg     # 리모콘 입력
├── IOStatus.msg          # I/O 상태
└── GripperControl.msg    # 그리퍼 제어
```

**주요 메시지:**
- `DriveControl.msg`: Differential drive (0x141, 0x142) + Lateral (0x143)
  - `float32 left_speed` (m/s)
  - `float32 right_speed` (m/s)
  - `float32 lateral_speed` (dps)

- `JointControl.msg`: 위치 제어 모터 (0x143~0x147)
  - `uint16 joint_id`
  - `float32 position` (degree)
  - `float32 velocity` (degree/s)
  - `uint8 control_mode` (absolute/relative)

- `Waypoint.msg`: 복합 네비게이션용 웨이포인트 ✅ NEW
  - `float32 x, y` (meter)
  - `uint8 motion_type` (DIFFERENTIAL=0, LATERAL=1, HYBRID=2)
  - `float32 max_speed` (m/s for differential, dps for lateral)

---

### 2.2 rebar_interfaces
**타입:** ROS2 Message/Action/Service 패키지 (CMake)
**역할:** 상위 계층 메시지, 액션, 서비스 정의

#### 인터페이스 정의
```
msg/
└── RebarStatus.msg       # 로봇 상태 정보

action/
├── PrecisionNav.action   # 정밀 네비게이션 액션
└── TyingSequence.action  # 철근 결속 시퀀스 액션

srv/
└── SetMode.srv           # 모드 전환 서비스
```

---

### 2.3 rebar_base_control
**타입:** Python ROS2 패키지
**역할:** 하드웨어 추상화 계층 (Hardware Abstraction Layer)

#### 노드 구성
```
rebar_base_control/
├── can_parser.py              # CAN 메시지 수신 및 파싱
├── can_sender.py              # CAN 메시지 송신
├── drive_controller.py        # 주행 제어 (리모콘/cmd_vel → DriveControl)
├── joint_controller.py        # 관절 모터 제어 (JointControl → CAN)
├── modbus_controller.py       # Modbus 장비 제어 (Seengrip, EZI-IO)
├── authority_controller.py    # 권한 관리 및 안전 감시
├── navigator_base.py          # 베이스 상태 머신 (State Machine)
└── velocity_profiler.py       # 속도 프로파일 생성 (가감속)
```

#### 설정 파일
```
config/
├── can_devices.yaml           # CAN 장비 설정 (모터 ID, 엔코더 CPR 등)
└── modbus_devices.yaml        # Modbus 장비 설정
```

#### Launch 파일
```
launch/
└── base_system.launch.py      # 하드웨어 계층 모든 노드 실행
```

#### 주요 기능
1. **CAN 통신**
   - CAN2 (1Mbps): 모터 통신 (0x141, 0x142, 0x143~0x147)
   - CAN3 (250kbps): 리모콘 통신

2. **State Machine** (navigator_base.py)
   - States: `idle`, `manual`, `auto`, `navigating`, `tying`, `emergency_stop`
   - python-statemachine 라이브러리 사용

3. **드라이브 제어** (drive_controller.py)
   - Manual 모드: 리모콘 조이스틱 → DriveControl
   - Auto 모드: /cmd_vel (Twist) → DriveControl
   - Differential drive kinematics 적용
   - 20Hz 주기적 발행 (Tire Roller 방식)

4. **관절 제어** (joint_controller.py) ✅ UPDATED
   - **Manual 모드**: 리모콘 S17/S18 → 횡이동 제어
   - **Auto 모드**: `/joint_control_cmd` 구독 → CAN 0xA4 명령 전송
   - 위치 제어 (절대/상대)
   - Lateral (0x143): 360도 단위, 200 dps, 50mm/rotation
   - Yaw (0x147): 5도 단위, 134 dps
   - 홈 캘리브레이션: 0x94 기준 154.94° (12시 방향)

---

### 2.4 rebar_control
**타입:** Python ROS2 패키지
**역할:** 상위 제어 계층 (Mission & Navigation)

#### 노드 구성
```
rebar_control/
├── zenoh_client.py            # UI ↔ ROS2 브릿지 (Zenoh)
├── navigator.py               # 미션 관리자 (웨이포인트 순회, motion type 감지) ✅ UPDATED
├── rebar_controller.py        # 복합 경로 제어 (Differential + Lateral) ✅ UPDATED
├── rebar_publisher.py         # 상태 정보 통합 발행
├── odom_to_pose.py            # Odometry → PoseStamped 변환
└── pose_mux.py                # ZED Odometry 멀티플렉서 (듀얼 ZED)
```

#### 설정 파일
```
config/
└── zenoh_config.yaml          # Zenoh 통신 설정
```

#### Launch 파일
```
launch/
├── control_system.launch.py   # 상위 제어 계층 노드 실행
└── full_system.launch.py      # 전체 시스템 실행 (base + control)
```

#### 주요 기능

##### 1. **zenoh_client.py** - UI 통신 브릿지
- **Zenoh → ROS2:**
  - `rebar/command` → `/mission/command` (String)
  - UI 명령어: `E-STOP`, `GO_HOME`, `START`, `STOP`, `ABORT`, `WAYPOINTS:<json>`

- **ROS2 → Zenoh (msgpack):**
  - `/mission/status` → `rebar/status` (10Hz)
  - `/robot_pose` → `rebar/pose` (20Hz)

##### 2. **navigator.py** - 미션 관리자
- **State Machine:**
  ```
  idle → planning → navigating → mission_done
              ↓         ↓            ↓
           emergency_stop (언제든지)
  ```

- **구독:**
  - `/mission/command` (String): UI 명령
  - `/control_mode` (String): navigator_base에서

- **발행:**
  - `/mission/target_pose` (PoseStamped): 현재 목표 위치
  - `/mission/feedback` (String): 진행 상황

- **기능:**
  - 웨이포인트 관리 및 순회
  - 목표 도달 시 다음 웨이포인트로 자동 이동
  - `/mission/waypoint_reached` 수신 시 진행

##### 3. **rebar_controller.py** - 복합 경로 제어 ✅ UPDATED
- **제어 알고리즘:** Motion Type 기반 하이브리드 제어
  - **DIFFERENTIAL**: PID 기반 전진/후진 제어
  - **LATERAL**: 횡이동 제어 (joint_controller 연동)

- **구독:**
  - `/robot_pose` (PoseStamped): ZED X Odometry
  - `/mission/target_pose` (PoseStamped): navigator에서 (하위 호환)
  - `/mission/enhanced_target` (Waypoint): motion type 포함 ✅ NEW

- **발행:**
  - `/cmd_vel` (Twist): drive_controller로 (differential + heading lock)
  - `/joint_control_cmd` (JointControl): joint_controller로 (lateral) ✅ NEW
  - `/mission/waypoint_reached` (String): 목표 도달 알림

- **제어 주기:** 20Hz

- **Differential Motion 파라미터:**
  ```yaml
  kp_linear: 0.5
  kd_linear: 0.1
  kp_angular: 1.0
  kd_angular: 0.2
  distance_tolerance: 0.05  # m (50mm)
  ```

- **Lateral Motion 파라미터:**
  ```yaml
  lateral_tolerance: 0.005   # m (5mm)
  lateral_speed_dps: 200.0   # degrees/sec
  mm_per_rotation: 50.0      # 50mm = 360° = 1 rotation
  ```

##### 4. **rebar_publisher.py** - 상태 통합 발행
- **구독:**
  - `/robot_pose` (PoseStamped)
  - `/control_mode` (String)
  - `/mission/feedback` (String)
  - `/battery_status` (옵션)

- **발행:**
  - `/mission/status` (String, JSON 형식)

- **JSON 필드:**
  ```json
  {
    "control_mode": "auto",
    "mission_status": "navigating",
    "position": {"x": 1.5, "y": 2.0, "theta": 0.5},
    "speed": 0.3,
    "heading": 45.0,
    "battery": 85.5,
    "current_waypoint": 2,
    "total_waypoints": 5,
    "errors": []
  }
  ```

##### 5. **pose_mux.py** - ZED Odometry 멀티플렉서 (옵션)
- **용도:** 듀얼 ZED 사용 시 전진/후진 방향별 센서 선택
- **입력:**
  - Front ZED odom
  - Back ZED odom
  - 방향 신호 (cmd_vel 부호)
- **출력:**
  - `/robot_pose` (PoseStamped, frame=odom)
- **로직:**
  - 전진 시 front ZED 사용
  - 후진 시 back ZED 사용
  - 저속 구간 히스테리시스로 바운싱 방지

---

## 3. 데이터 흐름

### 3.1 Manual 모드 (리모콘 제어)
```
리모콘 (CAN3)
    ↓
can_parser → /remote_control (RemoteControl)
    ↓
drive_controller → /drive_control (DriveControl)
    ↓
can_sender → CAN2 (모터)
```

### 3.2 Auto 모드 - Differential Motion (전진/후진)
```
UI (Zenoh)
    ↓
zenoh_client → /mission/command (String)
    ↓
navigator → /mission/enhanced_target (Waypoint, motion_type=DIFFERENTIAL)
    ↓
rebar_controller + /robot_pose (ZED) → /cmd_vel (Twist)
    ↓
drive_controller → /drive_control (DriveControl)
    ↓
can_sender → CAN2 (0x141, 0x142)

상태 피드백:
rebar_publisher → /mission/status (JSON) → zenoh_client → UI (Zenoh)
```

### 3.3 Auto 모드 - Lateral Motion (횡이동) ✅ NEW
```
UI (Zenoh)
    ↓
zenoh_client → /mission/command (String)
    ↓
navigator → /mission/enhanced_target (Waypoint, motion_type=LATERAL)
    ↓
rebar_controller → /joint_control_cmd (JointControl)
    ↓
joint_controller (auto mode) → CAN 0xA4 (위치 제어)
    ↓
can_sender → CAN2 (0x143 lateral motor)

Heading 유지:
rebar_controller → /cmd_vel (angular.z only) → drive_controller
```

### 3.4 Manual 모드 - 관절 제어 (리모콘)
```
리모콘 S17/S18 (CAN3)
    ↓
can_parser → /remote_control (RemoteControl)
    ↓
joint_controller (manual mode) → /joint_control (JointControl)
    ↓
can_sender → CAN2 (0x143)

---

## 4. 하드웨어 구성

### 4.1 CAN 버스
- **CAN2 (1Mbps):** 모터 통신
  - 0x141: 왼쪽 바퀴 (Differential drive)
  - 0x142: 오른쪽 바퀴 (반전 필요, 모터가 서로 마주보고 장착)
  - 0x143: Lateral (횡이동, RMD X4-36, 360 CPR)
  - 0x144~0x147: 기타 관절 모터

- **CAN3 (250kbps):** 리모콘

### 4.2 Modbus
- **Seengrip:** RTU, 115200 baud, /dev/ttyUSB0
- **EZI-IO:** TCP, 192.168.1.100

### 4.3 센서
- **ZED X Camera:** Odometry (/zed/odom) → /robot_pose

---

## 5. 리팩토링 진행 상황

### 5.1 완료된 Phase

#### Phase 1: 인터페이스 정의 ✅
- rebar_base_interfaces 메시지 정의
- rebar_interfaces 액션/서비스 정의

#### Phase 2: 하드웨어 추상화 계층 ✅
- can_parser, can_sender 구현
- drive_controller 구현 (리모콘 + cmd_vel 통합)
- joint_controller 구현 (위치 제어)
- modbus_controller 구현
- authority_controller 구현
- navigator_base 구현 (State Machine)
- config/can_devices.yaml, modbus_devices.yaml 작성
- launch/base_system.launch.py 작성

#### Phase 3: 상위 제어 계층 (진행 중) 🔄
- ✅ zenoh_client 구현
- ✅ navigator 구현 (기본 동작)
- ✅ rebar_controller 구현 (PID 기반)
- ✅ rebar_publisher 구현
- ✅ config/zenoh_config.yaml 작성
- ✅ launch/control_system.launch.py 작성

### 5.2 남은 작업

#### Phase 3 마무리
- [x] **navigator PAUSE/RESUME 로직** ✅ 완료 (2025-12-23)
  - [x] State Machine에 paused 상태 추가
  - [x] PAUSE/RESUME 명령 처리 구현
  - [x] 피드백에 paused 상태 반영
- [ ] **rebar_publisher 상세 필드** 보강 (waypoint 진행률 등)
- [x] **ZED X Odometry 연동** ✅ 완료 (2025-12-23)
  - [x] /zed/odom → /robot_pose 변환 노드 구현 (odom_to_pose.py)
  - [x] 듀얼/단일 ZED 환경 지원 (pose_mux + odom_to_pose)
  - [x] control_system.launch.py 조건부 실행 구현
  - [ ] zed-ros2-wrapper 실제 실행 확인 (실제 하드웨어 필요)
- [x] **통합 Launch 파일** ✅ 완료 (2025-12-23)
  - [x] launch/full_system.launch.py (base + control 통합)
  - [x] use_dual_zed 인자 추가 및 전달

#### Phase 4: 통합 및 검증
- [x] **빌드 테스트** ✅ 완료 (2025-12-23)
  - [x] colcon build --packages-select rebar_control
  - [x] 의존성 확인

- [x] **노드별 단위 테스트** ✅ 완료 (2025-12-23)
  - [x] odom_to_pose: 메시지 변환 검증 (테스트 스크립트로 확인)
  - [x] odom_to_pose: 발행 주파수 검증 (10Hz 확인)
  - [ ] zenoh_client: UI 명령 echo
  - [ ] rebar_publisher: 상태 메시지 생성 확인
  - [ ] navigator: 웨이포인트 로드 테스트
  - [ ] rebar_controller: 단순 목표 추종

- [ ] **통합 테스트**
  - [ ] Phase 2 + Phase 3 동시 실행
  - [ ] Laptop UI 테스트 (Zenoh)
  - [ ] ZED X Odometry 수신 확인
  - [ ] vcan 기반 안전 테스트

- [x] **성능 테스트** ✅ 스크립트 완료 (2025-12-23)
  - [x] performance_test.py 작성 (자동 600mm 전진/후진 반복)
  - [x] visualize_results.py 작성 (결과 시각화)
  - [x] setup.py entry_point 등록
  - [ ] 실제 하드웨어에서 테스트 실행
  - [ ] 600mm 전진 정확도 측정 (목표: ±10mm)
  - [ ] 600mm 후진 정확도 측정 (목표: ±10mm)
  - [ ] PID 파라미터 튜닝

- [x] **문서화** ✅ 부분 완료 (2025-12-23)
  - [x] 아키텍처 다이어그램 (ARCHITECTURE_DIAGRAM.md)
  - [x] odom_to_pose 노드 README (ODOM_TO_POSE_NODE.md)
  - [ ] 각 노드 README (zenoh_client, navigator 등)
  - [ ] 메시지/액션 인터페이스 문서
  - [ ] 설정 파라미터 가이드

- [ ] **기존 코드 정리**
  - [ ] iron_md_teleop_node.py 정리 (rebar_control에 남아있음)
  - [ ] 기타 미사용 파일 정리

#### Phase 5: 추가 기능 (Optional)
- [ ] RQT UI 개발
- [ ] 로깅 및 모니터링
- [ ] Vision 시스템 (rebar_vision)

---

## 6. 주요 이슈 및 해결 사항

### 6.1 모터 방향 설정
- **0x141 (왼쪽):** 그대로
- **0x142 (오른쪽):** 반전 필요 (모터가 서로 마주보고 장착)

### 6.2 Lateral 모터 속도 보정
- RMD X4-36 (0x143) 정격 속도: 83rpm (498 dps)
- 테스트 결과: 150dps → 실속도 37.6dps (25%)
- **현재 설정:** 200 dps로 상향 조정

### 6.3 Encoder CPR
- **0x143 (Lateral):** 360 CPR (실측값, 출력축 기준)
- **0x141/0x142 (Wheel):** wheel_radius = 0.02865m (1 rev = 0.18m)

### 6.4 State Machine 통합
- navigator_base (하드웨어 계층): 하드웨어 상태 관리
- navigator (상위 계층): 미션 상태 관리
- 두 계층의 상태를 /control_mode로 동기화

---

## 7. Claude Skill 추천 및 작업 가이드

### 7.1 현재 필요한 작업

#### 🎯 우선순위 1: ZED X Odometry 연동
**목적:** /robot_pose 토픽 공급으로 rebar_controller 동작 가능하게 하기

**작업 항목:**
1. zed-ros2-wrapper 상태 확인
   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py
   ros2 topic echo /zed/odom
   ros2 run tf2_ros tf2_echo odom base_link
   ```

2. Odometry → PoseStamped 변환 노드 작성 (필요시)
   - 또는 topic remap으로 해결
   - 파일: `src/rebar_control/rebar_control/odom_to_pose.py`

3. 듀얼 ZED일 경우 pose_mux 구현
   - 파일: `src/rebar_control/rebar_control/pose_mux.py` (이미 존재)
   - 로직: front/back ZED 선택, 히스테리시스
   - TF: static_transform_publisher로 상대 위치 등록

**Claude 작업 프롬프트:**
```
"ZED X Odometry를 /robot_pose (PoseStamped)로 변환하는 노드를 작성해줘.
- 입력: /zed/odom (nav_msgs/Odometry)
- 출력: /robot_pose (geometry_msgs/PoseStamped)
- frame_id: odom
- 10Hz 발행"
```

---

#### 🎯 우선순위 2: 통합 테스트 및 디버깅
**목적:** Phase 2 + Phase 3 통합 실행 및 동작 확인

**작업 항목:**
1. full_system.launch.py 작성
   - base_system.launch.py include
   - control_system.launch.py include

2. 빌드 및 실행
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select rebar_control rebar_base_control
   source install/setup.bash
   ros2 launch rebar_control full_system.launch.py
   ```

3. 토픽 플로우 확인
   ```bash
   ros2 topic list
   ros2 topic echo /mission/command
   ros2 topic echo /mission/status
   ros2 topic echo /cmd_vel
   ros2 topic echo /drive_control
   ```

4. 간단한 미션 테스트
   ```bash
   # E-STOP 테스트
   ros2 topic pub /mission/command std_msgs/String "data: 'E-STOP'"

   # 웨이포인트 테스트
   ros2 topic pub /mission/command std_msgs/String \
     "data: 'WAYPOINTS:[{\"x\":1.0,\"y\":0.0,\"theta\":0.0}]'"
   ```

**Claude 작업 프롬프트:**
```
"rebar_control의 full_system.launch.py를 작성해줘.
base_system.launch.py와 control_system.launch.py를 include해서
전체 시스템을 한 번에 실행할 수 있게 해줘."
```

---

#### 🎯 우선순위 3: navigator PAUSE/RESUME 기능 추가
**목적:** 미션 중단/재개 기능 구현

**작업 항목:**
1. State Machine에 paused 상태 추가
2. /mission/command 명령어 추가: `PAUSE`, `RESUME`
3. paused 상태에서 /mission/target_pose 발행 중단
4. RESUME 시 현재 웨이포인트부터 재개

**Claude 작업 프롬프트:**
```
"navigator.py에 PAUSE/RESUME 기능을 추가해줘.
- State: navigating ↔ paused
- paused 상태에서 /mission/target_pose 발행 중단
- RESUME 시 현재 웨이포인트부터 재개
- /mission/feedback에 PAUSED 상태 반영"
```

---

#### 🎯 우선순위 4: rebar_publisher 상태 필드 보강
**목적:** UI에 더 상세한 진행 상황 제공

**작업 항목:**
1. current_waypoint, total_waypoints 필드 추가
2. mission_status 상세화 (planning, navigating, paused 등)
3. 에러 메시지 수집 (/diagnostics 구독)

**Claude 작업 프롬프트:**
```
"rebar_publisher.py의 JSON 상태에 웨이포인트 진행률을 추가해줘.
- /mission/feedback에서 current_waypoint, total_waypoints 파싱
- JSON에 필드 추가
- 테스트 코드 작성"
```

---

#### 🎯 우선순위 5: 성능 테스트 및 PID 튜닝
**목적:** 정밀도 목표 달성 (±10mm)

**작업 항목:**
1. 정밀도 측정 스크립트 작성
2. PID 파라미터 자동 튜닝 (옵션)
3. 로그 분석 및 시각화

**Claude 작업 프롬프트:**
```
"rebar_controller의 PID 파라미터를 튜닝하기 위한 테스트 스크립트를 작성해줘.
- 600mm 전진/후진 반복
- 최종 위치 기록 (CSV)
- 평균, 표준편차, 최대 오차 계산
- matplotlib으로 시각화"
```

---

### 7.2 추가 개발 아이템

#### 문서화
1. **아키텍처 다이어그램**
   - Mermaid 또는 PlantUML
   - 노드 간 토픽 관계도

2. **각 노드 README**
   - 기능 설명
   - 파라미터 설명
   - 사용 예제

3. **메시지 인터페이스 문서**
   - 각 메시지 필드 설명
   - 사용 예제

#### 코드 품질 개선
1. **타입 힌팅 추가**
   - Python 3.10+ type hints
   - mypy 검증

2. **Unit Test 작성**
   - pytest 기반
   - 각 노드별 테스트

3. **로깅 개선**
   - 일관된 로그 레벨
   - 디버깅 정보 추가

#### Vision 연동 준비
1. **rebar_vision 패키지 정리**
   - ZED X 관련 파일 정리
   - 철근 인식 알고리즘 stub 작성

2. **위치 보정 인터페이스**
   - Vision → Navigator 피드백
   - /vision/rebar_detected 토픽 정의

---

## 8. 개발 환경 및 의존성

### 8.1 시스템 요구사항
- OS: Ubuntu 22.04 (ROS2 Humble)
- Python: 3.10+
- ROS2: Humble

### 8.2 Python 패키지
```bash
pip3 install python-statemachine  # State Machine
pip3 install zenoh                 # Zenoh 통신
pip3 install msgpack               # Zenoh 메시지 직렬화
```

### 8.3 ROS2 패키지
```bash
sudo apt install ros-humble-zed-ros2-wrapper  # ZED X
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-nav-msgs
```

---

## 9. 빌드 및 실행

### 9.1 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select \
  rebar_base_interfaces \
  rebar_interfaces \
  rebar_base_control \
  rebar_control
source install/setup.bash
```

### 9.2 실행

#### 하드웨어 계층만 실행
```bash
ros2 launch rebar_base_control base_system.launch.py
```

#### 상위 제어 계층만 실행
```bash
ros2 launch rebar_control control_system.launch.py
```

#### 전체 시스템 실행
```bash
ros2 launch rebar_control full_system.launch.py
```

---

## 10. 참고 자료

### 10.1 Tire Roller 코드 위치
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

### 10.2 관련 문서
- `REBAR_REFACTORING_PLAN.md`: 상세 리팩토링 계획
- `JOINT_CONTROLLER_REFACTOR.md`: 관절 제어 리팩토링 내역
- `LATERAL_MOTION_TEST_RESULT.md`: 횡이동 테스트 결과
- `S17_S18_LATERAL_FIX_SUMMARY.md`: S17/S18 수정 내역

---

## 11. Claude 작업 체크리스트

### Phase 3 완료
- [ ] ZED X Odometry → /robot_pose 변환 노드 작성
- [ ] full_system.launch.py 작성
- [ ] navigator PAUSE/RESUME 기능 추가
- [ ] rebar_publisher 상태 필드 보강
- [ ] 통합 테스트 실행 및 로그 확인

### Phase 4 통합 검증
- [ ] 빌드 에러 수정
- [ ] 토픽 플로우 검증
- [ ] 간단한 미션 테스트 (E-STOP, 웨이포인트)
- [ ] ZED Odometry 수신 확인
- [ ] 성능 테스트 스크립트 작성 및 실행

### 문서화
- [ ] 아키텍처 다이어그램 (Mermaid)
- [ ] 각 노드 README 작성
- [ ] 메시지 인터페이스 문서 작성
- [ ] 파라미터 가이드 작성

### 코드 품질
- [ ] Type hints 추가
- [ ] Unit test 작성
- [ ] 로깅 개선
- [ ] 기존 코드 정리 (iron_md_teleop_node.py 등)

---

## 12. 다음 단계 제안

1. **ZED X 연동 테스트**
   - zed-ros2-wrapper 실행
   - /zed/odom 데이터 확인
   - odom_to_pose 노드 작성

2. **전체 시스템 통합 테스트**
   - full_system.launch.py 작성
   - vcan으로 안전 테스트
   - 실제 하드웨어 연동

3. **UI 연동 테스트**
   - Zenoh 통신 확인
   - Laptop UI에서 명령 전송
   - 상태 피드백 확인

4. **성능 최적화**
   - PID 파라미터 튜닝
   - 정밀도 측정
   - 응답 지연 개선

---

## 13. 2025-12-23 작업 완료 내역

### 13.1 우선순위 4: 속도 제한 (정격의 25%)

**목적:** 안전한 테스트를 위해 0x141/0x142 모터 속도를 1/4로 제한

**구현 내용:**
- `can_devices.yaml`: `speed_scale_factor: 0.25` 파라미터 추가
- `drive_controller.py`: 속도 스케일링 로직 적용
  - Manual 모드 (리모콘) 스케일링
  - Auto 모드 (cmd_vel) 스케일링
  - 초기화 로그에 스케일 비율 표시

**효과:**
- 모든 주행 명령이 25% 속도로 제한
- 설정 파일 변경만으로 쉽게 조정 가능

---

### 13.2 우선순위 2: full_system.launch.py 개선

**목적:** 듀얼/단일 ZED 환경 자동 선택

**구현 내용:**
- Launch 인자 추가:
  - `use_dual_zed`: pose_mux vs odom_to_pose 선택
  - `single_zed_odom_topic`: 단일 ZED odometry 토픽
- control_system.launch.py로 인자 전달 구현

**사용법:**
```bash
# 듀얼 ZED (기본)
ros2 launch rebar_control full_system.launch.py

# 단일 ZED
ros2 launch rebar_control full_system.launch.py use_dual_zed:=false

# ZED 없이
ros2 launch rebar_control full_system.launch.py use_zed:=false
```

---

### 13.3 우선순위 3: navigator PAUSE/RESUME 기능

**목적:** 미션 중단/재개 기능 완성

**구현 내용:**
1. **State Machine 확장:**
   - `paused` 상태 추가
   - `pause` 전이: navigating → paused
   - `resume` 전이: paused → navigating
   - `on_enter_paused` hook: cmd_vel = 0 즉시 정지

2. **명령 처리:**
   - `PAUSE_MISSION`: State Machine 전이로 구현
   - `RESUME_MISSION`: 현재 웨이포인트부터 재개
   - 명확한 에러 메시지

3. **피드백 개선:**
   - `/mission/feedback`의 state 필드에 "paused" 반영
   - paused 상태에서도 피드백 발행
   - target_pose 발행 중단

4. **코드 정리:**
   - 불필요한 `self.paused` 플래그 제거
   - State Machine으로 통합 관리

---

### 13.4 우선순위 4: 성능 테스트 스크립트

**목적:** 600mm 전진/후진 정확1111도 자동 측정

**구현 파일:**

1. **performance_test.py** - 자동 테스트 노드
   - 10회 반복 (파라미터 조정 가능)
   - 600mm 전진 → 실제 위치 기록
   - 600mm 후진 → 실제 위치 기록
   - CSV 파일 자동 저장
   - 실시간 통계 출력

2. **visualize_results.py** - 결과 시각화
   - 위치 궤적 (X-Y Plot)
   - 오차 시계열
   - 오차 히스토그램
   - 전진 vs 후진 박스플롯
   - PNG 파일 저장

3. **setup.py** 업데이트
   - `performance_test` entry_point 등록
   - scripts 디렉토리 추가
   - docs 디렉토리 추가

**사용법:**
```bash
# 1. 전체 시스템 실행
ros2 launch rebar_control full_system.launch.py

# 2. 성능 테스트 (자동 시작, 3초 대기)
ros2 run rebar_control performance_test

# 3. 결과 시각화
python3 src/rebar_control/scripts/visualize_results.py 20251223_143052_performance_test_results.csv
```

**출력:**
- CSV: 모든 측정 데이터
- PNG: 4개 그래프 (궤적, 시계열, 히스토그램, 박스플롯)
- 터미널: 통계 요약 (평균, 표준편차, 최대/최소 오차)

---

### 13.5 문서 작성

**작성 문서:**
1. `ODOM_TO_POSE_NODE.md` - odom_to_pose 노드 가이드
2. `ARCHITECTURE_DIAGRAM.md` - 전체 시스템 아키텍처
3. `REBAR_DEVELOPMENT_SUMMARY.md` 업데이트

---

### 13.6 파일 변경 요약

**신규 파일:**
- `/home/koceti/ros2_ws/src/rebar_control/rebar_control/odom_to_pose.py`
- `/home/koceti/ros2_ws/src/rebar_control/rebar_control/performance_test.py`
- `/home/koceti/ros2_ws/src/rebar_control/scripts/visualize_results.py`
- `/home/koceti/ros2_ws/src/rebar_control/docs/ODOM_TO_POSE_NODE.md`
- `/home/koceti/ros2_ws/src/rebar_control/docs/ARCHITECTURE_DIAGRAM.md`

**수정 파일:**
- `/home/koceti/ros2_ws/src/rebar_base_control/config/can_devices.yaml` - 속도 스케일 추가
- `/home/koceti/ros2_ws/src/rebar_base_control/rebar_base_control/drive_controller.py` - 속도 스케일링
- `/home/koceti/ros2_ws/src/rebar_control/rebar_control/navigator.py` - PAUSE/RESUME
- `/home/koceti/ros2_ws/src/rebar_control/launch/control_system.launch.py` - 조건부 실행
- `/home/koceti/ros2_ws/src/rebar_control/launch/full_system.launch.py` - 인자 전달
- `/home/koceti/ros2_ws/src/rebar_control/setup.py` - entry_points 추가

---

---

## 14. 2025-12-23 복합 네비게이션 (Hybrid Navigation) 구현 완료

### 14.1 개요
전진/후진(X축)과 횡이동(Y축)을 결합한 복합 경로 네비게이션 시스템 구현

### 14.2 구현 내용

#### 1. Waypoint 메시지 확장
**파일:** `rebar_base_interfaces/msg/Waypoint.msg` ✅ NEW
- Motion type 상수 추가: DIFFERENTIAL(0), LATERAL(1), HYBRID(2)
- 위치 + 속도 + 모션 타입 포함

#### 2. Navigator 개선
**파일:** `rebar_control/navigator.py` ✅ UPDATED
- **Motion Type 자동 감지:** dy > 1mm && dx < 1mm → Lateral
- **Enhanced Waypoint 발행:** `/mission/enhanced_target` (Waypoint)
- **하위 호환성 유지:** 기존 `/mission/target_pose` (PoseStamped) 동시 발행
- **웨이포인트 JSON 형식:**
  ```json
  {
    "waypoints": [
      {"x": 0, "y": 0},       // 자동 감지
      {"x": 200, "y": 0},     // DIFFERENTIAL
      {"x": 200, "y": 50}     // LATERAL (자동 감지)
    ]
  }
  ```

#### 3. Rebar Controller 확장
**파일:** `rebar_control/rebar_controller.py` ✅ UPDATED
- **Differential Motion:** PID 기반 전진/후진 제어 (기존 로직)
- **Lateral Motion:** 횡이동 제어 추가 ✅ NEW
  - `/joint_control_cmd` 발행 → joint_controller로
  - 50mm = 360° = 1회전
  - Heading 유지 (angular.z 보정)
- **Motion Type 기반 제어 선택**

#### 4. Joint Controller 모드 분리
**파일:** `rebar_base_control/joint_controller.py` ✅ UPDATED
- **Manual 모드:** 리모콘 S17/S18 → 횡이동 제어 (기존)
- **Auto 모드:** `/joint_control_cmd` 구독 → CAN 명령 전송 ✅ NEW
- **토픽 분리:**
  - Manual: `/joint_control` 발행 (내부 처리)
  - Auto: `/joint_control_cmd` 구독 (외부 명령)

### 14.3 데이터 흐름

#### Differential Motion (전진/후진)
```
navigator → Waypoint(DIFFERENTIAL) → rebar_controller
    ↓
/cmd_vel → drive_controller → CAN (0x141, 0x142)
```

#### Lateral Motion (횡이동)
```
navigator → Waypoint(LATERAL) → rebar_controller
    ↓
/joint_control_cmd → joint_controller → CAN (0x143)
    +
/cmd_vel (angular.z only) → heading lock
```

### 14.4 사용 예시

#### 복합 경로 전송 (외부 PC)
```python
# send_hybrid_path.py
waypoints = [
    {"x": 0, "y": 0},       # Start
    {"x": 200, "y": 0},     # Forward 200mm (DIFFERENTIAL)
    {"x": 200, "y": 50},    # Lateral +50mm (LATERAL, auto-detect)
    {"x": 200, "y": 100},   # Lateral +50mm (LATERAL, auto-detect)
    {"x": 400, "y": 100},   # Forward 200mm (DIFFERENTIAL)
    {"x": 400, "y": 50},    # Lateral -50mm (LATERAL)
    {"x": 400, "y": 0},     # Lateral -50mm (LATERAL)
    {"x": 200, "y": 0},     # Backward 200mm (DIFFERENTIAL)
    {"x": 0, "y": 0},       # Backward 200mm (DIFFERENTIAL)
]
```

#### 시스템 실행
```bash
# 전체 시스템 실행
ros2 launch rebar_control full_system.launch.py

# 외부 PC에서 경로 전송
python3 send_hybrid_path.py --mode peer
```

### 14.5 핵심 특징

1. **자동 Motion Type 감지**: 웨이포인트 좌표만으로 이동 방식 판단
2. **Heading 유지**: 횡이동 시 로봇 방향 고정
3. **Dual ZED 지원**: 전진/후진별 카메라 자동 전환 (pose_mux)
4. **하위 호환성**: 기존 differential-only 경로도 동작
5. **간결한 아키텍처**:
   - ❌ 제거: hybrid_controller, motion_executors
   - ✅ 개선: joint_controller (Manual/Auto 분리), rebar_controller (Motion Type 지원)

### 14.6 파일 변경 요약

**신규 파일:**
- `rebar_base_interfaces/msg/Waypoint.msg`
- `rebar_control/scripts/send_hybrid_path.py`

**수정 파일:**
- `rebar_base_interfaces/CMakeLists.txt` - Waypoint.msg 추가
- `rebar_base_control/joint_controller.py` - Manual/Auto 모드 분리
- `rebar_control/navigator.py` - Motion type 감지, Enhanced Waypoint 발행
- `rebar_control/rebar_controller.py` - Lateral motion 지원 추가
- `rebar_control/launch/control_system.launch.py` - 단순화

**제거 파일:**
- `rebar_control/hybrid_controller.py`
- `rebar_control/motion_executors/` (전체 폴더)

### 14.7 테스트 방법

```bash
# 1. 빌드
colcon build --packages-select rebar_base_control rebar_control --symlink-install

# 2. 시스템 실행
ros2 launch rebar_control full_system.launch.py

# 3. 외부 PC에서 테스트 경로 전송
python3 src/rebar_control/scripts/send_hybrid_path.py --mode peer

# 4. 토픽 모니터링
ros2 topic echo /mission/enhanced_target  # Motion type 확인
ros2 topic echo /joint_control_cmd        # Lateral 명령 확인
ros2 topic echo /cmd_vel                  # Differential 명령 확인
```

---

---

## 15. 2025-12-26 상부체 3축 스테이지 (0x144~0x147) 구현

### 15.1 개요

철근 결속 작업을 위한 3축 스테이지 모터 제어 기능 추가

### 15.2 하드웨어 구성

| 모터 ID | 용도 | 제어 방식 | 리모콘 매핑 |
|---------|------|-----------|-------------|
| 0x144 | X축 스테이지 | 위치 제어 (0xA4) | AN1 (조이스틱) |
| 0x145 | Y축 스테이지 | 위치 제어 (0xA4) | AN2 (조이스틱) |
| 0x146 | Z축 스테이지 | 위치 제어 (0xA4) | S21(-)/S22(+) |
| 0x147 | Yaw 회전 | 위치 제어 (0xA4) | S23(+)/S24(-) ✅ 기존 |

### 15.3 구현 계획

#### Phase 1: Manual 모드 (현재 구현)

**수정 파일:**

| 파일 | 수정 내용 |
|------|----------|
| `can_parser.py` | 0x244~0x247 응답 파싱 추가 |
| `can_devices.yaml` | 3축 스테이지 파라미터 추가 |
| `joint_controller.py` | 파라미터 선언, 피드백 수신, 제어 핸들러 추가 |

**리모콘 버튼 매핑:**
```
조이스틱:
- AN1 (joysticks[0]) → 0x144 X축 연속 이동
- AN2 (joysticks[1]) → 0x145 Y축 연속 이동

버튼:
- S21 (buttons[4]) → 0x146 Z축 하강 (스텝)
- S22 (buttons[5]) → 0x146 Z축 상승 (스텝)
- S23 (buttons[6]) → 0x147 Yaw + (기존)
- S24 (buttons[7]) → 0x147 Yaw - (기존)
```

**제어 파라미터:**
```yaml
# 3축 스테이지 (can_devices.yaml)
stage_x_step_deg: 36.0      # 1mm = 36° (리드스크류 10mm/rev 가정)
stage_x_max_speed: 200.0    # dps
stage_y_step_deg: 36.0
stage_y_max_speed: 200.0
stage_z_step_deg: 36.0      # 스텝 이동량
stage_z_max_speed: 200.0
joystick_deadzone: 0.15     # 15% 데드존
```

#### Phase 2: Auto 모드 (추후 구현)

**목표:** ZED 카메라 기반 철근 결속 포인트 자동 이동

```
ZED 카메라 → rebar_detector.py → 결속 포인트 좌표 (x, y, z)
                    ↓
stage_controller.py → /joint_control_cmd → joint_controller → CAN
                    ↓
              결속 완료 신호 → 다음 포인트
```

**메시지 확장 (예정):**
```
# StageControl.msg
float32 target_x  # mm
float32 target_y  # mm
float32 target_z  # mm
float32 target_yaw  # degrees
float32 velocity  # dps
uint8 mode  # 0=absolute, 1=relative
```

### 15.4 데이터 흐름

#### Manual 모드 - 스테이지 제어
```
리모콘 (CAN3)
    ↓
can_parser → /remote_control (RemoteControl)
    ↓
joint_controller:
    ├── AN1 → 0x144 (X축)
    ├── AN2 → 0x145 (Y축)
    ├── S21/S22 → 0x146 (Z축)
    └── S23/S24 → 0x147 (Yaw)
    ↓
/joint_control (JointControl)
    ↓
can_sender → CAN2 (0xA4 위치 명령)

피드백:
CAN2 (0x244~0x247 응답)
    ↓
can_parser → /motor_feedback (MotorFeedback)
    ↓
joint_controller (위치 추적)
```

### 15.5 구현 단계

| 단계 | 파일 | 작업 내용 | 상태 |
|------|------|-----------|------|
| 1 | can_parser.py | 0x244~0x247 응답 파싱 추가 | ✅ 완료 |
| 2 | can_devices.yaml | 3축 스테이지 파라미터 추가 | ✅ 완료 |
| 3 | joint_controller.py | 파라미터 선언 및 초기화 | ✅ 완료 |
| 4 | joint_controller.py | recv_motor_feedback() 확장 | ✅ 완료 |
| 5 | joint_controller.py | _handle_stage_xy() 추가 | ✅ 완료 |
| 6 | joint_controller.py | _handle_stage_z() 추가 | ✅ 완료 |
| 7 | 테스트 | Manual 모드 동작 검증 | ⏳ 대기 (하드웨어 필요) |

### 15.6 주의사항

1. **리드스크류 피치 확인 필요**
   - 현재 가정: 10mm/rev → 36°/mm
   - 실제 하드웨어 사양에 맞게 조정

2. **모터 방향 확인**
   - +각도 → +방향(상승/전진) 매핑 확인
   - 필요시 부호 반전

3. **소프트 리밋 설정 (추후)**
   - 3축 스테이지 이동 범위 제한
   - 충돌 방지 안전 마진

4. **기존 0x147 Yaw 제어와 호환성**
   - S23/S24 매핑은 기존과 동일
   - 파라미터만 확인 필요

---

**작성자:** Claude Code (Opus 4.5)
**최종 업데이트:** 2025-12-26 (상부체 3축 스테이지 구현 시작)
