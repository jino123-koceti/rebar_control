# 정밀 주행 제어 시스템 설계 (Precision Navigation System)

## 📋 목차
1. [시스템 개요](#시스템-개요)
2. [문제 정의](#문제-정의)
3. [해결 방안](#해결-방안)
4. [시스템 아키텍처](#시스템-아키텍처)
5. [노드별 상세 설계](#노드별-상세-설계)
6. [메시지 정의](#메시지-정의)
7. [제어 알고리즘](#제어-알고리즘)
8. [UI 설계](#ui-설계)
9. [통합 방안](#통합-방안)
10. [구현 계획](#구현-계획)

---

## 시스템 개요

### 목표
무한궤도 로봇의 슬립 문제를 해결하여 정밀한 직진 주행 및 회전 제어 구현

### 핵심 기능
- **정밀 직진**: ZED X mini IMU 기반 Heading 제어 (Yaw 고정)
- **정확한 이동 거리**: Visual Odometry 기반 거리 피드백 제어
- **RQT UI 제어**: 사용자 친화적 인터페이스로 목표 명령 입력
- **기존 시스템 보존**: S20 Auto 모드 리모콘 동작 유지

---

## 문제 정의

### 현재 문제점
1. **슬립 발생**: 무한궤도 + 미끄러운 바닥 → 직진 불가, 틀어짐
2. **거리 부정확**: S20 모드 AN3 명령 (±600mm) 실제 이동 거리 오차 발생
3. **개루프 제어**: 위치 피드백 없이 속도 명령만 전송 (open-loop)

### 요구사항
- ✅ 직진 정확도: Yaw 오차 ±2도 이내
- ✅ 거리 정확도: 목표 거리 ±10mm 이내
- ✅ S20 모드 기존 기능 유지
- ✅ RQT UI 기반 제어
- ✅ 리모콘 명령과 UI 명령 배타적 처리 (동시 입력 방지)

---

## 해결 방안

### 센서 기반 폐루프 제어

| 센서 | 용도 | 피드백 데이터 |
|------|------|--------------|
| ZED X mini IMU | Heading 제어 | Yaw 각도 (quaternion) |
| ZED Visual Odometry | 거리 제어 | 3D 위치 (x, y, z) |

### 제어 전략
```
목표 명령 (예: 전진 600mm)
  ↓
[Precision Navigation Controller]
  ├─ Heading PID: 현재 Yaw vs 목표 Yaw → angular.z 보정
  └─ Distance PID: 현재 거리 vs 목표 거리 → linear.x 제어
  ↓
/cmd_vel_precise 발행
  ↓
iron_md_teleop (Command Mux)
  ├─ IF precision nav 활성 → /cmd_vel_precise 전달
  └─ ELSE → 리모콘 명령 처리
  ↓
position_control_node → 모터 제어
```

---

## 시스템 아키텍처

### 전체 구조도

```
┌─────────────────────────────────────────────────────────────┐
│                  RQT Precision Nav UI                       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │
│  │전진 600mm│ │후진 600mm│ │좌회전 90°│ │우회전 90°│       │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │
│  진행상황: ████████████░░░░ 75% (450/600mm)                │
│  현재 Yaw: 0.2° | 목표 Yaw: 0.0° | 오차: 0.2°              │
└────────────────────┬────────────────────────────────────────┘
                     │ /precision_nav/goal (PrecisionNavGoal)
                     ↓
┌─────────────────────────────────────────────────────────────┐
│         Precision Navigation Controller Node                │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ State Machine                                         │  │
│  │  IDLE → VALIDATE → MOVING → REACHED → IDLE           │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  Inputs:                                                     │
│  ├─ /zedxm/imu/data (sensor_msgs/Imu) - 100Hz              │
│  ├─ /zedxm/odom (nav_msgs/Odometry) - 30Hz                 │
│  └─ /precision_nav/goal (PrecisionNavGoal)                 │
│                                                              │
│  Controllers:                                                │
│  ├─ Heading PID Controller                                  │
│  │  - Input: Yaw error (목표 - 현재)                       │
│  │  - Output: angular.z correction                         │
│  │  - Gains: Kp=0.5, Ki=0.0, Kd=0.1                        │
│  │                                                           │
│  └─ Distance PID Controller                                 │
│     - Input: Distance error (목표 - 현재)                   │
│     - Output: linear.x velocity                             │
│     - Gains: Kp=0.3, Ki=0.0, Kd=0.05                        │
│                                                              │
│  Outputs:                                                    │
│  ├─ /cmd_vel_precise (geometry_msgs/Twist)                 │
│  ├─ /precision_nav/feedback (PrecisionNavFeedback)         │
│  └─ /precision_nav/status (PrecisionNavStatus)             │
└────────────────────┬────────────────────────────────────────┘
                     │ /cmd_vel_precise
                     ↓
┌─────────────────────────────────────────────────────────────┐
│         iron_md_teleop_node (수정: Command Mux 추가)        │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ Command Multiplexer Logic                             │  │
│  │                                                        │  │
│  │  IF /precision_nav/status == MOVING:                  │  │
│  │    ├─ Use /cmd_vel_precise                            │  │
│  │    ├─ Ignore remote AN3 input                         │  │
│  │    └─ Log: "Precision nav active, remote disabled"   │  │
│  │  ELSE:                                                 │  │
│  │    ├─ Use remote AN3 input (기존 동작)               │  │
│  │    └─ /cmd_vel_precise ignored                        │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  S20 Auto Mode 기존 기능:                                   │
│  ├─ AN3: 전후진 (슬립 있음, precision nav 비활성 시)        │
│  ├─ S17/S18: 횡이동                                         │
│  ├─ S21/S22: 작업 시퀀스                                    │
│  └─ S23/S24: Yaw 회전                                       │
│                                                              │
│  Output: /cmd_vel (geometry_msgs/Twist)                    │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│         position_control_node (변경 없음)                   │
│         → 0x141, 0x142 모터 속도 제어                       │
└─────────────────────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────┐
│              ZED X mini Camera (can2/can3와 독립)           │
│  Publishers:                                                 │
│  ├─ /zedxm/imu/data (100Hz)                                │
│  ├─ /zedxm/odom (30Hz)                                     │
│  ├─ /zedxm/pose (30Hz)                                     │
│  └─ /zedxm/rgb/image_rect_color (30Hz, optional)          │
└─────────────────────────────────────────────────────────────┘
```

---

## 노드별 상세 설계

### 1. Precision Navigation Controller Node

**파일**: `src/rebar_control/rebar_control/precision_navigation_node.py`

#### 클래스 구조
```python
class PrecisionNavigationNode(Node):
    def __init__(self):
        # 구독자
        self.imu_sub: IMU data
        self.odom_sub: Visual Odometry
        self.goal_sub: UI 명령

        # 발행자
        self.cmd_vel_pub: /cmd_vel_precise
        self.feedback_pub: 진행 상황
        self.status_pub: 노드 상태

        # 제어기
        self.heading_pid: PID Controller (Yaw)
        self.distance_pid: PID Controller (거리)

        # 상태 변수
        self.state: State Machine
        self.start_pose: 시작 위치
        self.current_pose: 현재 위치
        self.goal: 목표 (거리, 각도)
```

#### State Machine

```
┌──────┐
│ IDLE │ ← 초기 상태, 명령 대기
└───┬──┘
    │ Goal 수신
    ↓
┌──────────┐
│ VALIDATE │ ← 목표 유효성 검증 (거리/각도 범위)
└───┬──────┘
    │ Valid
    ↓
┌─────────┐
│ MOVING  │ ← PID 제어 실행, cmd_vel_precise 발행
└───┬─────┘
    │ Goal Reached (오차 < threshold)
    ↓
┌─────────┐
│ REACHED │ ← 정지, 결과 발행
└───┬─────┘
    │ 1초 대기
    ↓
┌──────┐
│ IDLE │
└──────┘

Error 발생 시 → ERROR 상태 → IDLE 복귀
```

#### 제어 주기
- **IMU 콜백**: 100Hz (Heading 제어)
- **Odom 콜백**: 30Hz (거리 제어)
- **제어 루프**: 50Hz (20ms) - 타이머 기반

---

### 2. RQT Precision Nav UI Plugin

**파일**: `src/rebar_control/rebar_control/rqt_precision_nav.py`

#### UI 레이아웃
```
┌─────────────────────────────────────────────────┐
│  Precision Navigation Control                   │
├─────────────────────────────────────────────────┤
│  Quick Commands:                                │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐        │
│  │전진 600mm│ │후진 600mm│ │정지      │        │
│  └──────────┘ └──────────┘ └──────────┘        │
│                                                  │
│  ┌──────────┐ ┌──────────┐                     │
│  │좌회전 90°│ │우회전 90°│                     │
│  └──────────┘ └──────────┘                     │
├─────────────────────────────────────────────────┤
│  Custom Command:                                │
│  Distance: [______] mm  [전진] [후진]          │
│  Angle:    [______] deg [좌회전] [우회전]      │
├─────────────────────────────────────────────────┤
│  Status:                                        │
│  State:     [IDLE / MOVING / REACHED]          │
│  Progress:  ████████████░░░░░░ 60%             │
│  Distance:  360 / 600 mm                       │
│  Yaw Error: 0.5°                                │
├─────────────────────────────────────────────────┤
│  Settings (PID Gains):                          │
│  Heading:  Kp [0.5] Ki [0.0] Kd [0.1]          │
│  Distance: Kp [0.3] Ki [0.0] Kd [0.05]         │
│  [Apply]                                        │
└─────────────────────────────────────────────────┘
```

#### 기능
1. **Quick Commands**: 미리 정의된 명령 (600mm, 90도)
2. **Custom Commands**: 사용자 정의 거리/각도
3. **실시간 피드백**: 진행 상황, 오차 표시
4. **PID 튜닝**: UI에서 게인 조정 가능

---

### 3. iron_md_teleop_node 수정

**파일**: `src/rebar_control/rebar_control/iron_md_teleop_node.py`

#### 추가 구독자
```python
self.cmd_vel_precise_sub = self.create_subscription(
    Twist, '/cmd_vel_precise', self.cmd_vel_precise_callback, 10
)
self.precision_status_sub = self.create_subscription(
    PrecisionNavStatus, '/precision_nav/status', self.precision_status_callback, 10
)
```

#### Command Mux 로직 (control_loop 수정)
```python
def control_loop(self):
    """20Hz 제어 루프"""

    # Precision nav 상태 확인
    if self.precision_nav_active:
        # Precision nav 명령 우선
        cmd_vel = self.cmd_vel_precise  # 저장된 precise 명령
        self.get_logger().debug('Using precision nav command')
    else:
        # 기존 리모콘 명령 처리 (S20 모드)
        if self.current_mode == 'S20':  # Auto mode
            cmd_vel = self.process_remote_an3()  # 기존 로직
        else:
            cmd_vel = Twist()  # S19 mode

    # 최종 cmd_vel 발행
    self.cmd_vel_pub.publish(cmd_vel)
```

#### 추가 변수
```python
self.precision_nav_active = False  # Precision nav 활성화 플래그
self.cmd_vel_precise = Twist()     # Precision nav 명령 저장
```

---

## 메시지 정의

### Custom Messages 패키지

**위치**: `src/rebar_msgs/`

#### 1. PrecisionNavGoal.msg
```
# 목표 명령
uint8 TYPE_LINEAR = 0   # 직진/후진
uint8 TYPE_ANGULAR = 1  # 회전

uint8 type              # 명령 타입
float32 distance        # 이동 거리 (mm, type=LINEAR)
float32 angle           # 회전 각도 (deg, type=ANGULAR)
float32 max_velocity    # 최대 속도 (m/s or rad/s)
```

#### 2. PrecisionNavFeedback.msg
```
# 진행 상황 피드백
float32 progress        # 진행률 (0.0 ~ 1.0)
float32 current_distance  # 현재 이동 거리 (mm)
float32 target_distance   # 목표 거리 (mm)
float32 yaw_error       # Yaw 오차 (deg)
float32 linear_velocity # 현재 선속도 (m/s)
```

#### 3. PrecisionNavStatus.msg
```
# 노드 상태
uint8 STATE_IDLE = 0
uint8 STATE_VALIDATE = 1
uint8 STATE_MOVING = 2
uint8 STATE_REACHED = 3
uint8 STATE_ERROR = 4

uint8 state            # 현재 상태
bool active            # 활성화 플래그 (MOVING일 때 true)
string error_msg       # 에러 메시지 (STATE_ERROR 시)
```

---

## 제어 알고리즘

### 1. Heading PID Controller (직진 유지)

#### 목표
전진/후진 시 Yaw 각도 고정 (슬립으로 인한 틀어짐 방지)

#### 입력/출력
- **Input**: Yaw error (deg) = 목표 Yaw - 현재 Yaw
- **Output**: angular.z (rad/s)

#### 알고리즘
```python
class HeadingPIDController:
    def __init__(self, Kp=0.5, Ki=0.0, Kd=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, yaw_error_deg, dt):
        """
        Args:
            yaw_error_deg: 목표 - 현재 (deg)
            dt: 시간 간격 (sec)
        Returns:
            angular.z (rad/s)
        """
        # deg → rad 변환
        error_rad = math.radians(yaw_error_deg)

        # PID 계산
        self.integral += error_rad * dt
        derivative = (error_rad - self.prev_error) / dt

        output = (self.Kp * error_rad +
                  self.Ki * self.integral +
                  self.Kd * derivative)

        self.prev_error = error_rad

        # Anti-windup
        self.integral = max(-1.0, min(1.0, self.integral))

        # 출력 제한 (±0.5 rad/s)
        output = max(-0.5, min(0.5, output))

        return output
```

#### 튜닝 가이드
- **Kp**: 비례 게인 (0.3 ~ 0.7) - 빠른 응답
- **Ki**: 적분 게인 (0.0 ~ 0.1) - 정상 상태 오차 제거
- **Kd**: 미분 게인 (0.05 ~ 0.2) - 오버슛 방지

---

### 2. Distance PID Controller (정확한 거리 제어)

#### 목표
목표 거리만큼 정확히 이동 (Visual Odometry 기반)

#### 입력/출력
- **Input**: Distance error (mm) = 목표 거리 - 현재 이동 거리
- **Output**: linear.x (m/s)

#### 알고리즘
```python
class DistancePIDController:
    def __init__(self, Kp=0.3, Ki=0.0, Kd=0.05):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, distance_error_mm, dt):
        """
        Args:
            distance_error_mm: 목표 - 현재 (mm)
            dt: 시간 간격 (sec)
        Returns:
            linear.x (m/s)
        """
        # mm → m 변환
        error_m = distance_error_mm / 1000.0

        # PID 계산
        self.integral += error_m * dt
        derivative = (error_m - self.prev_error) / dt

        output = (self.Kp * error_m +
                  self.Ki * self.integral +
                  self.Kd * derivative)

        self.prev_error = error_m

        # Anti-windup
        self.integral = max(-0.5, min(0.5, self.integral))

        # 출력 제한 (0.1 ~ 0.5 m/s)
        output = max(0.1, min(0.5, abs(output))) * (1 if error_m > 0 else -1)

        # 목표 근접 시 감속
        if abs(distance_error_mm) < 100:  # 100mm 이내
            output *= 0.5  # 속도 절반

        return output
```

---

### 3. Visual Odometry 거리 계산

#### 방법
ZED Odometry의 x, y 위치를 이용한 2D 거리 계산

```python
def calculate_distance_traveled(self, start_odom, current_odom):
    """
    Args:
        start_odom: 시작 위치 (nav_msgs/Odometry)
        current_odom: 현재 위치 (nav_msgs/Odometry)
    Returns:
        거리 (mm)
    """
    dx = current_odom.pose.pose.position.x - start_odom.pose.pose.position.x
    dy = current_odom.pose.pose.position.y - start_odom.pose.pose.position.y

    distance_m = math.sqrt(dx**2 + dy**2)
    distance_mm = distance_m * 1000.0

    return distance_mm
```

---

### 4. 목표 도달 판정

```python
def is_goal_reached(self, distance_error_mm, yaw_error_deg):
    """
    Args:
        distance_error_mm: 거리 오차 (mm)
        yaw_error_deg: Yaw 오차 (deg)
    Returns:
        bool: 목표 도달 여부
    """
    DISTANCE_THRESHOLD = 10.0  # ±10mm
    YAW_THRESHOLD = 2.0        # ±2도

    return (abs(distance_error_mm) < DISTANCE_THRESHOLD and
            abs(yaw_error_deg) < YAW_THRESHOLD)
```

---

## UI 설계

### RQT Plugin 구조

#### 클래스
```python
class PrecisionNavUI(Plugin):
    def __init__(self, context):
        # ROS2 노드
        self.node = context.node

        # Publisher
        self.goal_pub = self.node.create_publisher(
            PrecisionNavGoal, '/precision_nav/goal', 10
        )

        # Subscriber
        self.feedback_sub = self.node.create_subscription(
            PrecisionNavFeedback, '/precision_nav/feedback',
            self.feedback_callback, 10
        )
        self.status_sub = self.node.create_subscription(
            PrecisionNavStatus, '/precision_nav/status',
            self.status_callback, 10
        )

        # UI 위젯
        self.setup_ui()
```

#### UI 이벤트
```python
def on_forward_600_clicked(self):
    """전진 600mm 버튼"""
    goal = PrecisionNavGoal()
    goal.type = PrecisionNavGoal.TYPE_LINEAR
    goal.distance = 600.0  # mm
    goal.max_velocity = 0.3  # m/s
    self.goal_pub.publish(goal)

def on_backward_600_clicked(self):
    """후진 600mm 버튼"""
    goal = PrecisionNavGoal()
    goal.type = PrecisionNavGoal.TYPE_LINEAR
    goal.distance = -600.0  # mm
    goal.max_velocity = 0.3  # m/s
    self.goal_pub.publish(goal)

def on_rotate_left_90_clicked(self):
    """좌회전 90도 버튼"""
    goal = PrecisionNavGoal()
    goal.type = PrecisionNavGoal.TYPE_ANGULAR
    goal.angle = 90.0  # deg
    goal.max_velocity = 0.5  # rad/s
    self.goal_pub.publish(goal)
```

#### 피드백 표시
```python
def feedback_callback(self, msg):
    """진행 상황 업데이트"""
    # 진행률 프로그레스바
    self.progress_bar.setValue(int(msg.progress * 100))

    # 거리 표시
    self.distance_label.setText(
        f"{msg.current_distance:.1f} / {msg.target_distance:.1f} mm"
    )

    # Yaw 오차 표시
    self.yaw_error_label.setText(f"Yaw Error: {msg.yaw_error:.2f}°")
```

---

## 통합 방안

### integrated_control_debug.sh 수정

ZED 노드 추가:

```bash
# ZED X mini 카메라 노드
log_msg "[4/5] ZED X mini 카메라 노드 시작..."
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm >> $LOG_FILE 2>&1 &
ZED_PID=$!
log_msg "  - PID: $ZED_PID"
sleep 3

# Precision Navigation Controller 노드
log_msg "[5/5] Precision Navigation Controller 노드 시작..."
ros2 run rebar_control precision_navigation_node >> $LOG_FILE 2>&1 &
PRECISION_PID=$!
log_msg "  - PID: $PRECISION_PID"
sleep 2
```

### ROS2 Launch 파일

**파일**: `src/rebar_control/launch/precision_navigation.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ZED X mini 카메라
        Node(
            package='zed_wrapper',
            executable='zed_wrapper',
            name='zedxm',
            parameters=[
                {'camera_model': 'zedxm'},
                {'grab_resolution': 'HD1080'},
                {'grab_frame_rate': 30},
            ],
            output='screen'
        ),

        # Precision Navigation Controller
        Node(
            package='rebar_control',
            executable='precision_navigation_node',
            name='precision_navigation',
            parameters=[
                {'heading_kp': 0.5},
                {'heading_ki': 0.0},
                {'heading_kd': 0.1},
                {'distance_kp': 0.3},
                {'distance_ki': 0.0},
                {'distance_kd': 0.05},
                {'distance_threshold_mm': 10.0},
                {'yaw_threshold_deg': 2.0},
            ],
            output='screen'
        ),
    ])
```

---

## 구현 계획

### Phase 1: ZED 통합 및 토픽 확인 (Week 1)

#### 1.1 ZED SDK 버전 이슈 해결
- [ ] ZED SDK 버전 확인 및 업데이트
- [ ] wrapper 재빌드
- [ ] 토픽 확인: `/zedxm/imu/data`, `/zedxm/odom`

#### 1.2 테스트 스크립트
```bash
# ZED 토픽 확인
ros2 topic list | grep zedxm
ros2 topic hz /zedxm/imu/data
ros2 topic hz /zedxm/odom
ros2 topic echo /zedxm/odom --once
```

---

### Phase 2: Precision Navigation Controller 개발 (Week 2)

#### 2.1 기본 구조
- [ ] `precision_navigation_node.py` 생성
- [ ] IMU/Odom 구독자 구현
- [ ] State Machine 구현

#### 2.2 제어기 구현
- [ ] Heading PID Controller
- [ ] Distance PID Controller
- [ ] `/cmd_vel_precise` 발행

#### 2.3 테스트 (CLI)
```bash
# 목표 명령 테스트
ros2 topic pub /precision_nav/goal rebar_msgs/PrecisionNavGoal \
  "{type: 0, distance: 600.0, max_velocity: 0.3}" --once

# 피드백 확인
ros2 topic echo /precision_nav/feedback
```

---

### Phase 3: RQT UI 개발 (Week 3)

#### 3.1 플러그인 생성
- [ ] `rqt_precision_nav.py` 작성
- [ ] UI 레이아웃 (.ui 파일 또는 코드)
- [ ] RQT plugin 등록

#### 3.2 기능 구현
- [ ] Quick Commands 버튼
- [ ] Custom Commands 입력
- [ ] 실시간 피드백 표시
- [ ] PID 게인 조정

#### 3.3 테스트
```bash
rqt --standalone rebar_control.rqt_precision_nav
```

---

### Phase 4: iron_md_teleop 통합 (Week 4)

#### 4.1 Command Mux 구현
- [ ] `/cmd_vel_precise` 구독자 추가
- [ ] `/precision_nav/status` 구독자 추가
- [ ] `control_loop()` 수정 (Mux 로직)

#### 4.2 테스트 시나리오
1. S20 모드 + 리모콘 AN3 → 기존 동작 확인
2. S20 모드 + UI 명령 (600mm) → Precision nav 동작
3. Precision nav 동작 중 리모콘 AN3 → 무시되는지 확인
4. Precision nav 완료 후 리모콘 AN3 → 다시 동작하는지 확인

---

### Phase 5: PID 튜닝 및 최적화 (Week 5)

#### 5.1 실제 로봇 테스트
- [ ] 600mm 전진 테스트 (10회 반복)
  - 거리 오차 측정
  - Yaw 틀어짐 측정
- [ ] PID 게인 수동 튜닝
- [ ] 최적 게인 저장

#### 5.2 성능 검증
- [ ] 목표: 거리 오차 ±10mm 이내
- [ ] 목표: Yaw 오차 ±2도 이내
- [ ] 재현성 테스트 (20회)

---

## 추가 고려사항

### 1. 안전 기능
- **비상 정지**: Emergency Stop 시 Precision nav 중단
- **타임아웃**: 목표 도달 실패 시 10초 후 자동 중단
- **리미트 센서**: EZI-IO 센서와 연동하여 충돌 방지

### 2. 로깅
```python
# 디버그 로그
self.get_logger().info(f'Goal: {distance}mm, Current: {current}mm, Error: {error}mm')
self.get_logger().debug(f'PID output: linear.x={linear_x:.3f}, angular.z={angular_z:.3f}')
```

### 3. 확장성
이 시스템은 추후 상위 제어 노드에서 재사용 가능:
```python
# 자동화된 작업 시퀀스
mission_planner.move_to(x=600, y=0)  # Precision nav 사용
mission_planner.rotate(90)
mission_planner.move_to(x=0, y=400)
```

---

## 예상 토픽 구조

```
/zedxm/imu/data (100Hz)
  └─> [Precision Nav Controller]
        ├─> /cmd_vel_precise (50Hz)
        │     └─> [iron_md_teleop (Mux)]
        │           └─> /cmd_vel
        │                 └─> [position_control_node]
        │                       └─> 0x141, 0x142 모터
        ├─> /precision_nav/feedback
        │     └─> [RQT UI] (표시)
        └─> /precision_nav/status
              └─> [iron_md_teleop (Mux)]

/zedxm/odom (30Hz)
  └─> [Precision Nav Controller]

/precision_nav/goal
  ├─> [RQT UI] (발행)
  └─> [Precision Nav Controller] (구독)
```

---

## 다음 단계

✅ **즉시 시작**: Phase 1 - ZED SDK 이슈 해결

1. ZED SDK 버전 확인
2. wrapper 재빌드
3. 토픽 확인 스크립트 실행
4. IMU/Odom 데이터 검증

---

**작성일**: 2025-12-10
**최종 수정**: 2025-12-10
**담당자**: Koceti Robotics Team
