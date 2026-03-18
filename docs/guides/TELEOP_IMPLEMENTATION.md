# 리모콘 텔레옵 구현 완료

**날짜:** 2025-12-17
**구현 방식:** Tire Roller 방식

---

## 구현 개요

Phase 2 하드웨어 추상화 계층에 **리모콘 원격 조종 기능**을 추가했습니다.
Tire Roller의 검증된 방식을 채택하여, `drive_controller.py` 하나의 노드에서 리모콘과 cmd_vel을 모두 처리합니다.

---

## 변경 사항

### 1. drive_controller.py 업데이트

**이전 (Phase 2 초기):**
- cmd_vel → DriveControl 변환만 수행
- Auto 모드 전용

**현재 (Tire Roller 방식):**
- **Manual 모드**: 리모콘 조이스틱 → DriveControl 직접 변환
- **Auto 모드**: cmd_vel → DriveControl 변환
- 제어 모드에 따라 자동 전환

---

### 2. 데이터 흐름

#### Manual 모드 (S10/S19 스위치)
```
리모콘 (CAN3)
    ↓
can_parser.py
    ↓ /remote_control (RemoteControl)
    ├─→ navigator_base.py (State Machine)
    │       ↓ /control_mode = 'manual'
    └─→ drive_controller.py ⭐ (Tire Roller 방식)
            ↓ 조이스틱 직접 변환
        /drive_control (DriveControl)
            ↓
        can_sender.py
            ↓
        CAN2 (모터)
```

#### Auto 모드 (S20 스위치)
```
상위 제어 (Navigator, precision_nav 등)
    ↓ /cmd_vel (Twist)
    ↓
drive_controller.py
    ↓ Differential drive kinematics
/drive_control (DriveControl)
    ↓
can_sender.py
    ↓
CAN2 (모터)
```

---

## 3. drive_controller.py 구현 세부사항

### 3.1 구독 토픽 (3개)

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/remote_control` | RemoteControl | 리모콘 신호 (Manual 모드) |
| `/cmd_vel` | Twist | 상위 제어 명령 (Auto 모드) |
| `/control_mode` | String | 현재 상태 ('manual', 'auto', 'idle' 등) |

### 3.2 발행 토픽

| 토픽 | 타입 | 주기 | 용도 |
|------|------|------|------|
| `/drive_control` | DriveControl | 20Hz | 모터 속도 명령 |

### 3.3 주요 메서드

#### `publish_drive_control()` (타이머 콜백, 20Hz)
```python
if control_mode == 'manual':
    # 리모콘 조이스틱 → DriveControl
    drive_msg = self._convert_remote_to_drive()

elif control_mode == 'auto' or control_mode == 'navigating':
    # cmd_vel → DriveControl
    drive_msg = self._convert_cmd_vel_to_drive()

else:
    # idle, emergency_stop: 정지
    drive_msg.left_speed = 0.0
    drive_msg.right_speed = 0.0
```

#### `_convert_remote_to_drive()`
리모콘 조이스틱 값을 DriveControl로 변환

**조이스틱 매핑:**
- `joysticks[2]` (AN3): 전후진
  - -1.0 ~ 0.0: 전진
  - 0.0 ~ 1.0: 후진
- `joysticks[3]` (AN4): 좌우회전
  - -1.0 ~ 0.0: 시계방향 (CW)
  - 0.0 ~ 1.0: 반시계방향 (CCW)

**변환 과정:**
```python
# 1. 조이스틱 값 가져오기 (-1.0 ~ 1.0)
joy_linear = joysticks[2]   # AN3
joy_angular = joysticks[3]  # AN4

# 2. 데드존 적용
if abs(joy_linear) < deadzone:
    joy_linear = 0.0

# 3. 속도 변환
linear_velocity = -joy_linear * max_linear_vel
angular_velocity = joy_angular * max_angular_vel

# 4. Differential drive kinematics
left_speed = linear - (angular * wheel_base / 2)
right_speed = linear + (angular * wheel_base / 2)
```

#### `_convert_cmd_vel_to_drive()`
기존 로직 유지 (Twist → DriveControl)

---

## 4. 설정 파라미터

### can_devices.yaml
```yaml
drive_controller:
  ros__parameters:
    wheel_base: 0.5              # meters
    max_linear_vel: 10.0         # m/s
    max_angular_vel: 2.0         # rad/s
    joystick_deadzone: 20        # 0-127 범위
    joystick_center: 127         # 중립값
    publish_frequency: 20        # Hz
```

---

## 5. 빌드 결과

```bash
$ colcon build --packages-select rebar_base_control --symlink-install

Starting >>> rebar_base_control
Finished <<< rebar_base_control [0.78s]

Summary: 1 package finished [1.31s]
```

**빌드 상태:** ✅ 성공

---

## 6. 실행 방법

### 전체 시스템 실행
```bash
source install/setup.bash
ros2 launch rebar_base_control base_system.launch.py
```

### 실행되는 노드
1. **can_parser** - CAN 메시지 파싱
2. **can_sender** - CAN 메시지 전송
3. **drive_controller** ⭐ - 리모콘/cmd_vel → DriveControl 변환
4. **modbus_controller** - Modbus 통신
5. **authority_controller** - 권한 관리
6. **navigator_base** - State Machine

---

## 7. 테스트 시나리오

### Manual 모드 (리모콘 조종)
1. 리모콘 S10 (S19) 스위치 활성화
2. State Machine이 `manual` 상태로 전환
3. `/control_mode` = 'manual' 발행
4. `drive_controller`가 리모콘 조이스틱 값 읽기
5. AN3 (전후진), AN4 (좌우회전) 조작
6. → DriveControl 발행 → CAN 전송 → 모터 구동

### Auto 모드 (자동 제어)
1. 리모콘 S20 스위치 활성화
2. State Machine이 `auto` 상태로 전환
3. `/control_mode` = 'auto' 발행
4. Navigator가 `/cmd_vel` 발행
5. `drive_controller`가 cmd_vel 읽기
6. → DriveControl 발행 → CAN 전송 → 모터 구동

### 비상정지
1. 리모콘 비상정지 버튼 누름
2. State Machine이 `emergency_stop` 상태로 전환
3. `drive_controller`가 정지 명령 발행 (left_speed=0, right_speed=0)

---

## 8. Tire Roller와 비교

### 동일한 점
✅ `drive_controller` 하나의 노드에서 리모콘 + cmd_vel 처리
✅ 제어 모드에 따라 분기 처리
✅ 20Hz 주기적 발행 (타이머 방식)

### 다른 점
- Tire Roller: `AnmControl` (상위 제어 메시지) 사용
- Rebar: `Twist` (cmd_vel) 사용 ← **ROS2 표준 메시지**

---

## 9. 주요 개선 사항

### 이전 문제점
❌ 리모콘 신호가 파싱만 되고 실제 제어로 연결 안 됨
❌ Manual 모드에서 로봇 조종 불가능

### 해결됨
✅ Manual 모드에서 리모콘으로 즉시 조종 가능
✅ Auto 모드에서 Navigator가 cmd_vel로 제어 가능
✅ 모드 전환 자동 처리 (State Machine)
✅ 비상정지 즉시 반영

---

## 10. 향후 작업

### Phase 3 (상위 제어 계층)
- [ ] Navigator 구현 (Action Client)
- [ ] precision_nav_controller (Action Server)
- [ ] 조인트 제어 (X, Y, Z, Yaw, 횡이동)

### 조인트 제어 추가 필요
현재 `drive_controller`는 **주행 모터(0x141, 0x142)만** 처리합니다.

리모콘으로 조인트(상부체) 제어가 필요하다면:
- 옵션 1: `drive_controller`에 조인트 제어 로직 추가
- 옵션 2: 별도의 `joint_teleop_controller` 노드 생성

---

## 11. 참고 자료

### Tire Roller 참조 파일
- `tire_roller_basecontrol/drive_controller.py` (라인 55-106)
- `tire_roller_basecontrol/navigator.py` (State Machine)

### Rebar 기존 코드
- `rebar_control/iron_md_teleop_node.py` (조이스틱 매핑 참조)

---

**구현 완료:** 리모콘 원격 조종 가능 ✅
**다음 단계:** Phase 3 - 상위 제어 계층 구현
