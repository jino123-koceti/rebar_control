# Joint Controller Refactoring Summary

**날짜:** 2025-12-18
**목적:** Tire Roller 스타일로 간결하게 리팩토링 + uint8 오버플로우 버그 수정

---

## 🐛 해결된 핵심 버그

### uint8 오버플로우 문제
```python
# ❌ 이전 (JointControl.msg)
uint8 joint_id  # 0-255 범위만 가능

# 실제 모터 ID
0x143 = 323 (decimal)  # ❌ uint8 범위 초과!
0x147 = 327 (decimal)  # ❌ uint8 범위 초과!

# ✅ 수정 후
uint16 joint_id  # 0-65535 범위 (0x143=323, 0x147=327 모두 커버)
```

### 잘못된 우회 시도 제거
```python
# ❌ 이전 joint_controller.py
joint_id=0x43,  # 0x143 모터의 uint8 표현 → CAN ID 0x43으로 전송됨!

# ✅ 수정 후
joint_id=0x143,  # 올바른 CAN ID 직접 사용
```

---

## 🔄 개발 스토리

1. **위치 제어 (0xA4) 1차 시도**
   - `joint_controller.py.bak` (281 lines)
   - 모터별 위치 추적, 복잡한 구조
   - uint8 오버플로우 문제 존재

2. **속도 제어 (0xA2) 전환**
   - `drive_controller.py`에 통합 시도
   - 속도 적분 방식으로 위치 추적
   - 정밀도 문제 발생 (270° + 90° 역회전)

3. **위치 제어 (0xA4) 복귀 + Tire Roller 스타일 리팩토링** ✅
   - 152 lines (34% 코드 감소)
   - uint16 타입 수정으로 버그 해결
   - 간결한 구조, 명확한 로직

---

## 📊 코드 비교

### 라인 수 변화
| 버전 | 라인 수 | 특징 |
|------|---------|------|
| 백업 (v1) | 281 lines | 복잡한 구조, 모터별 상태 추적 |
| 이전 (v2) | 231 lines | 단순화 시도, uint8 버그 |
| **현재 (v3)** | **152 lines** | **Tire Roller 스타일, 버그 수정** |

### 구조 비교

#### ❌ 이전 구조 (Event-driven)
```python
def remote_control_callback(self, msg):
    # 콜백에서 직접 처리
    if self.control_mode != 'manual':
        return

    self._handle_lateral_move(msg)
    self._handle_yaw_rotation(msg)
    self.prev_buttons = list(msg.buttons)
```

#### ✅ 현재 구조 (Timer-driven, Tire Roller 패턴)
```python
def recv_remote(self, msg: RemoteControl):
    # 메시지 저장만
    self.remote_msg = msg

def process_joint_control(self):
    # Timer (20Hz)에서 주기적으로 처리
    if self.control_mode != 'manual':
        return

    msg = self.remote_msg
    self._handle_lateral(msg)
    self._handle_yaw(msg)
    self.prev_buttons = list(msg.buttons)
```

---

## 🎯 Tire Roller 스타일 적용

### 1. Timer 기반 주기 처리
```python
# 20Hz 주기 (tire_roller와 동일)
self.control_frequency = 20
self.timer = self.create_timer(1/self.control_frequency, self.process_joint_control)
```

### 2. recv_* 콜백 패턴
```python
def recv_status(self, msg: String):
    self.control_mode = msg.data

def recv_remote(self, msg: RemoteControl):
    self.remote_msg = msg
```

### 3. QoS 프로파일 적용
```python
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# Sensor data (리모콘)
self.remote_subscriber = self.create_subscription(
    RemoteControl, '/remote_control', self.recv_remote, qos_profile_sensor_data)

# System default (제어 명령)
self.joint_publisher = self.create_publisher(
    JointControl, '/joint_control', qos_profile_system_default)
```

### 4. 간결한 상태 관리
```python
# State
self.control_mode = 'idle'
self.remote_msg = RemoteControl()
self.prev_buttons = [0] * 8
self.last_command_time = None
```

---

## 🗑️ 제거된 불필요한 기능

### 1. 모터별 위치 추적
```python
# ❌ 이전 (불필요한 복잡도)
self.current_positions = {
    0x143: 0.0,
    0x144: 0.0,
    0x145: 0.0,
    0x146: 0.0,
    0x147: 0.0,
}
self.current_positions[motor_id] += delta_position
```

**이유:** `can_sender.py`가 이미 `self.joint_positions`로 추적 중

### 2. 모터별 명령 간격 제어
```python
# ❌ 이전 (과도한 세분화)
self.last_command_time = {}  # 모터별 딕셔너리

if motor_id in self.last_command_time:
    elapsed = current_time - self.last_command_time[motor_id]
    ...
```

**이유:** 전역 간격 제어로 충분 (리모콘은 하나의 모터만 동시 제어)

### 3. 절대 위치 제어 메서드
```python
# ❌ 이전 (미사용 코드)
def _send_absolute_position(self, motor_id, target_position, max_speed, name='Joint'):
    # Auto 모드용, 추후 구현
    ...
```

**이유:** 현재 Manual 모드만 사용, 필요 시 추가 가능

### 4. time.time() 사용
```python
# ❌ 이전
import time
current_time = time.time()

# ✅ 수정 후
self.get_clock().now()  # ROS2 권장 방식
```

---

## ✅ 테스트 체크리스트

- [x] JointControl.msg uint16 변경
- [x] rebar_base_interfaces 빌드 성공
- [x] rebar_base_control 빌드 성공
- [x] ros2 interface show 검증 (uint16 확인)
- [ ] vcan 테스트
  - [ ] S17 → CAN ID 0x143 확인
  - [ ] S18 → CAN ID 0x143 확인
  - [ ] S23/S24 → CAN ID 0x147 확인
- [ ] 실제 모터 테스트
  - [ ] 횡이동 +360° (5cm)
  - [ ] 횡이동 -360° (5cm)
  - [ ] 누적 이동 (3×360° = 1080°)
  - [ ] Yaw 회전 ±5°

---

## 📝 남은 작업

### 우선순위 HIGH
1. vcan 환경에서 CAN 메시지 검증
   ```bash
   # Terminal 1
   ros2 launch rebar_base_control base_system.launch.py

   # Terminal 2
   candump vcan0 -x | grep -E "143|147"

   # Terminal 3 (리모콘 시뮬레이션)
   ros2 topic pub /remote_control rebar_base_interfaces/msg/RemoteControl \
     "{buttons: [0,0,1,0,0,0,0,0]}"  # S17 누름
   ```

2. 실제 하드웨어 테스트
   - 0x143 모터 반응 확인
   - 정밀도 측정 (±5mm 이내)

### 우선순위 MEDIUM
3. S17_S18_LATERAL_FIX_SUMMARY.md 업데이트
   - 최종 해결 방법 기록
   - Tire Roller 스타일 적용 언급

4. PHASE2_SUMMARY.md 업데이트
   - joint_controller 완성 표시
   - 코드 라인 수 업데이트

---

## 🎉 결론

**상태:** ✅ **완성**

- 치명적 버그(uint8 오버플로우) 해결
- Tire Roller 스타일로 간결하게 리팩토링 (34% 코드 감소)
- 명확한 구조, 유지보수 용이
- 빌드 성공, 인터페이스 검증 완료

**다음 단계:** vcan 테스트 → 실제 모터 테스트
