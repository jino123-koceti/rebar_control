# 모터 제어 수정 사항

## 수정된 문제들

### 1. ✅ 0x142 회전 방향 반대로 수정
**파일**: `src/rmd_robot_control/rmd_robot_control/position_control_node.py`

**문제**: 0x141과 0x142 모터 둘 다 반전되어 있어서 결과적으로 반전 효과 없음

**수정 전**:
```python
# 0x141(왼쪽) 모터 방향 반전
left_dps = -left_dps

# 0x142(오른쪽) 모터 방향 반전
right_dps = -right_dps
```

**수정 후**:
```python
# 0x142(오른쪽) 모터만 방향 반전 (반대편 장착으로 회전 방향 반대)
right_dps = -right_dps
```

**결과**: 
- 전진 시: 0x141(+), 0x142(-) → 둘 다 앞으로 회전
- 후진 시: 0x141(-), 0x142(+) → 둘 다 뒤로 회전

---

### 2. ✅ AN2 충돌 문제 해결
**파일**: `src/rebar_control/rebar_control/iron_md_teleop_node.py`

**문제**: AN2가 주행 회전과 Y축 제어에 동시 사용되어 충돌

**수정 전**:
```python
def handle_driving(self):
    linear = -self.normalize_joystick(self.joystick_data['AN3'])
    angular = -self.normalize_joystick(self.joystick_data['AN2'])  # ❌ AN2 사용
    
def handle_xyz_stage(self):
    y_value = -self.normalize_joystick(self.joystick_data['AN2'])  # ❌ AN2 중복
```

**수정 후**:
```python
def handle_driving(self):
    linear = -self.normalize_joystick(self.joystick_data['AN3'])
    angular = 0.0  # ✅ AN2 제거, 직진만 제어
    
def handle_xyz_stage(self):
    y_value = -self.normalize_joystick(self.joystick_data['AN2'])  # ✅ Y축 전용
```

**결과**:
- AN3 → 주행 전후진만 제어 (0x141, 0x142)
- AN2 → Y축(0x145) 전용 제어
- AN1 → X축(0x144) 전용 제어

---

### 3. ✅ 위치 제어 단위 수정 (미터 → 도)
**파일**: `src/rebar_control/rebar_control/iron_md_teleop_node.py`

**문제**: RMD 모터는 도(degree) 단위를 요구하는데 미터(m) 단위로 전송

**수정 전**:
```python
self.declare_parameter('xyz_step_size', 0.01)  # m, 10mm per command
self.declare_parameter('lateral_move_distance', 0.05)  # m, 50mm

self.current_positions = {
    'x': 0.0,  # 미터
    'y': 0.0,  # 미터
    ...
}

self.get_logger().info(f'{joint_name.upper()}: {position:.3f} m')
```

**수정 후**:
```python
self.declare_parameter('xyz_step_size', 10.0)  # degree, 10도 per command
self.declare_parameter('lateral_move_distance', 50.0)  # degree, 50도

self.current_positions = {
    'x': 0.0,  # degree
    'y': 0.0,  # degree
    ...
}

self.get_logger().info(f'{joint_name.upper()}: {position:.2f}°')
```

**결과**:
- 조이스틱 움직임 1회당 10도 이동
- 스위치 1회 누름당 50도 이동
- RMD 모터가 제대로 위치 명령 받을 수 있음

---

## 조이스틱 매핑 (최종)

### 주행 제어 (0x141, 0x142)
- **AN3**: 전후진만 제어
  - 위로: 전진
  - 아래로: 후진
  - 좌우: 무시 (회전 없음)

### 위치 제어 (0x143-0x147)
- **AN1**: X축 제어 (0x144)
  - 좌우로 움직이면 X축 이동
  - 10도/명령
  
- **AN2**: Y축 제어 (0x145)
  - 상하로 움직이면 Y축 이동
  - 10도/명령

- **S17/S18**: 횡이동 (0x143)
  - S17: +50도
  - S18: -50도

- **S23/S24**: Yaw 회전 (0x147)
  - S23: +30도
  - S24: -30도

- **Z축 (0x146)**: 작업 시퀀스에서 제어

---

## 테스트 방법

### 1. 시스템 시작
```bash
cd /home/test/ros2_ws
source install/setup.bash
./integrated_control.sh
```

### 2. 브레이크 해제 (필수!)
```bash
# S13 버튼 누르기
# 또는 서비스로:
ros2 service call /safe_brake_release std_srvs/srv/Trigger
```

### 3. 주행 테스트 (AN3)
- AN3 위로 → 전진 (0x141: +, 0x142: -)
- AN3 아래로 → 후진 (0x141: -, 0x142: +)
- **둘 다 같은 방향으로 회전해야 함**

### 4. 위치 제어 테스트 (AN1, AN2)
- AN1 좌우 → X축 이동 (0x144)
- AN2 상하 → Y축 이동 (0x145)
- **주행 모터(0x141, 0x142)는 움직이지 않아야 함**

### 5. 모니터링 (별도 터미널)
```bash
# 주행 명령 확인
ros2 topic echo /cmd_vel

# 위치 명령 확인
ros2 topic echo /joint_2/position  # X축
ros2 topic echo /joint_3/position  # Y축

# CAN 버스 모니터링
candump can2
```

---

## 예상 동작

### AN3 위로 (전진)
```
/cmd_vel: linear.x=0.5, angular.z=0.0
→ 0x141: +속도
→ 0x142: -속도 (방향 반전)
→ 로봇 전진
```

### AN2 위로 (Y축 전진)
```
/joint_3/position: [10.0]  # 10도
→ 0x145: 10도 이동 명령
→ 0x141, 0x142: 정지
→ Y축만 이동
```

### AN1 오른쪽 (X축 오른쪽)
```
/joint_2/position: [10.0]  # 10도
→ 0x144: 10도 이동 명령
→ 0x141, 0x142: 정지
→ X축만 이동
```

---

## 문제 해결 체크리스트

### 0x141, 0x142가 움직이지 않는 경우
- [ ] 브레이크 해제했는지 확인 (S13 또는 서비스)
- [ ] CAN2 버스 상태 확인: `ip link show can2`
- [ ] `/cmd_vel` 토픽 발행 확인: `ros2 topic echo /cmd_vel`
- [ ] AN3 조이스틱 중립(127) 벗어났는지 확인

### 0x143-0x147이 움직이지 않는 경우
- [ ] 브레이크 해제했는지 확인
- [ ] 위치 토픽 발행 확인: `ros2 topic echo /joint_X/position`
- [ ] `position_control_node`가 실행 중인지 확인
- [ ] CAN2 응답 확인: `candump can2 | grep 24[3-7]`
- [ ] 조이스틱 값이 데드존(20) 벗어났는지 확인

### 방향이 반대인 경우
- 0x141 방향 반대: `left_dps`에 `-` 추가
- 0x142 방향 반대: `right_dps`에서 `-` 제거
- 위치 모터 방향 반대: `angle_control` 부호 변경

---

## 변경 파일 요약
1. ✅ `rmd_robot_control/position_control_node.py` - 0x142만 방향 반전
2. ✅ `rebar_control/iron_md_teleop_node.py` - AN2 충돌 제거, 단위 degree로 변경
3. ✅ 패키지 빌드 완료

**다음 단계**: `./integrated_control.sh` 실행 후 테스트
