# Homing Controller 분리 및 신규 시퀀스 구현 계획

## 1. 목표

`joint_controller.py` (1772줄)에서 호밍 관련 코드(~500줄)를 `homing_controller.py`로 분리하고,
새로운 호밍 시퀀스를 구현한다.

## 2. 현재 구조

### 기존 호밍 시퀀스 (joint_controller.py)
```
Z_UP (z_min까지 상승)
  → COARSE_HOME (X+Y+Yaw 동시 리미트 이동)
  → BACK_OFF (리미트에서 후퇴)
  → FINE_HOME (느린 속도로 리미트 재접근)
  → MOVE_TO_READY (준비 위치 이동)
  → COMPLETE
```

### 기존 호밍 코드 위치 (joint_controller.py)
| 라인 | 내용 |
|------|------|
| 26~35 | `HomingState` Enum |
| 81~92 | 호밍 파라미터 선언 |
| 115~126 | 호밍 파라미터 값 로드 |
| 191~195 | `/homing_cmd` 구독, `/homing_status` 발행 |
| 233~242 | 호밍 상태 변수 초기화 |
| 341~343 | `_recv_limit_sensor`에서 호밍 중 리미트 트리거 호출 |
| 567~572 | `process_joint_control`에서 `_homing_loop()` 호출 |
| 931~938 | `_send_homing_speeds()` |
| 1197~1239 | `_mission_command_callback` 내 EMERGENCY_STOP 처리 (호밍 중지) |
| 1242~1284 | `_recv_homing_cmd()` (START/STOP/Z_HOME/SET_READY) |
| 1286~1298 | `_homing_enter_state()`, `_publish_homing_status()` |
| 1301~1599 | `_homing_loop()` 전체 상태머신 |
| 1601~1663 | `_homing_on_limit_triggered()`, `_homing_fail()` |
| 1665~1751 | `_move_axes_to_ready()`, `_set_ready_position()` |

### 호밍이 사용하는 공유 리소스
- **모터 명령 함수**: `_send_joint_command_abs()`, `_send_joint_command_rel()`, `_send_speed_command()`
- **리미트 센서**: `self.limit_sensors` dict (joint_controller가 구독 중)
- **모터 각도**: `stage_x_angle`, `stage_y_angle`, `stage_z_angle`, `yaw_angle` (0x92 피드백)
- **Yaw 엔코더**: `yaw_encoder_90` (0x90 싱글턴 엔코더)
- **발행 토픽**: `/joint_control` (JointControl), `/homing_status` (String)

## 3. 새 호밍 시퀀스

```
1. Z_SAFE     — Z_min 리미트 체크 → 미도달 시 Z_min까지 상승, 도달 시 skip
2. X_SAFE     — X_min 리미트 체크 → 미도달 시 X_min까지 이동, 도달 시 skip
3. YAW_CHECK  — Yaw 0x90 싱글턴 엔코더로 현재 자세 판단
                - 좌측(Left) 자세이면 → L→R 자세복귀 시퀀스 실행
                - 우측(Right) 자세이면 → skip
4. YAW_SAFE   — Yaw 리미트(home) 체크 → 미도달 시 Yaw home까지 이동, 도달 시 skip
5. Y_SAFE     — Y_min 리미트 체크 → 미도달 시 Y_min까지 이동, 도달 시 skip
6. BACK_OFF   — X+Y+Yaw 리미트에서 살짝 후퇴 (센서 해제될 때까지)
7. FINE_HOME  — 느린 속도로 X+Y+Yaw 리미트 재접근 (정밀 호밍, 레퍼런스 기록)
8. READY      — 준비 위치로 이동 (X 3단계 분할, Y/Z/Yaw 동시)
9. COMPLETE   — 레퍼런스 발행, IDLE 복귀
```

### Yaw 자세 판단 기준
- 0x90 싱글턴 엔코더 값 기반
- `yaw_home_encoder_90` (16328) = Right 자세 (home 근처)
- `yaw_max_encoder_90` (44710) = Left 자세 (풀 스트로크)
- 판단 임계: 중간값 (~30000) 기준, 이상이면 Left, 이하면 Right
- tying_orchestrator.yaml 참고: Right 작업자세 = -3°, Left 작업자세 = 393°

### L→R 자세복귀 시퀀스 (tying_orchestrator 자세변경 역순)
1. X → 0mm (홈), Y → 250mm 이동
2. Yaw → 접근자세(390°)
3. Y → 100mm 이동
4. Yaw → 중간자세(173°)
5. X → 0mm (홈), Y → 0mm 이동

## 4. 구현 작업

### 4.1 homing_controller.py 신규 생성
- **패키지**: `rebar_base_control`
- **노드명**: `homing_controller`
- **구독 토픽**:
  - `/homing_cmd` (String) — START, STOP, Z_HOME, SET_READY
  - `/motor_feedback` (MotorFeedback) — 0x92/0x90 모터 피드백
  - `/limit_sensors/*` (Bool) — x_min, x_max, y_min, y_max, z_min, z_max, yaw_home
  - `/mission/command` (String) — EMERGENCY_STOP 시 호밍 중지
- **발행 토픽**:
  - `/joint_control` (JointControl) — 모터 명령
  - `/homing_status` (String) — 호밍 상태/완료 발행
  - `/encoder_request` (JointControl) — 0x90 엔코더 요청
- **파라미터**: can_devices.yaml에서 로드
  - homing_speed, homing_fine_speed, homing_timeout
  - yaw_home_encoder_90, yaw_max_encoder_90, yaw_full_stroke_deg
  - stage_x_step_deg, stage_y_step_deg, stage_z_step_deg
  - ready_x_mm, ready_y_mm, ready_z_mm, ready_yaw_deg
  - (신규) yaw_left_threshold_90: Left/Right 판단 임계값

### 4.2 joint_controller.py에서 제거할 코드
- `HomingState` Enum (homing_controller로 이동)
- 호밍 파라미터 선언/로드 (81~92, 115~126)
- `/homing_cmd` 구독, `/homing_status` 발행 (191~195)
- 호밍 상태 변수 (233~242)
- `_recv_limit_sensor`에서 호밍 트리거 호출 부분 (341~343) → 제거
- `process_joint_control`에서 `_homing_loop()` 호출 (567~572) → 제거
- `_send_homing_speeds()` (931~938) → 제거
- EMERGENCY_STOP 호밍 중지 (1197~1239) → 제거
- `_recv_homing_cmd()` ~ `_set_ready_position()` (1242~1751) → 전부 제거

### 4.3 joint_controller.py에 유지할 코드
- 모터 명령 함수: `_send_joint_command_abs/rel()`, `_send_speed_command()` → 유지 (조이스틱에도 사용)
- 리미트 센서 구독/상태: `_setup_limit_sensor_subscribers()`, `_recv_limit_sensor()`, `_check_limit_safe()` → 유지 (조이스틱 안전에도 사용)
- 모터 피드백: `recv_motor_feedback()` → 유지

### 4.4 setup.py 수정
```python
'homing_controller = rebar_base_control.homing_controller:main',
```

### 4.5 launch 파일 수정
- `full_system.launch.py`에 homing_controller 노드 추가
- params-file: can_devices.yaml

### 4.6 테스트
1. 빌드 후 서비스 재시작
2. GO_HOME 명령 → 새 시퀀스 순서대로 동작 확인
3. Left 자세에서 GO_HOME → 자세복귀 후 호밍 확인
4. 이미 홈 위치일 때 GO_HOME → 각 축 skip 확인
5. Z_HOME 명령 → Z축 단독 호밍 확인

## 5. 주의사항

- joint_controller에서 `self.is_homed` 플래그가 사라지므로, 필요 시 `/homing_status` 구독으로 대체
- `_recv_limit_sensor` 내 호밍 트리거 부분만 제거, 리미트 안전 체크는 유지
- tying_orchestrator가 `/homing_status`의 `COMPLETE:` 메시지를 파싱하므로 메시지 형식 유지 필수
- navigator.py도 `/homing_status` 구독 중 → 형식 유지
- can_devices.yaml에 신규 파라미터 추가 필요 (`yaw_left_threshold_90`)
