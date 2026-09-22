# RMD V4 Dual Encoder 출력축 절대 엔코더 활용 — 소스 수정 가이드

**대상 레포**: [jino123-koceti/rebar_control](https://github.com/jino123-koceti/rebar_control)
**대상 패키지**: `src/rmd_robot_control/rmd_robot_control/`
**작성일**: 2026-05-28
**예상 작업 분량**: 핵심 수정 ≤ 100 줄, 검증 포함 1~2일

---

## 0. TL;DR

| 항목 | 현재 | 수정 후 |
|---|---|---|자
| 위치 피드백 CAN 명령 | `0x92` (READ_MULTI_TURN_ANGLE — 모터부 누적 각도) | **`0x90` (READ_ENCODER_DATA — 출력축 절대 엔코더 포함 듀얼 엔코더)** |
| 피드백 출처 | 모터 rotor 측 엔코더 (감속비 × 백래시 영향) | **출력축 절대 엔코더 (백래시·슬립 무관, 17-bit ≈ 0.0027°)** |
| 전원 오프 + 임의 회전 후 | 0x92 누적값 잃음 → homing 필요 | **부팅 직후 즉시 절대 위치 복원** |
| 보고서 사례 #6, #7, #8 | HW 교체 검토 대상 | **SW 수정으로 해결 가능** |

수정 핵심: **명령 송신 3곳 + 응답 파서 1곳**. 프로토콜 레이어(`rmd_x4_protocol.py`)에는 dual encoder 파싱 함수가 이미 구현되어 있어, 컨트롤 노드에서 호출만 추가하면 됨.

---

## 1. 배경

### 1.1 HW는 이미 dual encoder 지원

`RMD-X4-P12-10-E` (V4 dual encoder) 모터는 두 개의 엔코더를 내장한다.

- **Encoder 1 (모터부, rotor 측)**: 18-bit, 모터 single-turn + 누적 멀티턴 카운트(배터리 백업)
- **Encoder 2 (출력축, after planetary gear)**: 17-bit single-turn absolute (≈ 0.0027°)

출력축 엔코더는 **감속기 뒤에서 직접 측정**하므로 백래시·슬립·전원 오프 후 외부 강제 회전과 무관하게 항상 출력축의 진짜 절대 위치를 알고 있다. Yaw(0x147)는 ±30° 가동범위이므로 17-bit 단일턴만으로 충분, 멀티턴 불필요.

### 1.2 SW가 출력축 엔코더를 미사용

`position_control_node.py` 분석 결과:
- `CommandType.READ_ENCODER_DATA (0x90)` 명령이 코드 전체에서 **한 번도 호출되지 않음**
- 대신 `CommandType.READ_MULTI_TURN_ANGLE (0x92)` 명령만 사용
- `0x92` 응답은 **모터부 누적 각도**(int32, 0.01도/LSB) 1개 값만 포함 → 감속기 뒤 백래시·슬립을 보지 못함

### 1.3 결과적으로 발생한 문제

- 보고서 Known Issues #2: "Yaw 0x90 encoder drift" → **실제로는 0x92 드리프트**
- 전원 오프 후 사용자 임의 회전 시 자기 위치 미지 → 매번 homing
- Yaw 5단계 pose change 시퀀스는 *알 수 없는 시작 자세*에서 안전하게 가기 위한 보수적 우회

`rmd_x4_protocol.py`에는 `_parse_encoder_response()`가 이미 구현되어 있고 18-bit/17-bit 두 엔코더를 정확히 파싱한다. **컨트롤 노드에서 0x90 명령을 송신하고 응답을 분기 처리하기만 하면 출력축 엔코더가 즉시 살아난다.**

---

## 2. 현재 코드 구조 분석

### 2.1 `rmd_x4_protocol.py` (변경 거의 불필요)

이미 다음 두 가지가 구현되어 있어 그대로 사용 가능.

```python
# CommandType IntEnum (이미 정의됨)
READ_ENCODER_DATA = 0x90            # ✅ 사용해야 할 명령
READ_MULTI_TURN_ANGLE = 0x92        # 현재 사용 중 (모터부만)
READ_SINGLE_TURN_ANGLE = 0x94

# 응답 파싱 (이미 구현됨)
def _parse_encoder_response(self, data: bytes) -> Dict[str, Any]:
    if len(data) < 9:
        return {'error': '데이터 길이 부족'}
    try:
        # 듀얼 엔코더 데이터 파싱
        encoder1 = struct.unpack('<i', data[1:5])[0]
        encoder2 = struct.unpack('<i', data[5:9])[0]
        return {
            'parsed': True,
            'encoder1': encoder1,
            'encoder2': encoder2,
            'encoder1_angle': encoder1 * 360.0 / (2**18),   # 18비트 (모터 측 추정)
            'encoder2_angle': encoder2 * 360.0 / (2**17)    # 17비트 (출력축 추정)
        }
    except Exception as e:
        return {'error': f'엔코더 응답 파싱 오류: {e}'}
```

> ⚠️ **검증 필요**: `encoder1`이 모터부인지 출력축인지, `encoder2`가 출력축인지 모터부인지는 **실측 검증 필요**. 한쪽 모터에 0x90을 보내고 손으로 출력축을 약간 회전시켰을 때 백래시 없이 즉시 따라오는 쪽이 출력축 엔코더. 본 가이드는 코드 주석을 따라 `encoder2`를 출력축으로 가정한다.

### 2.2 `position_control_node.py` 수정 필요 위치

| 위치 | 함수/맥락 | 현재 동작 | 수정 내용 |
|---|---|---|---|
| **Line 369~388** | `read_all_encoder_positions()` — 초기화/브레이크 해제 시 1회 호출 | `0x92` 전송 | `0x90` 전송으로 변경 |
| **Line 593~622** | 이동 중 주기적 위치 폴링 (control loop) | `0x92` 전송 | `0x90` 전송으로 변경 |
| **Line 1073~1075** | 또 다른 주기 폴링 지점 | `0x92` 전송 | `0x90` 전송으로 변경 |
| **Line 1143~1182** | `_handle_motor_response()` 내부 `elif command == READ_MULTI_TURN_ANGLE` 분기 | `data[4:8]`에서 angle_raw 파싱, `motor_states[id]['position']`에 저장 | **`elif command == READ_ENCODER_DATA` 분기 추가**, `encoder2_angle`을 `motor_states[id]['position']`에 저장 |

### 2.3 위치 피드백이 사용되는 곳

`motor_states[motor_id]['position']`과 `current_positions[motor_id]` 두 자료구조가 모든 모션 제어의 single source of truth. 다음 위치에서 참조:

- Line ~950: 목표 vs 현재 위치 비교 후 도달 판정
- Line ~960: `position_diff = abs(target_position - current_position)`
- 그 외 위치 제어 PID, homing, 이동 명령 등에서 직접 참조

→ **`motor_states[motor_id]['position']`만 출력축 절대값으로 갱신하면 나머지 로직은 변경 불필요.**

---

## 3. 수정 전략

### 3.1 점진적 마이그레이션 (권장)

리스크를 최소화하기 위해 **모든 축을 한 번에 바꾸지 말고 단계적으로** 적용한다.

| Phase | 적용 축 | 효과 | 리스크 |
|---|---|---|---|
| **Phase 0** | (검증용 테스트 스크립트) | 0x90 응답에서 encoder1/encoder2 거동 실측, 어느 게 출력축인지 확정 | 거의 없음 |
| **Phase 1** | **0x147 Yaw** | 5단계 pose change 시퀀스 단축 가능 검증 | 낮음 — 0x147은 ±30°라 single-turn으로 안전 |
| **Phase 2** | **0x143 횡이동** | 멀티턴 백래시 드리프트 제거 검증 | 중간 — multi-turn 카운트 별도 처리 필요 (3.3 참조) |
| **Phase 3** | **0x144, 0x145, 0x146** (XY/Z 스테이지) | 정밀도 향상 | 낮음 |
| **Phase 4 (선택)** | **0x141, 0x142** (주행) | 속도 제어라 위치 피드백 중요도 낮음 | 변경 불필요 가능 |

### 3.2 Dual mode 운영 (선택)

확신이 들 때까지 **0x90과 0x92를 둘 다 송신하고 둘 다 로깅**, 위치 피드백은 0x90을 primary로 하되 0x92와 비교 로그를 남기는 방식. 디버깅 시 백래시 양 측정 가능.

### 3.3 Multi-turn 처리 (0x143 횡이동 한정)

0x147 Yaw는 ±30°라서 17-bit single-turn(0~360°)만으로 충분. 그러나 **0x143 횡이동은 ±360° multi-turn**이라 출력축 single-turn absolute만으로는 회전수를 잃는다. 대응 방안:

- **방안 A**: 출력축 17-bit single-turn은 0x90으로 정밀 위치 확보, **모터부 multi-turn 카운트는 0x92(또는 0x90의 encoder1)에서 계속 사용**. 두 값을 합성하여 멀티턴 절대 위치 구성.
- **방안 B**: 운영상 ±180° 안에서만 사용하도록 SW limit 적용 → single-turn만으로 처리.
- **방안 C**: 0x143도 multi-turn 절대값 처리가 필요하면 모터의 multi-turn 배터리 백업 기능을 활용 (배터리 사양·수명 별도 관리).

신규 HW 설계 단계에서는 방안 B가 가장 단순. 본 가이드는 우선 Yaw 적용에 초점을 두며 횡이동은 별도 task로 분리.

---

## 4. 수정 가이드 — 파일별 Diff

### 4.1 (선택) `rmd_x4_protocol.py` — 코드 주석 보강

이미 동작하므로 의무 수정 없음. 다만 주석에 다음을 추가하면 추후 유지보수에 도움.

```python
# === 추가 권고 (수정 아님, 주석 보강) ===
def _parse_encoder_response(self, data: bytes) -> Dict[str, Any]:
    """0x90 응답 파싱 — RMD V4 Dual Encoder

    encoder1: 모터 측 엔코더 (18-bit, single-turn). 백래시·슬립 영향 받음.
    encoder2: 출력축 엔코더 (17-bit, single-turn absolute). 감속기 뒤에서 측정.

    출력축 엔코더 사용 권고:
    - 전원 오프 + 외부 회전 후에도 즉시 절대 위치 복원
    - 백래시·슬립 무관
    - Yaw(0x147), Z(0x146) 등 단일턴 동작 축에 적합
    """
```

### 4.2 `position_control_node.py` — 핵심 수정

#### 수정 A. `read_all_encoder_positions()` (Line 369~388 부근)

**Before:**
```python
def read_all_encoder_positions(self):
    """모든 모터의 현재 엔코더 위치 읽기 (초기화 및 브레이크 해제 시 사용)"""
    multi_turn_cmd = self.protocol.create_system_command(CommandType.READ_MULTI_TURN_ANGLE)

    all_motors = list(self.motor_ids) + [self.left_motor_id, self.right_motor_id]
    for i, motor_id in enumerate(all_motors):
        if i > 0:
            time.sleep(0.2)
        # 멀티턴 각도 읽기 (0x92)
        self.can_manager.send_frame(motor_id, multi_turn_cmd)
        self.get_logger().info(f"  → 0x{motor_id:03X} 위치 읽기 요청")
    time.sleep(1.5)
```

**After:**
```python
# 축별로 사용할 명령 매핑 (점진적 마이그레이션 지원)
ABSOLUTE_ENCODER_AXES = {0x147}  # Phase 1: Yaw만. 검증 후 0x143 등 확장.

def read_all_encoder_positions(self):
    """모든 모터의 현재 엔코더 위치 읽기 (초기화 및 브레이크 해제 시 사용)

    축별 분기:
    - ABSOLUTE_ENCODER_AXES (예: 0x147): 0x90으로 출력축 절대 엔코더 사용
    - 그 외: 기존 0x92 (모터부 누적 각도) 유지
    """
    multi_turn_cmd = self.protocol.create_system_command(CommandType.READ_MULTI_TURN_ANGLE)
    encoder_cmd    = self.protocol.create_system_command(CommandType.READ_ENCODER_DATA)

    all_motors = list(self.motor_ids) + [self.left_motor_id, self.right_motor_id]
    for i, motor_id in enumerate(all_motors):
        if i > 0:
            time.sleep(0.2)

        if motor_id in self.ABSOLUTE_ENCODER_AXES:
            # 출력축 절대 엔코더 (0x90)
            self.can_manager.send_frame(motor_id, encoder_cmd)
            self.get_logger().info(f"  → 0x{motor_id:03X} 출력축 절대 위치 읽기 요청 (0x90)")
        else:
            # 모터부 누적 각도 (0x92, 기존)
            self.can_manager.send_frame(motor_id, multi_turn_cmd)
            self.get_logger().info(f"  → 0x{motor_id:03X} 위치 읽기 요청 (0x92)")

    time.sleep(1.5)
```

#### 수정 B. 주기적 폴링 (Line 593~622 부근)

**Before:**
```python
# 이동 중인 모터들의 위치를 읽기 위해 0x92 명령 전송
multi_turn_cmd = self.protocol.create_system_command(CommandType.READ_MULTI_TURN_ANGLE)
for motor_id in moving_motors:
    self.can_manager.send_frame(motor_id, multi_turn_cmd)
    self.debug_logger.debug(f"📤 [0x92] 0x{motor_id:03X} 멀티턴 각도 읽기 명령 전송")
```

**After:**
```python
# 이동 중인 모터들의 위치 읽기 — 축별로 명령 분기
multi_turn_cmd = self.protocol.create_system_command(CommandType.READ_MULTI_TURN_ANGLE)
encoder_cmd    = self.protocol.create_system_command(CommandType.READ_ENCODER_DATA)

for motor_id in moving_motors:
    if motor_id in self.ABSOLUTE_ENCODER_AXES:
        self.can_manager.send_frame(motor_id, encoder_cmd)
        self.debug_logger.debug(f"📤 [0x90] 0x{motor_id:03X} 출력축 엔코더 읽기 명령 전송")
    else:
        self.can_manager.send_frame(motor_id, multi_turn_cmd)
        self.debug_logger.debug(f"📤 [0x92] 0x{motor_id:03X} 멀티턴 각도 읽기 명령 전송")
```

#### 수정 C. Line 1073~1075 부근 (또 다른 폴링 지점)

같은 패턴으로 0x92 명령 전송부를 축별 분기로 변경. (위 수정 B와 동일 로직 적용)

#### 수정 D. 응답 파서 — `_handle_motor_response()` 분기 추가 (Line 1143~1182 부근)

**Before:**
```python
if command == CommandType.READ_MULTI_TURN_ANGLE:
    # 0x92 멀티턴 각도 응답 파싱: [cmd][reserved(3)][angle(4)] = 8바이트
    if len(data) >= 8:
        angle_raw = struct.unpack('<i', data[4:8])[0]   # int32, 0.01도 단위
        angle_degrees = angle_raw * 0.01

        # 부호 반전: 명령 시 부호를 반전했으므로 응답도 반전
        angle_degrees = -angle_degrees

        previous_position = self.motor_states[motor_id].get('position', angle_degrees)
        position_delta = angle_degrees - previous_position

        # 상태 업데이트
        self.motor_states[motor_id]['position'] = float(angle_degrees)
        self.current_positions[motor_id] = float(angle_degrees)
        # ...
    else:
        self.get_logger().warning(f"모터 0x{motor_id:03X} 0x92 응답 데이터 길이 부족: {len(data)} bytes")

elif command == CommandType.READ_MOTOR_STATUS:
    # ...
```

**After:**
```python
if command == CommandType.READ_MULTI_TURN_ANGLE:
    # 0x92 멀티턴 각도 응답 (기존 로직 유지 — 출력축 엔코더 미사용 축용)
    if len(data) >= 8:
        angle_raw = struct.unpack('<i', data[4:8])[0]
        angle_degrees = angle_raw * 0.01
        angle_degrees = -angle_degrees

        previous_position = self.motor_states[motor_id].get('position', angle_degrees)
        position_delta = angle_degrees - previous_position

        self.motor_states[motor_id]['position'] = float(angle_degrees)
        self.current_positions[motor_id] = float(angle_degrees)
        # ...
    else:
        self.get_logger().warning(f"모터 0x{motor_id:03X} 0x92 응답 데이터 길이 부족: {len(data)} bytes")

# ───────── 추가 분기: 0x90 출력축 절대 엔코더 응답 ─────────
elif command == CommandType.READ_ENCODER_DATA:
    # 0x90 듀얼 엔코더 응답 파싱: [cmd][encoder1(4)][encoder2(4)] = 9바이트
    parsed = self.protocol._parse_encoder_response(data)
    if parsed.get('parsed'):
        encoder2_angle = parsed['encoder2_angle']     # 17-bit 출력축 절대값 (0~360°)
        encoder1_angle = parsed['encoder1_angle']     # 18-bit 모터부 (디버그용)

        # 출력축 절대값을 robot frame 기준으로 변환
        # (필요 시 offset/방향 보정)
        position_degrees = self._convert_output_encoder_to_axis_angle(motor_id, encoder2_angle)

        # 상태 업데이트 — 0x92와 동일 자료구조 사용
        self.motor_states[motor_id]['position'] = float(position_degrees)
        self.current_positions[motor_id] = float(position_degrees)

        # 0x92와 비교 로그 (디버깅 — 검증 끝나면 제거)
        self.debug_logger.debug(
            f"📥 [0x90] 0x{motor_id:03X} 출력축={encoder2_angle:.3f}°, "
            f"모터부={encoder1_angle:.3f}°, 변환된 위치={position_degrees:.3f}°"
        )
    else:
        self.get_logger().warning(
            f"모터 0x{motor_id:03X} 0x90 응답 파싱 실패: {parsed.get('error', 'unknown')}"
        )

elif command == CommandType.READ_MOTOR_STATUS:
    # ... (기존 유지)
```

#### 수정 E. 좌표 변환 헬퍼 추가

출력축 엔코더는 모터 출력축 기준 0~360° 값. 로봇 frame 기준 각도(예: 0x147 Yaw는 ±30°)로 변환하려면 offset과 방향 처리가 필요.

```python
def _convert_output_encoder_to_axis_angle(self, motor_id: int, raw_angle_deg: float) -> float:
    """출력축 절대 엔코더 값(0~360°)을 로봇 frame 축 각도로 변환

    축별 offset과 방향:
    - 0x147 Yaw: -180~+180으로 unwrap 후 offset 적용 (±30° 범위)
    - 다른 축은 적용 시 별도 설정
    """
    # 0~360 → -180~+180 unwrap
    if raw_angle_deg > 180.0:
        raw_angle_deg -= 360.0

    # 축별 offset (homing 시 측정한 값을 yaml/config에 저장 권장)
    offset = self.axis_zero_offsets.get(motor_id, 0.0)
    direction = self.axis_directions.get(motor_id, 1.0)  # 1.0 또는 -1.0

    return (raw_angle_deg - offset) * direction
```

`self.axis_zero_offsets`, `self.axis_directions`는 `__init__`에 추가하거나 yaml config에서 로드. **homing은 여전히 필요하되, 부팅 후 1회만 수행하면 그 후 절대 위치는 영구 유지**.

---

## 5. 검증 절차

### 5.1 Phase 0 — 사전 검증 스크립트 (제어 노드 수정 전)

별도 테스트 스크립트로 0x90 응답을 살펴 encoder1/encoder2 어느 쪽이 출력축인지 확정.

```python
#!/usr/bin/env python3
"""rmd_dual_encoder_test.py — 0x90 응답에서 출력축 엔코더 식별"""
import can, struct, time, sys

MOTOR_ID = 0x147  # 검증 대상 (Yaw)
CAN_IFACE = 'can2'

def send_read_encoder(bus, motor_id):
    msg = can.Message(
        arbitration_id=motor_id,
        data=bytes([0x90, 0, 0, 0, 0, 0, 0, 0]),
        is_extended_id=False
    )
    bus.send(msg)

def parse(data):
    enc1 = struct.unpack('<i', data[1:5])[0]
    enc2 = struct.unpack('<i', data[5:9])[0]
    return enc1 * 360.0 / (2**18), enc2 * 360.0 / (2**17)

bus = can.Bus(CAN_IFACE, bustype='socketcan', bitrate=1_000_000)

# 1. 정지 상태 측정
send_read_encoder(bus, MOTOR_ID)
msg = bus.recv(timeout=1.0)
e1, e2 = parse(msg.data)
print(f"[정지] encoder1={e1:.3f}°, encoder2={e2:.3f}°")

# 2. 사용자에게 모터 출력축을 손으로 5도 정도 회전시키도록 요청
input("출력축을 손으로 ~5° 회전시킨 후 Enter…")
send_read_encoder(bus, MOTOR_ID)
msg = bus.recv(timeout=1.0)
e1_new, e2_new = parse(msg.data)
print(f"[수동회전 후] encoder1={e1_new:.3f}° (Δ={e1_new-e1:.3f}), "
      f"encoder2={e2_new:.3f}° (Δ={e2_new-e2:.3f})")

# 3. 분석
# - 출력축에서 직접 측정하는 엔코더는 5° 그대로 따라옴
# - 모터부 엔코더는 백래시만큼 적게 움직이거나 안 움직임
# - 따라서 변화량 큰 쪽이 출력축
```

검증 절차:
1. 위 스크립트로 Δ가 큰 쪽이 출력축임을 확인 (예상: encoder2)
2. **전원 오프 → 출력축 임의 회전 → 전원 재인가 → 즉시 측정**: 출력축 엔코더는 새 위치 반영, 모터부는 부팅 시 0으로 리셋되는지 확인 (전원 오프 + 배터리 백업 유무에 따라 다름)
3. **백래시 측정**: 출력축은 동일 위치 유지, 모터부는 백래시 양만큼 회전 → 백래시 정량값 얻음

### 5.2 Phase 1 — Yaw(0x147) 단독 적용

1. Phase 0 검증 후 `ABSOLUTE_ENCODER_AXES = {0x147}` 로 시작
2. `_convert_output_encoder_to_axis_angle()` 의 offset을 현재 SW가 인식하는 yaw=0° 위치에서 측정한 encoder2 raw 값으로 설정
3. 다음 시나리오로 회귀 테스트:
   - Homing → 정상 동작 확인
   - Yaw 명령(예: +20°) → 위치 도달 확인
   - 손으로 출력축 살짝 회전 → 위치 변화가 즉시 반영되는지 확인 (수정 전엔 안 됐음)
   - 전원 오프 → 임의 회전 → 재부팅 → homing 없이 이전 위치 즉시 복원되는지 확인
   - 100점 결속 시퀀스 실행 → 5단계 pose change가 단순화 가능한지 평가
4. 사이클 시간, drift 누적, 충돌 여부 로그 분석

### 5.3 Phase 2 — 횡이동(0x143) 적용 (선택)

multi-turn 운영 시 §3.3 방안 A/B/C 중 결정 후 적용.

### 5.4 회귀 테스트 체크리스트

- [ ] 0x92 사용하는 다른 축의 동작 정상
- [ ] PID, homing, 브레이크 동작 정상
- [ ] cmd_vel 주행 정상
- [ ] CAN 버스 트래픽 증가 확인 (응답 9바이트 vs 8바이트, 큰 변화 없음)
- [ ] 0x90 응답 누락 빈도 (CAN bus-off 등) 측정
- [ ] 100-point 자율 결속 시 사이클 시간 변화 측정

---

## 6. 롤백 시나리오

`ABSOLUTE_ENCODER_AXES = set()` 으로 비우면 모든 축이 기존 0x92 동작으로 복귀. 응답 파서의 `elif command == READ_ENCODER_DATA` 분기는 호출 안 되므로 안전.

코드 변경이 작아 git revert도 간단:
```bash
git revert <commit>
```

---

## 7. 적용 후 보고서 사례 영향

작성된 「철근 결속 로봇 HW 애로사항 분석 및 신규 HW 개선 제안」(2026-05-28)의 사례 중 다음이 영향을 받음.

| 사례 | 기존 결론 | 수정 후 결론 |
|---|---|---|
| **사례 6 (Yaw 5단계 pose change)** | HW: Yaw ±30° + 백래시 → 신규 모터 필요 (★★★) | 부분 해결. 출력축 절대 엔코더 활용으로 *시작 자세 불명* 문제 제거. 작업공간만 확보되면 1단계 가능. **신규 HW의 Yaw 무한회전+절대 엔코더는 여전히 cycle time 단축에 유효하나 우선순위 ↓ (★)** |
| **사례 7 (횡이동 백래시 → 절대 위치 불가)** | HW: 절대 엔코더 부재 (★★★) | **HW에 이미 있음, SW만 수정으로 해결** (★→삭제 가능) |
| **사례 8 (X축 first homing timeout)** | HW: 드라이버 초기화 (★★) | 일부 영향 가능. 0x90 사용 시에도 같은 timeout 발생하는지 별도 검증 필요 |
| **사례 35 (position_control_node 단일 책임 과중)** | SW refactoring 진행 중 | 본 수정도 refactoring 패치에 통합. rebar_base_control 분리 시 0x90 사용을 표준으로 |

→ 신규 HW 사양서에서는 **"Yaw·횡이동에 dual encoder 모델 채택 + SW 측에서 0x90 명령으로 출력축 엔코더 사용"** 을 명문화. 모터 vendor 변경 불필요.

---

## 8. 부록

### 8.1 RMD V4 CAN 명령 비교

| 명령 | 코드 | 응답 길이 | 응답 내용 | 백래시 영향 | 절대값 |
|---|---|---|---|---|---|
| READ_ENCODER_DATA | `0x90` | 9 byte | encoder1(4B) + encoder2(4B) | encoder1: O, **encoder2: X** | encoder2: 1회전 절대 |
| WRITE_ENCODER_OFFSET | `0x91` | – | 영점 설정 | – | – |
| READ_MULTI_TURN_ANGLE | `0x92` | 8 byte | int32 누적각 (0.01°) | **O** (모터부 누적) | 누적값 (전원 오프 시 0) |
| READ_SINGLE_TURN_ANGLE | `0x94` | – | 단일턴 각도 | – | 1회전 내 |

### 8.2 사용된 코드 위치 요약 (grep 기준)

```
position_control_node.py:
  Line  371 — read_all_encoder_positions() 내부
  Line  381 — can_manager.send_frame(motor_id, multi_turn_cmd)
  Line  595 — multi_turn_cmd 생성
  Line  620 — 주기 폴링 send
  Line  622 — debug log "📤 [0x92]"
  Line 1074 — multi_turn_cmd 생성 (또 다른 폴링)
  Line 1143 — if command == READ_MULTI_TURN_ANGLE  ← 분기 추가 지점
  Line 1183 — elif command == READ_MOTOR_STATUS

rmd_x4_protocol.py:
  CommandType.READ_ENCODER_DATA = 0x90              ← 이미 정의됨, 사용만 하면 됨
  CommandType.READ_MULTI_TURN_ANGLE = 0x92          ← 현재 사용 중
  _parse_encoder_response()                          ← 듀얼 엔코더 파싱, 이미 구현
```

### 8.3 참고 문서

- [RMD-X Motor Motion Protocol V4.01](https://www.scribd.com/document/858068253/RMD-X-Motor-Motion-Protocol-V4-01)
- [V4-X4-10 Details (MyActuator)](https://www.myactuator.com/x4-10details)
- [position_control_node.py (rebar_control)](https://raw.githubusercontent.com/jino123-koceti/rebar_control/main/src/rmd_robot_control/rmd_robot_control/position_control_node.py)
- [rmd_x4_protocol.py (rebar_control)](https://raw.githubusercontent.com/jino123-koceti/rebar_control/main/src/rmd_robot_control/rmd_robot_control/rmd_x4_protocol.py)
- 사내 보고서: 「철근 결속 로봇 HW 애로사항 분석 및 신규 HW 개선 제안」(2026-05-28)

---

**작성**: J / Claude 협업, 2026-05-28
