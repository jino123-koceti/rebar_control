# Yaw 축 엔코더 초기화 가이드

## 개요
Yaw 모터(0x147)의 0x90 싱글턴 앱솔루트 엔코더 값은 기계적 간섭이나 충돌로 틀어질 수 있다.
이때 `can_devices.yaml`의 `yaw_home_encoder_90`, `yaw_max_encoder_90` 값을 재실측하여 업데이트해야 한다.

## 엔코더 종류
| 명령 | 타입 | 전원 OFF 유지 | 용도 |
|------|------|:---:|------|
| 0x90 | 싱글턴 앱솔루트 (16-bit) | O | 호밍 기준 (position 필드 사용) |
| 0x92 | 멀티턴 누적 각도 | X | 제어 중 각도 추적 |

## 0x90 응답 바이트 구조 (프로토콜 V4.3)
```
DATA[0]    = 0x90 (명령)
DATA[2:3]  = position (uint16, little-endian) ← config에 저장하는 값
DATA[4:5]  = raw (uint16, 원본 엔코더)
DATA[6:7]  = offset (uint16, ROM 저장 영점)

position = (raw - offset) mod 65536
```

## 재조정 절차

### 1단계: Home(리밋센서) 위치 실측
Yaw를 **홈센서(리밋) 위치**로 수동 이동시킨 후 CAN에서 읽기:

```bash
python3 -c "
import can, time
bus = can.Bus(channel='can2', interface='socketcan', bitrate=1000000)
for cmd in [0x90, 0x92]:
    msg = can.Message(arbitration_id=0x147, data=[cmd, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False)
    bus.send(msg)
    deadline = time.time() + 2.0
    while time.time() < deadline:
        resp = bus.recv(timeout=1.0)
        if resp and resp.arbitration_id == 0x247 and resp.data[0] == cmd:
            if cmd == 0x90:
                position = int.from_bytes(resp.data[2:4], 'little', signed=False)
                raw = int.from_bytes(resp.data[4:6], 'little', signed=False)
                offset = int.from_bytes(resp.data[6:8], 'little', signed=False)
                print(f'0x90: position={position}, raw={raw}, offset={offset}')
            elif cmd == 0x92:
                angle = int.from_bytes(resp.data[4:8], 'little', signed=True)
                print(f'0x92: deg={angle/100.0:.2f}')
            break
bus.shutdown()
"
```

출력 예시:
```
0x90: position=58121, raw=10376, offset=49231
0x92: deg=-7.67
```
→ **position=58121** 을 기록

### 2단계: Max 위치 실측
Yaw를 **반대쪽 끝(max)** 으로 수동 회전시킨 후 동일 스크립트 실행:

출력 예시:
```
0x90: position=24584, raw=2184, offset=49231
0x92: deg=393.76
```
→ **position=24584** 를 기록

### 3단계: config 업데이트
`src/rebar_base_control/config/can_devices.yaml` 수정:

```yaml
yaw_home_encoder_90: 58121    # ← 1단계 position 값
yaw_max_encoder_90: 24584     # ← 2단계 position 값
```

### 4단계: 빌드 & 재시작
```bash
colcon build --packages-select rebar_base_control
sudo systemctl restart robot-control
```

## 실측 이력
| 날짜 | home (position) | max (position) | 0x92 home | 0x92 max | 비고 |
|------|:---:|:---:|:---:|:---:|------|
| 2026-03-13 | 62056 | 27128 | - | - | 기존값 |
| 2026-03-15 | 58121 | 24584 | -7.67° | 393.76° | 요축 간섭 후 재실측 |
| 2026-03-15 | 61857 | 25896 | -5.05° | 395.56° | 2차 간섭 후 재실측 |

## 주의사항
- 0x90의 **position** 필드(DATA[2:3])를 config에 저장한다 (raw, offset 아님)
- offset(DATA[6:7])은 ROM에 저장된 영점값으로, 0x91 또는 0x64 명령으로 변경 가능
- 모터 전원을 껐다 켜도 0x90 position은 유지됨 (앱솔루트)
- 0x92 멀티턴은 전원 OFF 시 리셋되므로 호밍 기준으로 사용 불가
- 반드시 **모터가 정지한 상태**에서 읽을 것
