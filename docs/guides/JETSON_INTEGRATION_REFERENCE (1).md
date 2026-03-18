# Jetson(로봇) 측 연동 참고 사항

External PC(rebar_ui)와 Zenoh로 통신할 때 **Jetson에서 구현·참고해야 할 내용**을 정리한 문서입니다.  
Jetson 팀에서 이 문서를 기준으로 명령 수신 처리와 상태/피드백 발행을 구현하면 됩니다.

---

## 1. Zenoh 키 정리

| 키 | 방향 | 형식 | 설명 |
|----|------|------|------|
| `rebar/command` | UI → Jetson | JSON string | 외부 PC에서 보내는 명령 수신 |
| `rebar/status` | Jetson → UI | MessagePack binary | 로봇 상태·미션·**결속 상태** 발행 |
| `rebar/pose` | Jetson → UI | MessagePack binary | 고주파 위치(pose) 발행 |

- UI는 **구독**: `rebar/status`, `rebar/pose`  
- UI는 **발행**: `rebar/command`  
- Jetson은 **구독**: `rebar/command`  
- Jetson은 **발행**: `rebar/status`, `rebar/pose`

---

## 2. Jetson이 수신하는 명령 (`rebar/command`)

외부 PC에서 보내는 메시지는 **UTF-8 JSON 문자열**입니다.

### 2.1 단순 명령 (JSON 문자열)

- 형식: `{"command": "<명령어>"}`
- 예: `{"command": "TYING_START"}`, `{"command": "PAUSE"}`, `{"command": "GO_HOME"}`

Jetson에서 파싱 후 `command` 값에 따라 분기 처리하면 됩니다.

| command 값 | 설명 |
|-------------|------|
| `TYING_START` | **결속 시작** – 결속 작업을 시작할 때 사용 (UI의 "결속 시작 (TYING_START)" 버튼) |
| `GO_HOME` | 홈(원점) 복귀 |
| `START_MISSION` | 자율 작업(미션) 시작 |
| `PAUSE` | 일시 정지 |
| `RESUME` | 재개 |
| `CANCEL` | 작업 중단 |
| `EMERGENCY_STOP` | 비상 정지 |

### 2.2 웨이포인트 미션 (JSON 문자열)

- 형식: `{"waypoints": [{"x": 0.0, "y": 0.0}, ...]}`
- 좌표 단위: **미터(m)**  
- `"command"` 필드는 포함되지 않습니다.

### 2.3 상부 결속부 이동 (JSON 문자열)

- 형식: `{"command": "UPPER_BINDING_MOVE", "target": {"x": float, "y": float, "z": float, "yaw": float}}`
- 대상: 상부 결속부 목표 위치/자세 (UI에서 mm, ° 단위로 입력 후 전송)

---

## 3. Jetson이 발행해야 하는 것

### 3.1 상태 메시지: `rebar/status`

- **형식**: MessagePack 바이너리
- **주기**: 자유(예: 5~10 Hz). 결속 중에는 주기적으로 보내면 UI 반응이 좋음.

**기본 필드 (기존 프로토콜)**  
- 로봇/미션 상태, 위치, 진행률 등 (기존 문서 참고).

**결속 피드백용 추가 필드 (선택)**  
아래 필드가 **하나라도 있으면** UI의 **「결속 작업 상태」** 창에서 자동으로 표시합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `tying_state` 또는 `tying_status` | string | `"idle"` / `"tying"` / `"success"` / `"failed"` (소문자 권장) |
| `tying_progress` | int/float | 0~100 (진행률 %) |
| `tying_message` 또는 `tying_feedback` | string | 결속 중/완료/실패 시 사용자에게 보여줄 메시지 |
| `tying_result` | string | 최근 결속 결과 요약 (예: "결속 완료", "와이어 끊김") |

- `rebar/status` 한 메시지에 **기존 상태 필드 + 위 결속 필드**를 같이 넣어서 보내면 됩니다.
- 별도 토픽 없이 `rebar/status`만 사용합니다.

**예시 (Python dict → MessagePack으로 직렬화 후 발행)**

```python
# 결속 대기
status = {
    "state": "IDLE",
    "position": {"x": 0.0, "y": 0.0},  # mm 단위로 보내도 UI 호환
    "tying_state": "idle",
}

# 결속 진행 중
status = {
    "state": "NAVIGATING",  # 또는 현재 로봇 상태
    "position": {"x": 100.0, "y": 50.0},
    "tying_state": "tying",
    "tying_progress": 45,
    "tying_message": "와이어 감는 중",
}

# 결속 완료
status = {
    "state": "IDLE",
    "tying_state": "success",
    "tying_progress": 100,
    "tying_message": "결속 완료",
    "tying_result": "결속 완료",
}

# 결속 실패
status = {
    "state": "IDLE",
    "tying_state": "failed",
    "tying_message": "와이어 끊김",
    "tying_result": "결속 실패",
}
```

- **위치 단위**: 현재 UI는 `position.x`, `position.y`를 **mm**로도 해석하므로, 기존처럼 mm로 보내도 됩니다.

### 3.2 위치 메시지: `rebar/pose`

- **형식**: MessagePack 바이너리
- **주기**: 예: 20 Hz  
- 필드: `x`, `y`, `yaw`, `timestamp` 등 (기존 프로토콜 유지).  
- UI는 이걸로 실시간 궤적·로봇 위치를 그립니다.

---

## 4. TYING_START 처리 흐름 (Jetson 쪽 권장)

1. `rebar/command`에서 JSON 수신.
2. `command == "TYING_START"` 이면 결속 시퀀스 시작.
3. 결속 진행 중:
   - `rebar/status`에 `tying_state: "tying"`, `tying_progress`, `tying_message` 등을 넣어 주기적으로 발행.
4. 결속 완료 시:
   - `rebar/status`에 `tying_state: "success"`, `tying_progress: 100`, `tying_result` 등 발행.
5. 결속 실패 시:
   - `rebar/status`에 `tying_state: "failed"`, `tying_message`/`tying_result` 발행.
6. 대기 상태일 때:
   - `tying_state: "idle"` (또는 필드 생략)로 UI에 "대기"로 표시.

---

## 5. UI 측 동작 요약 (참고)

- **결속 시작 버튼** 클릭 시: `rebar/command`에 `{"command": "TYING_START"}` 전송.
- **결속 작업 상태** 창: `rebar/status`를 구독하여, 위 `tying_*` 필드가 있으면 자동으로 상태/진행률/피드백/최근 결과를 갱신.

---

## 6. 체크리스트 (Jetson 구현 시)

- [ ] `rebar/command` 구독 후 JSON 파싱.
- [ ] `command == "TYING_START"` 수신 시 결속 로직 실행.
- [ ] `rebar/status` 발행 시 MessagePack 사용, 필요 시 `tying_state`, `tying_progress`, `tying_message`, `tying_result` 포함.
- [ ] 결속 진행 중/완료/실패 시 해당 상태를 `rebar/status`로 주기적으로 전송.
- [ ] (기존) `rebar/pose` 20 Hz, `rebar/status` 적절한 주기 유지.

이 문서를 Jetson 쪽에 전달해 두면, UI와의 결속 연동에 필요한 수신·발행 규격을 한 곳에서 참고할 수 있습니다.
