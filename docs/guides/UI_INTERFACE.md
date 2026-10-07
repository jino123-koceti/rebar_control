# 외부 UI 연동 규격 (3차년도)

노트북에서 UI 를 만들 때 쓰는 **장비 쪽 인터페이스**다. 2차년도 UI 체인
(`zenoh_client` → `/mission/command`)과는 **토픽이 다르다** — 그대로 쓰면 명령이
아무 데도 안 닿는다.

- 도메인: **`ROS_DOMAIN_ID=33`**  (설정 안 하면 아무것도 안 보인다)
- 2차년도 장비는 도메인 0 을 쓴다. 33 으로 분리해 둔 이유가 그것이다.

---

## 1. 명령 — UI → 로봇

| 토픽 | 타입 | 값 | 하는 일 |
|---|---|---|---|
| `/homing_cmd` | `std_msgs/String` | `"all"` | **전체 호밍** |
| `/homing_cmd` | `std_msgs/String` | `"stop"` | 호밍 중단 |
| `/mission/start` | `std_msgs/Empty` | — | 미션 시작 (경로 주행 + 결속) |
| `/mission/abort` | `std_msgs/Empty` | — | 미션 중단 |
| `/stage/stop` | `std_msgs/Empty` | — | 상부 축 즉시 정지 |

### 꼭 지킬 것

- **호밍은 `"all"` 뿐이다.** 축 이름(`"x"` 등)은 **거부된다.** 축마다 간섭 구간이
  달라서, 일부만 호밍한 상태로 움직이면 프레임끼리 부딪친다 — 실제로 `/homing_cmd x`
  하나로 상부 프레임을 쳤다.
- **정지 버튼은 세 개를 모두 보내라.** `/homing_cmd "stop"` + `/mission/abort` +
  `/stage/stop`. 어느 한 노드가 죽어 있어도 나머지가 멈춘다. 멈추는 쪽은 중복이 안전하다.
- ⚠ **UI 정지를 1차 수단으로 삼지 말 것.** 가장 빠르고 확실한 것은 **리모콘 비상정지**다.
  UI 는 와이파이→HTTP→ROS 를 거치므로 느리고, 통신이 끊기면 눌러도 안 간다.
- **호밍은 자동으로 돌지 않는다.** 사람이 눌러야 한다. 되살아난 즉시 자동으로 돌면
  아무도 시키지 않았는데 장비가 움직이고, `respawn` 과 겹치면 크래시 루프가
  재호밍 루프가 된다.

---

## 2. 상태 — 로봇 → UI

전부 `std_msgs/String` 에 **JSON** 이 들어 있다.

### `/homing_status`

```json
{"state": "seek", "axis": "x", "detail": "x_min 탐색",
 "queue": ["yaw", "y"], "refs": {"z": -61.7, "x": -25.0}}
```

| 필드 | 뜻 |
|---|---|
| `state` | `idle` / `arm` / `seek` / `back_off` / `fine` / `offset` / `ready` / **`done`** / **`failed`** |
| `axis` | 지금 호밍 중인 축 (`null` 이면 없음) |
| `detail` | 사람이 읽을 설명. **실패 사유가 여기 들어온다** |
| `refs` | 축별 원점. **4개(`x`,`y`,`z`,`yaw`)가 다 차야 호밍 완료**다 |

### `/mission/status`

```json
{"phase": "run", "total": 5, "current": 1,
 "steps": [{"step": "tie", "state": "완료"}, {"step": "drive -500", "state": "진행"}],
 "detail": "..."}
```

| 필드 | 뜻 |
|---|---|
| `phase` | `idle` / `run` / **`done`** / **`failed`** |
| `total` / `current` | 전체 단계 수 / 지금 단계 번호(0-based, 없으면 `null`) |
| `steps[]` | `{step, state}` — `step` 은 `"tie"`·`"drive -500"` 같은 문자열 |
| `detail` | 실패 사유 |

### `/stage/status` (상부 축)

```json
{"pose": 1, "pose_detail": "1번 자세 (건 -16.75°, 오차 +0.02°)",
 "current_mm": {"x": -2.0, "y": 174.2, "z": 0.3},
 "homed": ["x","y","yaw","z"], "moving": false, "yaw_moving": false,
 "rejects": 0, "detail": "대기",
 "limit_mm": {"x": [-5.0, 408.7], "y": [50.9, 275.1]}}
```

| 필드 | 뜻 |
|---|---|
| `pose` | yaw 자세 1~4 (`0`=12시, `null`=모름/자세 사이) |
| `current_mm` | 현재 위치(mm). **호밍 전에는 `null`** 이다 |
| `homed` | 호밍된 축 목록 |
| `rejects` | 거부 누적. **늘어났으면 방금 거부된 것**이다 — `detail` 에 사유가 있다 |

### `/drive/status` (주행)

```json
{"moving": true, "goal_mm": -500.0, "moved_mm": -312.4, "left_mm": 187.6,
 "drift_mm": 0.4, "angles_ok": true, "rejects": 0, "detail": "..."}
```

`drift_mm` 은 좌우 바퀴 차이에서 온 **직진 이탈**이다. 30mm 를 넘으면 노드가 스스로 선다.

### `/safety/state` (`rebar_base_interfaces/msg/SafetyState`)

JSON 이 아니라 **메시지 타입**이다. 주요 필드: `estop`(bool), `stop_switch`,
`inputs_stale`, `blocked_axes`(문자열 배열).

---

## 3. UI 에서 보여줄 것 — 권장

**버튼 3개면 충분하다:** 전체 호밍 / 미션 시작 / 정지.

**반드시 보여야 하는 것:**

1. **호밍 여부** — `/stage/status` 의 `homed` 가 4개인가. 호밍 전에는 mm 이동이
   전부 거부되므로, 이게 안 보이면 "왜 안 움직이지" 로 헤매게 된다.
2. **비상정지** — `/safety/state` 의 `estop`. 걸려 있으면 **호밍도 미션도 거부된다.**
   실제로 "제어 권한을 못 받았다" 라는 엉뚱해 보이는 사유로 실패한다.
3. **실패 사유** — `state`/`phase` 가 `failed` 일 때 `detail` 을 그대로 띄울 것.
   사유 문장이 원인을 직접 말하도록 써 두었다.
4. **거부 누적** — `rejects` 가 늘면 `detail` 과 같이 띄울 것. 조용히 안 움직이는
   경우의 대부분이 거부다.
5. **통신 끊김** — 상태 토픽이 N 초 이상 안 오면 그것도 표시할 것. 로봇이 멈춘 것과
   UI 가 못 받는 것은 다른 문제인데 화면상으로는 똑같아 보인다.

**폴링 주기:** `/stage/status` 는 20Hz 로 나온다. UI 는 2~5Hz 면 충분하다.

---

## 4. 연결 방법

### (a) 노트북에 ROS 2 가 있으면 — 가장 단순

같은 네트워크에서 `ROS_DOMAIN_ID=33` 만 맞추면 토픽이 그대로 보인다. 중계가 필요 없다.

```bash
export ROS_DOMAIN_ID=33
ros2 topic echo /mission/status --field data
ros2 topic pub --once /mission/start std_msgs/Empty "{}"
```

### (b) 웹 UI 로 만들 거면

브라우저는 ROS 토픽을 직접 못 읽는다. 둘 중 하나가 필요하다.

- **`rosbridge_suite`** — 웹소켓으로 토픽을 그대로 노출한다. `roslibjs` 로 붙는다.
  설치 필요: `sudo apt install ros-humble-rosbridge-suite`
- **작은 HTTP 중계를 장비에 두기** — 상태를 모아 `GET /state` 로 주고 `POST /cmd/...`
  로 명령을 쏘는 노드. 표준 라이브러리만으로 되고 의존이 적다.

⚠ 어느 쪽이든 **장비에서 돌아야 한다** (로봇과 같은 도메인에 있어야 하므로).
필요하면 장비 쪽에 노드를 추가해 줄 수 있다 — UI 는 노트북에서 만들고,
중계만 장비에 두는 구성이 깔끔하다.

---

## 5. 리허설 흐름

```
전원 ON
   ↓  (약 1분 — 교차점 검출 모델 적재에 40초쯤 걸린다. 그 전에 미션을 걸면
   ↓   첫 검출이 버려져 "검출이 안 온다" 로 실패한다)
[전체 호밍]   /homing_cmd "all"
   ↓  refs 4개가 찰 때까지 (약 1~2분)
[미션 시작]   /mission/start
   ↓  test_path.txt 경로: 시점 결속 → 후진 500mm 결속 → 전진 500mm 결속
완료
```

**호밍 전에 yaw 를 12시로 맞출 필요는 없다.** 1번·4번 자세는 단회전만으로 못 가리는데,
X 를 먼저 호밍한 뒤 작업영역 카메라 영상으로 가린다. 못 가리면 **거부**되고 그때만
사람이 12시로 옮기면 된다.
