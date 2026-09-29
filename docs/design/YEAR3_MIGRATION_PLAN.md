# 2차년도 코드 → 3차년도 장비 이식 · 리팩토링 계획

**작성일:** 2026-09-29
**대상 독자:** 이 저장소에서 3차년도 이식을 진행하는 개발자
**전제:** 3차년도 하드웨어 기본 셋업 완료 (CAN·EZIO·WiFi·리모콘 확인)

> 새 기능을 만드는 작업이 아니다. **이미 개발된 2차년도 기능을 3차년도 장비로 옮기면서
> 구조를 정리하는 작업**이다. 그래서 "동작을 보존한 채 구조를 바꾼다"가 모든 판단의 기준이다.
>
> 실측으로 확인된 값은 출처를 적었다. "미확인" 표시는 그대로 미확인으로 다룰 것.

---

## 1. 2차년도 시스템 전체 지도

약 24,200줄 / 80개 파일, 4계층으로 설계되어 있다.

```
[외부 UI PC] ── Zenoh ──┐
                        ▼
┌─ 상위 제어 (rebar_control, 5,284줄) ─────────────────────────────┐
│  zenoh_client      UI ↔ ROS 브릿지 (rebar/command, rebar/status)  │
│  navigator         미션 상태기계 (idle→planning→navigating)        │
│  rebar_controller  경로추종 PID → /cmd_vel                         │
│  rebar_publisher   전체 상태를 JSON 으로 모아 /mission/status       │
│  pose_mux          전/후방 ZED odom 중 진행방향 쪽 선택             │
└───────────────────────────────────────────────────────────────────┘
      │ /cmd_vel, /joint_control_cmd, /mission/*
      ▼
┌─ 하드웨어 추상화 HAL (rebar_base_control, 6,372줄) ───────────────┐
│  can_parser        CAN 수신 → /motor_feedback, /remote_control     │
│  can_sender        /drive_control·/joint_control → CAN 송신         │
│  drive_controller  /cmd_vel → /drive_control (모드·비상정지 반영)   │
│  joint_controller  관절 명령 총괄 (1,186줄, 횡이동 포함)            │
│  homing_controller 원점복귀 (982줄)                                │
│  sequence_controller S21/S22 작업 시퀀스 (그리퍼 + Z축)             │
│  authority_controller 리모콘 S19/S20 → Manual/Auto 권한 중재         │
│  navigator_base    전체 상태기계 (idle/manual/auto/tying/estop)     │
│  encoder_odom      모터 엔코더 → /encoder_odom                     │
│  modbus_controller 그리퍼(RTU) + EZIO(TCP) → /io_status            │
└───────────────────────────────────────────────────────────────────┘
      │ CAN2 / CAN3 / Modbus
      ▼
    모터 · 리모콘 · EZIO · 그리퍼

┌─ 비전·결속 (rebar_vision, 5,740줄) ──────────────────────────────┐
│  rebar_detection      교차점 검출 서비스 /rebar/detect_crossings   │
│  tying_orchestrator   결속 자동화 2,103줄 — 6포인트(2x3) 순회       │
│  obstacle_detector    YOLO + 깊이 → /obstacle_pause                │
│  dual_camera_recorder 학습데이터 수집                               │
└───────────────────────────────────────────────────────────────────┘
```

**자율주행 흐름:**
`UI → zenoh_client → /mission/command → navigator → /mission/target_pose →
rebar_controller → /cmd_vel → drive_controller → /drive_control → can_sender → CAN`

**자율결속 흐름:**
`TYING_START → tying_orchestrator → detect_crossings → /joint_control(XY) →
Z하강 → 트리거 → Z상승` 을 6포인트 반복. 중간에 Yaw 자세를 바꿔 좌/우 카메라를 번갈아 사용.

**설계 원칙:** 상위 계층은 하드웨어를 모른다. 모터 ID·CAN 은 HAL 안에만 있다.
이 원칙은 잘 잡혀 있고, 이식 작업의 기준으로 그대로 쓸 수 있다.

---

## 2. 검증 상태 — 이 표가 모든 판단의 출발점

`rmd_robot_control`(5,581줄)은 계층 구조를 우회해 CAN2 를 직접 잡는 **별도 스택**이다.
`position_control_node` ≒ `can_sender` + `joint_controller` + `drive_controller` 로 역할이 겹친다.
그런데 **3차년도에서 동작이 검증된 것은 이 스택뿐이다.**

| 노드 | 3차년도 |
|---|---|
| position_control_node (1,946줄) | 동작 — 주행·XY·횡이동 실측 검증 |
| lateral_node (217줄) | 동작 — 횡이동 2축 |
| teleop_keyboard (351줄) | 동작 |
| remote_teleop_node (281줄) | 신규, 미테스트 |
| **나머지 26개 노드** | **미검증** |

**검증된 코드는 전체의 약 12% 다.** 리팩토링 범위를 정할 때 이 비율을 잊지 말 것.

---

## 3. 3차년도 하드웨어 델타

### 3.1 모터 ID 가 한 칸씩 밀렸다 (가장 위험)

| 역할 | 2차년도 | 3차년도 (실측) |
|---|---|---|
| 우측 주행 | 0x141 | 0x141 |
| 좌측 주행 | 0x142 | 0x142 |
| 횡이동 | 0x143 **(1축)** | **0x143 + 0x144 (2축)** |
| X축 | 0x144 | **0x145** |
| Y축 | 0x145 | **0x146** |
| Z축 (리프팅) | 0x146 | **0x147** |
| Yaw | 0x147 | **0x148** |

`0x14X` 가 **10개 파일에 하드코딩**되어 있고 2차/3차 주석이 섞여 있다
(`position_control_node` 주석의 `0x145=Yaw` 는 실제 X축). **코드 주석을 믿으면 안 된다.**
`0x148`(Yaw)은 `motor_ids` 에 없어 토픽이 아예 없다.
현재 매핑: `joint_1=0x143, joint_2=0x144, joint_3=0x145(X), joint_4=0x146(Y), joint_5=0x147(Z)`.

### 3.2 그 외 델타

| 항목 | 2차년도 | 3차년도 실측 |
|---|---|---|
| 횡이동 | 0x143 1축, ±360°=50mm / ±2880°=400mm | 0x143(+1)/0x144(−1) 2축, `/lateral/step` |
| EZIO | 1보드, 192.168.0.3, Modbus 502, L16O16 | **2보드** 0.5(IN16 하부)·0.6(I8O8 상부), **Plus-E UDP 3001** |
| EZIO 채널 | 문서상 매핑 | 전부 다름. 하부 실측: 범퍼 전IN07·후IN05(NO), 좌IN13·우IN10(NC), IN00=START, IN01=STOP. 상부는 미확인 |
| 교차점 검출 | ZED X mini 2대(좌·우) | **Gemini 2 L 1대 (교차점 전용)** |
| 좌·우 | — | **Gemini 305 2대** (2차년도에 없던 카메라) |
| 전·후 | zed_front / zed_back | **ZED X 2대** (현재 배럴잭 전원 미연결로 미인식) |
| 리모콘 | can3 250kbps | 동일. 프레임·비트 매핑 실측 확정, `can_parser` 수정 완료 |
| 그리퍼 | /dev/ttyUSB0 | 동일하나 **udev 규칙 없음** → 포트 순서에 따라 바뀜 |
| 브레이크 | — | X/Y/Z 홀딩 브레이크가 재시작마다 잠김. 해제(0x77)는 휘발성 |
| 전압 | — | 하부 32~41V 변동, 상부 24V. 속도 상한이 전압에 따라 변함 |
| 0x145 온도 | — | 센서 불량(0xEC 고정). 과열 보호 불가 |

---

## 4. 리팩토링이 필요한 지점 — 중복 실측

같은 일을 하는 코드가 여러 벌 있다. 이식하면서 정리할 대상이다.

**횡이동 — 구현이 4개**

| 파일 | 줄 | 상태 |
|---|---|---|
| `rmd/lateral_node.py` | 217 | **3차년도 동작 검증됨** |
| `rmd/lateral_axes.py` | 574 | 위 노드의 하위 모듈 |
| `base/lateral_motion.py` | 380 | entry point 없음 (사용 안 됨) |
| `base/lateral_encoder_calibration.py` | 232 | entry point 없음 |
| `base/joint_controller.py` 내부 | 1,186 중 일부 | 2차년도 1축 기준 |

**CAN 접근 — 2벌**

| 파일 | 줄 | |
|---|---|---|
| `rmd/can_manager.py` | 239 | position_control_node 가 사용 (동작) |
| `base/can_sender.py` | 493 | HAL 송신 (미검증) |
| `base/can_parser.py` | 496 | HAL 수신 + 리모콘 (리모콘 부분만 확인) |

can2 를 여는 파일이 **6개**다. 버스는 하나인데 소유자가 없다.
`can_parser`/`can_sender` 와 `position_control_node` 를 같이 띄우면 충돌한다.

**자세·odom — 3개**: `encoder_odom`(302) · `pose_mux`(315) · `odom_to_pose`(115)

**거대 파일**: `tying_orchestrator` 2,103 · `position_control_node` 1,946 ·
`rebar_controller` 1,528 · `navigator` 1,458 · `joint_controller` 1,186

---

## 5. 리팩토링 원칙

이 규모(24,000줄, 검증 12%)에서 한 번에 구조를 바꾸면 실패한다. 다음을 지킨다.

1. **인터페이스를 먼저 고정한다.** `rebar_base_interfaces` 의 메시지
   (`DriveControl`, `JointControl`, `MotorFeedback`, `RemoteControl`, `GripperControl`, `IOStatus`)를
   계층 경계로 확정한다. 이후 리팩토링은 이 경계 **안쪽만** 바꾼다. 경계를 바꿀 땐 별도 단계로.

2. **동작하는 것을 끄지 않는다 (strangler).** `position_control_node` 를 한 번에 HAL 로
   교체하지 않는다. 이 노드를 can2 의 **유일한 소유자**로 두고, HAL 인터페이스
   (`/drive_control`·`/joint_control` 구독, `/motor_feedback` 발행)를 **추가**한다.
   그러면 상위 계층은 수정 없이 붙고, 검증된 주행 동작이 유지된다.
   `can_sender`/`can_parser`(can2 부분)는 당분간 띄우지 않는다.
   `can_parser` 는 **can3 리모콘 전용**으로만 돌린다.

3. **기준선을 먼저 기록한다.** 구조를 건드리기 전에 현재 동작 수치를 남긴다:
   바퀴 반지름 0.02912, 속도 상한 0.25 m/s · 0.5 rad/s, 횡이동 12/12 완주 전류 12.13A,
   각 축 무부하 전류. 리팩토링 후 같은 값이 나오는지로 회귀를 판단한다.

4. **하드웨어 없이 검증할 수단을 만든다.** `vcan` + 기록된 candump 로그로 재생 테스트를 만든다.
   리모콘 파싱은 이미 이 방식으로 검증했다(캡처 재생으로 비상정지 판정 수정 확인).
   모터 쪽도 최소한의 응답 모사기를 만들면 리팩토링 회귀를 하드웨어 없이 잡을 수 있다.
   **이게 있어야 나머지 단계가 안전해진다.**

5. **삭제는 마지막에, 근거를 남기고.** 중복 구현을 바로 지우지 않는다.
   대체가 실측으로 확인된 뒤에 지우고, 왜 지웠는지 커밋 메시지에 남긴다.

6. **한 단계에 한 종류만 바꾼다.** "ID 정리 + 파일 분해"를 같이 하지 않는다.
   문제가 생겼을 때 원인을 가릴 수 없게 된다.

---

## 6. 단계별 계획

각 단계는 **직전 단계가 실측으로 확인된 뒤에만** 진행한다.
각 단계에 검증 기준과 롤백 방법을 함께 적었다.

### R0. 기준선 기록 + 재생 테스트 골격
- 현재 동작 수치를 문서에 남긴다 (§5-3)
- `vcan0` + candump 로그 재생 하네스 작성. 리모콘 파싱부터 적용
- **검증:** 하네스가 현재 코드에서 통과. **롤백:** 해당 없음 (추가만)

### R1. 축 레지스트리 단일화 ★ 가장 효과 큼
- `config/axes.yaml` 하나에 축 정의를 모은다:
  CAN ID · 역할명 · 감속비 · 부호 · 속도/전류 상한 · 브레이크 유무 · 리미트 채널
- 하드코딩된 `0x14X` 를 이 설정 참조로 교체 (10개 파일)
- `motor_ids` 에 `0x148`(Yaw) 추가 → `joint_6`
- 2차년도 기준 주석 정리
- **검증:** 1축씩 저속 명령 → 의도한 축만 움직임. 기준선 수치 재현
- **롤백:** 설정 파일과 참조만 되돌림. 로직 변경 없음

### R2. CAN 소유권 단일화
- `position_control_node` 를 can2 유일 소유자로 확정
- `lateral_axes`·`robot_control_gui` 의 직접 개방 제거 → 토픽 경유
- `can_parser` 는 can3 전용 기동
- **검증:** `candump` 로 프레임 중복 송신 없음, bus-error 0 유지
- **롤백:** 기동 구성(launch)만 되돌림

### R3. HAL 인터페이스 이식 (strangler)
- `position_control_node` 에 `/drive_control`·`/joint_control` 구독과
  `/motor_feedback` 발행을 추가. 기존 `/cmd_vel`·`/joint_N/speed` 는 유지
- 이 시점부터 상위 계층(navigator, orchestrator, sequence, authority)이 붙을 수 있다
- **검증:** 같은 동작을 두 경로로 각각 실행해 결과 일치
- **롤백:** 추가한 구독/발행만 제거

### R4. 안전 계층 ★ 자율주행 전 필수
- EZIO ROS 노드 재작성 (2보드, UDP, IN16/I8O8, 극성 반영) → `/limit_sensors/*`·범퍼·STOP
- 비상정지·범퍼·STOP·리미트·데드맨을 **한 노드에서 판단**해 명령 차단
- **지금은 범퍼를 눌러도 로봇이 서지 않는다**
- **검증:** 주행 중 각 범퍼·STOP 으로 정지, 리미트 도달 시 해당 축만 정지

### R5. 횡이동 구현 통합
- `lateral_node`(검증됨) 기준으로 하나만 남긴다
- `lateral_motion`·`lateral_encoder_calibration` 은 대체 확인 후 삭제
- `joint_controller` 의 1축 가정 횡이동 로직 제거, `/lateral/step` 으로 위임
- 각도 상수(±360°, ±2880°)를 2축 구조로 재계산
- **검증:** 50mm·400mm 이동 거리 실측, 12/12 완주 재현

### R6. 원점복귀·관절
- `homing_controller` 를 3차년도 ID·2축 횡이동·상부 리미트에 맞춤
- 브레이크 자동 해제 정책 결정 (Z축 낙하 위험 → 무조건 해제 금지)
- **검증:** 각 축 홈 복귀 반복 재현성

### R7. 상위 제어 복원 (자율주행)
- `encoder_odom` → `rebar_controller` → `drive_controller` 경로 복원
- ZED X 전원 확보 후 `pose_mux` 검증. `odom_to_pose` 와 역할 중복 정리
- **검증:** 직선·회전 웨이포인트 추종 오차

### R8. 비전·결속
- 카메라 5대 구성에 맞춰 `rebar_detection` 토픽·캘리브레이션 교체
- `tying_orchestrator` 6포인트 흐름 재설계 여부 결정 (§7-1)
- 그리퍼 udev 규칙 + `modbus_controller` IP 수정
- **검증:** 단일 포인트 결속 → 6포인트 순회

### R9. 거대 파일 분해 (여유 생길 때)
- `position_control_node` 1,946줄 → 주행 / 관절 / 진단·보호
- `tying_orchestrator` 2,103줄 (`TYING_ORCHESTRATOR_REFACTOR_PLAN.md` 참고)
- 각 분해는 R0 의 재생 테스트 통과를 조건으로

---

## 7. 미결정 사항

1. **결속 작업 흐름** — 3차년도는 교차점 전용 카메라가 따로 있다. 2차년도의
   "Yaw 자세를 바꿔 좌/우 카메라로 6포인트" 방식을 유지할지, 교차점 카메라 기준으로
   다시 설계할지. `tying_orchestrator` 2,103줄의 운명이 여기서 갈린다.
2. **Yaw(0x148) 조작** — 리모콘 S23/S24(±5°)를 그대로 쓸지.
3. **좌우 Gemini 305 의 역할** — 장애물? 결속 보조? 2차년도에 없던 카메라다.
4. **상부 EZIO 출력 8점에 무엇이 연결되어 있는가** — 트리거·그리퍼 여부 미확인.
5. **외부 UI(Zenoh) 연동을 3차년도에도 쓰는가.** 쓰지 않으면
   `zenoh_client`·`rebar_publisher` 는 R7 에서 제외할 수 있다.

---

## 8. 위험

| 위험 | 완화 |
|---|---|
| 모터 ID 오배정으로 엉뚱한 축 구동 | R1 을 먼저, 1축씩 저속 확인 |
| CAN2 이중 소유로 버스 사고 | R2 소유자 1개 원칙, `can_sender` 동시 기동 금지 |
| 범퍼·STOP 미연동 상태의 자율주행 | R4 완료 전 자율주행 금지 |
| Z축 브레이크 해제 시 자중 낙하 | 자동 해제 정책은 기구 상태 확인 후 |
| 0x145 과열 보호 불가 | 장시간 부하 작업 시 수동 감시 |
| 리팩토링 회귀를 못 잡음 | R0 기준선·재생 테스트를 먼저 |
| 미검증 코드를 검증된 것으로 착각 | §2 표를 기준으로 판단 |
