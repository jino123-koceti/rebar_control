# 3차년도 장비에 필요한 기능 목록

**작성일:** 2026-09-29
**근거:** 2차년도 작업본 `~/ros2_ws_2nd` (HEAD `a8b28c6`, 2026-09-22 — GitHub 최신과 동일)
**목적:** 리팩토링·이식 전에 "무엇을 옮겨야 하는가"를 기능 단위로 확정한다.

> 상태 표기 — **완료**: 3차년도에서 실측 확인 / **부분**: 일부만 동작 / **필요**: 이식해야 함 /
> **판단필요**: 3차년도에 쓸지 결정이 필요함 / **미확인**: 사실 확인이 안 된 것

---

## 결정 사항 (2026-09-29 확인)

| 질문 | 결정 |
|---|---|
| UI 연동 | **쓴다. 단 새로 구현한다.** 웨이포인트 생성 등 불필요 기능은 제거. 세부는 추후 논의 |
| 주행 방식 | **[A] 웨이포인트 + [B] 배근인지 둘 다 쓴다** |
| 리모콘 S23/S24 | **자율주행 시작·정지 전용.** Yaw 수동회전은 쓰지 않는다 |
| `rebar_detection` (zedxmini) | **제거.** Orbbec 으로 대체됨. zedxmini 는 3차년도 장비에 없다 |
| Pololu 트리거 | **동일하게 사용** |
| `orbbec_crossing.pt` | 신규 모델 **재학습 중**. 완료 후 반영 |

### 카메라 대응 — 2차년도 → 3차년도

| 2차년도 | 3차년도 | 영향받는 노드 |
|---|---|---|
| `zed_front` / `zed_back` | **ZED X 2대 (전·후방)** — 그대로 | `obstacle_detector` |
| `zedxmini1` / `zedxmini2` (좌·우) | **없음 → 제거** | `rebar_detection`(제거), `dual_camera_recorder`(재작성), `deck_edge`(경로 정리) |
| `zedxone` / `zedxone_left` | **없음 → 제거** | `zedxone_publisher`(제거), `deck_edge`, `tying_orchestrator` 일부 |
| `/camera` (Orbbec Gemini 2L) | **Gemini 2L (교차점)** — 그대로 | `rebar_drive`, `tying_orchestrator` |
| `/camera_left` (Orbbec) | **Gemini 305 좌·우** | `deck_edge`, `image_rotate` |
| `usbcam` | **제거** — 측면판정을 Orbbec 305 로 대체 | `usbcam_publisher`(제거), 측면판정 재작성 |

`rebar_drive_node` 와 `tying_orchestrator` 는 이미 `/camera/color`·`/camera/depth`(Orbbec)를
쓰므로 **교차점 검출 경로는 카메라 변경 영향이 없다.** `deck_edge_node` 는 zedxmini·zedxone·
`/camera_left` 를 모두 지원하도록 만들어져 있어, 3차년도는 `/camera_left`(305) 경로만 남기면 된다.

---

## 0. 2차년도 실제 운영 구성

현장에서 도는 진입점은 하나다.

```
systemd(robot-control.service) → scripts/service/robot_control_service.sh
  ├ 기동 시 모터 ROM 파라미터 재적용 (tools/motor/rmd_accel.py)
  ├ Orbbec Gemini 2L 카메라 런치 (serial_number 지정)
  └ ros2 launch rebar_control full_system.launch.py
        ├ base_system.launch.py   (rebar_base_control 13개 노드)
        │   can_parser · can_sender · drive_controller · joint_controller
        │   sequence_controller · modbus_controller · ezi_io_controller
        │   bumper_node · authority_controller · navigator_base
        │   encoder_odom · homing_controller · pololu_node
        ├ control_system.launch.py (rebar_control 4개 노드)
        │   zenoh_client · navigator · rebar_controller · rebar_publisher
        ├ 카메라: zed_front · zed_back · zedxmini1 · zedxmini2 · zedxone
        │         zedxone_left · usbcam_publisher · orbbec_camera
        └ 비전: rebar_detection · tying_orchestrator · obstacle_detector
                deck_edge · rebar_drive · auto_tying_launcher · data_acq_launcher
```

**주행 경로가 두 개 있고 서로 독립이다.**

```
[A] UI 웨이포인트 : navigator → rebar_controller ─┐
[B] 배근인지 자율 : rebar_drive_node ─────────────┴→ /cmd_vel → drive_controller → CAN
                                                        ↑
                    deck_edge_node · bumper_node · obstacle_detector (공통 안전차단)
```

안전 차단은 모터 직전(`drive_controller`)에 모은다. 상위 미션 흐름과 대화하지 않는다.
이유가 코드에 적혀 있다: navigator 는 경로를 일괄발행하고 빠져서 실시간 개입 경로가 없고,
rebar_controller 에 물리면 결속·횡이동 상태머신 인덱스가 꼬인다.

---

## A. 운영·기동

| # | 기능 | 2차년도 구현 | 3차년도 상태 |
|---|---|---|---|
| A1 | 전원 인가 시 자동 기동 | `robot-control.service` + `robot_control_service.sh` | **부분** — `can2-up`·`can3-up`·`rebar-teleop` 만 있음 |
| A2 | **기동 시 모터 ROM 가감속 재적용** | `rmd_accel.py --index 2/3` | **필요** — X4-36 은 0x43 이 ROM 에 안 남아 **매 기동 재적용이 필수**. 안 하면 0→최고속 14.9초 |
| A3 | CAN 인터페이스 자동 기동 | 스크립트 내 `ip link` | **완료** — `can2-up`/`can3-up` + 이름 고정(`pcan-fix-names`) |
| A4 | 시계 동기(chrony)·화면·자동로그인 | `scripts/service/*` | **보류** — 로봇 동작 구현 후에 판단 |
| A5 | 감시 프로브 (freeze·전류·전압·온도) | `freeze-probe` `motor-current-probe` `volt-monitor` `temp-monitor` | **필요** — 젯슨 프리즈·과부하 추적에 쓰였음 |
| A6 | 로그 정리 | `cleanup_logs.sh` | 필요 (낮은 우선도) |

**A2 는 성능 문제가 아니라 기능 문제다.** 감속비 12.5→36 교체로 같은 설정값의 실효 램프가
1/2.9 이 됐고, 0x43 이 **모터축 기준**이라는 것이 실측으로 확정됐다 (`process/260914.md`).

---

## B. 하드웨어 계층 (CAN·Modbus)

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| B1 | CAN 수신 → 모터 피드백 | `can_parser` (0x241/0x242) | **필요** (현재는 `position_control_node` 내부) |
| B2 | CAN 송신 (주행·관절·그리퍼) | `can_sender` 713줄 | **판단필요** — 3차년도는 `position_control_node` 가 담당 중 |
| B3 | **주행 토크 상한** (0xA2 DATA[1], 정격 %) | `max_torque_pct`, 좌우 개별 | **필요** — 과부하 시 젯슨 즉사 재현으로 도입된 보호 |
| B4 | **횡이동 토크 상한** | 0xA4 에 토크 바이트가 없어 **0xA9(Force Control Position)** 로 교체 | **필요** — 3차년도 횡이동은 2축이라 재검증 |
| B5 | **모터측 통신두절 보호 (0xB3)** | `drive_motor_watchdog_ms: 500` | **필요** — 젯슨 프리즈 시 SW 워치독은 같이 멈춘다. 주행모터에만, 스테이지엔 절대 금지 |
| B6 | cmd_vel 소프트 워치독 | `drive_controller` 0.3s | **완료** — `position_control_node` 에 0.5s 구현 |
| B7 | 직진 드리프트 보정 (좌우 속도 배율) | `can_devices.yaml` | **필요** — 3차년도 실측값으로 재교정 |
| B8 | can3 수신 오류 시 **버스 재연결** | 1초 후 `_reopen_remote()` | **필요** — USB 허브 EMI 로 포트가 껐다 켜지면 옛 소켓이 영구 오류 (7회 실측) |
| B9 | 엔코더 오도메트리 | `encoder_odom` | **필요** — 자율주행 전제 |
| B10 | 그리퍼 제어 (Modbus RTU) | `modbus_controller`, `seengrip_node` | **필요** — `/dev/ttyUSB0`, **udev 규칙 없음** |
| B11 | EZIO 입출력 (Modbus/Plus-E) | `ezi_io_controller`, `modbus_controller` | **재작성** — 2보드·UDP·IN16/I8O8 (§D 참고) |
| B12 | Pololu 트리거 제어 | `pololu_node` | **필요** — 동일 사용 확정 |

---

## C. 조작 (수동)

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| C1 | 리모콘 프레임 파싱 | `can_parser` → `/remote_control` | **완료** — 3차년도 수신기 실측 확정, 중립·비상정지 판정 수정 |
| C2 | 리모콘 → 주행·XY | `iron_md_teleop`(구) / `drive_controller`+`joint_controller`(신) | **부분** — `remote_teleop_node` 신규 작성, 미테스트 |
| C3 | 리모콘 → 횡이동 (S17/S18) | `joint_controller` ±360°/±2880° | **필요** — 2축 구조로 각도 재계산 |
| C4 | 리모콘 → Z축 | 2차년도는 S13=브레이크, S14=위치리셋 | **변경 확정** — 3차년도는 S13=상승, S14=하강 |
| C5 | 리모콘 → 자율주행 시작·정지 (S23/S24) | `rebar_drive_node` | **필요** — Yaw 수동회전은 쓰지 않기로 결정. 단 Yaw(0x148)는 결속 자세전환에 필요하므로 `motor_ids` 추가는 해야 한다 |
| C6 | 리모콘 → 작업 시퀀스 (S21/S22) | `sequence_controller` | **필요** — 그리퍼+Z 시퀀스 |
| C7 | 모드 중재 (S19 Manual / S20 Auto) | `authority_controller`, `navigator_base` | **필요** — 지금은 중재 없음 |
| C8 | 키보드 텔레옵 | 없음 (3차년도 신규) | **완료** — GUI 대신 벤치 조작 수단 |
| C9 | ~~GUI~~ | `robot_control_gui` 1,276줄 | **제거 확정** — 외부 UI + Zenoh 로 조작한다. can2 를 직접 잡는 파일이라 소유권 단일화에도 걸림돌 |

---

## D. 안전

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| D1 | 리모콘 비상정지 | `can_parser` → `/emergency_stop` | **부분** — 판정 로직은 수정 완료, 차단 경로 없음 |
| D2 | **범퍼 → 방향별 주행 차단** | `bumper_node` — 부딪힌 방향만 막고 반대는 열어둠 | **필요** — 채널·극성이 3차년도와 다름 (아래) |
| D3 | **데크끝 감지 → 방향별 차단** | `deck_edge_node` 926줄, seg 추론 | **필요** — 카메라 구성 변경 반영 |
| D4 | 장애물·사람 감지 | `obstacle_detector` (YOLO+깊이) | **필요** — 전·후방 ZED X |
| D5 | 상부 리미트 센서 | `ezi_io_node` → `/limit_sensors/*` | **필요** — 상부 보드 채널 미확인 |
| D6 | STOP 스위치 | — | **필요** — 3차년도 하부 IN01 실측 |
| D7 | Z축 과부하 감지 | `z_torque_monitor`, 2단계 토크 감지 | **필요** |
| D8 | 모터측 하드 워치독 | B5 참고 | **필요** |

**범퍼 채널·극성이 다르다.** 2차년도 `bumper_node` 는 IN08 전방 / IN09 우측 / IN10 후방 /
IN11 좌측, **전부 b접점(평상시 ON)** 을 가정한다 (2026-08-19 8회 실측).
3차년도 실측은 전방 IN07(NO) · 후방 IN05(NO) · 좌측 IN13(NC) · 우측 IN10(NC) —
**채널도 다르고 극성이 섞여 있다.** 그대로 옮기면 전방·후방 범퍼가 거꾸로 동작한다.

---

## E. 원점복귀·관절

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| E1 | 관절 명령 총괄 | `joint_controller` 1,454줄 | **필요** |
| E2 | 호밍 시퀀스 | `homing_controller` 1,106줄 | **필요** — ID·2축·리미트 반영 |
| E3 | Yaw 호밍 보호 | `a8b28c6` 에 포함 | **필요** |
| E4 | 횡이동 제어 | `lateral_motion`·`lateral_encoder_calibration` | **부분** — 3차년도는 `lateral_node`(2축)로 동작 중. 구현 4벌 정리 필요 |
| E5 | 브레이크 해제·잠금 | `safe_brake_release` 서비스 | **부분** — 자동 해제가 주석 처리. 재시작마다 잠김 |

---

## F. 자율주행 [A] — UI 웨이포인트

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| F1 | 미션 상태기계 | `navigator` 1,469줄 | **필요** — 웨이포인트 주행을 쓰기로 결정. 단 UI 재구현에 맞춰 불필요 기능 정리 |
| F2 | 경로추종 PID | `rebar_controller` 1,531줄 | **필요** (F1 과 한 묶음) |
| F3 | 전/후방 odom 선택 | `pose_mux` | **필요** — ZED X 전원 확보 후 |
| F4 | 경로 생성 | `path_generation/` (강화학습 시뮬 포함) | **제거 방향** — 웨이포인트 생성 기능은 UI 재구현 시 빼기로 결정 |

---

## G. 자율주행 [B] — 배근 인지 (핵심)

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| G1 | 배근 인지 스텝 주행 | `rebar_drive_node` 1,777줄 | **필요** — 좌표를 모른 채 보이는 배근을 따라가는 반응형 |
| G2 | 배근 간격 추정·이동거리 계산 | `coverage_planner` (순수함수) | **필요** — ROS 무관이라 이식 쉬움 |
| G3 | 리모콘 S20+S23/S24 로 시작·정지 | `rebar_drive_node` | **필요** — C5 와 충돌 (S23/S24 용도 결정 필요) |
| G4 | UI 버튼 → 자율결속 실행 | `auto_tying_launcher` | **판단필요** |

---

## H. 자율결속

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| H1 | 교차점 검출 (Orbbec + YOLO) | `orbbec_detector` 369줄, `orbbec_crossing.pt` | **필요** — 모델 **재학습 중**. 완료 후 반영 |
| H2 | 자세별 호모그래피 → 로봇 XY | `orbbec_detector`, `homography_orbbec.yaml` | **필요** — 3차년도 재캘리브레이션 필수 |
| H3 | CAD 좌표 변환 | `orbbec_cad_transform` 601줄 | **필요** |
| H4 | 6포인트 순회 결속 | `tying_orchestrator` 2,168줄 | **필요** — 자세 전환(Yaw)·Y 범위 재측정 |
| H5 | ~~교차점 검출 서비스~~ | `rebar_detection` 991줄 (zedxmini 기반) | **제거 확정** — Orbbec 으로 대체. zedxmini 가 3차년도에 없음 |
| H6 | 측면 판정 | `usb_side_cam_node`, `side_judge` 도구 | **재작성** — Orbbec 305 기반으로 대체 확정 |
| H7 | 그리퍼 동작 시퀀스 | `sequence_controller` | **필요** |
| H8 | 트리거 | `pololu_node` / `/trigger_control` | **필요** — 2차년도와 동일하게 사용 |

---

## I. 데이터 수집·캘리브레이션

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| I1 | 학습 데이터 수집 | `data_acq_launcher`, `dual_camera_recorder` | 필요 (낮은 우선도) |
| I2 | 호모그래피 적합 | `tools/calibration/fit_homography_orbbec.py` | **필요** — H2 전제 |
| I3 | YOLO 학습 파이프라인 | `tools/orbbec/orbbec_train.py` 등 | 필요 시 |
| I4 | 모터 진단·설정 도구 | `tools/motor/*` | **완료** — 3차년도 브랜치에 6종 이식됨 |

---

## J. UI 연동

| # | 기능 | 2차년도 | 3차년도 상태 |
|---|---|---|---|
| J1 | UI 연동 | `zenoh_client` | **재구현** — 쓰긴 하지만 새로 만든다. 프로토콜·범위는 추후 논의 |
| J2 | 통합 상태 발행 | `rebar_publisher` → `/mission/status` | **재구현** — J1 과 한 묶음 |

---

## 우선순위 제안

**1군 — 안전. 자율 동작 전에 반드시.**
A2(ROM 재적용) · B3/B4/B5(토크·모터측 워치독) · B8(can3 재연결) ·
D2(범퍼 방향별 차단) · D5/D6(리미트·STOP) · C7(모드 중재) · D1(비상정지 차단 경로)

**2군 — 수동 조작 완성.**
C2·C3·C4·C6 · E5(브레이크 정책) · C5/G3(S23/S24 용도 결정)

**3군 — 자율주행 [B] + 결속.** 이게 3차년도 본체다.
G1·G2 · H1·H2·H3·H4 · D3(데크끝) · D4(장애물) · E2(호밍)

**4군 — [A] 웨이포인트 주행과 UI.**
F1·F2·F3 · J1/J2(UI 재구현 — 프로토콜 확정 후) · C9(GUI 유지 여부만 판단)

**제거 대상.** `rebar_detection`(991줄) · `zedxone_publisher` · zedxmini 의존 코드 ·
`path_generation` 웨이포인트 생성 · `dual_camera_recorder`(zedxmini 기반, 필요하면 재작성) ·
`robot_control_gui`(1,276줄) · `usbcam_publisher`

---

## 남은 결정 사항

1. **UI 프로토콜·기능 범위** — 새로 구현하기로 했으니 무엇을 남기고 무엇을 뺄지 별도 논의.
2. **상부 EZIO 출력 8점 배선** — 보류. 로봇 동작 구현을 먼저 한다.
3. **A4(시계·화면 설정)** — 보류.

## 작업 순서 (2026-09-29 확인)

**호밍 동작을 먼저 구현한다.** 선행 조건이 있다 — §D5(상부 리미트 채널 매핑)와
0x148 등록, 브레이크 해제 정책이 호밍의 전제다. 상세는 별도 계획으로.

---

## 이식 불가·확인 필요 (하드웨어)

| 항목 | 상태 |
|---|---|
| ZED X 2대 (전·후방) | 캡처카드 배럴잭 전원 미연결로 미인식 |
| Yaw 0x148 | `motor_ids` 에 없어 토픽 없음 |
| 상부 EZIO 출력 8점 | 무엇이 연결됐는지 미확인 |
| 그리퍼 udev | 규칙 없음 |
| 0x145 온도센서 | 불량 (0xEC 고정) — 과열 보호 불가 |
| ZED 래퍼·Orbbec SDK | 2차년도 압축에서 제외됨. 별도 설치 필요 |
