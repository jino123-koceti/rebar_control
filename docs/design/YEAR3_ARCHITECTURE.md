# 3차년도 시스템 구조 — 설계 계약

**작성일:** 2026-09-29
**대상 독자:** 3차년도 구현을 진행하는 개발자
**같이 볼 문서:** `YEAR3_FEATURE_INVENTORY.md` (무엇을 옮길지)

> 이 문서는 **계약**이다. 새 기능을 넣을 때 "어느 계층의 어느 노드에 넣는가"를 여기서
> 먼저 찾는다. 해당되는 곳이 없으면 코드를 쓰기 전에 이 문서를 고친다.

---

## 1. 왜 계약이 필요한가

2차년도 코드(약 3만 줄)가 복잡해진 기전은 다섯 가지였고, 전부 증거가 있다.

| 기전 | 증거 |
|---|---|
| 자원에 소유자가 없다 | can2 를 여는 파일 **6개** |
| 새 걸 만들고 옛 걸 안 지운다 | 3중 스택 공존, 횡이동 구현 **4벌** |
| 상수를 참조 대신 복사 | `0x14X` 가 **10개 파일**, 주석은 2차년도 배치로 남아 오진 유발 |
| 인터페이스가 정본이 아니다 | `yaw_min` 발행 / `yaw_home` 구독 — 에러 없이 조용히 끊겨 있었다 |
| 새 책임을 둘 곳이 없어 기존 노드가 부푼다 | `tying_orchestrator` 2,103줄, `position_control_node` 1,946줄 |

2차년도에도 계층 설계와 리팩토링 계획 문서가 있었다. **그런데도 무너졌다.**
문서는 위반을 막지 못한다. 그래서 §6 에 **기계적으로 검사되는 규칙**을 둔다.

---

## 1.5 리팩토링 원칙

계층을 세우는 것과 별개로, **동작을 보존한 채 구조를 바꾸는 방법**에도 규칙이 필요하다.
검증된 코드가 전체의 12% 뿐인 상태에서 한 번에 갈아치우면 문제가 생겼을 때
하드웨어인지 코드인지 가릴 수 없다.

1. **인터페이스를 먼저 고정한다.** `rebar_base_interfaces` 의 메시지를 계층 경계로
   확정하고, 이후 리팩토링은 경계 **안쪽만** 바꾼다. 경계를 바꿀 땐 별도 단계로.
2. **동작하는 것을 끄지 않는다 (strangler).** `position_control_node` 를 한 번에
   교체하지 않는다. can2 의 유일한 소유자로 두고 기능을 **밖으로 덜어낸다.**
3. **기준선을 먼저 기록한다.** 구조를 건드리기 전에 현재 수치를 남긴다 —
   wheel_radius 0.02912, 속도상한 0.25 m/s · 0.5 rad/s, 횡이동 12/12 완주 전류 12.13A.
   리팩토링 후 같은 값이 나오는지로 회귀를 판단한다.
4. **하드웨어 없이 검증할 수단을 만든다.** `vcan` + 기록된 candump 로그 재생.
   리모콘 비상정지 판정 수정을 이 방식으로 검증했다(캡처 재생으로 3회 확인).
5. **삭제는 대체가 확인된 뒤에, 근거를 남기고.** 옛 코드는 브랜치·태그로만 보존한다.
6. **한 단계에 한 종류만 바꾼다.** "ID 정리 + 파일 분해" 를 같이 하면 원인을 가릴 수 없다.

---

## 2. 계층 구조

```
L6 외부 연동      ui_bridge · status_aggregator
                        ↕ /mission/command, /mission/status
L5 인지           crossing_detector · deck_edge · obstacle_detector · side_judge
                        ↓ 검출 결과·차단 신호
L4 작업/미션      mode_arbiter · navigator · path_follower
                  rebar_drive · tying_orchestrator · tying_sequence
                        ↓ /cmd_vel, /joint_cmd, /lateral/step  (축 단위 의도)
L3 동작           drive_node · stage_node · lateral_node · homing_node
                        ↓ /drive_control, /joint_control  (모터 단위 명령)
L2 안전           safety_node  →  /safety/state
                        ↓ (L1 이 최종 차단)
L1 하드웨어       motor_bridge(can2) · remote_bridge(can3) · ezi_io_node(EZIO×2)
                  gripper_node(Modbus RTU) · trigger_node(Pololu)
                        ↓ 물리 버스
L0 인터페이스     rebar_base_interfaces (메시지 정의만)
```

### 계층 규칙

1. **위에서 아래로만 명령한다.** L4 가 L1 을 직접 부르지 않는다.
   2차년도는 `tying_orchestrator` 가 `/joint_control`(L1 입력)을 직접 발행했다.
   그 결과 축 보호·좌표 변환이 두 곳에 생겼다. 3차년도는 L3 를 반드시 경유한다.
2. **아래에서 위로는 상태만 올린다.** L1 은 무엇이 자기를 부르는지 몰라야 한다.
3. **같은 계층끼리 명령하지 않는다.** 필요하면 한 단 위에서 중재한다.
4. **안전은 L1 에서 최종 차단한다.** 판정은 L2 가 하고, 차단은 모터 직전에서 한다.
   상위에서 막으면 "무엇이 명령했든" 을 보장할 수 없다 (2차년도도 같은 결론).

---

## 3. 자원 소유자 — 하나씩만

| 자원 | 소유 노드 | 비고 |
|---|---|---|
| **can2** (모터, 1 Mbps) | `motor_bridge` | 유일. 다른 노드는 토픽만 쓴다 |
| **can3** (리모콘, 250 kbps) | `remote_bridge` | 수신 오류 시 1초 후 재연결(2차년도 실측 필수) |
| **EZIO 2보드** (0.5/0.6) | `ezi_io_node` | 완료. 극성을 여기서 흡수 |
| **Modbus RTU** (그리퍼) | `gripper_node` | udev 규칙 필요 |
| **Pololu** (트리거) | `trigger_node` | |
| 카메라 | 외부 드라이버 | ZED wrapper · orbbec_camera |

`motor_bridge` 는 검증된 `position_control_node` 를 **줄여서** 만든다. 새로 쓰지 않는다.
축소는 S1 이 아니라 **S4** 에서 일어난다 — 이 파일의 부피는 주행 보호와 관절 처리이고,
그건 L3(`drive_node`·`stage_node`)로 옮겨야 줄어든다. S1 은 "누가 버스를 여는가" 만 정리한다.

⚠ **지금 can2 에 소유자가 둘이다.** `lateral_axes.py` 가 자기 소켓을 따로 연다
(검사 R2 가 잡아냈다). 서로 다른 모터 ID 를 쓰고 있어 증상이 없지만 구조상 위반이다.
횡이동은 3차년도에서 검증된 몇 개 안 되는 기능이라 회귀 위험이 있어, 이 항목만
S1 에서 뒤로 미루고 S4 에서 `lateral_node` 를 L3 로 정리할 때 함께 처리한다.
지금 이 파일에 들어 있는 것 중 **호밍·시퀀스·GUI 성격은 넣지 않는다** — 그게 2차년도가
2,000줄로 부푼 경로다.

---

## 4. 노드 목록과 책임

### L1 하드웨어

| 노드 | 책임 | 규모 | 2차년도 출처 |
|---|---|---|---|
| `motor_bridge` | can2 송수신, 모터 보호(전류·워치독·0xB3), `/drive_control`·`/joint_control` → CAN, `/motor_feedback` 발행 | ≤600 | `position_control_node`(줄임) + `can_sender`·`can_parser` 일부 |
| `remote_bridge` | can3 → `/remote_control`. 재연결 | ≤200 | `can_parser` can3 부분 |
| `ezi_io_node` | 리미트·범퍼·스위치 → Bool 토픽 (극성 흡수) | ≤300 | 재작성 완료 |
| `gripper_node` | 그리퍼 Modbus RTU | ≤300 | `seengrip_node` |
| `trigger_node` | 트리거 | ≤150 | `pololu_node` |

### L2 안전

| 노드 | 책임 |
|---|---|
| `safety_node` | 비상정지·범퍼·STOP·리미트·데크끝·장애물·하트비트를 모아 **판정만** 하고 `/safety/state` 발행 |

`/safety/state` 는 **방향별 마스크**를 담는다. 범퍼가 눌렸다고 전 방향을 막으면 빠져나올
수단이 없어져 그 자리에 갇힌다 (2차년도 `bumper_node`·`deck_edge` 가 같은 결론).

### L3 동작

| 노드 | 책임 | 규모 | 2차년도 출처 |
|---|---|---|---|
| `drive_node` | `/cmd_vel` → `/drive_control`. 슬루·좌우 배분·드리프트 보정 | ≤400 | `drive_controller` |
| `stage_node` | 상부 X/Y/Z/Yaw 축 동작. `/joint_cmd` → `/joint_control` | ≤620 ※ | `joint_controller`(횡이동 제외) |
| `lateral_node` | 횡이동 2축. `/lateral/step` | ≤300 | 3차년도 구현(동작 검증됨) |
| `homing_node` | 원점복귀 시퀀스. 리미트 구독 + 축 명령 | ≤700 | `homing_controller` |
| `remote_teleop` | 리모콘 입력(`/remote_control`) → 축 명령. CAN 을 모른다 | ≤300 | `iron_md_teleop_node` 조작 규칙 |

**`homing_node` 상한을 600 → 700 으로 올렸다 (2026-10-03).** 600 은 2차년도
`homing_controller` 이식분만 보고 잡은 값이었고, 3차년도에 **yaw 자세 판별 + 탐색 방향
유도**(리미트가 하나뿐이라 방향을 틀리면 기계 끝단에 박는다), **브레이크 해제 확인**(고정
대기로는 모자라다 — 실측 1.50초), **준비자세(READY)** 가 더해졌다. 한도를 지키려고
주석을 깎는 쪽이 더 나쁘다 — 그 주석이 실측으로만 얻은 지식이다.

※ **`stage_node` 500 → 620 (2026-10-03).** 500 은 X/Y/Z 만 보고 잡은 값인데 위 표가
맡기는 Yaw 가 들어오면서 **별개의 이동 방식**이 추가됐다 — mm 가 아니라 자세 번호로
받고, 도달을 단회전 자세 재판별로 확인하고, 회전이 **지나가는 중간 자세들의 교집합**을
먼저 검사한다. 거기에 자세별 가동 범위 강제가 함께 들어왔다. 자세한 근거는
`tools/check/check_structure.py` 의 `R4_LIMITS` 주석에 있다.

**`stage_node` 와 `homing_node` 를 나누는 이유:** 호밍은 "축을 어떻게 움직이나" 가 아니라
"어떤 순서로 원점을 찾나" 다. 2차년도도 `joint_controller` 에서 `homing_controller` 를
분리했다(커밋 `e494f8f`). 같은 판단을 유지한다.

### L4 작업/미션

| 노드 | 책임 | 규모 | 2차년도 출처 |
|---|---|---|---|
| `mode_arbiter` | **수동/자율 권한과 [A]/[B] 주행 중재.** 누가 `/cmd_vel` 을 쓸 수 있는지 결정 | ≤300 | `authority_controller` + `navigator_base` 통합 |
| `navigator` | [A] 웨이포인트 미션 상태기계 | ≤500 | `navigator`(1,469줄에서 축소) |
| `path_follower` | [A] 경로추종 제어 | ≤400 | `rebar_controller`(1,531줄에서 축소) |
| `rebar_drive` | [B] 배근인지 스텝 주행 | ≤800 | `rebar_drive_node` 1,777줄 |
| `tying_orchestrator` | 결속 포인트 순회 | ≤800 | `tying_orchestrator` 2,103줄 |
| `tying_sequence` | 그리퍼+Z 결속 동작 1회 | ≤400 | `sequence_controller` |

**`mode_arbiter` 가 새로 필요한 이유:** 2차년도는 [A] 와 [B] 가 둘 다 `/cmd_vel` 에 쏘면서
중재가 없었다. 코드에 "섞이면 덜컥거린다" 고 적혀 있고, 그래서 `auto_tying_launcher` 로
명령 이름을 분리하는 우회를 했다. 3차년도는 중재를 명시적으로 둔다.

### L5 인지

| 노드 | 책임 | 2차년도 출처 |
|---|---|---|
| `crossing_detector` | Orbbec 교차점 검출 → 로봇 XY. **호모그래피가 아니라 depth 역투영 + 강체변환**이다 (2차년도가 2026-08-05 에 교체했다). 모델은 RF-DETR (YOLO 아님) | `orbbec_detector` + `orbbec_cad_transform` |
| `deck_edge` | 주행가능 판정 → 방향별 차단 | `deck_edge_node` 926줄 |
| `obstacle_detector` | 사람·장애물 → 일시정지 | 그대로 |
| `side_judge` | 측면 판정 (Orbbec 305) | `usb_side_cam_node` 재작성 |

### L6 외부 연동

| 노드 | 책임 |
|---|---|
| `ui_bridge` | 외부 UI ↔ ROS (Zenoh). **새로 구현.** 웨이포인트 생성 등은 제외 |
| `status_aggregator` | 시스템 상태를 하나로 모아 발행 |

---

## 5. 토픽 계약

**이름이 정본이다.** 여기 없는 토픽을 쓰면 검사에서 실패한다.
2차년도 `yaw_min`/`yaw_home` 불일치가 바로 이 계약이 없어서 생겼다.

### 명령 (위 → 아래)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/cmd_vel` | Twist | L4, 텔레옵 | `drive_node` |
| `/joint_cmd` | JointControl | L4, 텔레옵, `homing_node` | `stage_node` |
| `/lateral/step` | Int32 | L4, 텔레옵 | `lateral_node` |
| `/drive_control` | DriveControl | `drive_node` | `motor_bridge` |
| `/joint_control` | JointControl | `stage_node`, `lateral_node` | `motor_bridge` |
| `/gripper_control` | GripperControl | `tying_sequence` | `gripper_node` |
| `/trigger_control` | Bool | `tying_sequence` | `trigger_node` |
| `/homing_cmd` | String | L4, UI | `homing_node` |
| `/control_mode_request` | String | L3·L4 (권한 요청자) | `mode_arbiter` |
| `/stage/goal` | Point | L4, 캘리브레이션 도구 | `stage_node` |
| `/stage/goal_deg` | Point | 캘리브레이션 도구 | `stage_node` |
| `/stage/stop` | Empty | L4, UI | `stage_node` |

### 상태 (아래 → 위)

| 토픽 | 타입 | 발행 |
|---|---|---|
| `/motor_feedback` | MotorFeedback | `motor_bridge` |
| `/remote_control` | RemoteControl | `remote_bridge` |
| `/limit_sensors/{x_min,x_max,y_min,y_max,z_min,z_max,yaw_home}` | Bool | `ezi_io_node` |
| `/bumpers/{front,rear,left,right}` | Bool | `ezi_io_node` |
| `/switches/{start_rear,stop}` | Bool | `ezi_io_node` |
| `/safety/state` | (신규) | `safety_node` |
| `/homing_status` | String | `homing_node` |
| `/control_mode` | String(JSON) | `mode_arbiter` |
| `/stage/status` | String(JSON) | `stage_node` |
| `/rebar/crossings` | RebarGrid | `crossing_detector` |
| `/rebar/detector_status` | String | `crossing_detector` |
| `/deck_edge_block` | String(JSON) | `deck_edge` |
| `/obstacle_pause` | Bool | `obstacle_detector` |
| `/encoder_odom` | PoseStamped | `odom_node` |
| `/mission/status` | String(JSON) | `status_aggregator` |

**규약:** Bool 은 항상 "True = 동작(도달·눌림·차단)". 극성은 L1 에서 흡수한다.

---

## 6. 기계적으로 검사할 규칙

`tools/check/` 에 스크립트로 둔다. 커밋 전에 돌린다.

| # | 규칙 | 검사 방법 | 막는 기전 |
|---|---|---|---|
| R1 | 모터 ID 리터럴 금지 | 소스에서 `0x14[1-8]` 검색. 허용 예외는 `axes.yaml` 과 프로토콜 파일 1개 | 상수 복사 |
| R2 | 자원 소유자 1개 | `can.Bus`·`AF_CAN`·`FAS_Connect`·`serial.Serial` 를 여는 파일 수를 자원별로 확인 | 소유자 없음 |
| R3 | 토픽 계약 일치 | 코드의 발행/구독 이름을 긁어 §5 표와 대조. 짝 없는 토픽은 실패 | 인터페이스 드리프트 |
| R4 | 노드 크기 상한 | §4 규모 초과 시 경고 | 노드 부풀기 |
| R5 | 중복 구현 금지 | 같은 기능의 모듈이 둘 이상이면 실패 (횡이동·CAN·odom) | 안 지우기 |

**R1 의 전제:** `config/axes.yaml` 을 단일 소스로 만든다.

```yaml
axes:
  drive_right: {can_id: 0x141, model: X4-36, gear: 36, ...}
  drive_left:  {can_id: 0x142, ...}
  lateral_1:   {can_id: 0x143, sign: +1, ...}
  lateral_2:   {can_id: 0x144, sign: -1, ...}
  x:   {can_id: 0x145, joint: joint_3, limit_min: x_min, limit_max: x_max, brake: true}
  y:   {can_id: 0x146, joint: joint_4, limit_min: y_min, limit_max: y_max, brake: true}
  z:   {can_id: 0x147, joint: joint_5, limit_min: z_min, limit_max: z_max, brake: true}
  yaw: {can_id: 0x148, joint: joint_6, limit_home: yaw_home}
```

---

## 7. 삭제 대상

대체가 실측으로 확인되면 **즉시 지운다.** 옛 코드는 브랜치·태그로만 보존한다.

| 대상 | 줄 | 사유 |
|---|---|---|
| `robot_control_gui` | 1,276 | 외부 UI 로 대체. can2 를 직접 잡는다 |
| `rebar_detection`(zedxmini) | 991 | Orbbec 으로 대체. zedxmini 가 3차년도에 없다 |
| `zedxone_publisher` · `usbcam_publisher` | 250 | 해당 카메라 없음. 측면판정은 305 로 |
| `lateral_motion` · `lateral_encoder_calibration` | 612 | entry point 없음. `lateral_node` 로 대체 |
| `can_manager` 중 중복 경로 | — | `motor_bridge` 단일화 |
| `path_generation` 웨이포인트 생성 | — | UI 재구현 시 제외 |
| `teleop_keyboard` | 351 | 리모콘으로 대체. **리모콘 주행 검증 후에** 지운다 |
| `rebar_control_old` 전체 | 3,948 | 참고만. 이식 완료 후 제거 |

---

## 7.5 중복 현황 (삭제·통합 대상 실측)

| 중복 | 파일 | 줄 |
|---|---|---|
| 횡이동 4벌 | `lateral_node`(검증됨) · `lateral_axes` · `lateral_motion`(미사용) · `lateral_encoder_calibration`(미사용) | 217·574·380·232 |
| CAN 접근 2벌 | `can_manager`(동작) · `can_sender`+`can_parser`(미검증) | 239·493·496 |
| 자세·odom 3개 | `encoder_odom` · `pose_mux` · `odom_to_pose` | 302·315·115 |

`lateral_motion` 과 `lateral_encoder_calibration` 은 entry point 가 없어 **아무도 쓰지 않는다.**

---

## 8. 구현 순서

각 단계는 **직전 단계가 실측 확인된 뒤에만** 진행한다.

| 단계 | 내용 | 완료 판정 |
|---|---|---|
| **S0** | `axes.yaml` + 검사 스크립트 3종(R1·R2·R3) | 검사 전부 통과 |
| **S1** | L1 소유자 정리 — `remote_bridge` 분리, `robot_control_gui` 삭제, can3 직접 읽기 제거 | can3 소유자 1개, GUI 제거 |
| **S2** | L2 `safety_node` + L1 최종 차단 + 0xB3 | 범퍼·STOP·비상정지로 실제 정지 |
| **S3** | **L3 `homing_node`** — 첫 기능. 골격의 본보기 | Z 단독 → X → Y → Yaw → 전체 시퀀스 |
| **S4** | L3 나머지 — `drive_node`·`stage_node` 분리, 텔레옵을 `/joint_cmd` 로 전환.<br>**`position_control_node` 의 실제 축소(1,960→600)가 여기서 일어난다** | 기존 동작 재현 |
| **S5** | L4 `mode_arbiter` + `tying_sequence` | 수동/자율 전환, 결속 1회 |
| **S6** | L5 인지 이식 — `crossing_detector`·`deck_edge`·`obstacle_detector` | 검출 좌표 정확도 |
| **S7** | L4 [B] `rebar_drive` + `tying_orchestrator` | 단일 포인트 → 순회 |
| **S8** | L4 [A] `navigator`·`path_follower` | 웨이포인트 추종 |
| **S9** | L6 `ui_bridge` 새 구현 | UI 조작·상태 표시 |

**S3 가 호밍이다.** S0~S2 가 선행 조건이다 — 축 정의(S0)와 리미트·안전(S2)이 없으면
호밍을 만들 수 없다. 리미트 토픽은 이미 나오고 있으므로 S2 는 판정·차단만 남았다.

---

## 9. 지금 상태 (2026-09-29)

### 끝난 것

| 항목 | 내용 |
|---|---|
| `ezi_io_node` (L1) | 2보드 재작성, 극성 흡수, 16개 토픽. 실장비 확인 |
| 리미트·범퍼 채널 매핑 | 상부 7개·하부 6개 실측 확정 |
| 리모콘 프로토콜 | 프레임·비트 실측 확정. `0x00`=출력비활성(송신기 꺼짐/START 전), `0x62`=활성, `0x80`=비상정지 |
| Yaw 0x148 등록 | `joint_6`. 재시작 후 7개 모터 초기화 확인 (`0x148: -427.2°`) |
| `axes.yaml` | 축 정의 단일 소스 |
| 검사 R1~R5 + 기준선 | 위반 314건 기록. 늘어날 때만 실패 |
| **S1** `robot_control_gui` 삭제 | −1,276줄. python-can 개방 3→2 파일 |
| **S1** `remote_bridge` 신규 (L1) | can3 소유자. 20Hz 발행, 4축 ±1.0 정규화 확인 |
| **S1** `remote_teleop` 전환 (L3) | can3 직접 읽기 제거 → `/remote_control` 구독. 계층 위반 하나 해소 |
| 설계 문서 정리 | 구버전 `YEAR3_MIGRATION_PLAN.md` 제거(단계 번호 R/S 충돌). 정본은 이 문서 + `FEATURE_INVENTORY` 둘 |
| 브랜치 분리 | 3차년도는 신규 장비이므로 `feature/year3-new-equipment` 로 분리 (기존 `feature/year3-equipment` 는 `cfb4da6` 에 보존) |

### S1 에 남은 것

| 항목 | 비고 |
|---|---|
| 리모콘 주행 실검증 | 신호 경로는 확인했다. 주행 검증은 보류 (2026-09-29 현장 판단) |
| `teleop_keyboard` 삭제 | **리모콘 주행이 검증된 뒤에.** 지금은 검증된 유일한 수동 조작 수단이다 |
| `lateral_axes` CAN 개방 정리 | S4 로 미룸 (횡이동 회귀 위험) |

### S0 에 남은 것

| 항목 | 비고 |
|---|---|
| `0x14X` 리터럴 290건 | **파일을 이식·수정하는 시점에 같이 처리한다.** 일괄 교체하면 이식 대상 파일(`can_sender` 20건, `homing_controller` 30여 건)을 두 번 고친다 |
| R3 검사를 §5 계약 표와 대조 | 지금은 "짝 없는 토픽" 만 본다 |

### 하드웨어 대기

| 항목 | 상태 |
|---|---|
| 상부 스테이지가 리미트에 닿지 않음 | 하드웨어 담당자 수정 예정. **호밍(S3) 실검증의 선행 조건** |
| ZED X 2대 | 캡처카드 배럴잭 전원 미연결로 미인식 |
| 상부 EZIO 출력 8점 배선 | 미확인 (보류) |
| 교차점 검출 모델 | **해결(2026-09-30)** — `new_weights_260930.pt` 수령, RF-DETR Medium 으로 확정. 클래스 순서만 미확인 (아래) |

### 작업 중 발견

- **`homing_controller.py` 가 CMakeLists 설치 목록에 없었다.** 이 패키지는 `ament_cmake` +
  `install(PROGRAMS)` 인데 `homing_controller` 는 `setup.py` entry point 에만 있어서
  `ros2 run` 이 애초에 불가능했다. 설치 목록에 추가했다.
- 2차년도 `ezi_io_node` 는 `/limit_sensors/yaw_min` 으로 발행하고 호밍·관절·송신 노드는
  `yaw_home` 을 구독했다 — **yaw 리미트가 실제로 안 붙어 있었을 수 있다.**
- **신규 교차점 모델은 YOLO 가 아니다.** `new_weights_260930.pt` 는 Roboflow 학습
  **RF-DETR Medium** 이라서 ultralytics 로 열리지 않는다. `rfdetr` 런타임을 2026-09-30 에
  설치했다 (numpy 1.26.4 · JetPack OpenCV 4.5.4 · cv_bridge 무손상 확인).
  Orin 실측 FP32 70ms(14fps) → **FP16 36ms(28fps)**, `model.inference(dtype=torch.float16)`.
- **모델 체크포인트에 클래스 이름이 없다** (`args.class_names is None`). rfdetr 은
  `class_id` 를 0-기반 인덱스로만 주므로 이름 순서를 코드가 공급해야 한다.
  `crossing,tie,untie` 는 알파벳 순 **가정**이다 — 실영상으로 확정할 것
  (`tools/test/rfdetr_check.py`).
- **2차년도의 75% 뒤집힘은 신규 모델에서 재현되지 않았다.** 실카메라 10프레임에서
  지점별 최고 신뢰도 슬롯이 내내 일정했다(뒤집힘 0%). 대신 6개 지점 중 3개에서
  **한 프레임 안에** 두 슬롯이 동시에 떴다 — DETR 은 NMS 가 없어 생기는 중복이고
  승자는 항상 같다. **지점당 최고 신뢰도 하나만 남기면 된다.**
- **분류 슬롯이 4개이고 뜨는 것은 slot2·slot3 뿐이다** (slot0·slot1 은 threshold 0.05
  에서도 안 뜬다). 이름 대응은 Roboflow 클래스 목록으로 확정할 것.
- 검출 십자가 교차점 위에 정확히 찍히는 것을 실영상에서 확인했다 — 2차년도 좌표변환이
  쓰는 **박스 중심**을 그대로 쓸 수 있다.

## 9.5 위험

| 위험 | 완화 |
|---|---|
| 모터 ID 오배정으로 엉뚱한 축 구동 | `axes.yaml` 참조, 1축씩 저속 확인 |
| 슬롯 이름 오배정으로 `tie` 를 `untie` 로 읽음 → **이중결속** | Roboflow 클래스 목록으로 slot2·slot3 확정. 확정 전에는 슬롯 번호로만 다룬다(`rfdetr_check.py`) |
| can2 이중 소유로 버스 사고 | 소유자 1개 원칙(§3), `can_sender` 동시 기동 금지 |
| 범퍼·STOP 미연동 상태의 자율주행 | S2 완료 전 자율주행 금지 |
| Z축 브레이크 해제 시 자중 낙하 | `axes.yaml` 의 `never_auto_release`. 기구 확인 후 판단 |
| 0x145 과열 보호 불가 (온도센서 불량) | 장시간 부하 시 수동 감시 |
| 리팩토링 회귀를 못 잡음 | §1.5 의 기준선·재생 테스트를 먼저 |
| 미검증 코드를 검증된 것으로 착각 | `YEAR3_FEATURE_INVENTORY.md` 의 검증 상태 표를 기준으로 |

---

## 10. 이 문서를 고칠 때

- 새 기능을 넣을 곳이 §4 에 없으면 **먼저 이 문서에 노드를 추가**하고 규모를 정한다.
- 토픽을 추가하면 §5 표에 넣는다. 넣지 않으면 R3 검사에서 실패한다.
- 규모 상한을 넘기게 되면 "상한을 올린다" 가 아니라 **책임을 쪼갠다.**
