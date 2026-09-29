# 3차년도 시스템 구조 — 설계 계약

**작성일:** 2026-09-29
**대상 독자:** 3차년도 구현을 진행하는 개발자
**같이 볼 문서:** `YEAR3_FEATURE_INVENTORY.md`(무엇을 옮길지) · `YEAR3_MIGRATION_PLAN.md`(순서)

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
| `stage_node` | 상부 X/Y/Z/Yaw 축 동작. `/joint_cmd` → `/joint_control` | ≤500 | `joint_controller`(횡이동 제외) |
| `lateral_node` | 횡이동 2축. `/lateral/step` | ≤300 | 3차년도 구현(동작 검증됨) |
| `homing_node` | 원점복귀 시퀀스. 리미트 구독 + 축 명령 | ≤600 | `homing_controller` |

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
| `crossing_detector` | Orbbec 교차점 검출 + 호모그래피 → 로봇 XY | `orbbec_detector` + `orbbec_cad_transform` |
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
| `rebar_control_old` 전체 | 3,948 | 참고만. 이식 완료 후 제거 |

---

## 8. 구현 순서

각 단계는 **직전 단계가 실측 확인된 뒤에만** 진행한다.

| 단계 | 내용 | 완료 판정 |
|---|---|---|
| **S0** | `axes.yaml` + 검사 스크립트 3종(R1·R2·R3) | 검사 전부 통과 |
| **S1** | L1 소유자 정리 — `motor_bridge` 범위 확정, `remote_bridge` 분리 | can2·can3 소유자 각 1개 |
| **S2** | L2 `safety_node` + L1 최종 차단 + 0xB3 | 범퍼·STOP·비상정지로 실제 정지 |
| **S3** | **L3 `homing_node`** — 첫 기능. 골격의 본보기 | Z 단독 → X → Y → Yaw → 전체 시퀀스 |
| **S4** | L3 나머지 — `drive_node`·`stage_node` 분리, 텔레옵을 `/joint_cmd` 로 전환 | 기존 동작 재현 |
| **S5** | L4 `mode_arbiter` + `tying_sequence` | 수동/자율 전환, 결속 1회 |
| **S6** | L5 인지 이식 — `crossing_detector`·`deck_edge`·`obstacle_detector` | 검출 좌표 정확도 |
| **S7** | L4 [B] `rebar_drive` + `tying_orchestrator` | 단일 포인트 → 순회 |
| **S8** | L4 [A] `navigator`·`path_follower` | 웨이포인트 추종 |
| **S9** | L6 `ui_bridge` 새 구현 | UI 조작·상태 표시 |

**S3 가 호밍이다.** S0~S2 가 선행 조건이다 — 축 정의(S0)와 리미트·안전(S2)이 없으면
호밍을 만들 수 없다. 리미트 토픽은 이미 나오고 있으므로 S2 는 판정·차단만 남았다.

---

## 9. 지금 상태

| 항목 | 상태 |
|---|---|
| `ezi_io_node` (L1) | **완료** — 2보드, 극성 흡수, 16개 토픽 |
| 리미트·범퍼 채널 매핑 | **완료** — 상부 7개·하부 6개 실측 |
| 리모콘 프로토콜 | **완료** — 프레임·비트 실측 확정 |
| Yaw 0x148 등록 | **완료** — `joint_6` |
| `position_control_node` | 동작하지만 1,946줄. S1 에서 `motor_bridge` 로 축소 |
| `remote_teleop_node` | 임시 — can3 직접 읽기. S1 에서 `/remote_control` 구독으로 전환 |
| 상부 스테이지 리미트 접촉 | **하드웨어 수정 대기** (2026-09-30 예정) |

---

## 10. 이 문서를 고칠 때

- 새 기능을 넣을 곳이 §4 에 없으면 **먼저 이 문서에 노드를 추가**하고 규모를 정한다.
- 토픽을 추가하면 §5 표에 넣는다. 넣지 않으면 R3 검사에서 실패한다.
- 규모 상한을 넘기게 되면 "상한을 올린다" 가 아니라 **책임을 쪼갠다.**
