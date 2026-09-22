# 배근 인지 자율주행 + 자율결속 구현 정리

**최종 갱신: 2026-08-06** — 첫 실구동·실결속 통합 테스트 완료 시점 기준

리모콘 S20(auto) + S23으로 시작하는 **[B] 비전 자율주행**과 `tying_orchestrator`
**자율결속**의 통합 동작을 정리한다. 기존 [A] UI 웨이포인트 주행
(navigator → rebar_controller)은 그대로 두고 별개 경로로 동작한다.

---

## 1. 아키텍처

```
  [A] UI 웨이포인트 ─▶ navigator ─▶ rebar_controller ──┐
  [B] 배근인지 자율 ─▶ rebar_drive_node ───────────────┴─▶ /cmd_vel ─▶ drive_controller
                            ▲                                            ▲
                            └─ /deck_edge_status ─ deck_edge_node ─ /deck_edge_block
                                                   (A·B 공통 안전차단)
                            │
                            └─ /mission/command ─▶ tying_orchestrator ─▶ /rebar_motion_cmd
                                                                        (TYING_COMPLETE:<n>)
```

`/cmd_vel` 소유권은 A가 우선이다. `/mission/feedback`의 state가 navigating/paused면
B는 **시작 거부**, 주행 중이면 **즉시 양보(ABORT)** 한다. 정지 후 `zero_hold_sec`(1.0s)
동안만 0을 발행하고 침묵해 버스를 A에 돌려준다.

`/control_mode`는 발행하지 않는다 — navigator_base가 유일 소유자이며, 주행권은
`/rebar_motion_cmd`로 요청한다.

### 관련 노드

| 노드 | 패키지 | 역할 |
|---|---|---|
| `rebar_drive_node` | rebar_vision | [B] 스텝 주행 상태머신, 결속 요청/대기 |
| `deck_edge_node` | rebar_vision | 배근 seg 기반 주행가능 판정 + heading 산출 |
| `tying_orchestrator` | rebar_vision | 실제 결속 수행 (스테이지 이동 + 결속기) |
| `drive_controller` | rebar_base_control | `/cmd_vel` → 주행모터, 최후 안전차단 |

### 주요 토픽

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/deck_edge_status` | deck_edge → drive | verdict(GO/SLOW/STOP), rebar_frac, cam, heading_deg |
| `/deck_edge_block` | deck_edge → drive_controller | 방향별 차단 (A·B 공통 안전망) |
| `/travel_direction` | drive → deck_edge | 진행방향 통보 (active_only 추론 대상 결정) |
| `/encoder_odom` | base → drive | 스텝 폐루프 기준 위치 (**ZED VSLAM odom 아님**) |
| `/remote_control` | can_parser → drive | S20/S23/S24 |
| `/mission/command` | drive → orchestrator | 결속 대상 교차점 좌표 |
| `/rebar_motion_cmd` | orchestrator → drive | `TYING_COMPLETE:<n>` |
| `/tying/status` | orchestrator → drive | 결속 후 스테이지 실제 자세(`pose`) |

---

## 2. 동작 시퀀스

```
IDLE ─S23─▶ FWD_DETECT ⇄ FWD_STEP ─(배근끝)─▶ FWD_SETTLE
       ─▶ REV_DETECT ⇄ REV_STEP ─(배근끝)─▶ REV_SETTLE ─▶ DONE

    S24 / estop / S20 해제 / A 미션 개입 → ABORT
```

한 사이클:

1. **검출** — Orbbec Gemini 2L YOLO로 교차점 검출 → CAD 변환으로 로봇좌표화
2. **간격 측정** — 검출 전체(넓은 범위)로 pitch(열/행 간격) 산출
3. **결속 대상 선별** — 스테이지 도달범위 안 교차점만, 좌/우 자세로 분류
4. **결속** — `/mission/command`로 요청 → `TYING_COMPLETE` 대기 (상한 180s)
5. **이동거리 산출** — 마지막 결속열 X + 마진 30mm
6. **스텝 주행** — 엔코더 폐루프로 그 거리만큼 이동, 매 tick deck_edge 감시
7. 반복 → deck_edge STOP이면 감속·정지 후 방향 전환

### 검출범위와 결속범위의 분리 (중요)

pitch 측정용 범위(`detect_*`, ±3000mm)와 결속 자세분류용 범위(`tie_*`)는
**반드시 분리**한다. 과거 두 가지 회귀가 있었다:

- 검출에 결속 도달범위를 쓰자 7개 중 6개가 버려져 pitch가 기본값으로 떨어짐
- 반대로 검출범위(±3000)로 자세분류를 하자 전부 `r`로 분류돼 좌측이 항상 0점 →
  지그재그(우→자세변경→좌)가 일어나지 않음 (2026-08-06 실주행에서 발견)

---

## 3. 실행 방법

### 전제

- 호밍 완료 + 스테이지가 검출자세(X≈0 부근) — Orbbec 시야 확보
- Orbbec `depth_registration:=true` (CAD 변환에 정합 depth 필요)
- 주행 카메라 2대 기동 (deck_edge 판정용)
- `robot-control.service` 실행 중

### 노드 기동

서비스는 `use_deck_edge` / `use_rebar_drive`가 **기본 false**라 두 노드를 띄우지
않는다. 테스트 시에는 별도 실행을 권한다 (서비스에 `rebar_drive_arm:=true`를
박아두면 다음 부팅부터 항상 실구동 상태가 되어 위험하다).

```bash
# 판정 노드 (토픽 기본값이 현재 배치와 일치하므로 인자 불필요)
ros2 run rebar_vision deck_edge

# 주행 노드 — dry-run(모션 없음)
ros2 run rebar_vision rebar_drive

# 주행 노드 — 실구동 + 실결속
ros2 run rebar_vision rebar_drive --ros-args -p arm:=true -p do_tying:=true
```

launch로 한 번에:

```bash
ros2 launch rebar_control full_system.launch.py use_zedxmini:=true \
    use_deck_edge:=true use_rebar_drive:=true rebar_drive_arm:=true
```

### 조작

| 입력 | 동작 |
|---|---|
| S20 | Auto 모드 (유지 필요 — 해제 시 ABORT) |
| S23 | 시작 / ABORT 후 재시작 |
| S24 | 정지 (ABORT) |

### 빌드

`rebar_vision` / `rebar_control` 모두 **symlink-install**(egg-link)이고 launch 파일도
`install → build → src` 심볼릭 링크다. 파이썬 소스와 launch 파일 수정은
**노드 재시작만으로 반영**되며 `colcon build`가 필요 없다.

---

## 4. 주요 파라미터

### rebar_drive_node

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `arm` | **false** | false = dry-run(모션 없음) |
| `do_tying` | **false** | true면 실제 결속 수행 |
| `speed` | 0.10 | m/s |
| `approach_mm` / `approach_scale` | 80.0 / 0.5 | 남은거리 80mm 이하에서 감속 |
| `tolerance_mm` | 15.0 | 도달 판정 허용오차 |
| `tie_timeout_sec` | 180.0 | 결속 완료 대기 상한 |
| `heading_enabled` | **false** | 배근 heading 조향 (부호 규약은 실측 확정) |
| `heading_deadband_deg` | 2.0 | 데드밴드 |
| `start_grace_sec` | 8.0 | 첫 판정 대기 유예 (DDS 디스커버리 경합 회피) |
| `stale_sec` | 1.0 | 판정 끊김 임계 |
| `max_step_mm` / `max_step_sec` | 1200 / 60 | 폭주 방지 |
| `detect_*_mm` | ±3000 | pitch 측정용 (넓게) |
| `tie_x_max_mm` | 345.0 | 결속 도달범위 X |
| `tie_right_y_*` / `tie_left_y_*` | 0~142 / 124~288 | 자세별 Y 도달범위 |

### deck_edge_node

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `front_topic` | `/zedxmini2/.../compressed` | **전방 카메라** |
| `back_topic` | `/zedxmini1/.../compressed` | **후방 카메라** |
| `seg_hz` | 4.0 | 총 추론 주기 (GPU 부하 감축으로 8.0→4.0) |
| `stop_thr` / `slow_thr` | 0.45 / 0.55 | rebar_frac 임계 |
| `active_only` | true | 진행방향만 추론 |
| `block_when_stale` | true | 판정 없으면 차단 (안전우선) |

> **카메라 배치 (2026-08-06 변경)**: zedxmini1/2를 좌·우 측면에서 전방/후방으로
> 물리 이동했다. **zedxmini2 = 전방(SN 54946194), zedxmini1 = 후방(SN 56755054)**.
> 실제 시야 스냅샷으로 방향을 확인했다. ZED X(zed_front/zed_back)는 비활성.

---

## 5. 안전 장치

| 계층 | 내용 |
|---|---|
| 기본 dry-run | `arm:=true` 없으면 모션 없음 |
| 결속 기본 off | `do_tying:=true` 없이는 결속기 미동작 |
| 주행 중 감시 | 매 tick deck_edge 판정 확인, STOP이면 스텝 미완이어도 즉시 중단 |
| staleness 워치독 | 판정 1.0s 끊기면 중단 (첫 판정은 8.0s 유예) |
| 스텝 제한 | 최대 1200mm / 60s, 한 방향 전체 600s |
| A 우선권 | 웨이포인트 주행 개입 시 즉시 양보 |
| 최후 방어선 | drive_controller가 cmd_vel 0.5s 끊기면 자동정지 + 방향차단 |
| 리모콘 | S24 즉시 정지, S20 해제 시 ABORT |

---

## 6. 실주행 결과 (2026-08-06)

첫 실구동 + 실결속 통합 테스트. 전진 2사이클 → 배근끝 감지 → 후진 2사이클.

| # | 방향 | 결속범위내 격자 | pitch 신뢰도 | 결속 | 목표 | 실제 |
|---|---|---|---|---|---|---|
| 1 | 전진 | 열2×행2 | high | 3점 | 258mm | 243mm |
| 2 | 전진 | 열2×행2 | high | 4점 | 319mm | 배근끝 감지로 중단 |
| 3 | 후진 | 열1×행2 | low | 2점 | 179mm | 164mm |
| 4 | 후진 | 열2×행2 | high | 5점 | 331mm | S24 정지 (278mm 지점) |

**누적 결속 14점.** 예외·타임아웃·판정 누락 없음. 로그의 유일한 ERROR는
의도된 S24 정지다.

검증된 항목:

- 검출 → 결속 → 이동 사이클이 끊김 없이 반복
- 전방 카메라(zedxmini2) 기반 배근끝 감지로 스텝 중간 안전 정지
  (`rebar_frac=0.45`에서 STOP)
- 방향 전환 시 후방 카메라(zedxmini1) 첫 판정이 `start_grace` 8s 안에 수신,
  `FWD_SETTLE → REV_DETECT` 1.5s 소요
- 감속 프로파일 정상 (목표 73mm 전부터 vx 0.100 → 0.050)
- 결속 소요시간 점당 약 7~8초 (3점 26.5s / 4점 33.0s / 2점 20.0s / 5점 38.7s),
  180s 상한 대비 여유 충분

---

## 7. 확인된 이슈 / 다음 작업

### (1) 스텝 이동거리가 일관되게 15mm 부족

243/258, 164/179 — 두 번 모두 정확히 **-15mm**. `tolerance_mm`(15.0)와 값이
일치하므로 도달 판정이 허용오차만큼 조기 종료하는 것으로 보인다. 다음 열까지
마진 30mm를 두고 있어 당장 열을 건너뛰지는 않지만 마진의 절반을 잠식한다.

- 확인 필요: 스텝마다 누적되는지, 1회성인지
- 검토안: 목표거리에 `tolerance_mm`를 더해 보정하거나 도달 판정을 목표 중심으로 변경

### (2) 조향 로직 미검증

전 구간 `wz=+0.000`이었다. `heading_enabled`가 **기본 false**라 조향이 아예
동작하지 않았기 때문이며, 데드밴드에 걸린 것이 아니다. 이번 테스트에서
heading 값 자체는 -1.4° ~ +0.9°로 안정적으로 산출됐다(`heading_deg`).

부호 규약은 실측으로 확정돼 있다 — `angular.z` 양수 = 좌회전(CCW), `heading_sign=+1`,
후진은 `heading_rev_sign=-1`. 다만 **RC 테스트베드는 배근이 고정돼 있지 않아**
궤도가 배근 위에서 자주·세게 비틀리면 격자가 무너진다. 그래서 게인을 러프하게
잡아뒀다(`heading_kp=0.008`, `heading_max=0.04`, 데드밴드 2°).

- 다음: `heading_enabled:=true`로 켜고 저속에서 조향 방향·강도 검증

### (3) 후진 진입 직후 pitch 신뢰도 low

사이클 3에서 결속범위 안에 열이 하나(`cols_x=[149.2]`)뿐이라 열 간격을 직접
재지 못하고 검출 전체(4열)에서 추정했다. 배근 끝 근처라 생긴 일시적 현상으로,
한 스텝 이동 후 사이클 4에서는 열2×행2 / high로 회복됐다. 설정 조정은 불필요해
보이나, 반복되면 검출자세 Y 또는 결속 도달범위를 재검토한다.

### (4) pose_mux odom 미가동 (이 기능과는 무관)

`rebar_drive_node`는 `/encoder_odom`을 쓰므로 영향 없다. 다만 [A] 웨이포인트
주행은 `/robot_pose`가 필요한데, 현재 zedxmini가 `depth_mode:=NONE` +
`pos_tracking_enabled:=false`라 odom이 발행되지 않는다. ZED 래퍼의
`startPosTracking()`이 `depth_mode`가 NONE이면 거부하기 때문이다.

- [A]를 쓰려면 두 카메라의 `depth_mode`를 PERFORMANCE 이상 +
  `pos_tracking_enabled:=true`로 올려야 한다 (GPU 부하 증가 — 프리즈 조사와 상충)

---

## 관련 파일

| 경로 | 내용 |
|---|---|
| `src/rebar_vision/rebar_vision/rebar_drive_node.py` | [B] 주행 상태머신 |
| `src/rebar_vision/rebar_vision/deck_edge_node.py` | 배근 판정 + heading |
| `src/rebar_vision/rebar_vision/coverage_planner.py` | 다음 이동거리 산출 |
| `src/rebar_vision/rebar_vision/tying_orchestrator_node.py` | 결속 수행 |
| `src/rebar_control/launch/full_system.launch.py` | 통합 launch |
| `src/rebar_vision/config/tying_orchestrator.yaml` | 결속 도달범위 실측치 |
| `scripts/service/robot_control_service.sh` | 서비스 기동 스크립트 |
