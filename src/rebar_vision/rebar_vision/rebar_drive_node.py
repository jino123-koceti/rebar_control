#!/usr/bin/env python3
"""배근 인지 자율주행 노드 — orbbec 교차점 간격 기반 **스텝 주행**.

기존 [A] UI 웨이포인트 주행(navigator → rebar_controller)은 **그대로 두고**, 그와 별개인
[B] 비전 자율주행을 담당한다. 좌표를 미리 모른 채 보이는 배근을 따라가는 반응형이라
웨이포인트 파이프라인에 맞지 않아 별도 노드로 둔다.

  [A] UI 웨이포인트 ─▶ navigator ─▶ rebar_controller ──┐
  [B] 배근인지 자율 ─▶ rebar_drive_node(이 노드) ──────┴─▶ /cmd_vel ─▶ drive_controller
                            ▲                                              ▲
                            └─ /deck_edge_status ─ deck_edge_node ─ /deck_edge_block
                                                   (A·B 공통 안전차단)

## 동작 (리모콘 S20 auto + S23 시작 / S24 정지)
  전진: orbbec 교차점 검출 → 간격(pitch) 측정 → **다음 주행거리 = 열수 × pitch_x**
        → 그 거리만큼 전진(엔코더 폐루프) → 반복
        → 배근 끝(deck_edge STOP) 감지 시 감속·정지
  후진: 방향만 바꿔 동일 반복 → 배근 끝 감지 시 감속·정지 → 완료
  ※ 결속은 `do_tying`으로 켠다(기본 False=주행만). 켜면 검출한 교차점을
    `/mission/command` TYING_START로 tying_orchestrator에 넘기고 완료를 기다린 뒤
    다음 스텝으로 간다. 도달범위 내 점이 0개면 결속을 건너뛴다.

  IDLE ─S23─▶ FWD_DETECT ⇄ FWD_STEP ─(배근끝)─▶ FWD_SETTLE ─▶ REV_DETECT ⇄ REV_STEP
        ─(배근끝)─▶ REV_SETTLE ─▶ DONE          (S24/estop/S20해제 → ABORT)

  횡이동(lateral_enabled)을 켜면 'ㄹ'자 커버리지가 된다 — 데크 끝마다 옆 레인으로:
    FWD_SETTLE ─▶ LATERAL_JUDGE ⇄ LATERAL_MOVE ─▶ REV_DETECT ─▶ … ─▶ REV_SETTLE
               ─▶ LATERAL_JUDGE ⇄ LATERAL_MOVE ─▶ FWD_DETECT ─▶ …
    · 이동거리 = orbbec 교차점 **Y간격** 기반 next_lateral_mm (전후진과 같은 방식)
    · ⚠ 70mm 회전 N회로 쪼개되 **1회전마다 측면 카메라 재판정**한다. 일괄 지령하면
      2회째에 배근이 끊겨도 못 멈춘다(데크 밖으로 떨어짐).
    · 방향은 **첫 전환에서만** 좌우 비교로 정하고 이후 고정 — 매번 정하면 갔던
      레인으로 되돌아가 같은 곳을 다시 결속한다.

## 전제
  · **호밍 완료 + 스테이지가 검출자세(X=0 부근)** 여야 Orbbec 시야가 안 가린다
  · orbbec 카메라 `depth_registration:=true` (CAD 변환에 정합 depth 필요)
  · deck_edge_node 실행 중 (배근 끝 판정 제공)

## 안전
  · 기본 **dry-run**(모션 없음). 실제 구동은 arm:=true
  · 주행 중 매 tick deck_edge 판정 감시 — STOP이면 스텝 미완이어도 즉시 중단
  · 판정 staleness 워치독, 스텝별 최대 시간/거리 제한
  · drive_controller: cmd_vel 0.5s 끊기면 자동정지 + /deck_edge_block 방향차단(최후 방어선)

## 소유권 규약 (중요)
  · **/cmd_vel**: A와 B가 동시에 쏘면 명령이 뒤섞여 덜컥거린다. A(기존 경로)에 우선권을 주고
    B가 비킨다 — `/mission/feedback`의 state가 navigating/paused면 **시작 거부**,
    주행 중이면 **즉시 양보(ABORT)**. 주행 소유 중에만 발행하고, 정지 후 zero_hold_sec
    동안만 0을 낸 뒤 침묵해 버스를 A에 돌려준다.
  · **/control_mode**: 발행 금지. navigator_base가 유일 소유자(5Hz, 구독자 7개)다.
    주행권은 `/rebar_motion_cmd`로 요청한다 ('MOVE_*' / 'NAVIGATION_COMPLETE').
"""
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from rebar_base_interfaces.msg import RemoteControl, JointControl

from rebar_vision.coverage_planner import compute_next_move

# 리모콘 buttons 인덱스 (can_parser): [s13,s14,s17,s18,s21,s22,s23,s24]
S23_I, S24_I = 6, 7
DRIVING = ('FWD_DETECT', 'FWD_STEP', 'FWD_SETTLE',
           'REV_DETECT', 'REV_STEP', 'REV_SETTLE')
STEPPING = ('FWD_STEP', 'REV_STEP')
LATERAL = ('LATERAL_JUDGE', 'LATERAL_MOVE')
# 작업 중(= [A]에 양보해야 하고, S20 해제 시 중단해야 하는) 상태 전부.
# 횡이동은 cmd_vel을 안 쓰지만 /joint_control 0x143을 쓰므로 [A]와 배타적이어야 한다.
BUSY = DRIVING + LATERAL


class RebarDriveNode(Node):

    def __init__(self):
        super().__init__('rebar_drive_node')

        self.declare_parameter('arm', False)            # False = dry-run(모션 없음)
        self.declare_parameter('speed', 0.10)           # m/s
        self.declare_parameter('slow_scale', 0.4)       # SLOW 판정 시 속도배율
        self.declare_parameter('approach_scale', 0.5)   # 목표 근접 시 감속배율
        self.declare_parameter('approach_mm', 80.0)     # 남은거리 이 이하면 감속
        self.declare_parameter('tolerance_mm', 15.0)    # 도달 판정 허용오차
        self.declare_parameter('start', 'fwd')          # 'fwd' | 'rev'
        # ★ [2026-09-08] 한 방향만 하고 끝낸다. 현장 운용이 이렇게 간다:
        #     전진 결속 → 주행불가로 정지 → **사람이 다음 라인으로 옮김** →
        #     다시 실행(`start:=rev`)해서 후진 결속 → 정지 → 옮김 → 반복.
        #   기본(False)이면 예전대로 전진 막히면 그 자리에서 곧바로 후진해 되돌아온다.
        self.declare_parameter('one_way', False)
        self.declare_parameter('settle_sec', 1.5)       # 방향전환 전 정지 대기
        # ★ STOP 채터링 방어 (2026-08-19)
        #   실측: 기둥을 지나칠 때 deck_edge가 **같은 초에 GO→STOP→GO**로 뒤집혔다.
        #   그때 이 노드는 STOP **한 프레임**만 보고 SETTLE로 넘어가고, settle이
        #   끝나면 판정을 다시 보지도 않고 후진/횡이동을 확정했다.
        #   = 순간 노이즈 하나가 레인 하나를 통째로 건너뛰게 만든다.
        #   deck_edge 쪽에도 해제 히스테리시스를 넣었지만(OBS_RELEASE_*),
        #   on_rebar 플리커 등 다른 경로가 남아 있어 여기서도 막는다.
        self.declare_parameter('stop_confirm_frames', 3)  # 연속 STOP 이만큼이어야 수용
        self.declare_parameter('settle_recheck', True)    # 전환 커밋 직전 판정 재확인
        self.declare_parameter('step_pause_sec', 0.6)   # 스텝 완료 후 정지 안정화 시간
        # 도착 지점에서 '결속한다고 가정'하고 머무는 시간. 실결속은 안 하지만
        # 실제 사이클 타임을 모사하려고 둔다(결속 연동 시 이 자리에 실제 결속이 들어감).
        self.declare_parameter('tie_dwell_sec', 5.0)
        # ★ 실제 결속 수행 여부. True면 검출한 교차점을 tying_orchestrator에 넘겨
        #   결속시키고 TYING_COMPLETE를 기다린 뒤 이동한다(tie_dwell_sec 대신).
        #   ⚠ 기본 false — 켜면 결속기가 실제로 동작하므로 명시적으로 지정할 것.
        self.declare_parameter('do_tying', False)
        self.declare_parameter('tie_timeout_sec', 180.0)   # 결속 완료 대기 상한
        self.declare_parameter('tie_speed_pct', 70)        # orchestrator 스테이지 속도 %
        # ★ 배근 heading 조향 (deck_edge_node가 같은 seg 마스크로 계산해 status에 실어줌)
        #   정렬되면 가로철근이 화면에서 수평 → heading_deg≈0. 틀어지면 기울어진다.
        #   ⚠ **부호 규약 미확정** — 어느 회전방향인지 지면 주행으로 확인 후
        #      heading_sign을 ±1로 맞출 것. 기본 false로 꺼둔다.
        #   ⚠ **RC 테스트베드는 배근이 고정돼 있지 않다.** 궤도가 배근 위에서 자주·세게
        #      비틀리면 격자가 밀려 무너진다. → 일부러 **러프하게** 잡는다:
        #      큰 데드밴드로 웬만한 오차는 무시하고, 걸릴 때도 아주 약하게만 튼다.
        #      (정렬 정확도보다 배근 보존이 우선. 실제 데크에선 키워도 됨)
        # ★ 2026-08-19 재튜닝 — 그 전 설정(kp 0.008 / max 0.04 / db 2.0)은
        #   **사실상 보정이 안 됐다.** 원인이 셋이었다:
        #     ① heading 추정이 과소평가 → rebar_grid.heading()을 스팬 가중으로 수정
        #        (같은 프레임 0.64° → 1.15°). 이걸 안 고치면 데드밴드만 낮춰도 절반만 본다.
        #     ② 데드밴드 2.0°가 **실측 노이즈(σ=0.12°, 범위 ±0.35°)의 17배**.
        #        오차가 2° 미만이면 영원히 무보정 → 실주행에서 head=+2.0°에 눌러앉았다.
        #        2° 틀어진 채 직진 = 거리의 3.5%씩 횡방향 이탈(3m에 10cm).
        #     ③ 게인 자체가 낮음.
        #   ⚠ 배근 보존 취지(아래 주석)는 유지한다. 데드밴드를 좁히는 대신 **작은 오차를
        #      자주·약하게** 잡는 쪽이, 2°까지 방치했다가 세게 트는 것보다 격자에 덜 무리다.
        self.declare_parameter('heading_enabled', False)
        self.declare_parameter('heading_kp', 0.015)        # rad/s per deg (0.008→)
        self.declare_parameter('heading_max', 0.08)        # angular.z 상한 rad/s (0.04→)
        self.declare_parameter('heading_deadband_deg', 0.6)  # 노이즈 ±0.35°의 약 1.7배 (2.0→)
        # 부호 실측 확정 (2026-08-06): `angular.z=-0.1` 발행 → **우회전** 관측.
        #   즉 이 로봇도 **angular.z 양수 = 좌회전(CCW)** 표준 규약이다.
        #   (drive_controller가 linear만 반전해서 뒤집힐 거라 추정했으나 실측은 반대였음)
        #   당시 heading=-1.07°에서 좌회전이 필요했으므로 w>0 → sign=+1.
        self.declare_parameter('heading_sign', 1.0)
        self.declare_parameter('heading_min_bars', 2)      # 가로철근 이 개수 미만이면 조향 안 함
        # 후진 부호. **실측으로 +1.0 확정 (2026-08-13)** — 기존 기본값 -1.0은 틀렸다.
        #   같은 로봇을 기울였다 정렬시키며 두 카메라를 동시에 읽으니
        #     front +3.18 → -0.05 (Δ -3.23),  back +1.49 → -0.92 (Δ -2.41)
        #   로 **같은 방향으로** 움직였다. 즉 같은 요 틀어짐을 두 카메라가 같은 부호로 읽는다.
        #   (전→후방은 단일축 미러가 아니라 **이미지 180° 회전**이고, 180° 회전은
        #    직선의 각도를 보존한다. '시선이 반대니 부호도 반대'라는 추론이 오류였다.)
        #   정렬은 로봇 자세 문제라 필요한 각속도 부호도 진행방향과 무관 → 그대로 +1.0.
        # ⚠ 크기는 다르다(3.23 vs 2.41, 34% 차): 카메라 틸트/화각이 달라 지면 직선각→
        #   이미지각 이득이 다르다. 러프 제어라 무시하지만, 후진 조향이 약하게 느껴지면
        #   그 탓이다(같은 오차에 약 3/4 세기).
        self.declare_parameter('heading_rev_sign', 1.0)
        # ★ 카메라별 heading 영점(장착 롤 오프셋). **로봇이 격자에 정렬된 상태에서
        #   읽히는 값**을 여기 넣으면 그만큼 빼고 제어한다.
        #   ⚠ 왜 필요한가 (2026-08-13 실측): 로봇을 격자에 **정렬시킨 상태**에서
        #      front -0.05° / back **-0.92°**. front는 거의 0인데 back만 1° 가까이 치우쳤다
        #      = 후방 카메라 장착 롤. 보정 없이 켜면 컨트롤러가 '로봇 정렬'이 아니라
        #      '영상 수평'을 목표로 삼아 후진 때마다 0.9°씩 틀어버린다.
        #      캡처: tools/vision_test/heading_offset_calib.py (정렬시킨 뒤 실행)
        #   ★ 2026-08-19 재캘리브 (스팬 가중 추정기 기준 — 추정기를 바꾸면 무효).
        #     사람이 정렬로 판단한 자세에서 front -1.13° / back +1.52°.
        #     ⚠ 이 값은 **아직 주행 검증 전**이다. 카메라만으로는 절대 영점을 못 정한다
        #       (front·back이 같은 부호라 서로 견제 못 함 — 측정 2개 vs 미지수 5개).
        #       직진 후 횡이탈로 확정할 것: tools/drive/heading_zero_drive.py
        #   ★★ 2026-09-22 실주행 재캘리브 (run_20260922_120018, 5레인·33스텝).
        #     기준 = **Orbbec 교차점 행 기울기**(스테이지 mm 좌표 = 실제 요각, 독립 경로).
        #     레인 간 연속(횡이동은 안 돈다)으로 신뢰성 확인. 회귀 head = a·yaw + b:
        #       front  a=0.06  b=+0.77 (r=0.47)   ← 실제 요를 거의 못 본다
        #       back   a=0.25  b=-2.37 (r=0.85)
        #     영점이 b만큼 틀려 있어 컨트롤러가 **없는 오차를 계속 교정** → 레인마다
        #     실제로 6~8° 돌아가는 지그재그(전진 +1→-3.5°, 후진 -2.4→+5°).
        #     영점 = 기존 + b.  front -1.13→-0.36, back +1.52→-0.85.
        #   ✔ 정지 교차검증(같은 날): 반시계로 틀어둔 자세에서 Orbbec 요 +6.5°,
        #     front 원시 +1.47° → 감도 (1.47+0.36)/6.5 = **0.28**, 영점 -0.36과 일치.
        #     주행 중 회귀의 0.06은 요 폭이 좁고 노이즈(±0.8°)에 묻힌 것.
        #     0.28은 **전방 카메라가 거의 수평이라 생기는 기하 한계**(가로철근 화면각
        #     ≈ 요 × sin(틸트)) → 실효 게인이 약하다. 후속: 원근 보정 또는 Orbbec 기울기를
        #     heading 원천으로. 데이터: data/rebar_map/run_20260922_120018.jsonl
        self.declare_parameter('heading_offset_front_deg', -0.36)
        self.declare_parameter('heading_offset_back_deg', -0.85)
        # ★ 후진 카메라 이득 보정 (2026-08-19 실측).
        #   같은 요 변화를 두 카메라가 얼마나 다르게 읽는지 3자세 2전이로 측정:
        #     P0→P1  Δfront -2.10  Δback -4.02   비 1.91
        #     P1→P2  Δfront +3.60  Δback +6.78   비 1.88
        #   → **back이 front의 1.90배**. 보정 없이 후진하면 조향이 1.9배 과하고,
        #     안정성 여유가 2.4배 → 1.3배로 떨어져 진동 위험 구간에 들어간다.
        #   ⚠ 방향이 예전과 **뒤집혔다**: 단순 중앙값 시절엔 back이 0.75배(약함)였는데
        #     스팬 가중으로 바꾸니 1.90배(강함)가 됐다. 추정기를 손대면 반드시 재측정.
        #   검증: back = 1.90·front + 3.78 이 3자세에서 산포 0.09°로 성립.
        self.declare_parameter('heading_rev_gain', 0.53)   # = 1/1.90
        # heading 중앙값 필터 샘플 수. front 편차가 ±0.8°(back은 ±0.1°)라 데드밴드
        # 경계에서 조향이 깜빡인다. 중앙값이라 튄 값 1개는 그대로 버려진다.
        self.declare_parameter('heading_median_n', 3)
        self.declare_parameter('max_step_mm', 1200.0)   # 한 스텝 최대 거리(폭주 방지)
        self.declare_parameter('max_step_sec', 60.0)    # 한 스텝 최대 시간
        self.declare_parameter('max_sec', 600.0)        # 한 방향 전체 최대 시간
        self.declare_parameter('stale_sec', 1.0)        # deck_edge 판정 끊김 임계
        # 첫 판정 대기 유예. DDS 디스커버리에 0.7~1.1s가 걸려 stale_sec(1s)과 경합한다.
        # "한 번도 못 받음"(기동 중)과 "받다가 끊김"(고장)은 다른 조건이라 분리한다.
        self.declare_parameter('start_grace_sec', 8.0)
        # 웨이포인트 주행[A] 피드백이 이 시간 이상 없으면 A 미실행으로 판단
        self.declare_parameter('mission_stale_sec', 2.0)
        self.declare_parameter('rate', 15.0)            # 제어 루프 Hz
        self.declare_parameter('require_remote', True)  # 리모콘 S20/S23 필요 여부
        self.declare_parameter('zero_hold_sec', 1.0)    # 정지 후 0 발행 유지 시간
        # 교차점 검출 실패 시 이 거리로 진행(0이면 중단). 배근 pitch 기본값 근처.
        self.declare_parameter('fallback_step_mm', 0.0)
        self.declare_parameter('detect_retry', 3)       # 검출 재시도 횟수
        # 스테이지 자세 초기값. 호밍 직후는 우측('r'). 이후에는 orchestrator가
        # `/tying/status`의 'pose'로 알려주는 **실제 자세**를 따라간다 —
        # 결속이 좌측에서 끝나면 좌측 검출자세로 복귀하므로 CAD 오프셋도 좌측이어야 한다.
        # ★ 배근 작업도용 기록 (2026-08-19).
        #   주행 중엔 **JSONL 한 줄 append만** 한다 — 플롯/집계는 전부 오프라인
        #   (tools/drive/plot_rebar_map.py). 실시간 처리를 넣으면 검출 스레드가
        #   느려져 주행에 영향을 준다.
        #   ⚠ 기록 실패는 **절대 주행을 막지 않는다**(전부 try/except).
        #   빈 문자열이면 기록 안 함.
        # ★ 결속 이력 메모리 (2026-08-19). 이미 결속한 자리를 **좌표로** 기억해 건너뛴다.
        #   ⚠ 왜 필요한가: `merge_policy=prefer_tie`는 **한 정지 지점 안에서** 여러
        #     프레임을 합칠 때만 듣는다. 스텝이 바뀌면 기억이 없어서, 지난 스텝에
        #     결속한 점을 모델이 untie로 오분류하면 **또 결속한다.**
        #     실측(2026-08-19 한 사이클): 결속 요청 12점 중 고유 7지점,
        #     **중복 4지점(한 곳은 3회) = 42% 낭비.** 전진·후진으로 같은 구간을
        #     두 번 지나므로 특히 잘 걸린다.
        #   0이면 기능 끔. 배근 pitch(~205mm)의 1/3 정도가 적당하다.
        self.declare_parameter('tie_memory_mm', 70.0)
        # S23 재시작 때 결속 이력을 지울지. 기본 True(지운다).
        #   ⚠ 안 지우면 이전 실행의 이력이 남아 **이번에 결속해야 할 점이 통째로
        #     건너뛰어질 수 있다**(예: 지난번에 결속기 전원이 꺼져 실제로는 안 묶였는데
        #     이력에는 '묶음'으로 남은 경우). 결속 누락이 중복보다 나쁘므로 지우는 쪽이
        #     기본이다. 중간에 멈췄다 이어서 하실 때만 False로 두면 중복이 줄어든다.
        self.declare_parameter('tie_memory_reset_on_start', True)
        self.declare_parameter('record_path',
                               '/home/koceti/ros2_ws/data/rebar_map')
        # ★ 학습데이터 수집 (2026-08-19). 검출 시점의 원본 컬러 프레임을 저장한다.
        #   ⚠ 왜: 지금 기록은 좌표뿐이라 **재학습에 못 쓴다**. 오검출을 실시간
        #     필터로 거르는 건 진짜 교차점을 잃을 위험이 커서 포기했고(사용자 판단),
        #     모델을 키우는 쪽으로 간다.
        #   ★ 이 프레임의 가치: 작업도가 **여러 시점에서 같은 점을 교차검증**하므로,
        #     한 프레임만 봐서는 알 수 없는 "이 검출이 진짜였나"를 나중에 판정할 수
        #     있다(2회 이상 관측 = 확인됨). 그게 곧 라벨 힌트가 된다.
        self.declare_parameter('save_frames', True)
        self.declare_parameter('detect_pose', 'r')
        self.declare_parameter('orbbec_enabled', True)  # False면 fallback 거리로만 주행
        self.declare_parameter('orbbec_model',
                               '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt')
        # ⚠️ [2026-08-18] 결속 대상 클래스. 신모델(tie_untie_weights.pt)은
        #   crossing / tie / untie 를 구분한다. **이미 결속된 점(tie)을 다시 결속하지
        #   않도록** 여기 나열된 클래스만 결속 대상으로 삼는다(이중결속 방지).
        #   ⚠ 이동거리 계산에는 **검출된 전부**를 쓴다 — untie만 남기면 격자에서
        #     행이 빠져 피치가 틀어진다. 필터는 결속 대상에만 적용된다.
        #   'crossing'은 결속 상태가 불명확한 검출이다. 포함하면 누락은 줄지만
        #   이중결속 위험이 오르므로 기본값에서 뺐다.
        #   단일 클래스 구모델에서는 이 설정과 무관하게 전부 결속한다(하위호환).
        self.declare_parameter('tie_classes', 'untie')
        #   병합 클러스터에 두 클래스가 섞일 때의 판정. 'prefer_tie'(기본) = tie가
        #   하나라도 있으면 결속 제외 / 'majority' = 다수결.
        #   실측(2026-08-18): 다수결이면 이미 결속된 점을 untie로 잘못 봐 이중결속 발생.
        self.declare_parameter('merge_policy', 'prefer_tie')
        # ⚠️ [2026-08-18] 결속 밀도. 주행방향(X)으로 **한 줄씩 건너뛴다.**
        #   1 = 100%(전부)  2 = 50%(1줄 건너뛰기)  3 = 33%(2줄 건너뛰기)
        #   줄 = 같은 X에 있는 교차점들(좌우 Y 전부). 건너뛰는 것은 줄 단위이고,
        #   남긴 줄 안의 점은 전부 결속한다:
        #        1 2  ← 결속        3 4  ← 건너뜀        5 6  ← 결속
        #   순번은 정차 지점을 넘어 이어진다. 이동거리가 "마지막 줄 + 마진"이라
        #   다음 검출창은 지나온 줄 뒤에서 시작하므로 같은 줄을 두 번 세지 않는다.
        self.declare_parameter('tie_every_n', 1)
        # ⚠ 검출 범위는 **결속용 도달범위(x 0~345, y 0~142/124~288)를 쓰면 안 된다.**
        #   그건 스테이지가 닿는지를 거르는 필터인데, 주행은 스테이지 도달과 무관하게
        #   격자 '간격'만 알면 된다. 실측에서 7개 검출 중 6개가 '범위밖'으로 버려져
        #   교차점 1개만 남았고, pitch가 측정값이 아닌 기본값(195mm)으로 떨어졌다.
        #   → 주행용은 범위를 넓게 열어 검출된 교차점을 모두 pitch 계산에 쓴다.
        self.declare_parameter('detect_x_min_mm', -3000.0)
        self.declare_parameter('detect_x_max_mm', 3000.0)
        self.declare_parameter('detect_y_min_mm', -3000.0)
        self.declare_parameter('detect_y_max_mm', 3000.0)
        self.declare_parameter('detect_frames', 2)      # YOLO 프레임 수(작을수록 빠름)
        self.declare_parameter('detect_timeout_sec', 15.0)
        # ★ 결속 가능(스테이지 도달) 범위 — 이 안의 교차점만 "결속했다"고 보고
        #   그 사이 간격 × 열수로 다음 주행거리를 낸다. 범위 밖은 결속 안 된 열이라
        #   세면 건너뛰게 된다. 값은 tying_orchestrator.yaml 실측치와 맞출 것.
        self.declare_parameter('tie_x_min_mm', 0.0)
        self.declare_parameter('tie_x_max_mm', 345.0)   # max_stage_x_mm
        self.declare_parameter('tie_y_min_mm', 0.0)     # 좌우 자세 합친 Y 전 범위
        self.declare_parameter('tie_y_max_mm', 288.0)
        # ★ 결속 **자세별** Y 도달범위 (tying_orchestrator.yaml 실측치).
        #   ⚠ 이걸로 자세를 분류해야 orchestrator가 지그재그(우→자세변경→좌)를 한다.
        #   검출용 범위(detect_*)를 ±3000으로 넓힌 탓에 localize()가 전부 'r'로
        #   분류해버려 좌측이 항상 0점 → 자세변경이 안 일어났다(2026-08-06 실주행에서 발견).
        #   → pitch 측정용(넓은 범위)과 결속 자세분류용(실제 도달범위)을 분리한다.
        self.declare_parameter('tie_right_y_min_mm', 0.0)
        self.declare_parameter('tie_right_y_max_mm', 142.0)
        self.declare_parameter('tie_left_y_min_mm', 124.0)
        self.declare_parameter('tie_left_y_max_mm', 288.0)
        # ★ 횡이동('ㄹ'자 커버리지) — 전/후진 한 레인을 끝내면 옆 레인으로 넘어간다.
        #   거리는 전후진과 같은 방식(orbbec 교차점 Y좌표 간격)으로 낸다: next_lateral_mm.
        #   ⚠ **한 번에 몰아서 보내지 않는다.** 횡이동은 70mm 단위 회전이라 200mm면
        #      3회전인데, 그 3회를 일괄 지령하면 2회째에 배근이 끊겨도 멈출 수 없다.
        #      → **1회전(70mm)마다 측면 카메라로 재판정**하고 가능할 때만 다음 회전.
        self.declare_parameter('lateral_enabled', False)
        # ★ 횡이동 방향 고정 (2026-09-22). ''=자동(첫 전환에서 좌·우 비교), 'left'|'right'=고정.
        #   고정하면 **반대쪽 카메라는 아예 판정하지 않는다**(측면 요청을 그쪽 한 대로만 보낸다).
        #   왜: 좌측 모노 카메라가 GMSL 링크 불량으로 죽어 있다(9-0029 #0, 재부팅·재결선에도
        #   복구 안 됨). 좌측을 'both'에 넣으면 '영상 없음=불가'가 되어 판정은 안전하지만,
        #   운용상 **좌측 끝단에서 출발해 우측으로만** 덮으면 되므로 굳이 볼 이유가 없다.
        #   ⚠ 끝단 정지는 그대로 동작한다 — 고정 방향 쪽 카메라는 **매 1회전마다** 재판정하고,
        #     한 칸도 못 가면 _finish_lateral 이 DONE 으로 끝낸다.
        self.declare_parameter('lateral_fixed_dir', '')
        self.declare_parameter('lateral_mm_per_turn', 70.0)   # 360° = 70mm (HW 실측)
        self.declare_parameter('lateral_speed_dps', 90.0)   # joint_controller.lateral_max_speed와 동일하게 유지 ([[zed_argus_hang_lateral]])
        # 좌/우 방향 부호. **-1.0 = 실측 확정값**(2026-08-13): left 지령 → 우측 이동이었다.
        # _send_lateral_turn() 주석 참조. 기구 변경 시 눈으로 1회전 재확인할 것.
        self.declare_parameter('lateral_sign', -1.0)
        self.declare_parameter('lateral_max_turns', 6)        # 한 레인 전환 최대 회전수
        self.declare_parameter('lateral_max_lanes', 4)        # 총 레인 전환 상한
        self.declare_parameter('lateral_turn_timeout_sec', 30.0)
        # ★ [2026-09-14] 20 → 45초. 측면 카메라가 **판정할 때 기동**되도록 바뀌어
        #   (deck_edge side_manage_nodes) 응답까지 시간이 더 걸린다:
        #     카메라 기동 4.5s + DDS 디스커버리 ~1s + AE 수렴 1.5s + 3프레임 ~0.5s ≈ 8s
        #   'both' 판정이면 stagger 포함 **약 17초** → 20초는 오탐 ABORT 를 낸다.
        self.declare_parameter('lateral_side_timeout_sec', 45.0)
        self.declare_parameter('lateral_settle_sec', 1.5)     # 회전 후 정지·영상 안정화
        # 상태 전이 fsync 로그 경로(빈 문자열이면 끔). 프리즈 진단용 — _trace() 주석 참조.
        self.declare_parameter('state_trace_path',
                               '/var/log/robot_control/drive_state_trace.csv')

        g = self.get_parameter
        self.dry = not bool(g('arm').value)
        self.speed = float(g('speed').value)
        self.slow_scale = float(g('slow_scale').value)
        self.approach_scale = float(g('approach_scale').value)
        self.approach_m = float(g('approach_mm').value) / 1000.0
        self.tol_m = float(g('tolerance_mm').value) / 1000.0
        self.start_dir = g('start').value
        self.one_way = bool(g('one_way').value)
        self.settle = float(g('settle_sec').value)
        self.stop_confirm = max(1, int(g('stop_confirm_frames').value))
        self.settle_recheck = bool(g('settle_recheck').value)
        self._stop_run = 0          # 연속 STOP 카운트
        self._settle_resume = None  # 전환 취소 시 돌아갈 상태
        self._bumper = None         # {'forward','backward','left','right','reason'}
        self._bumper_t = 0.0        # 마지막 수신 시각
        self._blocked_since = None  # 현재 차단이 시작된 시각
        self._blocked_total = 0.0   # 이번 스텝에서 차단된 누적 시간(타임아웃 제외용)
        self.step_pause = float(g('step_pause_sec').value)
        self.tie_dwell = float(g('tie_dwell_sec').value)
        self.do_tying = bool(g('do_tying').value)
        self.tie_classes = {c.strip() for c in
                            str(g('tie_classes').value).split(',') if c.strip()}
        self.merge_policy = str(g('merge_policy').value)
        self.tie_every_n = max(1, int(g('tie_every_n').value))
        self._col_seq = 0          # 주행 중 만난 줄의 누적 순번 (건너뛰기 판정용)
        self._col_abs = []         # [[절대x_mm, 순번, 관측수]] — 한 줄에 번호 하나
        # 절대좌표에서 같은 줄로 볼 거리. 배근 피치(180~220mm)보다 확실히 작아야
        # 옆줄을 삼키지 않고, 검출 흔들림(±20mm)+오도메트리 드리프트보다는 커야
        # 한다. 110mm면 피치 180 기준 옆줄까지 70mm 여유가 남는다.
        self.col_merge_mm = 110.0
        # 줄(같은 X) 묶는 허용오차. coverage_planner의 cluster_tol_mm(50)과 맞춘다 —
        # 다르면 피치 계산이 센 줄 수와 건너뛰기가 센 줄 수가 어긋난다.
        self.col_tol_mm = 50.0
        self.tie_timeout = float(g('tie_timeout_sec').value)
        self.tie_speed_pct = int(g('tie_speed_pct').value)
        self.tie_points = {'r': [], 'l': []}   # 이번 지점에서 결속할 교차점
        self.tie_done = False                  # TYING_COMPLETE 수신
        self.tie_tied = 0                      # 실제 결속된 점 수
        self.heading_on = bool(g('heading_enabled').value)
        self.heading_kp = float(g('heading_kp').value)
        self.heading_max = float(g('heading_max').value)
        self.heading_db = float(g('heading_deadband_deg').value)
        self.heading_sign = float(g('heading_sign').value)
        self.heading_min_bars = int(g('heading_min_bars').value)
        self.heading_rev_sign = float(g('heading_rev_sign').value)
        self.heading_rev_gain = float(g('heading_rev_gain').value)
        self.heading_offset = {'front': float(g('heading_offset_front_deg').value),
                               'back': float(g('heading_offset_back_deg').value)}
        self.heading_med_n = max(1, int(g('heading_median_n').value))
        self.heading_buf = {'front': [], 'back': []}
        self.max_step_m = float(g('max_step_mm').value) / 1000.0
        self.max_step_sec = float(g('max_step_sec').value)
        self.max_sec = float(g('max_sec').value)
        self.stale_sec = float(g('stale_sec').value)
        self.start_grace = float(g('start_grace_sec').value)
        self.mission_stale_sec = float(g('mission_stale_sec').value)
        self.require_remote = bool(g('require_remote').value)
        self.zero_hold_sec = float(g('zero_hold_sec').value)
        self.fallback_step_m = float(g('fallback_step_mm').value) / 1000.0
        self.detect_retry = int(g('detect_retry').value)
        self.detect_pose = g('detect_pose').value
        self.tie_memory_mm = float(g('tie_memory_mm').value)
        self.tie_memory_reset = bool(g('tie_memory_reset_on_start').value)
        self.save_frames = bool(g('save_frames').value)
        self._frame_dir = None
        self._tied_global = []      # 결속 완료한 전역좌표 [(gx, gy, lane), ...]
        self._rec_path = None
        rp = str(g('record_path').value or '')
        if rp:
            try:
                os.makedirs(rp, exist_ok=True)
                stamp = time.strftime('run_%Y%m%d_%H%M%S')
                self._rec_path = os.path.join(rp, stamp + '.jsonl')
                self.get_logger().warn(f'📝 배근 작업도 기록: {self._rec_path}')
                if self.save_frames:
                    self._frame_dir = os.path.join(rp, 'frames', stamp)
                    os.makedirs(self._frame_dir, exist_ok=True)
                    self.get_logger().warn(f'🖼  학습용 프레임: {self._frame_dir}')
            except Exception as e:
                self.get_logger().error(f'기록 경로 준비 실패(무시하고 진행): {e}')
        self.detect_frames = int(g('detect_frames').value)
        self.detect_timeout = float(g('detect_timeout_sec').value)
        self.tie_x_range = (float(g('tie_x_min_mm').value), float(g('tie_x_max_mm').value))
        self.tie_y_range = (float(g('tie_y_min_mm').value), float(g('tie_y_max_mm').value))
        self.lateral_on = bool(g('lateral_enabled').value)
        _fd = str(g('lateral_fixed_dir').value or '').strip().lower()
        if _fd not in ('', 'left', 'right'):
            self.get_logger().error(f"lateral_fixed_dir='{_fd}' 무효 → 자동(좌·우 비교)으로")
            _fd = ''
        self.lat_fixed_dir = _fd
        self.lat_mm_turn = float(g('lateral_mm_per_turn').value)
        self.lat_speed = float(g('lateral_speed_dps').value)
        self.lat_sign = float(g('lateral_sign').value)
        self.lat_max_turns = int(g('lateral_max_turns').value)
        self.lat_max_lanes = int(g('lateral_max_lanes').value)
        self.lat_turn_timeout = float(g('lateral_turn_timeout_sec').value)
        self.lat_side_timeout = float(g('lateral_side_timeout_sec').value)
        self.lat_settle = float(g('lateral_settle_sec').value)
        self.pose_y_range = {
            'r': (float(g('tie_right_y_min_mm').value), float(g('tie_right_y_max_mm').value)),
            'l': (float(g('tie_left_y_min_mm').value), float(g('tie_left_y_max_mm').value))}

        # 상태 전이 추적 파일 (프리즈에서 살아남게 fsync)
        self._trace_f = None
        tp = g('state_trace_path').value
        if tp:
            try:
                os.makedirs(os.path.dirname(tp) or '.', exist_ok=True)
                self._trace_f = open(tp, 'a')
            except OSError as e:
                self.get_logger().warn(f'상태 추적 로그 열기 실패({tp}): {e}')

        # 검출 워커 스레드 연동 상태
        self._stop = False
        self._det_req = False        # 검출 요청
        self._det_busy = False       # 검출 진행 중
        self._det_done = False       # 결과 준비됨
        self._det_result = None      # 다음 거리(m) 또는 None
        self._det_t = 0.0            # 요청 시각(타임아웃용)
        self._pending = None         # (이동거리 m, 결속대기 종료시각) — 결속 가정 대기 중
        self.orbbec_enabled = bool(g('orbbec_enabled').value)

        self.state = 'IDLE'
        self.state_t = time.time()
        self.dir_t = time.time()            # 현재 방향 시작 시각(max_sec 기준)
        self._zero_until = 0.0
        self._motion_req_t = 0.0            # 마지막 주행권 요청 시각
        self._motion_held = False           # 주행권 보유 중인가(반납 1회성 판단)

        # deck_edge 판정
        self.judge = {'front': None, 'back': None}
        self.judge_t = {'front': 0.0, 'back': 0.0}

        # 횡이동 상태
        self.lat_dir = None          # 'left'|'right' — 첫 판정에서 정하고 이후 고정
        self.lat_target_mm = 0.0     # 이번 레인 전환 목표 거리(측정값)
        self.lat_moved_mm = 0.0      # 이번 레인 전환에서 실제 이동한 거리
        self.lat_turns_left = 0      # 남은 회전 수
        self.lat_lane = 0            # 완료한 레인 전환 수
        self.lat_next = 'FWD_DETECT'  # 횡이동 후 이어갈 주행 방향
        self.lat_side = None         # 측면 판정 결과
        self.lat_side_t = 0.0
        self.lat_req_t = 0.0         # 측면 판정 요청 시각
        self.lat_move_done = False   # /lateral_motion_complete 수신
        self.last_lateral_mm = None  # 최근 검출의 next_lateral_mm
        self.last_lateral_window = None   # [lo, hi] 다음 n행이 다 들어오는 거리 구간

        # 엔코더 (제어부호: x_ctrl = -encoder_odom.x, cmd_vel +x = 전진 = x_ctrl 증가)
        self.x_ctrl = None
        self.step_start_x = None
        self.step_target_m = 0.0
        self.step_no = 0
        self.prev_pitch = (None, None)
        self.detect_fail = 0

        # 리모콘
        self.s20 = self.s24 = self.estop = False
        self.prev_s23 = 0
        self.s23_edge = False

        # 웨이포인트 주행[A] 활성 감시 — /cmd_vel 소유권 상호배제용.
        # navigator는 navigating/paused/mission_done일 때만 5Hz로 feedback을 낸다.
        self.mission_state = None
        self.mission_state_t = 0.0
        self.create_subscription(String, '/mission/feedback', self._mission_cb, 10)

        self.create_subscription(String, '/deck_edge_status', self._status_cb, 10)
        # 범퍼 차단 (bumper_node). **판단은 drive_controller가 한다** — 여기선
        # ① 로그에 드러내고 ② 차단된 시간을 스텝 타임아웃에서 제외하려고 받는다.
        #   ⚠ 왜 필요한가(2026-08-19 실측): 범퍼를 눌러 로봇이 완전히 멈춰 있는데
        #     로그는 `vx=+0.400 GO`를 계속 찍었다. rebar_drive는 자기가 차단당한 걸
        #     몰라서 "명령은 나가는데 왜 안 서지?"로 읽힌다.
        #   ⚠ 타임아웃: 차단 시간을 빼지 않으면 범퍼를 오래 누르고 있는 것만으로
        #     max_step_sec에 걸려 몇 분짜리 작업이 죽는다.
        self.create_subscription(String, '/bumper_block', self._bumper_cb, 10)
        self.create_subscription(RemoteControl, '/remote_control', self._remote_cb, 10)
        self.create_subscription(PoseStamped, '/encoder_odom', self._odom_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # ⚠ /control_mode는 **발행하지 않는다**. navigator_base가 유일 소유자로
        #   자기 FSM 상태를 5Hz로 계속 쏜다(구독자 7개). 여기서 같이 쏘면
        #   last-write-wins로 auto↔idle/manual이 초당 수회 뒤집혀 drive_controller가
        #   "cmd_vel 변환 ↔ 정지"를 토글 → 주행이 덜컥거린다.
        #   대신 tying_orchestrator와 같은 정규 경로인 /rebar_motion_cmd로 **요청**한다:
        #     'MOVE_*' → navigator_base FSM auto→navigating (권한이 auto일 때만 수락)
        #     'NAVIGATION_COMPLETE' → navigating→auto 복귀
        self.motion_cmd_pub = self.create_publisher(String, '/rebar_motion_cmd', 10)
        # 결속 요청은 tying_orchestrator가 듣는 /mission/command 로 (rebar_controller와 동일 경로)
        self.mission_cmd_pub = self.create_publisher(String, '/mission/command', 10)
        # 결속 완료는 /rebar_motion_cmd 에 'TYING_COMPLETE:<n>' 으로 돌아온다.
        # (내가 MOVE_* 를 쏘는 토픽과 같으므로 내 메시지는 걸러진다)
        self.create_subscription(String, '/rebar_motion_cmd', self._motion_cmd_cb, 10)
        # 스테이지 실제 자세 추적 (결속 후 좌/우 어느 쪽으로 끝났는지)
        self.create_subscription(String, '/tying/status', self._tying_status_cb, 10)
        # 횡이동: 측면 주행가능 판정(요청형) + 0x143 회전 지령/완료
        self.side_req_pub = self.create_publisher(String, '/deck_edge/side_request', 10)
        self.create_subscription(String, '/deck_edge/side_status', self._side_status_cb, 10)
        # ⚠ **`/joint_control_cmd` 이지 `/joint_control` 이 아니다** (2026-08-14 실주행에서 발견).
        #   체인:  상위노드 ─▶ /joint_control_cmd ─▶ joint_controller ─▶ /joint_control ─▶ can_sender ─▶ CAN
        #                      (지령 입력)            (12시 정렬·홈기준 변환      (절대위치)
        #                                              ·완료판정·완료신호 발행)
        #   `/joint_control`에 직접 쏘면 can_sender만 받아 **모터는 돌지만**
        #   joint_controller가 관여하지 않아 12시 정렬도, 완료 판정도,
        #   `/lateral_motion_complete` 발행도 없다 → 30s 완료신호 타임아웃 ABORT.
        #   (rebar_controller의 기존 횡이동도 `/joint_control_cmd`를 쓴다 — 같은 경로로 맞춘 것)
        self.joint_pub = self.create_publisher(JointControl, '/joint_control_cmd', 10)
        self.create_subscription(String, '/lateral_motion_complete',
                                 self._lateral_complete_cb, 10)
        self.state_pub = self.create_publisher(String, '/rebar_drive/state', 10)
        self.info_pub = self.create_publisher(String, '/rebar_drive/step', 10)
        self.dir_pub = self.create_publisher(String, '/travel_direction', 10)
        # 사람이 읽을 진행 메시지 → rebar_publisher가 모아 UI로 보낸다.
        #   ⚠ depth를 크게 잡는다: UI 도달 경로가 10Hz 폴링이라 순간적으로 여러 건이
        #     몰리면(예: 스텝완료→검출→결속생략→다음스텝) 얕은 큐에서 밀린다.
        self.event_pub = self.create_publisher(String, '/rebar_drive/event', 50)
        self._ev_seq = 0
        self._ev_verdict = None            # 감속/복귀 전이 추적

        self.orbbec = None
        if self.orbbec_enabled:
            self._init_orbbec()

        self._det_thread = threading.Thread(target=self._detect_worker, daemon=True)
        self._det_thread.start()

        self.timer = self.create_timer(1.0 / float(g('rate').value), self._tick)
        self.get_logger().warn(
            f"배근 스텝주행 {'[DRY-RUN 모션X]' if self.dry else '[ARM 실구동]'} "
            f"speed={self.speed} start={self.start_dir}"f"{' one_way' if self.one_way else ''} "
            f"orbbec={'ON' if self.orbbec else 'OFF(fallback)'} "
            f"{'리모콘 S20+S23 시작' if self.require_remote else '자동시작'}")
        if not self.require_remote:
            self._begin()

    # ---------- 초기화 ----------
    def _init_orbbec(self):
        """Orbbec 로컬라이저(YOLO+CAD변환). 실패하면 fallback 거리 주행으로 강등."""
        try:
            from rebar_vision.orbbec_detector import OrbbecLocalizer
            g = self.get_parameter
            xmin = float(g('detect_x_min_mm').value)
            xmax = float(g('detect_x_max_mm').value)
            ymin = float(g('detect_y_min_mm').value)
            ymax = float(g('detect_y_max_mm').value)
            self.orbbec = OrbbecLocalizer(
                self,
                model_path=g('orbbec_model').value,
                color_topic='/camera/color/image_raw',
                depth_topic='/camera/depth/image_raw',
                info_topic='/camera/color/camera_info',
                yrange={'r': (ymin, ymax), 'l': (ymin, ymax)},
                x_min=xmin, x_max=xmax,
                merge_policy=self.merge_policy)
            self.get_logger().info('  [orbbec] 로컬라이저 로드 ✅')
        except Exception as e:
            self.orbbec = None
            self.get_logger().error(
                f'  [orbbec] 초기화 실패 → 교차점 검출 없이 fallback 거리 주행: {e}')

    # ---------- 입력 ----------
    def _status_cb(self, msg):
        try:
            r = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        cam = r.get('cam')
        if cam in self.judge:
            self.judge[cam] = r
            self.judge_t[cam] = time.time()
            # heading은 판정마다(≈4Hz) 들어온다 → 여기서 중앙값 창을 채운다.
            hd, bars = r.get('heading_deg'), r.get('heading_bars', 0)
            if hd is not None and bars >= self.heading_min_bars:
                b = self.heading_buf[cam]
                b.append(float(hd) - self.heading_offset[cam])
                del b[:-self.heading_med_n]
            else:
                self.heading_buf[cam].clear()   # 근거 없으면 옛 값도 버린다

    def _remote_cb(self, m):
        self.s20 = bool(m.switch_s20)
        self.estop = bool(m.emergency_stop)
        b = list(m.buttons)
        s23 = b[S23_I] if len(b) > S23_I else 0
        self.s24 = bool(b[S24_I]) if len(b) > S24_I else False
        if self.prev_s23 == 0 and s23 == 1:
            self.s23_edge = True
        self.prev_s23 = s23

    def _tying_status_cb(self, msg):
        """orchestrator가 알려주는 실제 스테이지 자세를 따라간다.

        결속이 좌측에서 끝나면 좌측 검출자세(Y=160)로 복귀하므로, 다음 검출의
        CAD 오프셋·겹침구간 우선순위도 좌측이어야 한다. 고정 'r'이면 어긋난다.
        """
        try:
            p = json.loads(msg.data).get('pose')
        except (ValueError, TypeError):
            return
        new = {'right': 'r', 'left': 'l'}.get(p)
        if new and new != self.detect_pose:
            self.get_logger().warn(f'  ↻ 스테이지 자세 변경 감지: {self.detect_pose} → {new}')
            self.detect_pose = new

    def _motion_cmd_cb(self, msg):
        """TYING_COMPLETE:<n> 수신 (내가 발행한 MOVE_* 는 무시)."""
        d = msg.data
        if d.startswith('TYING_COMPLETE'):
            try:
                self.tie_tied = int(d.split(':')[1])
            except (IndexError, ValueError):
                self.tie_tied = 0
            self.tie_done = True

    def _side_status_cb(self, msg):
        try:
            self.lat_side = json.loads(msg.data)
            self.lat_side_t = time.time()
        except (ValueError, TypeError):
            pass

    def _lateral_complete_cb(self, msg):
        """joint_controller의 횡이동 완료 신호. 'AUTO:<+->:<turns>' 형식."""
        if msg.data.startswith('AUTO:'):
            self.lat_move_done = True

    def _mission_cb(self, msg):
        try:
            self.mission_state = json.loads(msg.data).get('state')
            self.mission_state_t = time.time()
        except (ValueError, TypeError):
            pass

    def _mission_active(self):
        """웨이포인트 주행[A]이 /cmd_vel을 쓰고 있는가.

        A와 B가 동시에 /cmd_vel을 쏘면 명령이 뒤섞여 주행이 덜컥거린다.
        A가 기존 주행경로이므로 **A에 우선권**을 주고 B가 비킨다.
        navigator는 navigating/paused/mission_done에서만 feedback을 내므로,
        일정 시간 소식이 없으면 미션 없음으로 본다.
        """
        if self.mission_state is None:
            return False
        if time.time() - self.mission_state_t > self.mission_stale_sec:
            return False                       # 피드백 끊김 = 미션 종료/미실행
        return self.mission_state in ('navigating', 'paused')

    def _odom_cb(self, msg):
        # rebar_controller와 동일 규약: 부호 반전해 cmd_vel +전진 = 증가
        self.x_ctrl = -msg.pose.position.x

    # ---------- UI 이벤트 ----------
    #   왜 별도 채널인가 (2026-09-15): 진행 상황이 **로그에만** 남아 현장 UI에는
    #   "자율 결속 전송" 한 줄 뒤로 아무것도 안 보였다. UI(외부 노트북)는
    #   `/mission/status` 하나만 보므로, rebar_publisher가 여기를 모아 실어 보낸다.
    #   ⚠ `seq`가 핵심이다 — UI는 10Hz로 폴링하는데 "최신 한 줄" 필드만 두면
    #     그 사이에 지나간 이벤트를 통째로 놓친다. UI는 seq가 커진 것만 append하면 된다.
    _EV_ERROR = ('ABORT',)
    _EV_INFO = ('DETECT', 'STEP_PROGRESS')

    def _ev(self, level, text, log=True):
        """진행 메시지를 로그와 UI에 **동시에** 낸다. 문구는 한 곳에서만 쓴다.

        log=False: 로그엔 이미 상세본이 있고 UI엔 요약만 보낼 때 (검출·결속 요청).
                   로그는 사후분석용이라 길어도 되지만 UI는 한 줄이어야 읽힌다.
        """
        self._ev_seq += 1
        if not log:
            pass
        elif level in self._EV_ERROR:
            self.get_logger().error(text)
        elif level in self._EV_INFO:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        m = String()
        m.data = json.dumps({'seq': self._ev_seq, 'ts': time.time(),
                             'level': level, 'state': self.state, 'text': text},
                            ensure_ascii=False)
        self.event_pub.publish(m)

    # ---------- 상태 ----------
    def _set_state(self, s):
        if s != self.state:
            self.get_logger().warn(f"상태: {self.state} → {s}")
            self._trace(f'{self.state} -> {s}')
        self.state = s
        self.state_t = time.time()

    def _trace(self, what):
        """상태 전이를 **fsync**해서 남긴다 — 프리즈에서 살아남는 유일한 주행 기록.

        ⚠ 왜 필요한가 (2026-08-13): 프리즈가 두 번 났는데 **죽는 순간 주행이 어느
           단계였는지를 끝내 확인하지 못했다.** ROS 로그도 journald도 fsync를 하지
           않아 하드 리셋에 꼬리가 통째로 날아갔기 때문이다(1차는 syslog에 우연히
           남은 drive_controller 메시지로 역추적했고, 2차는 아예 못 했다).
           남은 유일한 단서가 "횡이동 단계에서만 죽는다"인데, 그걸 확증하려면
           **어느 상태에서 멈췄는지**가 기록돼야 한다.
        상태 전이 때만 쓰므로 초당 수 줄 수준 — fsync 비용(실측 6~9ms)은 무시 가능.
        """
        if self._trace_f is None:
            return
        try:
            self._trace_f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')},"
                f"{float(open('/proc/uptime').read().split()[0]):.0f},{what}\n")
            self._trace_f.flush()
            os.fsync(self._trace_f.fileno())
        except OSError:
            self._trace_f = None        # 한 번 실패하면 조용히 포기(주행은 계속)

    def _begin(self):
        now = time.time()
        for c in self.judge_t:
            self.judge_t[c] = now           # 첫 판정까지 유예
        self.step_no = 0
        self.detect_fail = 0
        self.prev_pitch = (None, None)
        self.dir_t = now
        self._det_req = self._det_done = False      # 이전 실행의 낡은 결과 폐기
        self._det_result = None
        self._pending = None
        # 결속 이력 — 새 실행이면 지운다(위 파라미터 주석 참조)
        if self.tie_memory_reset and self._tied_global:
            self.get_logger().warn(
                f'  📌 결속 이력 {len(self._tied_global)}점 초기화 (새 실행)')
            self._tied_global = []
        # 횡이동 상태도 리셋 — 방향은 매 실행마다 새로 판정한다
        #   (lateral_fixed_dir 이 있으면 처음부터 그 방향 → 'both' 비교를 건너뛴다)
        self.lat_dir = self.lat_fixed_dir or None
        if self.lat_fixed_dir:
            self._ev('LATERAL', f'↔ 횡이동 방향 고정: {self.lat_fixed_dir} '
                                f'(반대쪽 판정 생략 · 끝단에서는 {self.lat_fixed_dir} 판정으로 정지)')
        self.lat_lane = 0
        self.lat_turns_left = 0
        self.lat_moved_mm = 0.0
        self.last_lateral_mm = None
        self.lat_side = None
        if self.start_dir == 'rev':
            self.get_logger().warn('🟢 시작 → 후진 스텝주행')
            self._set_state('REV_DETECT')
        else:
            self.get_logger().warn('🟢 시작 → 전진 스텝주행')
            self._set_state('FWD_DETECT')

    def _release_motion(self):
        """주행권 반납: navigator_base FSM navigating → auto 복귀 (1회성, 중복 무해)."""
        if self._motion_held:
            self._motion_held = False
            m = String(); m.data = 'NAVIGATION_COMPLETE'
            self.motion_cmd_pub.publish(m)

    def _abort(self, why):
        self._ev('ABORT', f"⛔ ABORT: {why} → 정지")
        self._publish(0.0)
        self._release_motion()
        self._set_state('ABORT')

    def _angular_for(self, fwd):
        """배근 heading 기반 조향 각속도(rad/s). 조향 불가/불필요면 0.

        deck_edge_node가 같은 seg 마스크로 계산해 `/deck_edge_status`에 실어준
        heading_deg(가로철근 기울기, 정렬=0)를 P 제어로 각속도에 매핑한다.
        후진은 후방카메라라 화면 기울기와 로봇 회전의 관계가 반대 → heading_rev_sign.
        """
        if not self.heading_on:
            return 0.0
        hd = self._heading_now()
        if hd is None:
            return 0.0                                   # 근거 부족 → 조향 안 함
        if abs(hd) < self.heading_db:
            return 0.0                                   # 데드밴드
        sign = self.heading_sign * (1.0 if fwd else self.heading_rev_sign)
        # 후진 카메라는 같은 요를 1.9배로 읽는다 → 게인을 그만큼 낮춰 전진과 맞춘다.
        gain = self.heading_kp * (1.0 if fwd else self.heading_rev_gain)
        w = -sign * gain * hd                            # P 제어
        return max(-self.heading_max, min(self.heading_max, w))

    def _heading_now(self):
        """진행방향 카메라의 heading(오프셋 보정 + 중앙값). 근거 부족이면 None.

        창이 다 안 찼으면 조향하지 않는다 — 방향 전환 직후 첫 1~2 샘플은
        아직 반대방향 카메라 값이 섞여 있을 수 있다.
        """
        b = self.heading_buf[self._active_cam()]
        if len(b) < self.heading_med_n:
            return None
        # ⚠ 짝수 개일 때 `sorted(b)[n//2]`는 중앙값이 아니라 **위쪽 값**이다
        #   (n=2면 최댓값). heading은 부호 있는 오차라 그러면 한쪽 방향만
        #   과대평가하는 조향 편향이 된다. 짝수는 가운데 둘의 평균으로 낸다.
        v = sorted(b)
        n = len(v)
        return float(v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2.0)

    def _publish(self, vx, wz=0.0):
        """cmd_vel 발행. **자기가 주행을 소유할 때만** 발행한다.

        ⚠ dry-run이나 IDLE에서도 0을 계속 쏘면 웨이포인트 주행[A]의 cmd_vel을
           덮어써 A가 안 움직인다(/cmd_vel 공유). 그래서 주행 소유 중에만 발행하고,
           정지 직후 zero_hold_sec 동안만 0을 낸 뒤 침묵해 버스를 A에 돌려준다.
        """
        driving = self.state in DRIVING
        if self.dry:
            return
        now = time.time()
        if driving:
            self._zero_until = now + self.zero_hold_sec
            # 주행권 요청 유지 (놓친 메시지 대비 1Hz 재요청. 이미 navigating이면 무해)
            if now - self._motion_req_t > 1.0:
                self._motion_req_t = now
                self._motion_held = True
                m = String(); m.data = 'MOVE_VISION_DRIVE'
                self.motion_cmd_pub.publish(m)
        elif now > self._zero_until:
            return
        t = Twist()
        t.linear.x = float(vx) if driving else 0.0
        t.angular.z = float(wz) if driving else 0.0      # 배근 heading 조향
        self.cmd_pub.publish(t)

    # ---------- 판정 ----------
    def _active_cam(self):
        return 'front' if self.state.startswith('FWD') else 'back'

    def _save_frame(self, tag):
        """검출 시점의 원본 컬러 프레임 저장. 실패는 무시한다."""
        if not self._frame_dir or self.orbbec is None:
            return None
        try:
            buf = getattr(self.orbbec, 'buf', None)
            if not buf:
                return None
            import cv2
            fn = f'{tag}.jpg'
            cv2.imwrite(os.path.join(self._frame_dir, fn), buf[-1])
            return fn
        except Exception as e:
            self.get_logger().error(f'프레임 저장 실패(무시): {e}',
                                    throttle_duration_sec=30.0)
            return None

    def _pose_norm(self, pose):
        """자세별 좌표를 'r' 기준으로 맞추는 보정값.

        ★ 검출 좌표는 교차점의 물리위치가 아니라 **'결속기를 보낼 스테이지 좌표'**다.
          자세가 다르면 같은 교차점도 값이 다르다(CAD 오프셋 차 l-r=(-13.0,+20.6)mm).
          이력을 좌표로 대조하려면 반드시 한 자세 기준으로 통일해야 한다.
        """
        po = getattr(self.orbbec, 'pose_offset', None)
        if not po or pose not in po or 'r' not in po:
            return 0.0, 0.0
        return float(po['r'][0]) - float(po[pose][0]), \
            float(po['r'][1]) - float(po[pose][1])

    def _to_global(self, pose, x, y):
        """검출 좌표 → 전역(주행거리 포함, 'r' 기준). odom 없으면 None."""
        if self.x_ctrl is None:
            return None
        nx, ny = self._pose_norm(pose)
        return self.x_ctrl * 1000.0 + float(x) + nx, float(y) + ny

    def _already_tied(self, pose, x, y):
        """이 자리를 이번 주행에서 이미 결속했는가."""
        if self.tie_memory_mm <= 0 or not self._tied_global:
            return False
        g = self._to_global(pose, x, y)
        if g is None:
            return False            # 위치를 모르면 막지 않는다(결속 누락이 더 나쁘다)
        gx, gy = g
        r2 = self.tie_memory_mm ** 2
        for tx, ty, lane in self._tied_global:
            if lane != self.lat_lane:
                continue            # 레인이 다르면 다른 자리다
            if (tx - gx) ** 2 + (ty - gy) ** 2 <= r2:
                return True
        return False

    def _remember_tied(self):
        """이번에 결속 요청한 점들을 전역좌표로 기억한다."""
        if self.tie_memory_mm <= 0:
            return
        n = 0
        for pose, pts in self.tie_points.items():
            for x, y in pts:
                g = self._to_global(pose, x, y)
                if g is not None:
                    self._tied_global.append((g[0], g[1], self.lat_lane))
                    n += 1
        if n:
            self.get_logger().info(
                f'  📌 결속 이력 {n}점 기록 (누적 {len(self._tied_global)}점)')

    def _record(self, rec):
        """작업도 기록 한 줄 append. **어떤 실패도 주행을 막지 않는다.**"""
        if not self._rec_path:
            return
        try:
            rec['t'] = time.time()
            with open(self._rec_path, 'a') as f:
                f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        except Exception as e:
            self.get_logger().error(f'기록 실패(무시): {e}',
                                    throttle_duration_sec=30.0)

    def _bumper_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._bumper = {k: bool(d.get(k, False))
                        for k in ('forward', 'backward', 'left', 'right')}
        self._bumper['reason'] = str(d.get('reason', ''))
        self._bumper_t = time.time()

    def _bumper_blocking(self, fwd):
        """진행방향이 범퍼로 막혔는가. 막혔으면 사유 문자열, 아니면 None.

        ⚠ 신호가 없거나 낡으면 **막지 않는다**(fail-open). 실제 차단은
          drive_controller가 하고 거기엔 stale 시 마지막 상태 유지가 들어 있다.
          여기서 또 막으면 이중 안전장치가 아니라 이중 오작동이 된다.
        """
        if self._bumper is None:
            return None
        if time.time() - self._bumper_t > self.stale_sec:
            return None
        key = 'forward' if fwd else 'backward'
        if self._bumper.get(key):
            return self._bumper.get('reason') or '범퍼'
        return None

    def _stop_confirmed(self, verdict, ok):
        """STOP이 **연속으로** 확인됐는가. 한 프레임 노이즈로 방향을 바꾸지 않는다.

        판정이 아직 없으면(ok=False) 카운트를 건드리지 않는다 — 영상이 잠깐
        끊긴 것과 배근이 끝난 것은 다르다(끊김은 stale 워치독이 따로 본다).
        """
        if not ok:
            return False
        if verdict == 'STOP':
            self._stop_run += 1
        else:
            self._stop_run = 0
        return self._stop_run >= self.stop_confirm

    def _verdict(self):
        """(verdict, rebar_frac, ok). ok=False면 아직 판정 없음(대기해야 함)."""
        cam = self._active_cam()
        r = self.judge[cam]
        if r is None:
            return 'WAIT', 0.0, False
        return r.get('verdict', 'STOP'), float(r.get('rebar_frac', 0.0)), True

    # ---------- 검출 → 다음 거리 ----------
    def _detect_worker(self):
        """검출 전용 스레드.

        ⚠ YOLO 추론(n_frames×~0.9s)을 제어 타이머에서 직접 돌리면 executor가 통째로
           막혀 `/deck_edge_status` 콜백이 안 돈다. 실측에서 2.64초 블로킹 → 판정이
           낡아 곧바로 staleness ABORT가 났다. 그래서 검출은 여기서만 돌린다.
        """
        while not self._stop:
            if self._det_req:
                self._det_req = False
                self._det_busy = True
                try:
                    self._det_result = self._plan_step()
                except Exception as e:
                    self.get_logger().error(f'  [orbbec] 검출 오류: {e}')
                    self._det_result = None
                self._det_done = True
                self._det_busy = False
            time.sleep(0.05)

    def _plan_step(self):
        """orbbec 교차점 검출 → 다음 주행거리(m). None이면 실패. (워커 스레드에서 호출)"""
        if self.orbbec is None:
            return self.fallback_step_m or None
        try:
            sets = self.orbbec.localize(self.detect_pose, n_frames=self.detect_frames)
        except Exception as e:
            self.get_logger().error(f'  [orbbec] 검출 오류: {e}')
            return None
        pts_xyuv = [c for pose in ('r', 'l') for c in sets.get(pose, [])]
        # ⚠️ 이동거리(격자 피치)는 **검출된 전부**로 계산한다. 결속 상태와 무관하게
        #   교차점은 교차점이므로, untie만 남기면 행이 빠져 피치가 틀어진다.
        pts = [(c[0], c[1]) for c in pts_xyuv]
        if not pts:
            return None
        # 결속 대상 = 도달범위 안 교차점만, **자세 분류를 보존**해서 저장.
        # 이 좌표를 그대로 orchestrator에 넘겨 결속시킨다 → 이동거리 계산의 전제
        # ("이 열들을 결속했다")와 실제 결속 대상이 항상 일치한다.
        # 결속 대상 분류. 두 가지를 여기서 바로잡는다:
        #  ① localize()의 자세 분류는 못 쓴다 — 검출범위를 ±3000으로 넓혀놔서 전부
        #     현재자세로 몰린다(좌측 0점 → 자세변경 없음 → 지그재그 안 됨).
        #  ② ⚠ **좌표를 자세별 오프셋으로 다시 계산해야 한다.** CAD 오프셋이
        #     r=(-50.8,146.8) / l=(-63.8,159.4)로 (13.0,-12.6)mm 다르다. r 좌표를
        #     그대로 l로 재분류하면 그만큼 틀어진 위치에 결속한다.
        #     (pitch는 차분이라 오프셋 무관 → 위 compute_next_move는 그대로 둬도 됨)
        tx0, tx1 = self.tie_x_range
        other = 'l' if self.detect_pose == 'r' else 'r'
        self.tie_points = {'r': [], 'l': []}
        # ⚠️ [2026-08-18] **결속 대상은 클래스로 거른다** — 이미 결속된 점(tie)을
        #   다시 결속하면 와이어 낭비에 결속기 간섭까지 생긴다(이중결속 방지).
        #   단일 클래스 구모델에서는 거를 근거가 없으므로 전부 대상으로 둔다.
        state_aware = getattr(self.orbbec, 'state_aware', False)
        # 도달범위 안 교차점을 **클래스와 무관하게** 먼저 모은다.
        # ⚠ 밀도(tie_every_n) 순번은 **이미 결속된 줄까지 포함해** 세어야 한다.
        #   tie를 먼저 빼고 세면 그 줄이 순번 자리를 비워, 다음 줄이 대신 결속되면서
        #   간격이 불규칙해지고 목표 밀도를 넘긴다(예: 50% 배근에 50% 모드 → 60%).
        reach = []                                       # [(pose, [x,y], cls), ...]
        for c in pts_xyuv:
            u, v = c[2], c[3]
            cls = c[4] if len(c) > 4 else None
            for p in (self.detect_pose, other):          # 겹침구간은 현재자세 우선
                xy = self.orbbec._to_robot(p, u, v)      # p 자세 오프셋으로 변환
                if xy is None:
                    continue
                px, py = xy
                y0, y1 = self.pose_y_range[p]
                if tx0 <= px <= tx1 and y0 <= py <= y1:
                    reach.append((p, [round(px, 1), round(py, 1)], cls))
                    break

        # ── ① 밀도: 도달범위 안 **모든 줄**로 순번을 매겨 남길 줄을 정한다 ──
        keep_reach = reach
        if self.tie_every_n > 1 and reach:
            cols = {}
            for item in reach:
                cols.setdefault(round(item[1][0] / self.col_tol_mm), []).append(item)
            # ⚠ [2026-09-08] 순번은 **절대좌표(odom+local)로 기억**해야 한다.
            #   스텝당 전진은 180~310mm인데 도달범위는 0~345mm라, 같은 가로배근이
            #   두 스텝에 걸쳐 다시 잡힌다. 스텝마다 무조건 +1 하던 옛 코드는 같은
            #   줄에 번호를 두 번 매겨 1/2 밀도가 실제로는 불규칙해졌다.
            #   (실측 2026-09-08: 가로배근 6줄인데 순번이 10까지 올라갔고,
            #    50mm bin 경계에 걸린 한 줄이 271mm→bin5 / 287mm→bin6으로
            #    쪼개져 또 한 번 겹쳐 셌다. 절대좌표 병합이 둘 다 막는다.)
            # ⚠ 정렬은 **주행방향 순**이어야 한다. local x는 자세 기준이라
            #   후진 중에는 큰 x가 앞(다가가는 쪽)이다 — rev면 내림차순.
            base = 0.0 if self.x_ctrl is None else self.x_ctrl * 1000.0
            rev = self.state.startswith('REV')
            keep_reach = []
            n_keep_col = n_skip_col = n_seen_col = 0
            for key in sorted(cols, reverse=rev):
                ax = base + sum(i[1][0] for i in cols[key]) / len(cols[key])
                j = next((i for i, c in enumerate(self._col_abs)
                          if abs(c[0] - ax) <= self.col_merge_mm), None)
                if j is None:                            # 처음 보는 줄
                    self._col_seq += 1
                    hit = self._col_seq
                    self._col_abs.append([ax, hit, 1])
                    if len(self._col_abs) > 40:
                        self._col_abs.pop(0)
                else:
                    # ⚠ 기억 위치를 **누적평균으로 갱신**한다. 한 줄의 검출 x는
                    #   스텝마다 ±20mm 흔들려서, 첫 관측만 붙들고 있으면 나중
                    #   관측이 병합범위 밖으로 밀려나 없는 줄이 새로 생긴다
                    #   (실측: 1202로 고정 → 1308이 106mm 떨어져 신규로 샜다).
                    c = self._col_abs[j]
                    c[2] += 1
                    c[0] += (ax - c[0]) / c[2]
                    hit = c[1]
                    n_seen_col += 1
                if hit % self.tie_every_n == 1:
                    n_keep_col += 1
                    keep_reach.extend(cols[key])
                else:
                    n_skip_col += 1
            self.get_logger().info(
                f'  🔩 결속밀도 1/{self.tie_every_n}: {len(cols)}줄 중 '
                f'{n_keep_col}줄 대상 / {n_skip_col}줄 건너뜀'
                + (f' (재검출 {n_seen_col}줄)' if n_seen_col else '')
                + f'   누적 순번 {self._col_seq}')

        # ── ② 클래스: 남긴 줄 안에서 결속 대상만 고른다 ──
        n_skip_cls = {}
        n_skip_mem = 0
        for p, xy, cls in keep_reach:
            if state_aware and cls is not None and cls not in self.tie_classes:
                n_skip_cls[cls] = n_skip_cls.get(cls, 0) + 1
                continue                                 # 이미 결속됨 → 패스
            # ★ 이력 대조 — 모델이 untie로 오분류해도 이미 묶은 자리는 안 간다.
            if self._already_tied(p, xy[0], xy[1]):
                n_skip_mem += 1
                continue
            self.tie_points[p].append(xy)
        if n_skip_mem:
            self.get_logger().info(
                f'  📌 이력 제외 {n_skip_mem}점 (이번 주행에서 이미 결속한 자리)')
        if n_skip_cls:
            detail = ' '.join(f'{k} {v}개' for k, v in sorted(n_skip_cls.items()))
            self.get_logger().info(
                f'  🔩 결속 제외({detail}) — 대상 클래스 {sorted(self.tie_classes)}')

        r = compute_next_move(pts, prev_pitch=self.prev_pitch,
                              tie_x_range=self.tie_x_range,
                              tie_y_range=self.tie_y_range)
        if 'error' in r or not r.get('next_forward_mm'):
            return None
        self.prev_pitch = (r['pitch_x'], r['pitch_y'])
        # 횡이동 거리도 같은 검출에서 나온다(행 Y좌표 기준). 데크 끝에서는 검출을
        # 다시 하지 않으므로 **마지막 성공 검출값**을 레인 전환에 쓴다.
        if r.get('next_lateral_mm'):
            self.last_lateral_mm = float(r['next_lateral_mm'])
        # 유효 구간도 같이 보관 — 회전수를 이 안에서 최소로 고른다(_enter_lateral)
        self.last_lateral_window = r.get('lateral_window_mm')
        dist_m = float(r['next_forward_mm']) / 1000.0
        if dist_m > self.max_step_m:
            self.get_logger().warn(
                f"  계산거리 {dist_m*1000:.0f}mm > 상한 {self.max_step_m*1000:.0f}mm → 상한으로 제한")
            dist_m = self.max_step_m
        self.get_logger().warn(
            f"  📏 교차점 {len(pts)}개(결속범위내 열{r['n_cols']}×행{r['n_rows']} / "
            f"검출전체 열{r.get('n_cols_all', '?')}) "
            f"pitch=({r['pitch_x']},{r['pitch_y']})mm "
            f"cols_x={r['cols_x']} rows_y={r['rows_y']} "
            f"→ 다음 {r['next_forward_mm']}mm "
            f"(마지막열 {r.get('last_col_x')}+마진 {r.get('clear_margin_mm')}) "
            f"[{r['confidence']} {r['pitch_src']}]")
        self._ev('DETECT',
                 f"📏 교차점 {len(pts)}개 감지 "
                 f"(결속범위 열{r['n_cols']}×행{r['n_rows']}) "
                 f"pitch=({r['pitch_x']},{r['pitch_y']})mm → 다음 {r['next_forward_mm']}mm",
                 log=False)
        im = String()
        im.data = json.dumps({'step': self.step_no, 'dir': self._active_cam(),
                              'n_points': len(pts), **r})
        self.info_pub.publish(im)

        frame_fn = self._save_frame(
            f"s{self.step_no}_{'fwd' if self.state.startswith('FWD') else 'rev'}"
            f"_{self.detect_pose}_{int(time.time() % 100000)}")
        # ★ 작업도 기록 — 검출 **전부**(결속 대상 아닌 것 포함)를 로봇 위치와 함께.
        #   전역 좌표 복원은 오프라인에서: global_x = odom_x_mm + local_x
        #   (전진하면 odom_x는 늘고 같은 교차점의 local_x는 그만큼 줄어 상쇄된다)
        self._record({
            'type': 'detect',
            'step': self.step_no,
            'dir': 'fwd' if self.state.startswith('FWD') else 'rev',
            'pose': self.detect_pose,
            'lane': self.lat_lane,          # 횡이동 레인 번호(0=시작 레인)
            'lane_dir': self.lat_dir,       # 마지막 레인전환 방향(left/right)
            'odom_x_mm': None if self.x_ctrl is None else self.x_ctrl * 1000.0,
            'heading_deg': self._heading_now(),
            'frame': frame_fn,          # 이 검출에 쓰인 원본 프레임 (학습용)
            # [x, y, cls, conf, u, v] — x,y는 stage mm / u,v는 **원본 픽셀좌표**.
            #   u,v가 있어야 나중에 프레임 위에 박스를 다시 그려 라벨로 쓸 수 있다.
            'det': [[round(float(c[0]), 1), round(float(c[1]), 1),
                     (c[4] if len(c) > 4 else None),
                     (round(float(c[5]), 3) if len(c) > 5 else None),
                     round(float(c[2]), 1), round(float(c[3]), 1)]
                    for c in pts_xyuv],
            'tie_req': {k: [[round(float(a), 1), round(float(b), 1)]
                            for a, b in v] for k, v in self.tie_points.items()},
            'pitch': [r.get('pitch_x'), r.get('pitch_y')],
            'next_mm': r.get('next_forward_mm'),
        })
        return dist_m

    # ---------- 제어 루프 ----------
    def _tick(self):
        now = time.time()
        sm = String(); sm.data = self.state
        self.state_pub.publish(sm)

        # 공통 안전 게이트
        if self.estop:
            if self.state not in ('IDLE', 'ABORT', 'DONE'):
                self._abort('리모콘 emergency_stop')
            self._publish(0.0); return
        if self.s24 and self.state not in ('IDLE', 'ABORT', 'DONE'):
            self._abort('S24'); return
        if self.state in BUSY:
            if self._mission_active():
                # A가 주행을 시작하면 B가 즉시 비킨다 (/cmd_vel·/joint_control 충돌 방지)
                self._abort(f'웨이포인트 주행[A] {self.mission_state} 시작 → 주행권 양보')
                return
            if self.require_remote and not self.s20:
                self._abort('S20 해제(auto 이탈)'); return
        if self.state in DRIVING:
            if now - self.dir_t > self.max_sec:
                self._abort(f'max-sec {self.max_sec}s 초과'); return
            cam = self._active_cam()
            if self.judge[cam] is None:
                # 아직 한 번도 못 받음 = 기동 중(디스커버리). 넉넉히 기다렸다 포기.
                if now - self.dir_t > self.start_grace:
                    self._abort(f'{cam} 판정 {self.start_grace}s 동안 수신 없음 '
                                f'(deck_edge_node 실행 중인가?)'); return
            elif now - self.judge_t[cam] > self.stale_sec:
                self._abort(f'{cam} 판정 staleness {self.stale_sec}s (감지 끊김)'); return

        if self.state == 'IDLE':
            self._publish(0.0)
            if self.s23_edge and (self.s20 or not self.require_remote):
                self.s23_edge = False
                if self._mission_active():
                    self.get_logger().error(
                        f"⛔ 시작 거부: 웨이포인트 주행[A]이 {self.mission_state} 중 "
                        f"— /cmd_vel 충돌. UI에서 미션을 중단한 뒤 다시 S23")
                else:
                    self._begin()
            else:
                self.s23_edge = False
            return

        if self.state in ('FWD_DETECT', 'REV_DETECT'):
            self._tick_detect(now); return
        if self.state in STEPPING:
            self._tick_step(now); return
        if self.state in ('FWD_SETTLE', 'REV_SETTLE'):
            self._tick_settle(now); return
        if self.state == 'LATERAL_JUDGE':
            self._tick_lateral_judge(now); return
        if self.state == 'LATERAL_MOVE':
            self._tick_lateral_move(now); return

        # DONE / ABORT — 주행권 반납 (navigating → auto 복귀). idle 강제는 금지:
        # /control_mode는 navigator_base 소유이고, 여기서 쏘면 모드가 뒤집힌다.
        self._publish(0.0)
        self._release_motion()
        if self.s23_edge and (self.s20 or not self.require_remote) \
                and not self.estop and not self.s24:
            self.s23_edge = False
            self.get_logger().warn('🟢 S23 재입력 → 재시작')
            self._begin()
        else:
            self.s23_edge = False

    def _tick_detect(self, now):
        """정지 상태에서 교차점 검출 → 다음 스텝 거리 결정."""
        self._publish(0.0)
        fwd = self.state.startswith('FWD')
        dm = String(); dm.data = 'forward' if fwd else 'backward'
        self.dir_pub.publish(dm)

        # ★ 결속 대기는 **데크끝 판정보다 먼저** 본다. 순서를 바꾸면 안 된다.
        #   (2026-09-15 실주행 사고) 이 블록이 아래에 있던 탓에, 결속 대기 중
        #   데크끝 STOP이 확정되자 대기를 버리고 SETTLE→횡이동으로 빠져나갔다.
        #   그 결과 **상부가 결속 시퀀스를 수행하는 동안 하부가 횡이동**했다:
        #       602.4 하부 FWD_DETECT→FWD_SETTLE (배근 끝)
        #       609.1 상부 X 202mm 뻗음
        #       610.6 하부 횡이동 [1/6회] 시작      ← 기체를 들어 옆으로
        #       612.0 상부 Z하강 712°               ← 횡이동 중에 내림
        #       613.6 상부 트리거 발사
        #   기구 충돌 위험 + 결속 좌표가 이동 전 기준이라 엉뚱한 자리에 쏜다.
        #   데크끝은 도망가지 않는다 — 결속이 끝난 뒤 판정해도 늦지 않다.
        # 검출 끝 → 결속(실제 또는 가정) → 그 다음 이동
        if self._pending is not None:
            dist, until = self._pending
            if self.do_tying:
                if self.tie_done:
                    self._ev('TIE', f'🔩 결속 완료: {self.tie_tied}점')
                    self._record({'type': 'tie_done', 'step': self.step_no,
                                  'n': self.tie_tied,
                                  'points': self.tie_points})
                    self._pending = None
                    self._start_step(dist, now)
                elif now >= until:
                    self._abort(f'결속 응답 없음 {self.tie_timeout:.0f}s '
                                f'(tying_orchestrator 상태/호밍 확인)')
                else:
                    self.get_logger().info(
                        f'  🔩 결속 중… ({until - now:.0f}s 내 완료 대기) '
                        f'→ 이후 {dist*1000:.0f}mm 이동', throttle_duration_sec=2.0)
                return
            if now < until:
                self.get_logger().info(
                    f'  🔩 결속 중(가정) {until - now:.1f}s 남음 → 이후 {dist*1000:.0f}mm 이동',
                    throttle_duration_sec=1.0)
                return
            self._pending = None
            self._start_step(dist, now)
            return

        # 배근 끝이면 검출할 것도 없이 방향 전환
        verdict, frac, ok = self._verdict()
        if not ok:
            self.get_logger().info(f'[{self.state}] 첫 판정 대기 중…',
                                   throttle_duration_sec=0.5)
            return
        if self._stop_confirmed(verdict, ok):
            self.get_logger().warn(
                f'🛑 {self._active_cam()} 배근 끝 (rebar_frac={frac:.2f}, '
                f'STOP {self._stop_run}연속) → 정지')
            self._settle_resume = 'FWD_DETECT' if fwd else 'REV_DETECT'
            self._set_state('FWD_SETTLE' if fwd else 'REV_SETTLE')
            return
        if verdict == 'STOP':                          # 아직 확정 전 = 대기
            self.get_logger().info(
                f'[{self.state}] STOP {self._stop_run}/{self.stop_confirm} 확인 중…',
                throttle_duration_sec=0.5)
            return

        if now - self.state_t < self.step_pause:       # 정지 안정화 후 검출
            return

        # 검출은 워커 스레드가 수행 (제어 루프를 막지 않게)
        if not self._det_done:
            if not self._det_busy and not self._det_req:
                self._det_req = True
                self._det_t = now
                self._ev('DETECT', '🔍 교차점 검출 중…')
            elif self._det_busy and now - self._det_t > self.detect_timeout:
                self._abort(f'교차점 검출 타임아웃 {self.detect_timeout}s')
            return
        self._det_done = False
        dist = self._det_result
        if dist is None:
            self.detect_fail += 1
            if self.detect_fail <= self.detect_retry:
                self._ev('WARN',
                          f'⚠ 교차점 검출 실패 {self.detect_fail}/{self.detect_retry} → 재시도')
                self.state_t = now                     # step_pause 만큼 대기 후 재시도
                return
            self._abort(f'교차점 검출 {self.detect_retry}회 실패 '
                        f'(fallback_step_mm 설정 시 그 거리로 진행 가능)')
            return

        self.detect_fail = 0
        if self.x_ctrl is None:
            self.get_logger().warn('  ⚠ /encoder_odom 수신 없음 → 거리제어 불가',
                                   throttle_duration_sec=2.0)
            return
        if self.do_tying:
            n = len(self.tie_points['r']) + len(self.tie_points['l'])
            if n == 0:
                self._ev('TIE', '🔩 결속 대상 없음 (도달범위 내 0점) → 결속 생략')
                self._start_step(dist, now)
                return
            self._request_tying()
            self._pending = (dist, now + self.tie_timeout)
            return
        if self.tie_dwell > 0:
            self._pending = (dist, now + self.tie_dwell)   # 결속 가정 대기 후 이동
            self._ev('TIE', f'🔩 결속 가정 대기 {self.tie_dwell:.0f}s')
            return
        self._start_step(dist, now)

    def _request_tying(self):
        """검출한 교차점을 tying_orchestrator에 넘겨 결속 요청.

        `/mission/command` TYING_START에 `points`를 실으면 orchestrator가
        **검출을 건너뛰고 그 좌표로** 결속한다(자세별 지그재그 시퀀스는 그대로).
        자세 분류를 보존하려고 dict 형태로 넘긴다.
        """
        self.tie_done = False
        self.tie_tied = 0
        cmd = {
            'command': 'TYING_START',
            'speed': self.tie_speed_pct,
            'direction': 'forward',
            'points': self.tie_points,        # {'r': [[x,y],...], 'l': [...]}
        }
        m = String(); m.data = json.dumps(cmd)
        self.mission_cmd_pub.publish(m)
        self._remember_tied()
        # 좌표까지 남긴다 — 재부팅으로 orchestrator 저널이 날아가도 "어느 점을 결속했나"를
        # 이 로그만으로 재구성할 수 있어야 중복결속/누락 판정이 된다.
        self._ev('TIE',
                 f"🔩 결속 시작: 우{len(self.tie_points['r'])} "
                 f"좌{len(self.tie_points['l'])} = "
                 f"{len(self.tie_points['r']) + len(self.tie_points['l'])}점",
                 log=False)
        self.get_logger().warn(
            f"  🔩 결속 요청: 우{len(self.tie_points['r'])} "
            f"좌{len(self.tie_points['l'])}점 → TYING_COMPLETE 대기\n"
            f"      r={self.tie_points['r']}\n"
            f"      l={self.tie_points['l']}")

    def _start_step(self, dist, now):
        """검출·결속(가정)이 끝난 뒤 실제 이동 시작."""
        fwd = self.state.startswith('FWD')
        self.step_no += 1
        self.step_start_x = self.x_ctrl
        self.step_target_m = dist
        self._ev('STEP',
                 f"▶ 스텝 {self.step_no}: {'전진' if fwd else '후진'} {dist*1000:.0f}mm 시작")
        self._blocked_total = 0.0          # 스텝마다 차단 누적 초기화
        self._blocked_since = None
        self._ev_verdict = None            # 스텝마다 감속/복귀 알림 다시 켠다
        self._set_state('FWD_STEP' if fwd else 'REV_STEP')

    def _tick_step(self, now):
        """엔코더 폐루프로 목표거리만큼 이동. 주행 중 배근 끝 감시."""
        fwd = self.state.startswith('FWD')
        sign = 1.0 if fwd else -1.0

        verdict, frac, ok = self._verdict()
        if ok and verdict == 'STOP':
            # 바퀴는 **첫 프레임에 바로** 세운다 — 확정을 기다리며 계속 굴리면
            # 진짜 데크끝일 때 그만큼 더 나간다. 상태 전이만 확정 후에 한다.
            self._publish(0.0)
            if self._stop_confirmed(verdict, ok):
                self._ev('STOP',
                          f'🛑 주행불가 임계감지 [정지] — 배근 끝 '
                          f'(rebar_frac={frac:.2f}, STOP {self._stop_run}연속) → 스텝 중단')
                self._settle_resume = 'FWD_STEP' if fwd else 'REV_STEP'
                self._set_state('FWD_SETTLE' if fwd else 'REV_SETTLE')
            else:
                self.get_logger().info(
                    f'[{self.state}] STOP {self._stop_run}/{self.stop_confirm} '
                    f'확인 중… (정지 상태로 대기)', throttle_duration_sec=0.5)
            return
        self._stop_confirmed(verdict, ok)              # GO/SLOW면 카운트 리셋
        # 차단된 시간은 '못 간' 게 아니라 '안 간' 것이므로 타임아웃에서 뺀다.
        # ⚠ **진행 중인 차단도 포함**해야 한다 — 누적(_blocked_total)은 해제 시점에만
        #   갱신되므로, 범퍼를 계속 누르고 있으면 누적이 0인 채로 타임아웃에 걸린다.
        blocked = self._blocked_total + (
            0.0 if self._blocked_since is None else now - self._blocked_since)
        if now - self.state_t - blocked > self.max_step_sec:
            self._abort(f'스텝 시간 초과 {self.max_step_sec}s '
                        f'(범퍼 차단 {blocked:.0f}s 제외)'); return
        if self.x_ctrl is None:
            self._abort('/encoder_odom 끊김'); return

        moved = (self.x_ctrl - self.step_start_x) * sign     # 진행 방향 이동량(m)
        remain = self.step_target_m - moved
        if remain <= self.tol_m:
            self._ev('STEP',
                      f"✅ 스텝 {self.step_no} 완료: {moved*1000:.0f}mm "
                      f"(목표 {self.step_target_m*1000:.0f}mm)")
            self._publish(0.0)
            self._set_state('FWD_DETECT' if fwd else 'REV_DETECT')
            return

        # ★ 범퍼 차단 — 실제 게이팅은 drive_controller가 하지만, 여기서도 0을 내
        #   보내야 로그의 vx가 실제와 맞는다(안 그러면 멈춰 있는데 vx=0.400이 찍힌다).
        blk = self._bumper_blocking(fwd)
        if blk:
            if self._blocked_since is None:
                self._blocked_since = now
                self._ev('BUMPER', f'🛑 범퍼 차단 → 일시정지 ({blk})')
            self._publish(0.0)
            self.get_logger().info(
                f'[{self.state}] 스텝{self.step_no} {moved*1000:6.0f}/'
                f'{self.step_target_m*1000:.0f}mm **범퍼 일시정지** '
                f'{now - self._blocked_since:.1f}s ({blk})',
                throttle_duration_sec=1.0)
            return
        if self._blocked_since is not None:
            held = now - self._blocked_since
            self._blocked_total += held
            self._blocked_since = None
            self._ev('BUMPER', f'✅ 범퍼 해제 → 주행 재개 (정지 {held:.1f}s)')

        v = self.speed
        if ok and verdict == 'SLOW':
            v *= self.slow_scale
        # UI 이벤트는 **전이할 때만** — 매 틱 내보내면 10Hz 폴링 UI가 같은 줄로 덮인다.
        if ok and verdict != self._ev_verdict:
            if verdict == 'SLOW':
                self._ev('SLOW', f'⚠ 주행불가 감지 [감속] (rebar_frac={frac:.2f}) '
                                 f'→ {self.slow_scale*100:.0f}% 속도')
            elif verdict == 'GO' and self._ev_verdict == 'SLOW':
                self._ev('GO', f'✅ 주행가능 복귀 [정상속도] (rebar_frac={frac:.2f})')
            self._ev_verdict = verdict
        if remain < self.approach_m:                        # 목표 근접 감속
            v *= self.approach_scale
        wz = self._angular_for(fwd)
        self._publish(sign * v, wz)
        hd = self._heading_now()
        self.get_logger().info(
            f"[{self.state}] 스텝{self.step_no} {moved*1000:6.0f}/"
            f"{self.step_target_m*1000:.0f}mm {verdict} "
            f"vx={0.0 if self.dry else sign*v:+.3f}"
            f"{'' if hd is None else f' head={hd:+.1f}° wz={wz:+.3f}'}"
            f"{' (dry)' if self.dry else ''}",
            throttle_duration_sec=0.5)

    def _tick_settle(self, now):
        self._publish(0.0)
        if now - self.state_t < self.settle:
            return

        # ★ 전환 커밋 직전 재확인 — **이게 마지막 되돌릴 지점**이다.
        #   여기를 넘으면 후진/횡이동이 확정되고 취소할 방법이 없다.
        #   예전에는 settle 시간만 세고 판정을 다시 보지 않아서, 순간 STOP 하나로
        #   아직 갈 수 있는 배근인데 레인을 넘어가는 일이 가능했다.
        #   정지해 있는 동안 판정이 GO로 돌아왔다면 그건 노이즈였다는 뜻이다.
        if self.settle_recheck:
            verdict, frac, ok = self._verdict()
            if ok and verdict != 'STOP':
                back = self._settle_resume or (
                    'FWD_DETECT' if self.state == 'FWD_SETTLE' else 'REV_DETECT')
                self.get_logger().warn(
                    f'↩️ 전환 취소: 정지 중 판정이 {verdict}로 복귀 '
                    f'(rebar_frac={frac:.2f}) → {back} 재개')
                self._stop_run = 0
                self._settle_resume = None
                self._set_state(back)
                return

        self._settle_resume = None
        if self.state == 'FWD_SETTLE':
            # 'ㄹ'자 커버리지: 데크 끝에 닿았으니 옆 레인으로 넘어간 뒤 후진한다.
            # (횡이동을 끄면 예전대로 그 자리에서 바로 후진)
            if self._lateral_possible():
                self._enter_lateral('REV_DETECT', now)
                return
            if self.one_way:
                self.get_logger().warn(
                    '✅ 전진 종료 (one_way) — 되돌아오지 않는다. '
                    '다음 라인으로 옮긴 뒤 `start:=rev` 로 재실행할 것')
                self._set_state('DONE')
                return
            self._start_reverse(now)
        else:
            if self._lateral_possible():
                self._enter_lateral('FWD_DETECT', now)
                return
            if self.one_way:
                self.get_logger().warn(
                    '✅ 후진 종료 (one_way) — 다음 라인으로 옮긴 뒤 '
                    '`start:=fwd` 로 재실행할 것')
            else:
                self._ev('DONE', '✅ 전·후진 시퀀스 완료')
            self._set_state('DONE')

    def _start_reverse(self, now):
        self._ev('DIR', '🔄 후진 스텝주행 시작')
        self.step_no = 0
        self.detect_fail = 0
        self.dir_t = now
        # ⚠ 방향을 바꾸면 **새 방향의 옛 판정은 버린다.** deck_edge가 진행방향만
        #   추론(active_only)하면 그 판정은 방향 전환 전 것이라 오래됐고, 그대로
        #   두면 staleness 워치독에 즉시 걸린다. None으로 비워 '첫 판정 대기'로
        #   들어가게 하면 새 판정이 올 때까지 움직이지 않는다.
        self.judge['back'] = None
        self.judge_t['back'] = now
        self.heading_buf['back'].clear()    # 이 카메라가 마지막으로 활성이던 때의 옛 값
        self._set_state('REV_DETECT')

    def _start_forward(self, now):
        self._ev('DIR', '🔄 전진 스텝주행 시작')
        self.step_no = 0
        self.detect_fail = 0
        self.dir_t = now
        self.judge['front'] = None
        self.judge_t['front'] = now
        self.heading_buf['front'].clear()
        self._set_state('FWD_DETECT')

    # ---------- 횡이동 ('ㄹ'자 커버리지) ----------
    def _lateral_possible(self):
        if not self.lateral_on:
            return False
        if self.lat_lane >= self.lat_max_lanes:
            self._ev('DONE', f'↔ 레인 전환 상한 {self.lat_max_lanes}회 도달 → 종료')
            return False
        if not self.last_lateral_mm:
            self.get_logger().warn('↔ 횡이동 거리 미측정(교차점 행 정보 없음) → 종료')
            return False
        return True

    def _enter_lateral(self, next_state, now):
        """레인 전환 시작. 목표거리를 70mm 회전수로 쪼갠다."""
        self.lat_next = next_state
        self.lat_target_mm = self.last_lateral_mm
        self.lat_moved_mm = 0.0
        # ★ 회전수는 **목표거리(결속한 행 수 × 피치)에 가장 가까운 값**으로 고르되,
        #   유효구간 [lo, hi] 안으로 자른다 (2026-09-15).
        #
        #   유효구간은 "다음 미결속 행이 결속범위에 들어오는" **제약**이지 목표가 아니다.
        #   2026-09-02엔 구간 안에서 **최소** 회전을 골라 회전 1회를 아끼려 했는데,
        #   그게 lo가 우연히 작게 나올 때 무너진다:
        #     실측(2026-09-15) 구간 19~307mm, 목표 197mm → ceil(19/70)=1 → **70mm만 이동**.
        #     같은 주행의 다른 전환은 목표 192mm에 3회전(210mm)이었다. 간격이 일정한데
        #     전환마다 6/3/1/5회로 널뛰었다.
        #   당시에도 `lo == 0`에만 예외를 뒀는데, lo가 0이 아니라 **작기만 해도** 같은 일이
        #   난다 — 예외를 조건이 아니라 규칙으로 올린다.
        #
        #   ⚠ 회전을 아끼는 건 실익이 없다: 데크를 덮는 **총 횡이동 거리는 정해져 있고
        #     1회전은 항상 70mm**다. 지금 덜 가면 나중에 그만큼 더 간다. 회전 총량은
        #     그대로인데 **레인 전환 횟수와 전·후진 패스만 늘어난다**(전환마다 측면
        #     재판정 5~7초 + 왕복 주행 1회).
        #   구간 상한으로 자르므로 **행을 건너뛸 위험은 없다**(t_hi 이하로만 간다).
        win = self.last_lateral_window
        turns = None
        if win and len(win) == 2 and win[1] > 0:
            lo, hi = float(win[0]), float(win[1])
            t_lo = max(1, int(math.ceil(lo / self.lat_mm_turn)))   # 미달하면 행이 범위 밖
            t_hi = int(math.floor(hi / self.lat_mm_turn))          # 초과하면 지나쳐 건너뜀
            # 목표는 **반올림**한다. 올림이면 항상 목표보다 더 가 다음 전환이 짧아진다.
            t_tg = max(1, int(round(self.lat_target_mm / self.lat_mm_turn)))
            if t_hi >= t_lo:
                turns = min(max(t_tg, t_lo), t_hi)
                self.get_logger().info(
                    f'↔ 유효구간 {lo:.0f}~{hi:.0f}mm(={t_lo}~{t_hi}회전) · '
                    f'목표 {self.lat_target_mm:.0f}mm(={t_tg}회전) → '
                    f'{turns}회전({turns*self.lat_mm_turn:.0f}mm) 선택')
            else:
                # 구간이 1회전보다 좁다 — 어느 회전수도 구간에 안 들어온다.
                # 덜 가면 결속을 못 하므로 **하한 쪽**을 택한다(초과가 미달보다 낫다).
                turns = t_lo
                self.get_logger().warn(
                    f'↔ 유효구간 {lo:.0f}~{hi:.0f}mm이 1회전({self.lat_mm_turn:.0f}mm)보다 '
                    f'좁다 → 하한 {t_lo}회전({t_lo*self.lat_mm_turn:.0f}mm) 선택')
        if turns is None:                            # 구간을 못 구하면 목표 반올림
            turns = max(1, int(round(self.lat_target_mm / self.lat_mm_turn)))
            self.get_logger().info(
                f'↔ 유효구간 없음 → 목표 {self.lat_target_mm:.0f}mm 기준 {turns}회전')
        self.lat_turns_left = max(1, min(turns, self.lat_max_turns))
        if turns > self.lat_max_turns:
            self.get_logger().warn(
                f'↔ 필요 {turns}회전 > 상한 {self.lat_max_turns} → 상한으로 제한')
        self._ev('LATERAL',
                 f'↔ 레인 전환 {self.lat_lane + 1}: 목표 {self.lat_target_mm:.0f}mm '
                 f'→ {self.lat_turns_left}회 × {self.lat_mm_turn:.0f}mm '
                 f'= {self.lat_turns_left*self.lat_mm_turn:.0f}mm (1회마다 측면 재판정)')
        # 주행권 반납 — /control_mode가 'navigating'이면 joint_controller가
        # /joint_control을 무시한다('auto'에서만 수락). 횡이동 중엔 주행도 안 하므로
        # 여기서 돌려주는 게 맞다.
        self._release_motion()
        self.lat_side = None
        self._set_state('LATERAL_JUDGE')

    def _tick_lateral_judge(self, now):
        """이번 1회전(70mm)을 가도 되는지 측면 카메라로 판정."""
        self._publish(0.0)
        if self.lat_turns_left <= 0:                 # 목표 회전 다 함 → 주행 재개
            self._finish_lateral(now, ok=True)
            return
        if now - self.state_t < self.lat_settle:      # 회전 직후 영상 안정화
            return
        if self.lat_req_t < self.state_t:             # 이 상태에서 아직 요청 안 함
            self.lat_req_t = now
            self.lat_side = None
            side = 'both' if self.lat_dir is None else self.lat_dir
            m = String(); m.data = side
            self.side_req_pub.publish(m)
            self.get_logger().info(f'  🔍 측면 판정 요청({side})…')
            return
        if self.lat_side is None:
            if now - self.lat_req_t > self.lat_side_timeout:
                self._abort(f'측면 판정 응답 없음 {self.lat_side_timeout:.0f}s '
                            f'(deck_edge_node / 측면 카메라 확인)')
            return

        # 첫 전환에서만 좌우를 비교해 방향을 정하고, 이후 레인은 그 방향을 유지한다
        # (방향이 매번 바뀌면 갔던 레인으로 되돌아가 같은 곳을 다시 결속한다).
        if self.lat_dir is None:
            self.lat_dir = self._pick_side()
            if self.lat_dir is None:
                self._ev('DONE', '↔ 좌·우 모두 주행 불가 → 커버리지 종료')
                self._set_state('DONE'); return
            self._ev('LATERAL', f'↔ 횡이동 방향 결정: {self.lat_dir} (이후 고정)')

        v = (self.lat_side or {}).get(self.lat_dir) or {}
        if not v.get('ok'):
            # 목표 도중이라도 배근이 끊기면 더 못 간다 — 여기서 멈추는 게 안전하다.
            self.get_logger().warn(
                f"🛑 {self.lat_dir} 주행 불가 (rebar_frac={v.get('rebar_frac')}) "
                f"→ 횡이동 중단 ({self.lat_moved_mm:.0f}/{self.lat_target_mm:.0f}mm)")
            self._finish_lateral(now, ok=False)
            return
        self._send_lateral_turn(now)

    def _pick_side(self):
        """좌·우 중 주행 가능한 쪽. 둘 다 가능하면 배근이 더 많이 남은 쪽."""
        s = self.lat_side or {}
        cand = [k for k in ('left', 'right') if (s.get(k) or {}).get('ok')]
        if not cand:
            return None
        return max(cand, key=lambda k: s[k].get('rebar_frac', 0.0))

    def _send_lateral_turn(self, now):
        """0x143 1회전 지령 (rebar_controller와 동일 경로·규약)."""
        # ★ 부호 **실측 확정 (2026-08-13)**: `left`를 지령했더니 로봇이 **우측으로** 갔다.
        #   → 좌측 = **-360°**. (그 전에는 rebar_publisher 주석 `S17 = +Y (좌측)`을 근거로
        #     +360°=좌측이라 가정했는데 실기와 반대였다.)
        #
        #   ⚠ 이 버그가 왜 치명적이었나: 단순히 반대로 가는 게 아니라, **재판정이 보는
        #      쪽과 실제 가는 쪽이 반대**가 된다. `left`가 계속 "가능"으로 나오는 동안
        #      물리적으로는 주행 불가 판정된 우측으로 끝까지 밀고 간다 = 데크 이탈 경로.
        #      1회전마다 재판정하는 안전장치가 통째로 무력화된다.
        #
        #   ⚠ `lateral_sign` 파라미터로 뒤집을 수 있게 해뒀다. 기구를 손보면 재확인할 것:
        #      "좌측 지령 → 로봇이 좌측으로 가는가"를 눈으로 1회전 확인.
        sign = self.lat_sign * (1.0 if self.lat_dir == 'left' else -1.0)
        self.lat_move_done = False
        if not self.dry:
            m = JointControl()
            m.joint_id = 0x143
            m.position = 360.0 * sign
            m.velocity = self.lat_speed
            m.control_mode = JointControl.MODE_RELATIVE
            self.joint_pub.publish(m)
        _i = int(self.lat_moved_mm / max(self.lat_mm_turn, 1e-6)) + 1
        _n = _i + self.lat_turns_left - 1
        self._ev('LATERAL',
                 f"↔ 횡이동 시작 [{_i}/{_n}회] {self.lat_dir} "
                 f"{self.lat_mm_turn:.0f}mm{' (dry)' if self.dry else ''}")
        self._set_state('LATERAL_MOVE')

    def _tick_lateral_move(self, now):
        """1회전 완료 대기 → 완료되면 다시 판정으로 (한 번에 몰아 돌리지 않는다)."""
        self._publish(0.0)
        if self.dry:
            if now - self.state_t < 1.0:
                return
            self.lat_move_done = True
        if self.lat_move_done:
            self.lat_turns_left -= 1
            self.lat_moved_mm += self.lat_mm_turn
            self._ev('LATERAL',
                     f"✅ 횡이동 1회 완료 "
                     f"({self.lat_moved_mm:.0f}/{self.lat_target_mm:.0f}mm, "
                     f"남은 {self.lat_turns_left}회)")
            self._set_state('LATERAL_JUDGE')
            return
        if now - self.state_t > self.lat_turn_timeout:
            self._abort(f'횡이동 완료신호 없음 {self.lat_turn_timeout:.0f}s '
                        f'(0x143 / control_mode=auto 확인)')

    def _finish_lateral(self, now, ok):
        """레인 전환 종료 → 반대 방향 주행 재개. 한 칸도 못 갔으면 커버리지 종료."""
        self.lat_lane += 1
        if self.lat_moved_mm <= 0.0:
            self._ev('DONE', '✅ 횡이동 불가 → 커버리지 종료')
            self._set_state('DONE'); return
        self.get_logger().warn(
            f"↔ 레인 전환 완료: {self.lat_dir} {self.lat_moved_mm:.0f}mm "
            f"({'목표 도달' if ok else '중도 정지'}) → {self.lat_next}")
        if self.lat_next == 'REV_DETECT':
            self._start_reverse(now)
        else:
            self._start_forward(now)


def main(args=None):
    rclpy.init(args=args)
    node = RebarDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop = True
            node.cmd_pub.publish(Twist())
            node._release_motion()          # 주행권 반납 (control_mode는 건드리지 않음)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
