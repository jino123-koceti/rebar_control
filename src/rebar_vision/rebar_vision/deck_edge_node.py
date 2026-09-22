#!/usr/bin/env python3
"""데크끝(주행불가) 감지 노드 — 철근배근 기반 주행가능 판정.

⚠ 주행규칙: 로봇 무한궤도는 **철근 배근 위를 밟고** 주행 → **주행가능 = 배근이 이어진 곳**뿐.
   방수포(background)·floor·wall·obstacle 은 전부 주행불가. 판정 로직은 rebar_vision/deck_edge.py.

역할: seg 추론 1회/프레임으로 전·후방 주행가능 여부를 판정해 두 가지를 발행한다.
  1) `/deck_edge_status` (String JSON) — 판정 상세. 모니터링·UI·상위 주행노드용.
  2) `/deck_edge_block`  (String JSON) — **방향별 차단 신호**. drive_controller가 받아
     "데크 밖으로 나가는 방향"의 cmd_vel만 0으로 만든다.

이 노드는 미션 흐름(navigator/rebar_controller)과 **대화하지 않는다**.
obstacle_detector와 같은 위치의 하부 안전망이다. 이유:
  · navigator는 경로를 일괄발행하고 빠지는 구조라 주행 중 실시간 개입 경로가 없다
  · rebar_controller에 물리면 결속/횡이동 상태머신과 인덱스 상태가 꼬인다
  · drive_controller는 모터 직전이라 무엇이 명령하든 데크 이탈을 막을 수 있다
⚠ `/obstacle_pause`를 재사용하지 않는 이유: 그건 control_mode를 통째로 래치해 **전 방향**을
   정지시킨다. 데크끝에 쓰면 멈춘 뒤 후진 탈출까지 막혀 로봇이 갇힌다.

카메라 운용: 전·후방을 **교대로** 추론한다(각 seg_hz/2). 진행방향만 보면 반대방향 판정이
  stale이 되어 '탈출 방향'까지 차단되기 때문. `/travel_direction`은 진행방향 카메라의
  추론 비중을 높이는 데만 쓴다.

토픽:
  구독: /zedxmini1|zedxmini2 .../rgb/color/rect/image/compressed, /travel_direction
        ⚠ 토픽 경로는 zed-ros2-wrapper **v5.1에서 바뀌었다**(구 v4.2: rgb/image_rect_color).
        ([2026-08-06] 주행 카메라 교체: zed_front/zed_back(ZED X) → zedxmini2(전방)/zedxmini1(후방))
  발행: /deck_edge_status (String JSON), /deck_edge_block (String JSON)
"""
import os
import json
import signal
import subprocess
import time
import threading

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from rebar_vision.rebar_seg import RebarSegmenter
from rebar_vision import deck_edge as de
from rebar_vision import rebar_grid as rg

# ★ [2026-09-14] 9클래스 모델로 교체. 추가된 클래스:
#     level_rod — 노란 수직봉. 전엔 색 기반으로만 잡았고 **좌측 카메라에선 불가**했다
#                 (녹슨 철근이 봉보다 노랗게 나옴). 이제 색 기반과 **OR** 로 쓴다.
#     plate     — 데크 가장자리 H빔 형상 철판. **주행불가**로 분류(NONDECK).
#   ⚠ 클래스가 늘면서 인덱스가 밀렸다(rebar_h 4→6, rebar_v 5→7). 인덱스는
#     `de.set_class_map()` / `rg.set_class_map()` 이 **이름으로 재계산**한다 —
#     seg 로드 직후의 그 호출이 빠지면 주행판정이 통째로 뒤집힌다.
#   ⚠ 학습 데이터가 아직 적어 정확도가 낮을 수 있다. 구 모델로 되돌리려면
#     `weights:=~/ros2_ws/src/rebar_vision/model/retrain_best_260804.pt` (7클래스,
#     클래스맵도 자동으로 7클래스로 잡힌다).
DEFAULT_WEIGHTS = os.path.join(
    os.path.expanduser('~'), 'ros2_ws',
    'src/rebar_vision/model/rebar_seg_260914.pt')


class DeckEdgeNode(Node):

    def __init__(self):
        super().__init__('deck_edge_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('weights', DEFAULT_WEIGHTS)
        self.declare_parameter('imgsz', 512)
        self.declare_parameter('device', '')          # '' = auto(cuda 있으면 cuda)
        # 전체 추론 주기(Hz). active_only면 이 전부가 진행방향 카메라에 쓰인다.
        # 8.0 → 4.0으로 낮춤(2026-08-06, GPU 부하 감축·프리즈 완화). 속도 58mm/s에서
        # 판정 간 이동은 약 15mm라 배근 끝 감지 여유는 충분하다.
        self.declare_parameter('seg_hz', 4.0)
        self.declare_parameter('publish_hz', 10.0)    # block 신호 발행 주기
        self.declare_parameter('stop_thr', de.STOP_THR)
        self.declare_parameter('slow_thr', de.SLOW_THR)
        self.declare_parameter('obs_thr', de.OBS_THR)
        # 장애물 **해제** 임계·연속프레임 (진입 obs_thr보다 낮게 = 슈미트 트리거).
        # 기둥을 지나칠 때 obs가 진입임계에 걸터앉아 같은 초에 GO→STOP→GO로
        # 뒤집히던 걸 막는다(2026-08-19 실측). 정지 반응성은 건드리지 않는다.
        # ★ 레벨봉(배근에 꽂는 노란 수직봉) 감지 → 진행방향 차단 (2026-09-02).
        #   현장에 레벨봉이 설치돼 있는데 **세그 모델이 이걸 못 본다**(노란 픽셀의
        #   96.7%를 background로 분류). 그래서 색으로 따로 찾아 차단에 반영한다.
        #   같은 마스크를 재사용하므로 추론 비용은 0이다.
        self.declare_parameter('level_rod_enabled', True)
        # 가장 가까운 봉의 **밑동 행 / 화면높이**가 이 값을 넘으면 차단.
        #   실측(2026-09-02, 960x600 전방): 사용자가 "이 거리에서 멈췄으면" 한 위치가
        #   y_bot=414 → 0.69. 더 붙으면 536(0.89)까지 간다.
        #   ⚠ 정지거리(검출 4Hz + 감속)를 감안해 조금 앞에서 걸리게 0.66으로 둔다.
        #   ⚠ 결속기 간섭은 고려 불필요 — 작업공간이 전/후방 카메라 **사이**에 있어
        #      카메라가 봉을 보고 멈추면 결속기는 이미 봉에서 떨어져 있다(사용자 확인).
        # 2026-09-02: 0.66 → 0.55 (사용자 요청, 더 일찍 정지).
        #   실측 대응: 0.415/0.478=주행가능 · 0.692="여기서 멈춰라" · 0.893=너무 가까움
        self.declare_parameter('level_rod_stop_frac', 0.55)
        # ★ seg `level_rod` 소스만 따로 끄는 스위치 (2026-09-15).
        #   색 기반과 OR이라 **한쪽만 틀려도 즉시정지**가 걸린다. 실제로 헤딩 영점
        #   시험이 3번 연속 이것 때문에 중단됐다 — 3번 다 `색0.00/seg0.66~0.80`,
        #   즉 색 기반은 아무것도 안 보는데 seg만 혼자 봤다. `level_rod`는 2026-09-14에
        #   추가된 클래스로 학습데이터가 적다.
        #   전·후방은 색 기반만으로도 원래 동작했다(seg를 넣은 이유는 **좌측 측면**에서
        #   녹슨 철근이 노랗게 보여 색 기반을 못 쓰기 때문). 그래서 이걸 꺼도
        #   전·후방 보호막은 남는다.
        # ⚠ 기본값 **False** (2026-09-16). 전·후방에서 seg `level_rod`는 종일 오검출만 냈다:
        #   색 기반 0.00 인데 seg 0.65~0.71 이 **지속적으로** 뜬다(단발이 아니라 누적필터도
        #   3/3·4/3 으로 통과). 실체는 철근 받침(스페이서)·그림자로, 레벨봉처럼 바닥에
        #   수직으로 선 구조물이라 기하로는 못 가른다. 그 탓에 [A]/[B] 주행이 1~2초마다
        #   끊겼다(`🛑 데크끝 차단: 전진` ↔ `✅ 해제` 반복).
        #   전·후방은 **색 기반만으로 원래 동작했다** — seg 를 넣은 이유는 좌측 측면에서
        #   녹슨 철근이 노랗게 보여 색을 못 쓰기 때문이었다.
        #   ⟹ `level_rod` 학습데이터가 보강되면 다시 True 로 되돌릴 것.
        #      (측면 판정도 같이 꺼지는 점은 감안 — 지금은 횡이동 자체를 끈 상태)
        self.declare_parameter('level_rod_use_seg', False)
        # ★ 레벨봉 **누적 확인** (2026-09-15). 최근 window 프레임 중 n번 이상
        #   임계를 넘어야 즉시정지로 인정한다. n=1이면 끈다(즉시 반응).
        #
        #   왜: 오검출이 **단발**이다. 주행 중 포착한 9프레임 중 값이 뜬 건 1장뿐이었고
        #   (268px 덩어리, 나머지 8장은 0.000), 그 한 장이 즉시정지를 만들었다.
        #   실체는 **철근 받침(스페이서)** — 레벨봉처럼 바닥에 수직으로 선 구조물이라
        #   실루엣이 거의 같다. 원근폭 검사를 0.9px 차이로 통과했다. 기하로는 못 가른다.
        #   반면 **진짜 봉은 연속으로 안정되게 잡힌다**(색 0.49/0.50/0.49…).
        #   ⟹ 구분되는 건 모양이 아니라 **지속성**이다.
        #
        #   중앙값이 아니라 카운터인 이유: 중앙값은 **보고값 자체를 뭉갠다**(거리가
        #   한 프레임 뒤처진다). 카운터는 `rod_near_frac`을 그대로 두고 **정지 결정만**
        #   지연시킨다 — 관심사가 분리되고, n이 현장에서 조정하기 직관적이다.
        #
        #   ⚠ 대가는 지연이다. 판정이 4Hz이므로
        #       n=3/window=4 → 최악 4프레임 ≈ 1.0초, 0.1m/s 주행이면 **100mm**
        #     정지거리가 340mm 이상이라 감당된다. **주행속도를 올리면 반드시 재검토**할 것
        #     (0.3m/s면 같은 지연이 300mm가 된다).
        self.declare_parameter('level_rod_confirm_n', 3)
        self.declare_parameter('level_rod_confirm_window', 4)
        # 측면(횡이동)용 임계는 **따로** 둔다.
        #   ⚠ 화각·장착각이 달라 같은 frac이 같은 실거리를 뜻하지 않는다.
        #   ⚠ 더 중요한 이유: **횡이동은 70mm 단위로 뛴다.** 판정이 임계를 아슬아슬하게
        #     못 넘으면 한 스텝(70mm)을 더 가버리는데, 그 거리면 봉에 닿을 수 있다.
        #     전후진은 연속 감속이라 이런 위험이 작다.
        #   실측(2026-09-02): 사용자가 "여기서 멈춰야 한다"고 한 위치가 0.664.
        #     임계 0.66이면 여유 0.004뿐 → **0.60**으로 한 스텝분 여유를 준다.
        #     대가는 최대 한 레인 일찍 멈추는 것이고, 봉에 닿는 것보다 낫다.
        self.declare_parameter('level_rod_side_stop_frac', 0.60)
        self.declare_parameter('obs_release_thr', de.OBS_RELEASE_THR)
        self.declare_parameter('obs_release_frames', de.OBS_RELEASE_FRAMES)
        self.declare_parameter('smooth', 5)
        self.declare_parameter('stale_sec', 1.5)      # 판정이 이보다 오래되면 stale
        # stale일 때 그 방향을 막을지. True=안전우선(판정 없으면 못 감).
        # ⚠ 카메라 한 대가 아예 안 뜨면 그 방향이 영구 차단된다 → 경고 로그 확인할 것.
        self.declare_parameter('block_when_stale', True)
        # 프레임 자체의 신선도. ⚠ [2026-09-02] 이게 없어서 **눈이 먼 채로 GO**가 났다.
        #   ZED 3대가 리부팅 루프에 빠져 영상이 전혀 안 왔는데, _decode가 마지막으로
        #   받아둔 **화석 프레임 한 장**을 계속 돌려줘서 판정이 매 주기 '갱신'됐다.
        #   → result_t가 늘 신선하니 block_when_stale이 영영 발동하지 않았고,
        #     /deck_edge_block은 forward=False backward=False verdict=GO를 유지했다.
        #   프레임이 이 시간보다 낡으면 없는 것으로 친다(판정 중단 → stale → 차단
        #   → rebar_drive의 판정 staleness 워치독이 ABORT).
        self.declare_parameter('img_stale_sec', 2.0)
        # ★ 진행방향 카메라만 추론(부하↓). 반대방향은 판정하지 않는다.
        #   ⚠ 그러면 반대방향은 늘 stale이므로 block_when_stale에 걸려 **탈출 방향까지
        #     막힌다**. 그래서 active_only일 때는 **판정 안 한 방향은 차단하지 않는다**.
        #     안전 근거: 그 방향으로 주행을 시작하면 /travel_direction이 바뀌어 카메라가
        #     전환되고 ~0.2s 안에 새 판정이 나온다. 게다가 주행노드는 방향 전환 시
        #     **새 판정이 올 때까지 움직이지 않는다**(첫 판정 대기).
        #   ※ seg_hz는 '총' 추론 주기다. active_only만 켜면 그 전부가 진행방향에 몰릴 뿐
        #     총량은 그대로 → 부하를 줄이려면 seg_hz도 함께 낮출 것(예: 8 → 5).
        self.declare_parameter('active_only', True)
        # 조향용 heading 산출(같은 마스크 재사용). 640폭으로 줄여 계산 — 파라미터가
        # 그 해상도로 튜닝됐고 원본 1920폭은 모폴로지/연결성분이 비싸다.
        self.declare_parameter('heading_enabled', True)
        self.declare_parameter('heading_width', 640)
        # [2026-08-06] zedxmini2=전방, zedxmini1=후방으로 물리 이동 → 기본값도 재지정.
        #   (실제 시야 스냅샷으로 확인: zedxmini2가 배근 진행방향을 본다)
        self.declare_parameter('front_topic',
                               '/zedxmini2/zed_node/rgb/color/rect/image/compressed')
        self.declare_parameter('back_topic',
                               '/zedxmini1/zed_node/rgb/color/rect/image/compressed')
        # ── 측면(횡이동) 주행가능 판정 ──────────────────────────────────
        # ⚠ **요청 시에만** 추론한다(상시 아님). 횡이동은 'ㄹ' 한 턴에 한 번뿐이라
        #   상시로 돌리면 부하만 늘고 얻는 게 없다. → /deck_edge/side_request 수신 시 1회.
        # 판정은 전후방과 **동일한 rebar_frac + 동일 임계(0.45)**.
        #   실측(2026-08-13): 좌(305) 1.000/0.521/0.425/0.238, 우(ZED X One) 1.000/0.940/0.657/0.285
        #   → 기종·해상도·시야각이 달라도 같은 임계로 갈린다. 카메라별 보정 불필요.
        # ⚠ obstacle 조건은 **쓰지 않는다** — RC 테스트베드에서 의자/바닥반사를 obstacle로
        #   잡아 0.053~0.068이 나왔다(임계 0.06 초과). 측면은 "갈 수 있나"만 보면 된다.
        # ⚠ compressed로 받는다. raw는 ZED X One HD1200 = 9.2MB/프레임이라
        #   구독을 여는 몇 초 동안만도 수십 MB/s가 이 노드로 들어온다.
        # ★ [2026-09-14] 좌측 Orbbec 305 → **ZED X One 4K(SN 319430526)** 로 교체.
        #   305는 배선상 거꾸로 달려 있어 180° 회전이 필요했으나 새 카메라는 정상 장착 → 0.
        #   되돌릴 경우: '/camera_left/color/image_raw/compressed' + rotate 180.
        self.declare_parameter('side_left_topic',
                               '/zedxone_left/zed_node/rgb/color/rect/image/compressed')
        self.declare_parameter('side_right_topic',
                               '/zedxone/zed_node/rgb/color/rect/image/compressed')
        self.declare_parameter('side_left_rotate', 0)
        self.declare_parameter('side_right_rotate', 0)

        # ★★ [2026-09-14] **측면 카메라를 이 노드가 띄웠다 내린다** (on-demand).
        #
        # ## 왜
        # GMSL에 ZED X 스테레오 2대(전/후, 주행 안전판정에 상시 필요) + ZED X One 모노
        # 2대(좌/우)가 달렸다. **4대를 동시에 스트리밍하면 죽는다** — 2026-09-14 실측:
        # 스테레오2+모노1이 돌고 있는 상태에서 모노를 하나 더 열자 전방 스테레오가
        # **SIGSEGV(exit -11)** 로 죽었다. 3대 구성은 검증돼 있다([[zedxone_coexistence]]).
        # 측면 판정은 'ㄹ'자 한 턴에 한 번뿐이라 상시로 띄울 이유가 없다.
        #   ⟹ **동시 최대 = 스테레오 2 + 모노 1.** 코드상 이 불변식을 깬다.
        #
        # ## 죽이기를 어떻게 보장하나 (이게 설계의 핵심이다)
        # 정리에 실패하면 모노가 쌓여 결국 4대가 되고 전체가 죽는다. 그래서 3중으로 건다:
        #   ① **여는 쪽에서 보장** — 띄우기 직전에 *모든* 측면 노드를 먼저 죽인다.
        #      이전 회차 정리가 실패했어도 다음 회차가 자동 복구한다(누적되지 않는다).
        #   ② **한 번에 한 대만** — 'both' 판정도 좌 → 죽이고 → 우 순서.
        #   ③ **finally + 확인사살** — 예외/타임아웃에도 반드시 종료. SIGINT → 확인 →
        #      남으면 SIGTERM → SIGKILL.
        # ⚠ `ros2 launch` 는 **SIGTERM으로는 자식이 안 죽는다**(2026-09-14 실측: 보내고도
        #    robot_state_publisher·zed_container가 살아남아 카메라를 쥐고 있었다).
        #    **SIGINT** 여야 launch가 자식까지 정리한다.
        # ★★ [2026-09-14 실패 기록] 기본값 True → **False**.
        #   on-demand 를 실제로 돌렸더니 **스테레오 2대가 같이 죽었다**(exit -11).
        #   원인은 Argus 대수 초과가 아니라 내가 띄운 프로세스가 **비정상 종료**한 것:
        #     · 좌측: `side_warmup_sec` 5초가 **기동 시간보다 짧아** 초기화 중에 SIGINT를
        #       보냈다 → `failed to create guard condition: context is not valid`
        #     · 우측: `No plugins found!`(image_transport) → **SIGABRT(exit -6)**
        #   그 SIGABRT가 Argus 소켓을 끊었고(`Error EndOfFile: reading socket`),
        #   물려 있던 스테레오 2대가 연달아 SIGSEGV로 죽었다.
        #   ⟹ **Argus 클라이언트 하나가 비정상 종료하면 나머지를 같이 끌고 내려간다.**
        #      런타임에 ZED 노드를 띄웠다 내리는 것 자체가 이 플랫폼에선 위험하다.
        #   다시 켜기 전에 반드시 해결할 것: ① warmup 을 기동시간보다 넉넉히(15초+)
        #   ② 자식이 SIGABRT 로 죽지 않게 환경/플러그인 문제 규명 ③ 죽어도 다른
        #   카메라가 안 죽는다는 보장(없다면 이 방식 자체를 포기).
        self.declare_parameter('side_manage_nodes', False)
        self.declare_parameter('side_left_cam', 'zedxone_left')
        self.declare_parameter('side_left_serial', '319430526')
        self.declare_parameter('side_right_cam', 'zedxone')
        self.declare_parameter('side_right_serial', '319647570')
        self.declare_parameter('side_cam_model', 'zedxone4k')
        # ⚠ HD1200 필수 — SVGA/HD1080은 이 유닛에서 open 실패하고, 반복 재시도가 GMSL
        #   버스를 교란해 스테레오까지 죽인다. override yaml에 그 근거가 적혀 있다.
        self.declare_parameter(
            'side_cam_override',
            '/home/koceti/ros2_ws/install/rebar_vision/share/rebar_vision/'
            'config/zedxone_params.yaml')
        # 기동 대기 상한. 실측 개통 4.5초 + DDS 디스커버리 0.7~1.1초.
        self.declare_parameter('side_launch_timeout_sec', 20.0)
        self.declare_parameter('side_kill_grace_sec', 5.0)
        self.declare_parameter('side_stop_thr', de.STOP_THR)
        # 요청 시 구독을 새로 여니 DDS 디스커버리(실측 0.7~1.1s) + 프레임 도착까지 대기
        self.declare_parameter('side_warmup_sec', 5.0)
        # both 판정에서 좌→우 사이에 두는 텀. 앞 카메라의 인코딩 파이프라인이
        # 내려간 뒤 다음을 열어 순간 부하가 겹치지 않게 한다.
        self.declare_parameter('side_stagger_sec', 1.0)
        # 판정 1회당 프레임 수(중앙값). 임계 0.45를 검증한 방법과 동일하게 3장.
        self.declare_parameter('side_frames', 3)
        # 첫 프레임 도착 후 **AE 수렴**을 기다리는 시간. 위 _open_side 주석 참조.
        self.declare_parameter('side_settle_sec', 1.5)
        # ★ 측면 프레임 **촬영시각** 기준 신선도 (2026-09-22). 이보다 오래 전에 찍힌
        #   프레임은 판정에 쓰지 않는다. 아래 _judge_one 주석 참조.
        self.declare_parameter('side_stale_sec', 2.0)

        g = self.get_parameter
        self.enabled = g('enabled').value
        self.imgsz = int(g('imgsz').value)
        self.seg_hz = float(g('seg_hz').value)
        self.stale_sec = float(g('stale_sec').value)
        self.block_when_stale = bool(g('block_when_stale').value)
        self.img_stale_sec = float(g('img_stale_sec').value)
        self.active_only = bool(g('active_only').value)
        weights = g('weights').value
        device = g('device').value

        if not device:
            import torch
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'seg 모델 로드: {weights} (device={device})')
        self.seg = RebarSegmenter(weights, device, self.imgsz)
        # ★★ [2026-09-14] **인덱스를 모델 클래스맵으로 재계산한다. 이 호출이 빠지면
        #   9클래스 모델에서 "주행가능 = obstacle + plate" 가 되어 판정이 뒤집힌다.**
        #   (rebar_h 4→6, rebar_v 5→7 로 밀렸다. rebar_seg.CLASS_MAPS 주석 참조)
        _cls = de.set_class_map(self.seg.class_map)
        rg.set_class_map(self.seg.class_map)
        self.get_logger().warn(
            'seg 클래스 %d개 → 주행가능=%s · 즉시정지=%s · level_rod=%s · plate=%s'
            % (len(self.seg.class_map),
               [self.seg.class_map[i] for i in _cls['REBAR']],
               [self.seg.class_map[i] for i in _cls['OBSTACLE']],
               _cls['level_rod'], _cls['plate']))

        # 카메라별 상태
        self.msg = {'front': None, 'back': None}
        self.msg_t = {'front': 0.0, 'back': 0.0}   # 프레임 도착시각
        self.fsm = {c: de.VerdictFSM(float(g('stop_thr').value),
                                     float(g('slow_thr').value),
                                     int(g('smooth').value),
                                     int(g('obs_release_frames').value))
                    for c in ('front', 'back')}
        self.result = {'front': None, 'back': None}     # 최근 judge() dict
        self.result_t = {'front': 0.0, 'back': 0.0}     # 최근 판정 시각
        self.no_image_warned = {'front': 0.0, 'back': 0.0}
        self.travel_direction = 'forward'
        self.obs_thr = float(g('obs_thr').value)
        self.obs_release_thr = float(g('obs_release_thr').value)
        self.rod_enabled = bool(g('level_rod_enabled').value)
        self.rod_stop_frac = float(g('level_rod_stop_frac').value)
        self.rod_use_seg = bool(g('level_rod_use_seg').value)
        self.rod_confirm_n = max(1, int(g('level_rod_confirm_n').value))
        self.rod_confirm_w = max(self.rod_confirm_n,
                                 int(g('level_rod_confirm_window').value))
        self.rod_hist = {'front': [], 'back': []}   # 최근 '임계 초과' 여부
        self.rod_side_stop_frac = float(g('level_rod_side_stop_frac').value)
        self.heading_enabled = bool(g('heading_enabled').value)
        self.heading_width = int(g('heading_width').value)

        # ★ 런타임 변경 허용 (2026-09-15). 이 값들은 **현장에서 튜닝하는 값**인데
        #   콜백이 없어 한 번 바꾸려면 서비스 전체(카메라 4대 포함)를 재기동해야 했다.
        #   헤딩 영점 시험을 레벨봉 오검출이 막았을 때 그게 그대로 병목이 됐다.
        #   ⚠ 시험용으로 낮춘 값은 **반드시 되돌릴 것** — 재기동하면 yaml 값으로 돌아간다.
        self.add_on_set_parameters_callback(self._on_params)


        self.create_subscription(CompressedImage, g('front_topic').value,
                                 lambda m: self._store('front', m), qos_profile_sensor_data)
        self.create_subscription(CompressedImage, g('back_topic').value,
                                 lambda m: self._store('back', m), qos_profile_sensor_data)
        self.create_subscription(String, '/travel_direction',
                                 self._direction_cb, 10)

        self.status_pub = self.create_publisher(String, '/deck_edge_status', 10)
        self.block_pub = self.create_publisher(String, '/deck_edge_block', 10)

        # ── 측면 판정 (요청 시 1회) ──
        self.side_thr = float(g('side_stop_thr').value)
        self.side_rot = {'left': int(g('side_left_rotate').value),
                         'right': int(g('side_right_rotate').value)}
        self.side_topic = {'left': g('side_left_topic').value,
                           'right': g('side_right_topic').value}
        self.side_msg = {'left': None, 'right': None}
        self.side_sub = {'left': None, 'right': None}
        # 측면 카메라 수명주기 (위 side_manage_nodes 주석 참조)
        self.side_manage = bool(g('side_manage_nodes').value)
        self.side_cam = {'left': str(g('side_left_cam').value),
                         'right': str(g('side_right_cam').value)}
        self.side_serial = {'left': str(g('side_left_serial').value),
                            'right': str(g('side_right_serial').value)}
        self.side_model = str(g('side_cam_model').value)
        self.side_override = str(g('side_cam_override').value)
        self.side_launch_timeout = float(g('side_launch_timeout_sec').value)
        self.side_kill_grace = float(g('side_kill_grace_sec').value)
        self.side_proc = {'left': None, 'right': None}
        # 노드 시작 시 남아 있는 측면 카메라를 먼저 정리한다 — 이전 실행이
        # 비정상 종료했다면 고아 프로세스가 카메라를 쥔 채 남아 있다.
        if self.side_manage:
            self._kill_side_nodes(reason='기동 시 정리')
        self.side_warmup = float(g('side_warmup_sec').value)
        self.side_stagger = float(g('side_stagger_sec').value)
        self.side_frames = max(1, int(g('side_frames').value))
        self.side_settle = float(g('side_settle_sec').value)
        self.side_stale = float(g('side_stale_sec').value)
        self._side_req = None            # None | 'both' | 'left' | 'right'
        # ⚠ 측면은 **구독 자체를 요청 시에만** 만든다(상시 구독 금지).
        #   ZED X One HD1200 raw = 1920×1200×4B ≈ 9.2MB/프레임. 5Hz면 46MB/s가
        #   판정을 안 할 때도 계속 이 노드로 흘러든다(직렬화·전송·콜백 전부 비용).
        #   횡이동 판정은 레인 전환 때 몇 초뿐이라 그 순간만 열고 바로 닫는다.
        self.create_subscription(String, '/deck_edge/side_request',
                                 self._side_request_cb, 10)
        self.side_pub = self.create_publisher(String, '/deck_edge/side_status', 10)

        self.timer = self.create_timer(1.0 / float(g('publish_hz').value), self._publish_block)

        self._stop = False
        self._thread = threading.Thread(target=self._seg_worker, daemon=True)
        self._thread.start()
        self.get_logger().warn(
            f"데크끝 감지 시작 (철근배근 기준) stop<{g('stop_thr').value} "
            f"slow<{g('slow_thr').value} seg={self.seg_hz}Hz 교대추론 "
            f"block_when_stale={self.block_when_stale}")

    # ---------- 입력 ----------
    def _store(self, cam, m):
        self.msg[cam] = m
        self.msg_t[cam] = time.time()

    def _direction_cb(self, m):
        if m.data in ('forward', 'backward'):
            self.travel_direction = m.data

    # ---------- 측면 판정 ----------
    def _store_side(self, side, m):
        self.side_msg[side] = m

    @staticmethod
    def _stamp(m):
        """메시지의 **촬영 시각**(header.stamp, 초). 도착 시각이 아니다."""
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 if m is not None else None

    def _side_request_cb(self, m):
        """'both'|'left'|'right' 요청 → 워커가 다음 사이클에 1회 판정."""
        self._side_req = m.data if m.data in ('both', 'left', 'right') else 'both'

    ROT = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
           270: cv2.ROTATE_90_COUNTERCLOCKWISE}

    # ---------- 측면 카메라 수명주기 ----------
    def _cam_pids(self, names):
        """해당 카메라 이름으로 떠 있는 프로세스 PID 전부.

        ⚠ **argv 원소 단위로 정확히 비교**한다. 부분일치로 찾으면 `zedxone` 이
           `zedxone_left` 까지 잡아서 엉뚱한 카메라를 죽인다(이름이 접두사 관계다).
           launch가 만드는 프로세스들은 공통으로 `__ns:=/<이름>` 또는
           `camera_name:=<이름>` 을 **하나의 argv 원소**로 갖는다.
        """
        want = set()
        for n in names:
            want.add('__ns:=/%s' % n)
            want.add('camera_name:=%s' % n)
        pids = []
        for d in os.listdir('/proc'):
            if not d.isdigit():
                continue
            try:
                with open('/proc/%s/cmdline' % d, 'rb') as f:
                    argv = [x.decode('utf-8', 'replace')
                            for x in f.read().split(b'\0') if x]
            except Exception:
                continue                      # 그 사이 죽은 프로세스
            if any(a in want for a in argv):
                pids.append(int(d))
        return pids

    def _kill_side_nodes(self, sides=None, reason=''):
        """측면 카메라 노드를 확실히 종료한다. **죽었다고 가정하지 않고 확인한다.**

        SIGINT → 확인 → SIGTERM → 확인 → SIGKILL 순. 첫 단계가 SIGINT인 이유는
        `ros2 launch` 가 SIGTERM 으로는 자식을 정리하지 않기 때문이다(2026-09-14 실측).
        """
        names = [self.side_cam[s] for s in (sides or ('left', 'right'))]
        # 우리가 띄운 프로세스 그룹부터 (자식까지 한 번에)
        for s in (sides or ('left', 'right')):
            pr = self.side_proc.get(s)
            if pr is not None and pr.poll() is None:
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                    try:
                        os.killpg(os.getpgid(pr.pid), sig)
                    except Exception:
                        break
                    try:
                        pr.wait(timeout=self.side_kill_grace)
                        break
                    except Exception:
                        continue
            self.side_proc[s] = None

        # 그래도 남았는지 **직접 확인**한다 (고아·이전 실행 잔재 포함)
        t0 = time.time()
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            pids = self._cam_pids(names)
            if not pids:
                break
            for pid in pids:
                try:
                    os.kill(pid, sig)
                except Exception:
                    pass
            t1 = time.time()
            while time.time() - t1 < self.side_kill_grace and self._cam_pids(names):
                time.sleep(0.1)
        left = self._cam_pids(names)
        if left:
            self.get_logger().error(
                '⛔ 측면 카메라 %s 종료 실패 — PID %s 가 남았다. **다음 판정에서 4대가 될 수 '
                '있다**(전방 스테레오 사망 경로). 수동 확인 필요.' % (names, left))
        elif reason:
            self.get_logger().info('  측면 카메라 정리 (%s, %.1fs)' % (reason, time.time() - t0))
        return not left

    def _spawn_side_node(self, side):
        """측면 카메라 노드 1대를 띄우고 토픽이 살아날 때까지 기다린다.

        ⚠ **띄우기 전에 좌·우 모두를 먼저 죽인다.** 이것이 "동시 모노는 최대 1대"
           불변식을 강제하는 지점이다 — 이전 회차 정리가 실패했어도 여기서 복구된다.
        ⚠ `env -u DISPLAY` — DISPLAY가 설정돼 있으면 Argus가 원격 X에 GPU 버퍼를
           만들려다 **BadParameter로 segfault** 한다(SSH X11 포워딩 시 재현).
        """
        self._kill_side_nodes(reason='%s 띄우기 전 정리' % side)
        name = self.side_cam[side]
        cmd = ['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py',
               'camera_name:=%s' % name,
               'camera_model:=%s' % self.side_model,
               'serial_number:=%s' % self.side_serial[side],
               'publish_tf:=false',
               'ros_params_override_path:=%s' % self.side_override]
        env = dict(os.environ)
        env.pop('DISPLAY', None)
        log = '/var/log/robot_control/side_cam_%s.log' % name
        try:
            f = open(log, 'ab')
        except Exception:
            f = subprocess.DEVNULL
        try:
            self.side_proc[side] = subprocess.Popen(
                cmd, env=env, stdout=f, stderr=subprocess.STDOUT,
                start_new_session=True)      # 자체 프로세스 그룹 → killpg 로 통째 정리
        except Exception as e:
            self.get_logger().error('측면 카메라 %s 기동 실패: %s' % (name, e))
            self.side_proc[side] = None
            return False
        self.get_logger().info(
            '  📷 %s 기동 (SN %s, pid %d) — 로그 %s'
            % (name, self.side_serial[side], self.side_proc[side].pid, log))
        return True

    def _open_side(self, sides):
        """측면 카메라를 띄우고 구독을 열어 첫 프레임을 기다린다 (판정 직전에만).

        ⚠ `sides` 는 **한 번에 하나만** 넘어온다(_judge_side 가 그렇게 부른다).
           모노 2대를 동시에 띄우면 GMSL 4대가 되어 전방 스테레오가 죽는다.
        """
        if self.side_manage:
            if len(sides) > 1:
                self.get_logger().error(
                    '⛔ 측면 카메라를 한 번에 %d대 요청받았다 — 1대씩만 연다'
                    '(4대 동시 = 전방 스테레오 사망)' % len(sides))
                sides = tuple(sides)[:1]
            for sd in sides:
                if not self._spawn_side_node(sd):
                    return          # 못 띄웠으면 프레임도 안 온다 → 판정 불가로 떨어진다
        for s in sides:
            if self.side_sub[s] is None:
                self.side_msg[s] = None
                self.side_sub[s] = self.create_subscription(
                    CompressedImage, self.side_topic[s],
                    (lambda side: lambda m: self._store_side(side, m))(s),
                    qos_profile_sensor_data)
        # DDS 디스커버리(0.7~1.1s) + **AE(자동노출) 수렴** 대기.
        # ⚠ 프레임이 왔다고 바로 판정하면 안 된다 (2026-08-13 실측): 정지 상태에서
        #   같은 장면을 30회 판정했는데 rebar_frac이 0.168 / 0.475 **두 무리로
        #   갈렸다**(판정 내부 3프레임은 편차 0.005로 일치). 구독이 끊기면 발행이
        #   멈추고, 다시 붙는 순간부터 AE가 재수렴하기 때문이다. 임계 0.45를 정확히
        #   가로질러 횡이동 판정이 무작위로 뒤집힌다.
        #   → 첫 프레임 도착 후 `side_settle_sec` 만큼 더 기다린 뒤 판정한다.
        t0 = time.time()
        first = None
        while time.time() - t0 < self.side_warmup:
            if all(self.side_msg[s] is not None for s in sides):
                if first is None:
                    first = time.time()
                elif time.time() - first >= self.side_settle:
                    break
            time.sleep(0.05)

    def _close_side(self):
        """구독 해제 + **카메라 노드 종료** → 판정 안 할 때 측면 비용 0.

        구독만 닫으면 발행은 멈추지만 grab+rectify 는 계속 돈다. GMSL 대수를 줄이려면
        노드 자체를 내려야 한다 — 그게 4대 동시를 막는 유일한 방법이다.
        """
        for s, sub in self.side_sub.items():
            if sub is not None:
                self.destroy_subscription(sub)
                self.side_sub[s] = None
            self.side_msg[s] = None
        if self.side_manage:
            self._kill_side_nodes(reason='판정 종료')

    def _judge_frame(self, side):
        """현재 들어와 있는 프레임 1장 판정 → (rebar_frac, on_rebar). 없으면 None."""
        m = self.side_msg[side]
        img = (cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
               if m is not None else None)
        if img is None:
            return None
        rot = self.ROT.get(self.side_rot[side])
        if rot is not None:
            img = cv2.rotate(img, rot)
        mask = self.seg.predict(img)
        _, frac, on_rebar = de.rebar_edge(mask)
        # 레벨봉 — 측면은 **횡이동 경로**라 봉이 있으면 그쪽으로 못 간다.
        #   전방과 색 재현·형상이 달라 측면 전용 임계를 쓴다(ROD_SIDE_*).
        rod_f = 0.0
        if self.rod_enabled:
            try:
                rods = de.level_rods(img, mask,
                                     hsv_lo=de.ROD_SIDE_HSV_LO,
                                     min_ar=de.ROD_SIDE_MIN_AR,
                                     w_min_ratio=de.ROD_SIDE_W_RATIO,
                                     min_area=de.ROD_SIDE_MIN_AREA)
                if rods:
                    rod_f = rods[0][1] / float(mask.shape[0])
                # ★ [2026-09-14] seg `level_rod` 클래스와 **OR**. 색 기반은 좌측
                #   카메라에서 못 쓰고(녹슨 철근이 봉보다 노랗다), seg는 아직
                #   데이터가 적어 정확도가 낮다 → 어느 쪽이든 잡으면 멈춘다.
                if self.rod_use_seg:
                    # ⚠ 측면은 원근폭 계수가 없다(전방 실측값) → 검사를 끈다.
                    #   색 기반도 측면에선 같은 이유로 꺼져 있다.
                    rod_f = max(rod_f, de.level_rod_seg(mask, w_min_ratio=0.0))
            except Exception as e:
                self.get_logger().error(f'측면 레벨봉 검출 오류: {e}',
                                        throttle_duration_sec=10.0)
        return float(frac), bool(on_rebar), float(rod_f)

    def _judge_one(self, side):
        """한쪽만 열고 → **n프레임 중앙값**으로 판정 → 즉시 닫는다.

        · 한 대씩 여는 이유: 구독이 붙는 순간 그 카메라의 JPEG 인코딩이 기동된다.
          좌우를 겹치면 순간 부하가 두 배가 된다.
        · ⚠ **1프레임 판정은 못 쓴다** (2026-08-13): 로봇이 정지해 있는데도 우측이
          0.253/0.455/0.602/0.835/0.465로 흔들렸다. 임계 0.45 근처에서 판정이 뒤집힌다.
          애초에 임계 0.45를 검증한 방법도 **위치당 3프레임**(편차 ≤0.004)이었다.
          → 구독이 어차피 열려 있으니 몇 장 더 받아 중앙값을 쓴다(추론 ~0.1s/장).
        """
        self._open_side((side,))
        # ⚠ 아래 finally 가 **카메라를 내리는 유일한 보장 지점**이다. 예외로 빠지거나
        #   프레임이 안 와도 반드시 통과한다. 여기서 놓치면 모노가 쌓여 4대가 된다.
        # ★★ 촬영시각 검사 (2026-09-22). 전방엔 9/2에 `img_stale_sec`을 넣었는데
        #   측면 경로엔 없었다. 그 결과:
        #     11:07 좌측 카메라 사망(CAMERA NOT INITIALIZED, 이후 155회 재접속 실패)
        #     11:16 측면 판정  left = 불가(0.345, **3장 편차 0.0**)
        #   9분 전에 죽은 카메라에서 `0.345` 라는 판정값이 나왔다. 편차 0.0 = 같은
        #   사진을 세 번 판정한 것이다. 이번엔 '불가'라 멈췄지만, 옛 사진이 배근 위였다면
        #   **'가능'으로 나와 눈먼 채 횡이동**했을 것이다.
        #   → ① 촬영시각이 side_stale_sec 보다 오래되면 버린다
        #     ② 판정에 쓰는 프레임들은 **서로 다른 촬영시각**이어야 한다
        #     모자라면 '영상 없음'(= 불가, 안전한 쪽).
        try:
            fracs, rebars, rodfs, stamps, ages = [], [], [], [], []
            for i in range(self.side_frames):
                if i:
                    self._wait_new_frame(side)
                m = self.side_msg[side]
                st = self._stamp(m)
                if st is None:
                    break
                age = time.time() - st
                if age > self.side_stale:
                    self.get_logger().warn(
                        f'⚠ {side} 측면 프레임이 {age:.1f}s 전 촬영 — 판정 제외 '
                        f'(임계 {self.side_stale:.1f}s, 카메라 정지/재발행 의심)')
                    break
                if st in stamps:
                    self.get_logger().warn(
                        f'⚠ {side} 같은 촬영시각 프레임 반복 — 새 프레임이 안 온다')
                    break
                r = self._judge_frame(side)
                if r is None:
                    break
                fracs.append(r[0]); rebars.append(r[1]); rodfs.append(r[2])
                stamps.append(st); ages.append(age)
            if len(fracs) < self.side_frames:
                why = ('영상 없음' if not fracs else
                       f'신선한 프레임 부족 {len(fracs)}/{self.side_frames}')
                return {'ok': False, 'reason': why}
            frac = float(sorted(fracs)[len(fracs) // 2])          # 중앙값
            on_rebar = sum(rebars) * 2 > len(rebars)              # 다수결
            # obstacle은 보지 않는다(주석 참조). 배근이 이어져 있는지만 본다.
            rod_f = float(sorted(rodfs)[len(rodfs) // 2]) if rodfs else 0.0
            rod_block = (rod_f >= self.rod_side_stop_frac) if self.rod_enabled else False
            reason = ''
            if not on_rebar:
                reason = '배근이탈'
            elif frac < self.side_thr:
                reason = f'배근부족 {frac:.2f}'
            elif rod_block:
                reason = (f'레벨봉 근접 {rod_f:.2f}'
                          f'(임계 {self.rod_side_stop_frac:.2f})')
            return {'ok': bool(on_rebar and frac >= self.side_thr and not rod_block),
                    'rebar_frac': round(frac, 3), 'on_rebar': on_rebar,
                    'rod_near_frac': round(rod_f, 3),
                    'n': len(fracs), 'spread': round(max(fracs) - min(fracs), 3),
                    'age_max': round(max(ages), 2),
                    'reason': reason}
        finally:
            self._close_side()          # 실패해도 스트림은 반드시 닫는다

    def _wait_new_frame(self, side, timeout=1.0):
        """다음 프레임이 도착할 때까지 대기(같은 장면을 중복 판정하지 않게)."""
        # ⚠ 메시지 **객체**가 바뀌었는지가 아니라 **촬영시각**이 바뀌었는지를 본다.
        #   죽은 카메라가 마지막 사진을 계속 다시 발행하면 객체는 매번 새것이라
        #   `is prev` 로는 못 거른다(2026-09-22 좌측 `3장 편차 0.0` 의 정체).
        prev = self._stamp(self.side_msg[side])
        t0 = time.time()
        while self._stamp(self.side_msg[side]) == prev and time.time() - t0 < timeout:
            time.sleep(0.02)

    def _run_side(self, which):
        """요청된 방향(들)을 1회 판정해 /deck_edge/side_status 발행.

        ⚠ **좌우를 동시에 열지 않는다** (2026-08-13). 구독을 여는 순간 그 카메라의
           JPEG 인코딩 파이프라인이 새로 기동되는데(image_transport는 구독자가 생겨야
           인코딩 시작), ZED X One은 HD1200이라 그 비용이 작지 않다. 좌우를 겹치면
           순간 부하가 두 배가 되고, 이 판정은 이미 카메라 5대 + GPU seg + YOLO가
           도는 상태에서 일어난다(실측 load 16~18/12코어).
           → left 열기·판정·닫기 → 사이 텀 → right 열기·판정·닫기 로 쪼갠다.
           첫 레인 전환에서만 both가 필요하고, 이후 레인은 어차피 한쪽만 본다.
        """
        out = {}
        sides = ('left', 'right') if which == 'both' else (which,)
        for i, side in enumerate(sides):
            if i:
                time.sleep(self.side_stagger)   # 앞 카메라 파이프라인이 내려갈 틈
            out[side] = self._judge_one(side)
        msg = String()
        msg.data = json.dumps({'thr': self.side_thr, 'stamp': time.time(), **out})
        self.side_pub.publish(msg)
        parts = []
        for s, v in out.items():
            detail = str(v.get('rebar_frac', v.get('reason')))
            if 'n' in v:
                detail += f", {v['n']}장 편차{v['spread']} 촬영지연≤{v.get('age_max', '?')}s"
            parts.append(f"{s}={'가능' if v.get('ok') else '불가'}({detail})")
        self.get_logger().warn('↔ 측면 판정: ' + '  '.join(parts))

    def _decode(self, cam):
        m = self.msg[cam]
        # 낡은 프레임은 '없음'으로 친다 — 화석 프레임 재판정 금지(위 img_stale_sec 주석).
        if m is None or (time.time() - self.msg_t[cam]) > self.img_stale_sec:
            return None
        arr = np.frombuffer(m.data, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    # ---------- 추론 ----------
    def _seg_worker(self):
        """추론 워커.
        · active_only=True : 진행방향 카메라만 (부하↓)
        · active_only=False: 전·후방 교대, 진행방향 2회 : 반대방향 1회
        """
        order = ['active', 'active', 'other']
        i = 0
        period = 1.0 / max(self.seg_hz, 0.1)
        while not self._stop:
            t0 = time.time()
            # 측면 판정 요청이 있으면 먼저 처리 (1회성, 횡이동 직전에만 온다)
            if self._side_req is not None:
                req, self._side_req = self._side_req, None
                try:
                    self._run_side(req)
                except Exception as e:
                    self.get_logger().error(f'측면 판정 오류: {e}')
                    self._close_side()      # 실패해도 스트림은 반드시 닫는다
            if self.enabled:
                active = 'front' if self.travel_direction == 'forward' else 'back'
                other = 'back' if active == 'front' else 'front'
                if self.active_only:
                    cam = active
                else:
                    cam = active if order[i % len(order)] == 'active' else other
                i += 1
                try:
                    self._run_seg(cam)
                except Exception as e:
                    self.get_logger().error(f'{cam} seg 오류: {e}')
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    # 런타임에 바꿀 수 있는 값들. 이름 → 속성 + 형변환.
    _LIVE_PARAMS = {
        'level_rod_enabled': ('rod_enabled', bool),
        'level_rod_stop_frac': ('rod_stop_frac', float),
        'level_rod_use_seg': ('rod_use_seg', bool),
        'level_rod_confirm_n': ('rod_confirm_n', int),
        'level_rod_confirm_window': ('rod_confirm_w', int),
        'level_rod_side_stop_frac': ('rod_side_stop_frac', float),
        'obs_thr': ('obs_thr', float),
        'obs_release_thr': ('obs_release_thr', float),
        'heading_enabled': ('heading_enabled', bool),
        'img_stale_sec': ('img_stale_sec', float),
    }

    def _on_params(self, params):
        """`ros2 param set`으로 임계를 바꿀 수 있게 한다(재기동 불필요).

        stop_thr/slow_thr은 여기 없다 — VerdictFSM이 생성 시점에 값을 품고 있어
        객체를 새로 만들어야 하고, 그러면 진행 중인 히스테리시스 상태가 날아간다.
        그건 판정 도중에 조용히 바뀌면 안 되는 종류다.
        """
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            ent = self._LIVE_PARAMS.get(prm.name)
            if ent is None:
                continue
            attr, cast = ent
            try:
                val = cast(prm.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False, reason=f'{prm.name}: 형변환 실패')
            setattr(self, attr, val)
            self.get_logger().warn(f'⚙ 런타임 변경: {prm.name} = {val}')
        return SetParametersResult(successful=True)

    def _run_seg(self, cam):
        img = self._decode(cam)
        if img is None:
            now = time.time()
            if now - self.no_image_warned[cam] > 5.0:
                self.no_image_warned[cam] = now
                self.get_logger().warn(
                    f'⚠ {cam} 영상 없음 → 해당 방향 판정 불가'
                    f"{' (그 방향 주행 차단됨)' if self.block_when_stale else ''}")
            return
        mask = self.seg.predict(img)
        # 레벨봉을 **judge보다 먼저** 뽑는다 — verdict에 반영해야 rebar_drive가
        # STOP을 보고 방향전환/횡이동으로 넘어간다(block 토픽만으론 안 된다).
        # ★ 색 기반과 seg를 **따로 들고** 있다가 마지막에 OR 한다 (2026-09-15).
        #   왜: 둘을 바로 max()로 합치면 오검출 때 **어느 쪽이 범인인지 알 수 없다.**
        #   실제로 헤딩 영점 시험이 두 번 다 레벨봉 오검출로 중단됐는데
        #   (rebar_frac은 0.57/0.58로 정상, 멈추면 사라짐 = 주행 중 단발 오검출),
        #   로그가 합쳐진 값만 찍어 색인지 seg인지 못 갈랐다.
        #   색 기반은 녹슨 철근을 노랗게 보고, seg의 level_rod는 학습데이터가 적다 —
        #   둘 다 용의자라 **분리 기록이 있어야 어느 쪽을 고칠지 정해진다.**
        rods, rod_color, rod_seg = [], 0.0, 0.0
        if self.rod_enabled:
            try:
                rods = de.level_rods(img, mask)
                if rods:
                    rod_color = rods[0][1] / float(mask.shape[0])
                if self.rod_use_seg:
                    rod_seg = float(de.level_rod_seg(mask))
            except Exception as e:
                self.get_logger().error(f'레벨봉 검출 오류: {e}',
                                        throttle_duration_sec=10.0)
        rod_f = max(rod_color, rod_seg)          # 보고값은 **그대로**(거리 정보 보존)
        # 누적 확인 — 단발 오검출 제거 (위 level_rod_confirm_n 주석 참조).
        over = bool(self.rod_stop_frac > 0 and rod_f >= self.rod_stop_frac)
        h = self.rod_hist[cam]
        h.append(over)
        del h[:-self.rod_confirm_w]
        rod_hits = sum(h)
        rod_src = ('색' if rod_color >= rod_seg else 'seg') if rod_f > 0 else '-'
        rod_hard = bool(rod_hits >= self.rod_confirm_n)
        r = de.judge(mask, self.fsm[cam], self.obs_thr, self.obs_release_thr,
                     extra_hard=rod_hard,
                     extra_reason=f'레벨봉 근접 {rod_f:.2f}'
                                  f'[{rod_src} 색{rod_color:.2f}/seg{rod_seg:.2f}'
                                  f' 누적{rod_hits}/{self.rod_confirm_n}]')
        r['cam'] = cam
        r['rods'] = [[c[0], c[1], c[2]] for c in rods[:5]]
        r['rod_n'] = len(rods)
        r['rod_near_frac'] = round(rod_f, 3)
        r['rod_color_frac'] = round(rod_color, 3)
        r['rod_seg_frac'] = round(rod_seg, 3)
        r['rod_hits'] = int(rod_hits)              # 최근 창에서 임계 초과 횟수
        r['rod_confirm_n'] = int(self.rod_confirm_n)
        # 조향 신호(heading)도 **같은 마스크**로 뽑는다 — 추론을 두 번 하지 않으려고
        # 여기서 계산해 함께 발행한다. 주행노드는 이걸 받아 angular.z에 쓴다.
        if self.heading_enabled:
            hd, nb = rg.heading(mask, self.heading_width)
            r['heading_deg'] = None if hd is None else round(hd, 2)
            r['heading_bars'] = nb
        self.result[cam] = r
        self.result_t[cam] = time.time()

        msg = String()
        msg.data = json.dumps(r)
        self.status_pub.publish(msg)
        if r['verdict'] == 'STOP':
            self.get_logger().warn(
                f"🛑 {cam} 배근 끊김(데크끝) rebar_frac={r['rebar_frac']:.2f} {r['reason']}",
                throttle_duration_sec=1.0)

    # ---------- 출력 ----------
    def _blocked(self, cam, active=None):
        """(차단여부, 사유). 판정이 stale이면 정책에 따라 차단.

        active_only 모드에서 **판정 대상이 아닌 카메라는 차단하지 않는다** —
        안 보고 있다는 이유로 막으면 탈출 방향까지 잠긴다.
        """
        r = self.result[cam]
        if r is None or (time.time() - self.result_t[cam]) > self.stale_sec:
            if self.active_only and active is not None and cam != active:
                return False, ''            # 미판정 방향 → 차단 안 함
            if self.block_when_stale:
                return True, f'{cam} 판정 없음/stale'
            return False, ''
        if r['verdict'] == 'STOP':
            return True, f"{cam} 데크끝({r['reason'] or 'rebar_frac ' + str(r['rebar_frac'])})"
        # 레벨봉은 judge()에 extra_hard로 먹여 **verdict=STOP**이 되므로
        # 위 STOP 분기에서 이미 걸린다(reason에 '레벨봉 근접'이 실린다).
        return False, ''

    def _publish_block(self):
        if not self.enabled:
            return
        active = 'front' if self.travel_direction == 'forward' else 'back'
        fb, fr = self._blocked('front', active)
        bb, br = self._blocked('back', active)
        r = self.result[active]
        payload = {
            'forward': fb,             # front 카메라 방향 주행 차단
            'backward': bb,            # back 카메라 방향 주행 차단
            'reason': '; '.join(x for x in (fr, br) if x),
            'verdict': (r or {}).get('verdict', 'UNKNOWN'),
            'rebar_frac': (r or {}).get('rebar_frac', 0.0),
            'rod_n': (r or {}).get('rod_n', 0),
            'rod_near_frac': (r or {}).get('rod_near_frac', 0.0),
            'active_cam': active,
            'stamp': time.time(),
        }
        m = String()
        m.data = json.dumps(payload)
        self.block_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = DeckEdgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        # ⚠ 이 노드가 측면 카메라의 **소유자**다. 여기서 안 내리면 고아로 남아
        #   카메라를 쥔 채 다음 실행과 겹친다 → GMSL 4대 → 전방 스테레오 사망.
        try:
            if getattr(node, 'side_manage', False):
                node._kill_side_nodes(reason='노드 종료')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
