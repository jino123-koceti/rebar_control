#!/usr/bin/env python3
"""데크끝(주행불가) 판정 — 순수 함수 모듈 (ROS 의존 없음).

⚠ 주행규칙: 로봇 무한궤도는 **철근 배근 위를 밟고** 주행한다.
  → **주행가능 = rebar_h/rebar_v 가 이어져 있는 곳**뿐.
  → 방수포(background)·floor·wall·obstacle·human 은 전부 **주행불가**.
  (구버전은 background를 주행가능에 포함했으나 배근 없는 맨 방수포 위를 GO로
   오판했고 카메라별 baseline도 갈렸다. 2026-08-06 철근 기반으로 전환.)

핵심 지표 `rebar_frac` = 배근이 이어진 세로범위 (0~1):
  · 행별 철근비율 r(y) = 중앙컬럼 중 rebar_h|rebar_v 픽셀 비율 (수직 스무딩)
  · 발밑(하단) 지지행부터 위로 스캔, GAP_FRAC 넘게 끊기면 거기가 배근 끝 = edge_row
  · rebar_frac = (H-edge_row)/H
  RC 주행영상 실측: 정상주행 front 0.55~0.64 / back 0.55~0.68,
                    데크끝     front 0.17~0.44 / back 0.40~0.47
  → **front/back 공통 임계 stop 0.45 / slow 0.55** (철근기준이라 카메라 무관)

사용처: rebar_vision/deck_edge_node.py (실주행), tools/drive/*.py (분석·튜닝)
"""
import numpy as np

# ★★ [2026-09-14] **인덱스를 이름에서 유도한다.** 하드코딩 금지.
#
# 9클래스 모델(rebar_seg_260914)에서 인덱스가 전부 밀렸다 — rebar_h 4→6, rebar_v 5→7,
# obstacle 3→4, wall 6→8. 하드코딩된 `REBAR = [4, 5]` 를 그대로 두면 신규 모델에서
# **"주행가능 = obstacle + plate"**, **"주행불가 = 진짜 철근"** 이 된다. 판정이 통째로
# 뒤집혀 데크 밖으로 나간다. 앞으로 클래스가 또 늘어도 안전하도록 이름으로 푼다.
#
# 아래 값들은 **구 7클래스 기준 기본값**이고, 모델을 로드한 쪽이 `set_class_map()` 을
# 불러 실제 맵으로 덮어쓴다(deck_edge_node 가 seg 로드 직후 호출).
IDX_H, IDX_V = 4, 5        # rebar_h(가로), rebar_v(세로=주행방향)
REBAR = [IDX_H, IDX_V]     # ★ 주행가능면 = 철근 배근 (이것만!)
NONDECK = [0, 1, 2, 3, 6]  # 배근이 아닌 전부 = 주행불가
OBSTACLE = [2, 3]          # human, obstacle (안전 정지 대상)
DECK_LEGACY = [0, 4, 5]    # (구버전 비교용) background 포함 deck
IDX_LEVEL_ROD = None       # 9클래스 모델에만 있다. 없으면 색 기반만 쓴다
IDX_PLATE = None           # 데크 가장자리 H빔 형상 철판
CLASS_MAP = {0: 'background', 1: 'floor', 2: 'human', 3: 'obstacle',
             4: 'rebar_h', 5: 'rebar_v', 6: 'wall'}


def set_class_map(class_map):
    """활성 모델의 클래스맵으로 인덱스 집합을 재계산한다.

    규칙(인덱스가 아니라 **의미**로 정의한다):
      · REBAR    = rebar_h, rebar_v            ← 궤도가 밟고 갈 수 있는 유일한 면
      · NONDECK  = **REBAR 가 아닌 전부**       ← 여집합이라 클래스가 늘어도 자동으로
                                                 "모르는 것 = 주행불가"(안전한 쪽)
      · OBSTACLE = human, obstacle             ← 즉시정지(hard_stop) 대상
      · plate    = NONDECK 에 포함(여집합이라 자동). 데크 가장자리 표식이므로 밟지
                   않는다. 즉시정지는 아니다 — rebar_frac 스무딩 경로로 처리한다
                   (2026-09-14 결정: 모델 정확도가 아직 낮아 오검출 시 잦은 급정지를
                    만들지 않기 위함).
    """
    global IDX_H, IDX_V, REBAR, NONDECK, OBSTACLE, DECK_LEGACY
    global IDX_LEVEL_ROD, IDX_PLATE, CLASS_MAP
    name2i = {v: k for k, v in class_map.items()}
    missing = [n for n in ('rebar_h', 'rebar_v', 'background') if n not in name2i]
    if missing:
        raise RuntimeError(
            f'클래스맵에 필수 클래스가 없다: {missing}. 주행판정을 만들 수 없다. '
            f'받은 맵: {class_map}')
    CLASS_MAP = dict(class_map)
    IDX_H, IDX_V = name2i['rebar_h'], name2i['rebar_v']
    REBAR = [IDX_H, IDX_V]
    NONDECK = [i for i in class_map if i not in REBAR]
    OBSTACLE = [name2i[n] for n in ('human', 'obstacle') if n in name2i]
    DECK_LEGACY = [name2i['background']] + REBAR
    IDX_LEVEL_ROD = name2i.get('level_rod')
    IDX_PLATE = name2i.get('plate')
    return {'REBAR': REBAR, 'NONDECK': NONDECK, 'OBSTACLE': OBSTACLE,
            'level_rod': IDX_LEVEL_ROD, 'plate': IDX_PLATE}
CENTER = (0.2, 0.8)        # 중앙 컬럼 범위 (좌우 20% 제외)
FRAC_THR = 0.7             # (구버전) 컬럼이 'deck 지배'로 볼 최소 비율
# 철근배근 판정 파라미터 (2026-08-05 RC 주행영상 파라미터 스윕으로 확정)
SMOOTH_ROWS = 16           # 수직 스무딩 커널 = H/16 (작으면 세로바 구간이 끊겨 오검)
ROW_THR = 0.015            # 행별 철근비율 ≥ 이 값 = 그 행에 배근 있음
#   (가로바 행 0.2~0.5, 가로바 사이는 세로바만 0.03~0.07 → 0.015가 안전한 하한.
#    0.02+H/64 조합은 세로바 구간이 끊겨 정상주행 중 순간 STOP이 났음)
GAP_FRAC = 0.09            # H의 9% 넘게 연속 무철근 = 배근 끊김(데크끝)
ANCHOR_FRAC = 0.45         # 최하단 지지행이 화면 하단 45% 안에 있어야 '발밑 배근 있음'
#   ※ 스무딩(H/16) 때문에 이 경계는 ±3%H 정도 무뎌진다. 정밀 기준이 아니라
#     '발밑에 배근이 아예 없는가'를 거르는 굵은 게이트로 볼 것.
# 전방 밴드(세로, H 비율): near=바로앞, mid=전방, far=먼전방
BANDS = {'near': (0.75, 1.00), 'mid': (0.50, 0.75), 'far': (0.32, 0.50)}
# 기본 임계 (front/back 공통)
STOP_THR = 0.45
SLOW_THR = 0.55
OBS_THR = 0.06             # 전방밴드 obstacle/human 비율 > 이 값 → 즉시 STOP
# ★ 해제 히스테리시스 (2026-08-19 추가)
#   실측: 기둥을 지나칠 때 obs가 0.06 임계에 걸터앉아 **같은 초에 GO→STOP→GO**로
#   뒤집혔다. hard_stop은 스무딩·히스테리시스를 **의도적으로 우회**하므로(사람이
#   들어왔을 때 지연되면 안 된다) 진입에는 보호가 없어야 맞지만, **해제**까지
#   무방비인 건 별개 문제였다.
#   → 정지는 지금처럼 즉시, 해제는 더 낮은 값에서 N프레임 연속일 때만.
#   ⚠ 이게 왜 중요한가: 상위 rebar_drive_node는 STOP 한 프레임만 봐도 방향전환·
#     횡이동을 확정한다. 채터링 1회 = 레인 하나를 통째로 건너뛸 수 있다.
OBS_RELEASE_THR = 0.04     # 해제 판단 임계 (진입 0.06보다 낮게 = 슈미트 트리거)
OBS_RELEASE_FRAMES = 3     # 이 횟수 연속 '깨끗'해야 hard_stop 해제


# ── 레벨봉(배근에 꽂는 노란 수직봉) 검출 ──────────────────────────────
# ★ 왜 색으로 찾나 (2026-09-02):
#   현재 7클래스 세그 모델은 레벨봉을 **전혀 못 본다** — 노란 픽셀의 96.7%를
#   `background`로 분류한다(실측). 그래서 deck_edge가 봉 앞에서 안 멈춘다.
#   반면 색은 아주 잘 갈린다: 전체 화면의 0.24%만 노란색이고 그게 전부 봉이다.
#   재학습 없이 즉시 쓸 수 있고 GPU도 안 쓴다. (장기적으로는 세그에 클래스 추가)
# ★ 채도(S) 하한이 핵심이다 (2026-09-02 실측).
#   현장에는 작업자 통로용 **나무 판자**가 깔린다. 판자는 색상(H)이 18~38에
#   **94.8%**나 들어와 색상만으로는 레벨봉과 못 가른다. 갈라주는 건 채도다:
#       판자   채도 중앙값 44,  90퍼센타일 54
#       레벨봉 채도 중앙값 131, 최소 110
#   임계별 판자 오검출 픽셀(면적임계 40 기준):
#       S>=80 → 71개 ⚠ 위험 /  S>=90 → 35개 ✅ /  S>=110 → 5개
#   90으로 잡으면 판자는 여전히 면적임계 아래이고 **레벨봉 검출 픽셀은 24% 늘어**
#   어두운 현장에서의 여유가 생긴다(봉의 채도 최소가 110이라 110은 여유가 없다).
ROD_HSV_LO = (18, 90, 90)      # 노란색 하한 (H,S,V)
ROD_HSV_HI = (38, 255, 255)
ROD_MIN_AREA = 40              # 최소 픽셀
ROD_MIN_H = 12                 # 최소 높이(px)
# 세로/가로 하한. 레벨봉은 **설계상 가늘고 길다**.
# ★ 1.2 → 1.5 (2026-09-02): 배근 위에 놓인 **노란 케이블 뭉치**가 오검출됐다
#   (h/w 1.3, 면적 3539 — 덩어리라 봉과 크기·색이 비슷하다).
#   실측 비교:  케이블뭉치 1.3  ↔  가까운 레벨봉 1.7 / 2.9
#   ⚠ 바닥에 **늘어진** 케이블은 이 필터로 이미 걸러진다(가늘고 가로라 h/w<1).
#   ⚠ 봉은 가까울수록 굵어 보여 h/w가 준다. 실측 최소가 1.7이라 1.5면 여유가 있지만,
#     더 큰 봉을 쓰면 재확인할 것.
ROD_MIN_AR = 1.5
ROD_ROW_FRAC = 0.5             # 밑동 행에 배근이 있어야 하는 비율
# ★ 원근 폭 검사 (2026-09-02) — **세로로 놓인 노란 케이블**을 거른다.
#   세로 케이블은 색·세로비·발밑 조건을 전부 통과해 오검출됐다(frac 0.84로 오차단).
#   갈라주는 건 **원근**이다: 크기가 정해진 레벨봉은 가까울수록 굵어 보이는데
#   케이블은 아무리 가까워도 가늘다.
#     실측 맞춤 (960x600 전방, R²=0.9925):  w ≈ 0.217·y_bot − 40.3
#       y=250→11px(예측14) / y=415→48px(예측50) / y=536→76px(예측76)
#     세로 케이블:  y=503→27px(예측69의 39%) / y=599→30px(예측89의 34%)
#   ⚠ 해상도 의존 — 계수는 **960x600 기준**이다. 다른 해상도면 폭에 비례 보정한다.
#   ⚠ 더 굵거나 가는 레벨봉으로 바꾸면 재측정할 것.
ROD_W_SLOPE = 0.217            # 폭 예측 기울기 (px per row) @960폭
ROD_W_INTER = -40.3            # 폭 예측 절편 (px)
ROD_W_MIN_RATIO = 0.5          # 예측폭 대비 최소 비율. 미만이면 '너무 가늘다' → 제외
ROD_W_CHECK_FROM = 0.45        # 이 행비율보다 가까운 것만 검사(먼 것은 폭이 작아 무의미)
# ★ 채움률(면적 / w·h) 하한 — **바닥에서 휘어진 케이블**을 거른다 (2026-09-02 실측).
#   실주행 중 오탐: 세로로 놓인 케이블이 바닥에서 휘면서 바운딩 박스가
#   w=51 h=86 이 됐다. 예측폭 51.1과 **비율 1.00** — 원근 폭 검사를 그대로 통과했다.
#   (앞서 막은 '곧게 뻗은 세로 케이블'과 달리, 휘면 박스가 봉만큼 넓어진다.)
#   갈라주는 건 **박스가 얼마나 차 있느냐**다 — 휜 케이블은 박스 안의 가는 곡선이다:
#       휜 케이블 0.09  /  세로 케이블 0.24~0.35  /  레벨봉 0.30~0.67
#   0.25면 이번 오탐(0.09)을 확실히 걸러내고 봉 최소(0.30)는 여유 있게 통과한다.
#   ⚠ 폭 검사와 **둘 다** 필요하다: 곧은 케이블은 폭으로, 휜 케이블은 채움률로 걸린다.
ROD_MIN_FILL = 0.25

# ── 측면(좌/우) 카메라용 값 — 전방과 색 재현·형상이 다르다 (2026-09-02 실측) ──
#   좌측 Orbbec Gemini 305 (848x530, 180° 뒤집혀 장착):
#     레벨봉 채도 174~211(중앙 200) / 철근 52~78 / 방수포 60  → 3배 차이로 잘 갈린다
#     ⚠ 봉 색상이 16~23으로 전방보다 주황 쪽 → 하한을 12로 낮춰야 다 잡힌다
#     ⚠ 세로/가로: 봉 3개가 1.1 / 1.5 / 1.7 (사용자가 셋 다 봉으로 확인) → 하한 1.0
#        전방(1.5)을 그대로 쓰면 1.1짜리를 놓친다. 대신 케이블뭉치(1.3) 방어가 약해지는데,
#        측면은 **횡이동 경로**라 뭉치든 봉이든 넘어가면 안 되므로 막는 게 맞다.
#     ⚠ 원근 폭 계수는 전방 실측이라 측면에 못 쓴다 → 0으로 꺼둔다(계수 확보 시 켤 것)
ROD_SIDE_HSV_LO = (12, 130, 90)
ROD_SIDE_MIN_AR = 1.0
ROD_SIDE_W_RATIO = 0.0         # 0 = 원근 폭 검사 끔
#     ⚠ 원근 폭 검사를 끄는 대신 **면적 하한**으로 막는다. 측면 실측:
#        진짜 봉  720 / 814 / 1096 / 1211  (멀리 있는데도 이만큼 크다)
#        오검출    40 /  60 /  121 /  220  (철근 반사·색수차)
#        특히 면적 121짜리가 frac 0.900으로 잡혀 **잘못 차단**될 뻔했다.
#        300이면 둘 사이를 넉넉히 가른다.
ROD_SIDE_MIN_AREA = 300


ROD_SEG_MIN_PIXELS = 150   # seg level_rod 덩어리 최소 픽셀(노이즈 컷)


def level_rod_seg(mask, min_pixels=ROD_SEG_MIN_PIXELS,
                  w_min_ratio=ROD_W_MIN_RATIO):
    """★ [2026-09-14] seg 의 `level_rod` 클래스로 레벨봉 근접도를 낸다.

    반환값은 색 기반 `level_rods()` 와 **같은 지표**다 — 가장 가까운 봉의
    밑동 y / H (0~1, 클수록 가깝다). 같은 척도여야 둘을 max 로 OR 할 수 있다.

    ## 왜 둘 다 쓰나 (2026-09-14 결정)
    · 색 기반: 실물시험으로 오검출 방어 4겹을 검증했다(판자·케이블뭉치·세로케이블).
      다만 **좌측 카메라에서는 못 쓴다** — 녹슨 철근이 노랗게 보여 봉(72)보다 배경
      (79)이 더 노랗다. 그래서 "세그 클래스 추가가 정공법"이라고 남겨뒀었다.
    · seg: 그 한계를 푸는 정공법이지만 **데이터가 적어 아직 정확도가 낮다**.
    ⟹ 한쪽만 믿지 않고 **OR**. 어느 쪽이든 잡으면 멈춘다. 놓치는 것보다
       조금 더 자주 멈추는 편이 안전한 쪽이다.

    구 7클래스 모델이면 `IDX_LEVEL_ROD` 가 None 이라 0.0 을 반환한다(색 기반만 동작).
    """
    import cv2
    if IDX_LEVEL_ROD is None:
        return 0.0
    m = (mask == IDX_LEVEL_ROD).astype(np.uint8)
    if int(m.sum()) < min_pixels:
        return 0.0
    H, W = mask.shape[:2]
    n, _lab, stats, _c = cv2.connectedComponentsWithStats(m, 8)
    y_bot = 0
    for i in range(1, n):
        a = int(stats[i, cv2.CC_STAT_AREA])
        if a < min_pixels:
            continue                       # 조각은 버린다
        w = int(stats[i, cv2.CC_STAT_WIDTH])
        h = int(stats[i, cv2.CC_STAT_HEIGHT])
        yb = int(stats[i, cv2.CC_STAT_TOP]) + h
        # ★★ 원근폭 검사 — 색 기반(level_rods)이 쓰는 것과 **같은 규칙**이다.
        #   왜 필요한가 (2026-09-15 실측): 이 함수는 조건에 맞는 덩어리 중
        #   **가장 아래(max y_bot)**를 고른다. 즉 화면 아래쪽 노이즈 하나가
        #   진짜 봉을 제치고 "가장 가까운 봉"이 된다 — 노이즈를 거르는 게 아니라
        #   **증폭하는** 구조였다. 실제로:
        #       #1 진짜 봉  25x34 302px  y=248 → 0.41
        #       #2 그림자   15x15 152px  y=373 → 0.62  ← 이게 이겨서 STOP
        #   #2는 파란 방수포 위 철근받침 **그림자**로, 노랗지도 봉 모양도 아니었다.
        #   주행 중엔 이런 덩어리가 더 아래에 생겨 0.66~0.80까지 튀었고
        #   헤딩 영점 시험을 3번 연속 중단시켰다(그때 색 기반은 0.00이었다).
        #   원근폭(w ≈ 0.217y − 40.3, 전방 실측 R²0.99)은 "화면 아래일수록 굵게
        #   보여야 한다"는 기하 제약이라 그림자·잡덩어리를 깨끗하게 거른다.
        #   ⚠ 계수는 **전방 카메라 실측**이다. 측면 카메라는 계수가 없어
        #     w_min_ratio=0 으로 꺼서 호출한다(색 기반과 같은 처리).
        if w_min_ratio > 0 and yb >= H * ROD_W_CHECK_FROM:
            sc = W / 960.0                              # 계수는 960폭 기준
            w_exp = (ROD_W_SLOPE * (yb / (H / 600.0)) + ROD_W_INTER) * sc
            if w_exp > 0 and w < w_exp * w_min_ratio:
                continue
        # 채움률 — 박스만 크고 속이 빈 것(휜 케이블 등) 제외. 색 기반과 동일.
        if yb >= H * ROD_W_CHECK_FROM and a < ROD_MIN_FILL * w * h:
            continue
        y_bot = max(y_bot, yb)
    return float(y_bot) / float(H)


def level_rods(img_bgr, mask, anchor_frac=ANCHOR_FRAC,
               hsv_lo=None, min_ar=None, w_min_ratio=None, min_area=None):
    """노란 레벨봉 검출. [(cx, y_bottom, area), ...] 아래(가까운 것)부터.

    ★ 판정은 **두 근거 중 하나**면 인정한다:
      ① 밑동이 있는 **행**에 배근이 있다 — 멀리 있는 봉. 봉이 자기 발밑을 가려도
         같은 행 좌우로 철근이 보인다.
      ② 발밑 영역(화면 하단 `anchor_frac`) 안에 있다 — 가까운 봉.

    ⚠ ②가 **반드시** 필요하다. 처음에 ①만으로 만들었더니 **코앞의 봉을 놓쳤다**
      (2026-09-02 실측: y_bot=536인 봉이 자기가 꽂힌 철근을 가려 '배근 0%'로 나옴).
      정작 멈춰야 할 순간에 못 잡는 필터였다.
      ②의 근거: 카메라가 데크를 내려다보므로 발밑에 있는 노란 물체는 배경일 수 없다.
    """
    import cv2
    # ★ 카메라마다 색 재현이 달라 임계를 따로 준다 (2026-09-02 실측).
    #   같은 레벨봉인데 전방 채도 131 / 좌측 200. 배경도 다르다:
    #     전방 — 나무판자 44 (봉과 3배 차)
    #     좌측 — 철근 52~78 (봉과 3배 차, 하지만 절대값이 전방과 다름)
    #   그래서 하나의 임계로는 못 맞춘다. 호출자가 카메라별 값을 넘긴다.
    lo = np.array(hsv_lo if hsv_lo is not None else ROD_HSV_LO)
    min_ar = ROD_MIN_AR if min_ar is None else min_ar
    w_min_ratio = ROD_W_MIN_RATIO if w_min_ratio is None else w_min_ratio
    min_area = ROD_MIN_AREA if min_area is None else min_area
    H, W = mask.shape
    if img_bgr.shape[:2] != (H, W):
        img_bgr = cv2.resize(img_bgr, (W, H))
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    hi = np.array(ROD_HSV_HI)
    ymask = cv2.inRange(hsv, lo, hi)
    # 세로로 긴 커널 — 봉이 철근에 가려 끊긴 부분을 잇는다
    ymask = cv2.morphologyEx(ymask, cv2.MORPH_CLOSE,
                             cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9)))
    n, _lab, stats, cent = cv2.connectedComponentsWithStats(ymask, 8)
    rows_ok = rebar_profile(mask) >= ROW_THR
    anchor = int(H * (1.0 - anchor_frac))
    out = []
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if a < min_area or h < ROD_MIN_H or h / max(w, 1) < min_ar:
            continue
        yb = min(H - 1, y + h)
        # ③ 원근 폭 검사 — 가까운데 가늘면 봉이 아니다(세로 케이블 등).
        #   먼 것은 예측폭 자체가 작아 판별력이 없으므로 가까운 것만 본다.
        # w_min_ratio=0 이면 이 검사를 끈다 — 원근 계수는 **전방 카메라 실측**이라
        # 다른 카메라(측면)에는 그대로 쓸 수 없다. 측면은 아직 계수가 없어 끈다.
        # ④ 채움률 — 휜 케이블처럼 박스만 크고 속은 빈 것을 거른다.
        if yb >= H * ROD_W_CHECK_FROM and a < ROD_MIN_FILL * w * h:
            continue
        if w_min_ratio > 0 and yb >= H * ROD_W_CHECK_FROM:
            sc = W / 960.0                      # 계수는 960폭 기준
            w_exp = (ROD_W_SLOPE * (yb / (H / 600.0)) + ROD_W_INTER) * sc
            if w_exp > 0 and w < w_exp * w_min_ratio:
                continue
        lo_r, hi_r = max(0, yb - 8), min(H, yb + 12)
        row_ok = rows_ok[lo_r:hi_r].mean() if hi_r > lo_r else 0.0
        if row_ok >= ROD_ROW_FRAC or yb >= anchor:
            out.append((int(cent[i][0]), int(yb), int(a)))
    out.sort(key=lambda t: -t[1])          # 화면 아래 = 가까운 것 먼저
    return out


def band_features(mask):
    """전방 밴드별 클래스 비율 신호 (다중 주행불가 판단용).
    반환: vreb_mid/far(주행방향 철근), hreb_mid, nondeck_mid, obs_nearmid."""
    H, W = mask.shape
    c0, c1 = int(W * CENTER[0]), int(W * CENTER[1])

    def frac(cls, band):
        r0, r1 = int(H * band[0]), int(H * band[1])
        sub = mask[r0:r1, c0:c1]
        return float(np.isin(sub, cls).mean()) if sub.size else 0.0

    return {
        'vreb_mid': frac([IDX_V], BANDS['mid']),   # 주행방향 철근 전방
        'vreb_far': frac([IDX_V], BANDS['far']),   # 주행방향 철근 먼전방
        'hreb_mid': frac([IDX_H], BANDS['mid']),
        'nondeck_mid': frac(NONDECK, BANDS['mid']),
        'obs_nearmid': frac(OBSTACLE, (BANDS['mid'][0], BANDS['near'][1])),
    }


def rebar_profile(mask):
    """행별 철근(rebar_h|rebar_v) 비율 r(y) — 중앙컬럼, 수직 스무딩."""
    H, W = mask.shape
    c0, c1 = int(W * CENTER[0]), int(W * CENTER[1])
    r = np.isin(mask[:, c0:c1], REBAR).mean(axis=1).astype(np.float32)
    k = max(3, (int(H / SMOOTH_ROWS) | 1))                       # 홀수 커널
    return np.convolve(r, np.ones(k, np.float32) / k, mode='same')


def rebar_edge(mask):
    """★ 주(主)판정: seg 마스크 → (edge_row, rebar_frac, on_rebar).

    발밑 지지행에서 위로 스캔하며 배근이 이어지는 최상단 행을 찾는다.
    · edge_row  : 배근이 끊기는 행 (여기부터 위는 주행불가)
    · rebar_frac: (H-edge_row)/H — 배근이 이어진 세로범위. 클수록 갈 길이 남음
    · on_rebar  : 발밑(하단 ANCHOR_FRAC)에 배근이 있는가 (False=배근 이탈→즉시정지)
    """
    H = mask.shape[0]
    r = rebar_profile(mask)
    sup = r >= ROW_THR                                           # 행별 배근 유무
    idx = np.flatnonzero(sup)
    anchor_min = int(H * (1.0 - ANCHOR_FRAC))                    # 이 행보다 아래여야 발밑
    if idx.size == 0 or idx[-1] < anchor_min:
        return H, 0.0, False                                     # 발밑에 배근 없음
    gap_max = max(1, int(H * GAP_FRAC))
    top, run = int(idx[-1]), 0
    for y in range(int(idx[-1]), -1, -1):                        # 발밑 → 위로
        if sup[y]:
            top, run = y, 0
        else:
            run += 1
            if run > gap_max:                                    # 배근 끊김 = 데크끝
                break
    return top, (H - top) / H, True


def deck_edge(mask):
    """(구버전·비교용) background 포함 deck 비율 → (edge_row, deck_frac, nondeck_band).
    ⚠ 방수포를 주행가능으로 보므로 실주행 판정에는 쓰지 말 것. 분석 CSV 비교기록용."""
    H, W = mask.shape
    c0, c1 = int(W * CENTER[0]), int(W * CENTER[1])
    deck = np.isin(mask, DECK_LEGACY).astype(np.float32)[:, c0:c1]
    cum = np.cumsum(deck[::-1], axis=0)[::-1]                    # cum[y]=sum deck[y:H]
    cnt = np.arange(H, 0, -1).reshape(-1, 1)                     # (H-y)
    frac = cum / cnt
    dom = frac >= FRAC_THR
    has = dom.any(axis=0)
    boundary = np.where(has, np.argmax(dom, axis=0), H)
    edge_row = float(np.median(boundary))
    deck_frac = (H - edge_row) / H
    b0, b1 = int(0.40 * H), int(0.75 * H)
    nd = float(np.isin(mask[b0:b1, c0:c1], NONDECK).mean())
    return edge_row, deck_frac, nd


COLOR = {'GO': (0, 200, 0), 'SLOW': (0, 165, 255), 'STOP': (0, 0, 255)}


class VerdictFSM:
    """스무딩(중앙값)+히스테리시스 판정. STOP은 rebar_frac가 slow 위로 회복될 때만 해제
    (경계 근처 STOP/GO 깜빡임 방지). 실제 주행에선 STOP=전진종료→측면전환.

    hard_stop: 발밑 배근 이탈(on_rebar=False)·전방 장애물 등 즉시정지 신호.
               스무딩을 우회해 그 프레임에 바로 STOP (지연 없이)."""

    def __init__(self, stop=STOP_THR, slow=SLOW_THR, smooth=5,
                 release_frames=OBS_RELEASE_FRAMES):
        from collections import deque
        self.stop, self.slow = stop, slow
        self.buf = deque(maxlen=smooth)
        self.state = 'GO'
        # hard_stop 래치: 한 번 걸리면 '확실히 깨끗'한 프레임이 연속으로 쌓여야 풀린다.
        self.release_frames = release_frames
        self.hard_latched = False
        self._clean_run = 0

    def step(self, rebar_frac, hard_stop=False, hard_clear=None):
        """hard_stop  : 즉시 정지 조건(진입). True면 그 프레임에 바로 STOP.
        hard_clear : '확실히 해제해도 되는가'. None이면 `not hard_stop`으로 본다
                     (구버전 호출 호환). 진입보다 **엄격한** 조건을 넣어야
                     임계 경계에서 채터링이 안 난다."""
        self.buf.append(rebar_frac)
        s = sorted(self.buf)[len(self.buf) // 2]        # 중앙값 스무딩

        if hard_clear is None:
            hard_clear = not hard_stop
        if hard_stop:
            self.hard_latched = True
            self._clean_run = 0
        elif self.hard_latched:
            # 진입 임계와 해제 임계 사이(회색지대)면 카운트를 늘리지 않고 래치 유지.
            self._clean_run = self._clean_run + 1 if hard_clear else 0
            if self._clean_run >= self.release_frames:
                self.hard_latched = False
                self._clean_run = 0

        if self.hard_latched:                           # 배근이탈/장애물 = 즉시
            self.state = 'STOP'
        elif self.state == 'STOP':
            if s > self.slow:                           # 확실히 배근 복귀 시만 해제
                self.state = 'GO'
        elif s < self.stop:
            self.state = 'STOP'
        else:
            self.state = 'SLOW' if s < self.slow else 'GO'
        return self.state, s, COLOR[self.state]


def judge(mask, fsm, obs_thr=OBS_THR, obs_release_thr=OBS_RELEASE_THR,
          extra_hard=False, extra_reason=''):
    """마스크 한 장 → 판정 dict. 노드/도구 공용 진입점.

    반환: {verdict, rebar_frac(스무딩), rebar_frac_raw, edge_row, on_rebar,
           obs_nearmid, hard_stop, reason}
    """
    edge_row, rebar_frac, on_rebar = rebar_edge(mask)
    bf = band_features(mask)
    obs = bf['obs_nearmid']
    # extra_hard: 마스크 밖에서 판정한 즉시정지 사유(레벨봉 근접 등).
    #   ★ **verdict에 반영해야 한다.** block 토픽에만 넣으면 drive_controller는
    #     막지만 rebar_drive는 모른 채 계속 명령해 max_step_sec 초과로 ABORT 난다
    #     (방향전환/횡이동으로 안 넘어간다). FSM에 hard로 먹여야 STOP이 흐른다.
    hard = (not on_rebar) or obs > obs_thr or bool(extra_hard)
    # 해제는 더 낮은 임계에서만 인정한다(슈미트 트리거). 0.04~0.06 사이는
    # '아직 모르겠다' 구간이라 래치를 유지한다 — 여기서 뒤집히던 게 채터링이었다.
    clear = on_rebar and obs < obs_release_thr and not extra_hard
    verdict, sm, _ = fsm.step(rebar_frac, hard, clear)
    # ⚠ reason은 **래치 상태**를 반영해야 한다. hard가 내려가도 래치가 살아 있으면
    #   여전히 STOP인데 reason이 비어 있으면 "왜 멈춰 있는지" 알 수 없다.
    latched = getattr(fsm, 'hard_latched', hard)
    reason = ('' if not latched else
              (extra_reason if extra_hard and extra_reason else
               ('배근이탈' if not on_rebar else f'장애물 {obs:.2f}')))
    return {
        'verdict': verdict,
        'rebar_frac': round(float(sm), 3),
        'rebar_frac_raw': round(float(rebar_frac), 3),
        'edge_row': int(edge_row),
        'on_rebar': bool(on_rebar),
        'obs_nearmid': round(float(obs), 3),
        'hard_stop': bool(hard),
        'reason': reason,
    }
