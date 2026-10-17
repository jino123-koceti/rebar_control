#!/usr/bin/env python3
"""카메라 좌표 → **그 자세에서 보낼 스테이지 위치(mm)** 를 실측으로 뜬다.

## 무엇을 푸는가

`crossing_detector` 는 교차점을 **카메라 좌표(mm)** 로 낸다. 결속하려면 "그 점에
건 끝을 두려면 스테이지를 어디로 보내야 하나" 를 알아야 한다.

    stage_mm = A · P_cam + b        A 는 3x3, b 는 3  → 미지수 12개 (자세당)

⚠ **장비(하부체·횡이동)를 옮겨도 이 변환은 유효하다.** 카메라와 스테이지가 둘 다
상부체에 달려 있어 서로의 관계가 변하지 않기 때문이다 — 장비가 움직이면 교차점의
카메라 좌표와 보낼 스테이지 위치가 **함께** 바뀐다. 그래서 여러 위치에서 모은
짝을 섞어 써도 된다 (오히려 카메라 좌표가 넓게 퍼져 변환이 단단해진다).

⚠ 다만 **고정해 둔(frozen) 좌표는 장비가 움직이면 무효다.** 같은 교차점이 다른
카메라 좌표로 보이므로 `s` 로 다시 고정해야 한다. `r` 이 기록 전에 그 좌표에
지금도 점이 있는지 확인해 막는다.

[오판 기록] 2026-10-03 에 스냅샷 두 장의 카메라 x 가 -101mm 움직이고 같은 기간
스테이지 Y 가 +95mm 움직인 것을 보고 "카메라가 캐리지에 달려 있다" 고 결론 냈다.
**우연의 일치였다** — 카메라 x 이동은 횡이동 1회 때문이었고 Y 이동은 그 사이
리모콘 조작이었다. 인과를 확인하지 않고 비율만 보고 판단한 것이 잘못이었다.
확인하려면 **한 축만** 움직이고 나머지는 고정한 채 비교해야 한다.

⚠⚠ **자세마다 따로 뜬다.** 건이 yaw 축에 달려 있어 자세를 바꾸면 건 끝이 X·Y 로
움직인다. 실측 오프셋이 100~130mm 다 — 하나로 뭉치면 그만큼 틀린다.

⚠ 이것은 **좌표계 변환이 아니다.** 같은 세상의 점이라도 자세가 다르면 보낼
스테이지 위치가 다르다. 그래서 결과를 "stage 좌표" 라 부르지 않고 **"그 자세에서
보낼 위치"** 라 부른다.

## 왜 mm 인가 (각도가 아니라)

옛 버전은 **절대 모터 각도**로 떴고 "호밍 원점이 바뀌면 다시 떠야 한다" 는 경고가
붙어 있었다. 멀티턴은 전원에 날아가므로 절대 각도는 전원 세션 안에서만 산다.
mm 는 **원점 기준**이라 재호밍·전원 재투입을 견딘다 (원점 반복오차 ~0.4mm 만 남는다).
`/tying/goal` 과 자세별 가동범위·사분면 규칙도 전부 mm 라 그대로 맞물린다.

`/stage/status` 의 `current_mm` 을 그대로 쓴다 — `stage_node` 가 이미 원점과
`mm_per_deg` 로 환산해 발행한다. 여기서 또 환산하면 두 곳이 어긋난다.

## 절차

  1. 호밍 (`/homing_cmd` → `all`). 원점이 없으면 mm 가 안 나온다
  2. 이 도구를 띄운다. 현재 자세가 **판별되어 있어야** 한다
     (안 되면 `/stage/yaw_declare` 로 알려줄 것 — 1번·4번은 단회전이 모호하다)
  3. `s` — 지금 보이는 교차점을 번호와 함께 고정한다
  4. **리모콘으로 건 끝을 그 점에 맞춘다** (X·Y·Z 다)
  5. `r <번호>` — 그 순간의 스테이지 mm 를 짝으로 기록한다
  6. 3~5 를 **자세당 최소 4점, 권장 6점** 반복
  7. 리모콘 **S23/S24** 로 다음 자세로 옮기고 3~6 반복 (1~4번 전부)
  8. `f` — 자세별로 최소제곱을 풀고 yaml 로 저장한다

⚠ 점들이 **한 줄로 늘어서면 안 된다.** 시야 안에서 좌우·앞뒤로 흩어진 점을 고를 것.
⚠ **깊이(z)도 흩어져야 한다** — A 가 3x3 이라 z 가 거의 같으면 그 열이 결정되지
  않는다. 철근 배근이 기울어 있어 보통은 349~624mm 로 충분히 퍼진다.
⚠ 4점은 미지수와 같은 수라 잔차가 0 으로 나와 정확도 확인이 안 된다.

명령: s=고정  g <n>=예측위치로 X·Y 이동  r <n>=기록  rxy <n>=X·Y만 기록
      p <n>=예측검증  l=목록  u=마지막취소  f=산출·저장  q=종료
"""

import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from rebar_base_interfaces.msg import RebarGrid

OUT = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/stage_camera.yaml')
# 짝을 모으는 중간 상태. **한 명령씩 따로 띄워도 이어지도록** 파일에 남긴다
# (`--cmd` 비대화 모드용). 대화 모드에서도 같은 파일을 쓴다 — 중간에 끊겨도
# 다시 띄우면 이어서 할 수 있다. 자세당 6점을 손으로 맞추는 일이라 날리면 아깝다.
STATE = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/stage_camera_pairs.json')
# 장비가 실제로 쓰는 모델 — 예측은 **이것** 기준이어야 보정량이 뜻을 갖는다
MODEL = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/stage_camera.yaml')
# 고정할 때마다 **번호가 찍힌 그림**을 남긴다. 숫자만 보고는 어느 교차점인지
# 알 수 없어 사람이 건 끝을 맞출 수 없다. 그림과 번호가 **같은 스냅샷**에서
# 나와야 어긋나지 않으므로 도구가 직접 그린다 (따로 그리면 정렬 순서가 다르다).
SNAP = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/snapshot.png')
AXES = ('x', 'y', 'z')

# ⚠ 깊이를 X·Y 적합에 쓸지. **False 가 기본** — 위 `_solve_axis` 주석 참고.
#   되돌리려면 True 로만 바꾸면 된다 (짝 데이터는 그대로 쓴다).
# ⚠⚠ [2026-10-15 재결정] **깊이를 쓴다.** 끄고 쓰던 이유는 "깊이 열이 중복돼
#   최소제곱이 아무 값이나 고른다" 였는데, 그건 짝이 적던 때 이야기다.
#   오늘 짝 47개로 다섯 모델을 LOO 비교한 결과 (평균 / 중앙값 mm):
#     깊이사용·공통 12.0/10.4  ·  깊이사용·독립 13.3/10.9  ·  깊이열0·공통 14.1/11.7
#     깊이열0·독립 14.6/13.6   ·  깊이제거(화소비례)·독립 20.2/20.0
#   이 장비는 카메라가 비스듬해 **스테이지 X 와 깊이의 상관이 -0.89** 다 —
#   X 를 맞추는 정보가 깊이에 들어 있어서, 빼면 X 가 무너진다 (실측: 학습
#   깊이대 밖인 370mm 점에서 ΔX +48mm).
USE_DEPTH = True
# ⚠⚠ [사용자 지시 2026-10-15] **자세별 독립 모델을 기준으로 쓴다** (2차년도 방식).
#   공통 변환 + 자세별 오프셋은 "자세마다 A 가 같다" 를 가정하는데, 건·지그가 요와
#   같이 돌아 그 가정이 깨진다. 2차년도에도 좌/우 자세를 따로 처리했다.
#   ⚠ 독립은 자세당 미지수 6개다 — **자세당 6점 이상** 받아야 과적합을 면한다.
#     모자란 자세는 공통 모델로 적어 둔다 (yaml 의 source 로 구분된다).
MODEL_SOURCE = 'shared'            # 독립은 자세당 점이 더 모여야 한다 (위 비교)

# ⚠⚠ **평면이 바뀌면 옛 짝은 무효다 — 날짜로 끊는다.**
#   깊이빠진 모델은 "교차점이 어느 평면에 있다" 를 전제로 cx·cy 만으로 X·Y 를
#   낸다. 그래서 **그 평면 전용**이다. 데크플레이트를 깔기 전(2026-10-04)의 26짝은
#   같은 cx·cy 가 다른 물리점이라, 섞으면 모델을 평면 사이로 끌어당긴다 —
#   실제로 그러자 1번 자세 잔차가 1.83mm → 12.21mm 로 나빠졌다.
#   `archive` 는 전부 비우므로 쓸 수 없다(쓸 만한 10-07 이후 짝까지 날아간다).
#   지우지도 않는다 — 사람이 리모콘으로 맞춘 기록이라 평면이 되돌아오면 되살린다.
#   ⚠ 데크플레이트를 치우거나 배근 높이가 바뀌면 **이 날짜를 다시 잡아야 한다.**
PLANE_SINCE = '2026-10-15 00:00:00'   # [2026-10-15] 카메라가 움직였다 — 10-08 짝과 오늘 짝을 섞으니 X 잔차 2.58→11.84mm 로 악화. 오늘 것만 쓴다

# ⚠⚠ [2026-10-07 사용자 결정] **Z 는 자세와 무관하게 이 상수다.**
#   "데크플레이트 위 철근은 평면으로 봐도 되니까 하강깊이는 어디서나 같다."
#   그래서 Z 를 **적합하지 않는다.** 적합에 맡기면 자세마다 Z 오프셋이 생기는데
#   (2026-10-08: 2번이 -56.64, 나머지가 -58.80 으로 갈렸다) 그 차이는 물리가
#   아니라 **사람이 눈으로 맞춘 하강 깊이의 측정오차**다 — yaw 회전은 높이를
#   바꾸지 않으므로 자세별로 달라질 이유가 없다. 노이즈를 모델에 굳히는 셈이다.
#   잔차는 그대로 보고한다(상수가 실측과 얼마나 다른지 봐야 하므로).
#   ⚠ 데크플레이트를 치우거나 배근 높이가 바뀌면 **이 값부터 다시 잡아야 한다.**
Z_CONST = -70.0   # [사용자 결정 2026-10-16] -85 → -70 (시연 당일, 하강 깊이 축소)
#   이력: -58 → -62 → -67 → -72 → -85 → -70

# ⚠⚠ [사용자 지시 2026-10-12] **손으로 넣는 X·Y 보정.** 실장비에서 교차점 위치가
#   한쪽으로 치우쳐 보일 때, 짝을 다시 받지 않고 바로 미는 값이다.
#   X 범위는 -5.0~408.7mm 이므로 **+가 Xmax 방향**이다.
#   짝(`pairs.json`)에는 손대지 않는다 — 적합 결과 위에 **더하기만** 한다.
#   그래서 잔차 보고는 이 값을 반영하지 않는다 (적합이 아니라 사람이 준 치우침이다).
#   ⚠⚠ **이 값이 걸린 뒤에 `rxy` 로 새 짝을 받으면 이중 보정이 된다** — 그 짝의
#     스테이지 위치는 보정된 자리에서 맞춘 것이라 적합이 같은 치우침을 또 흡수한다.
#     새 짝을 받을 거면 **먼저 이 값을 0 으로 돌리고** 받아야 한다.
XY_OFFSET_MM = (0.0, 0.0)       # [2026-10-15 23:2x] 짝 70개로 재적합 — 손보정 0 으로 내린다.
# 뷰·깊이마다 부호가 바뀌며 오프셋을 쫓아다녔다 (공통 -34.3 → -10.9, 자세3 +10.9 등).
# 깊이 452 대 점들이 일관되게 음수(-5.9~-14.7)여서 **깊이 계수 문제**로 본다 —
# 평행이동으로 덮을 게 아니라 데이터로 A 를 다시 잡아야 한다.

XY_OFFSET_BY_POSE_MM = {
    # ⚠⚠⚠ [2026-10-16 06:40] **주행하면 이 값이 통째로 바뀐다 — 36mm 나.**
    #   같은 아침에 650mm 전진 전후로 네 자세의 ΔX 가
    #     전진 전: -13.2 / -14.4 / -12.2 / -13.9
    #     전진 후: +22.4 / +24.4 / +17.7 / +17.5
    #   으로 **부호까지 뒤집혔다.** 깊이 369~564mm 에 걸쳐 ΔX 가 같았으니
    #   깊이 탓이 아니라 자세 무관한 평행이동이다.
    #   카메라와 건은 둘 다 로봇 위에 있어 **주행만으로는 cam→stage 변환이
    #   바뀔 수 없다.** 그러니 물리적으로 뭔가 움직인 것이다. 후보 둘:
    #     ① 지그·건 체결이 주행 진동에 밀린다 (사용자가 전부터 의심한 쪽)
    #     ② 데크가 고르지 않아 로봇 피치가 바뀐다 — 건은 마스트 끝이라
    #        0.5° 만 기울어도 X 로 ~17mm 움직인다. 카메라는 덜 움직인다.
    #   **구분법**: 650mm 후진해서 제자리로 돌아왔을 때
    #     오프셋이 -13 쪽으로 돌아오면 ② (자세 의존, 재현됨)
    #     안 돌아오면 ① (누적 슬립) — 이 경우 체결을 조여야 한다.
    #   ⚠ 어느 쪽이든 **주행이 끼는 시나리오에서는 한 번 잡은 오프셋이
    #     다음 정차지에서 유효하지 않다.**
    1: (-1.1, -1.4),   # 전진 전 누계 (-23.5,-6.8) + 전진 후 (+22.4,+5.4)
    2: (+7.2, -1.1),   # 전진 전 누계 (-17.2,-8.8) + 전진 후 (+24.4,+7.7)
    3: (-5.8, -6.9),   # 전진 전 누계 (-23.5,-5.8) + 전진 후 (+17.7,-1.1)
    4: (-9.4, -9.6),   # 전진 전 누계 (-26.9,-9.3) + 전진 후 (+17.5,-0.3)
}



class Calib(Node):
    def __init__(self):
        super().__init__('stage_camera_calib')
        self.stage = None
        self.grid = None
        self.create_subscription(String, '/stage/status', self._on_stage, 10)
        self.create_subscription(RebarGrid, '/rebar/crossings', self._on_grid, 10)
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self.color = None
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._on_color, qos_profile_sensor_data)
        self.trig = self.create_publisher(Empty, '/rebar/detect', 10)
        # ⚠ 예측 위치로 **X·Y 만** 보낸다 (`g <n>`). Z 는 NaN 으로 둬서 내리지
        #   않는다 — 캘리 중에 Z 를 내리면 X·Y 가 아직 틀린 상태라 철근을 찍는다
        #   (2026-10-07 에 결속 9점이 전부 그렇게 -48mm 에서 걸렸다).
        from geometry_msgs.msg import Point
        self._Point = Point
        self.goal = self.create_publisher(Point, '/stage/goal', 10)
        self.frozen = []          # 고정한 검출점 [(x,y,z,conf), ...]
        self.pairs = {}           # 자세 → [{cam, stage, snap, gun, idx, t}, ...]
        self.snap_stage = None    # 마지막 스냅샷 당시 스테이지 mm
        self._load_state()

    # ---- 중간 상태 ---------------------------------------------------------
    # 짝 하나: {'cam':[3], 'stage':[3], 'gun':건각도, 'idx':교차점번호, 't':시각}
    # ⚠ **`gun` 이 핵심이다.** 자세 번호만으로는 모자란다 — 자세의 건 각도를
    #   재조정하면(오늘 3번 +6.81→+4.00, 4번 +18.80→+16.69 처럼) 번호는 같은데
    #   실제 각도가 달라져 **예전 짝이 조용히 틀린 데이터가 된다.** 기록해 두면
    #   불러올 때 설정값과 대조해 걸러낼 수 있다.
    def _load_state(self):
        try:
            d = json.load(open(STATE, encoding='utf-8'))
            out = {}
            for k, v in (d.get('pairs') or {}).items():
                rows = []
                for q in v:
                    if isinstance(q, dict):
                        rows.append(q)
                    else:                       # 옛 형식 [cam, stage] — gun 없음
                        rows.append({'cam': q[0], 'stage': q[1], 'gun': None,
                                     'idx': None, 't': None})
                out[int(k)] = rows
            self.pairs = out
            self.frozen = [tuple(f) for f in (d.get('frozen') or [])]
            self.snap_stage = d.get('snap_stage')
        except Exception:
            pass

    # 자세 각도가 이만큼 달라지면 **다른 자세의 데이터**로 본다.
    # ⚠ 건 끝이 yaw 축에서 약 640mm 떨어져 있다 (실측 오프셋 1→2 가 10.44° 에
    #   116mm). 그래서 **건 1° = 약 11mm** 다. 1.0° 로 두면 11mm 틀어진 짝이
    #   그대로 섞인다 — 2026-10-04 에 2번(0.87°)·3번(0.60°) 이 안 잡혔다.
    # ⚠⚠ 그렇다고 **도달 허용오차보다 작게 두면 안 된다.** yaw 도달 판정이
    #   모터축 3°(= 건 0.24°)이므로 정상 도달도 그만큼 벗어난다 — 0.2° 로 두었더니
    #   방금 받은 정상 짝(+4.83° vs 설정 +4.60°)이 즉시 무효로 걸렸다.
    #   0.35°(≈3.9mm) 로 둔다: 도달 오차 0.24° 는 통과하고, 자세 재정의(0.6~1.5°)는
    #   걸린다. 남는 도달 오차는 위치로 최대 2.7mm 다 — ±5mm 예산 안이지만
    #   무시할 수준은 아니다. 더 조이려면 yaw 도달 판정을 먼저 조여야 한다.
    STALE_DEG = 0.35

    def stale(self):
        """설정된 자세 각도와 **기록 당시 각도**가 다른 짝. {자세: [(i, 기록각, 설정각)]}"""
        try:
            from rmd_robot_control.axis_config import load_pose_id
            info = load_pose_id('yaw')
        except Exception:
            return {}
        if not info:
            return {}
        bad = {}
        for p, rows in self.pairs.items():
            want = (info['poses'].get(p) or {}).get('gun')
            if want is None:
                continue
            for i, q in enumerate(rows):
                g = q.get('gun')
                if g is not None and abs(g - want) > self.STALE_DEG:
                    bad.setdefault(p, []).append((i, g, want))
        return bad

    def _save_state(self):
        """⚠ **파일에 있던 다른 키를 덮어쓰지 않는다.** 전에는 dict 를 새로 만들어
        썼더니 `archive`(보관한 짝) 가 다음 저장에서 사라졌다 — 사람이 리모콘으로
        맞춘 15짝이 날아갔다(git 에서 복구했다). 읽어서 **고칠 것만 고친다.**
        """
        os.makedirs(os.path.dirname(STATE), exist_ok=True)
        try:
            d = json.load(open(STATE, encoding='utf-8'))
        except Exception:
            d = {}
        d['pairs'] = {str(k): v for k, v in self.pairs.items()}
        d['frozen'] = [list(f) for f in self.frozen]
        d['snap_stage'] = self.snap_stage
        d['saved'] = time.strftime('%Y-%m-%d %H:%M:%S')
        json.dump(d, open(STATE, 'w', encoding='utf-8'),
                  ensure_ascii=False, indent=1)

    def _on_stage(self, msg):
        try:
            self.stage = json.loads(msg.data)
        except ValueError:
            pass

    def _on_grid(self, msg):
        self.grid = msg

    def _on_color(self, msg):
        self.color = msg

    def stale_idx(self, p):
        """그 자세에서 **무효가 된 짝의 번호**. 자세 각도가 재조정된 뒤의 것들이다."""
        return {i for i, g, w in self.stale().get(p, [])}

    def off_plane(self, p):
        """그 자세에서 **다른 평면에서 받은 짝의 번호** (`PLANE_SINCE` 이전)."""
        return {i for i, q in enumerate(self.pairs.get(p, []))
                if not (q.get('t') or '') or q['t'] < PLANE_SINCE}

    def bad_idx(self, p):
        """사람이 **무효로 표시한** 짝의 번호 (`bad` 가 참인 짝).

        왜 지우지 않고 표시만 하는가: 어느 짝이 왜 틀렸는지가 다음 수집의 근거가
        된다. 실제로 2026-10-07 2번 자세 3짝은 `_on_declare` 가 측정값 대신
        **공칭 각도**(-6.33°)를 적어 넣은 동안 받은 것이라, 실제 건은 -4.92° —
        1.41°(건끝 약 16mm) 어긋난 자리에서 맞춘 짝이었다. 기록이 남아 있어
        '왜 2번만 Y 가 15mm 갈렸나' 를 역추적할 수 있었다.
        """
        return {i for i, q in enumerate(self.pairs.get(p, [])) if q.get('bad')}

    def skip_idx(self, p):
        """적합에서 뺄 짝 번호 — 자세 재정의분 + 다른 평면분 + 무효 표시분."""
        return self.stale_idx(p) | self.off_plane(p) | self.bad_idx(p)

    def rows(self, p):
        """(cam, stage) 짝. **절대 위치**로 푼다 — 카메라와 스테이지가 둘 다
        상부체에 있어 장비가 움직여도 둘 사이 관계가 변하지 않는다.

        `snap`(검출 당시 스테이지 위치)은 수식에 쓰지 않고 기록만 한다 — 나중에
        "이 짝이 어느 상황에서 나왔나" 를 되짚을 때 필요하다.
        """
        bad = self.skip_idx(p)
        return [(q['cam'], q['stage'])
                for i, q in enumerate(self.pairs.get(p, [])) if i not in bad]

    def spin(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---- 현재 상태 ---------------------------------------------------------
    def pose(self):
        return (self.stage or {}).get('pose')

    def mm(self):
        c = (self.stage or {}).get('current_mm') or {}
        v = [c.get(a) for a in AXES]
        return None if any(x is None for x in v) else v

    def show(self):
        s = self.stage or {}
        p, g = s.get('pose'), s.get('gun_deg')
        m = self.mm()
        print(f"  자세 {p if p is not None else '미확인'}"
              f"  건 {g if g is not None else '?'}°"
              f"  위치 " + (f"X {m[0]:.1f} Y {m[1]:.1f} Z {m[2]:.1f} mm" if m else '없음'))
        if p is None:
            print("  ⚠ 자세가 판별되지 않았습니다 — /stage/yaw_declare 로 알려주세요")
        if m is None:
            print("  ⚠ mm 를 못 받습니다 — 호밍이 필요합니다 (/homing_cmd all)")

    # ---- 동작 --------------------------------------------------------------
    def snapshot(self):
        self.grid = None
        # 툴이 막 떴을 때는 디스커버리가 끝나기 전이라 publish 가 유실된다 —
        # 검출기가 /rebar/detect 를 구독하기까지 기다린 뒤에 쏜다
        w = time.time() + 5.0
        while rclpy.ok() and self.trig.get_subscription_count() == 0 and time.time() < w:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.trig.get_subscription_count() == 0:
            print("  ⚠ crossing_detector 가 /rebar/detect 를 구독하지 않습니다")
        self.trig.publish(Empty())
        # RF-DETR Medium 추론은 유휴 뒤 첫 장이 느리다
        t = time.time() + 20.0
        while rclpy.ok() and self.grid is None and time.time() < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.grid is None:
            print("  검출 결과를 못 받았습니다 — crossing_detector 가 떠 있나요?")
            return
        self.frozen = [(d.x, d.y, d.z, d.confidence, d.pixel_u, d.pixel_v)
                       for d in self.grid.detections]
        # 카메라가 캐리지에 달려 있다 — 이 좌표들이 어느 위치에서 본 것인지
        # 남겨야 나중에 다른 위치의 스냅샷과 섞어 쓸 수 있다
        self.snap_stage = self.mm()
        print(f"  검출 당시 스테이지 " + (f"X {self.snap_stage[0]:.1f} "
              f"Y {self.snap_stage[1]:.1f} Z {self.snap_stage[2]:.1f} mm"
              if self.snap_stage else "— mm 없음 ⚠ 이 스냅샷은 쓸 수 없다"))
        print(f"  검출 {len(self.frozen)}개  ({self.grid.error_message})")
        if not self.frozen:
            print("  깊이가 확보된 검출이 없습니다. 시야·조명을 확인하세요.")
        self._save_state()
        for i, (x, y, z, c, u, v) in enumerate(self.frozen):
            print(f"    [{i}] 카메라 ({x:+8.1f}, {y:+8.1f}, {z:8.1f}) mm  "
                  f"픽셀 ({u:4d},{v:4d})  신뢰도 {c:.2f}")
        self._draw()

    def _draw(self):
        """번호가 찍힌 그림을 남긴다. 번호는 위 목록과 **같은 순서**다."""
        if self.color is None or not self.frozen:
            print("  (그림을 못 그렸습니다 — 컬러 영상 없음)")
            return
        try:
            import cv2
            import numpy as np
            c = self.color
            img = np.frombuffer(c.data, dtype=np.uint8).reshape(
                c.height, c.width, 3)[:, :, ::-1].copy()
            # 깊이 층을 색으로 구분한다 — z 가 퍼진 점을 고르게 돕는다
            zs = [f[2] for f in self.frozen]
            lo, hi = min(zs), max(zs)
            for i, (x, y, z, cf, u, v) in enumerate(self.frozen):
                t = 0.0 if hi - lo < 1 else (z - lo) / (hi - lo)
                col = (int(60 + 195 * t), 220, int(255 - 195 * t))   # 가까움→멀음
                cv2.circle(img, (u, v), 18, col, 3)
                cv2.circle(img, (u, v), 2, (0, 0, 255), -1)
                cv2.putText(img, str(i), (u - 9, v + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
                cv2.putText(img, str(i), (u - 9, v + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
                cv2.putText(img, f"{z:.0f}", (u + 22, v + 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
            s = self.stage or {}
            hdr = (f"pose {s.get('pose')}  gun {s.get('gun_deg')}deg   "
                   f"{len(self.frozen)} crossings   z {lo:.0f}-{hi:.0f}mm   "
                   f"(number = index, small = camera z)")
            cv2.rectangle(img, (0, 0), (c.width, 32), (0, 0, 0), -1)
            cv2.putText(img, hdr, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)
            os.makedirs(os.path.dirname(SNAP), exist_ok=True)
            cv2.imwrite(SNAP, img)
            print(f"  그림: {SNAP}")
        except Exception as e:
            print(f"  (그림 저장 실패: {e})")

    def _scene_moved(self, tol=8.0, need=0.5):
        """장면이 움직였는가. (움직였다, 설명).

        ⚠⚠ **목표 교차점 자리를 확인하면 안 된다.** 결속 위치에 건을 내려놓으면
        그 교차점은 **건에 가려진다** — 기록하는 순간엔 원리적으로 안 보인다.
        2026-10-04 에 그래서 정상 기록이 "그 좌표에 교차점이 없다(125mm)" 로 막혔다.

        대신 **지금 보이는 점들이 고정해 둔 집합과 맞는지** 본다. 장면이 그대로면
        가려지지 않은 점들은 제자리에 있다. 가림으로 수가 줄어드는 것은 정상이므로
        **몇 개가 맞는지가 아니라 맞는 것들이 얼마나 가까운지**로 판단한다.
        """
        self.grid = None
        self.trig.publish(Empty())
        t = time.time() + 5.0
        while rclpy.ok() and self.grid is None and time.time() < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.grid is None:
            return False, "새 검출을 못 받았습니다 — 확인할 수 없습니다"
        now = [(d.x, d.y, d.z) for d in self.grid.detections]
        if not now:
            return False, "지금 검출이 0개입니다 — 가림으로 확인 불가"
        F = np.array([f[:3] for f in self.frozen], dtype=float)
        ds = [float(np.min(np.linalg.norm(F - np.array(q), axis=1))) for q in now]
        near = sum(1 for d in ds if d <= tol)
        med = float(np.median(ds))
        if near >= max(1, int(len(now) * need)):
            return False, (f"장면 일치 (검출 {len(now)}개 중 {near}개가 {tol:.0f}mm "
                           f"안, 중앙값 {med:.1f}mm)")
        return True, (f"장면이 움직인 것 같습니다 — 검출 {len(now)}개 중 "
                      f"{tol:.0f}mm 안에 맞는 것이 {near}개뿐 (중앙값 {med:.1f}mm). "
                      f"`s` 로 다시 고정하세요")

    def record(self, idx, check=True, axes='xyz', stage=None):
        if not (0 <= idx < len(self.frozen)):
            print(f"  번호가 범위를 벗어났습니다 (0~{len(self.frozen) - 1})")
            return
        if check:
            moved, why = self._scene_moved()
            print(f"  장면 확인: {why}")
            if moved:
                print("  → **기록하지 않았습니다.** 강행하려면 `r! <n>`")
                return
        self.spin(0.8)
        p, m = self.pose(), self.mm()
        if p is None:
            print("  자세를 모릅니다 — 기록하지 않습니다 "
                  "(/stage/yaw_declare 로 알려주세요)")
            return
        if m is None:
            print("  스테이지 mm 를 못 받습니다 — 기록하지 않습니다 (호밍 필요)")
            return
        x, y, z = self.frozen[idx][:3]
        if stage is not None:
            m = list(stage) + [0.0] * (3 - len(stage))
        if self.snap_stage is None:
            print("  검출 당시 스테이지 위치를 모릅니다 — `s` 로 다시 고정하세요")
            return
        self.pairs.setdefault(p, []).append(
            {'cam': [x, y, z], 'stage': m, 'snap': list(self.snap_stage),
             'idx': idx, 'axes': axes, 'gun': (self.stage or {}).get('gun_deg'),
             't': time.strftime('%Y-%m-%d %H:%M:%S')})
        self._save_state()
        print(f"  {p}번 자세 짝 {len(self.pairs[p])} 기록 [{axes}] — "
              f"카메라 ({x:+.1f},{y:+.1f},{z:.1f}) ↔ "
              f"스테이지 (X {m[0]:.1f}, Y {m[1]:.1f}, Z {m[2]:.1f}) mm")

    def archive(self, why=''):
        """지금 짝을 **보관함으로 옮기고** 비운다. 지우지 않는다.

        자세 정의가 바뀌거나 수집 조건이 달라지면 기존 짝을 쓸 수 없다. 그렇다고
        지우면 "어느 각도·조건에서 받았나" 가 사라져 나중에 되짚을 수 없다 —
        사람이 리모콘으로 맞춘 노동의 기록이다. 그래서 옮겨만 둔다.
        """
        if not self.pairs:
            print("  보관할 짝이 없습니다")
            return
        try:
            d = json.load(open(STATE, encoding='utf-8'))
        except Exception:
            d = {}
        arc = d.get('archive') or []
        n = sum(len(v) for v in self.pairs.values())
        arc.append({'at': time.strftime('%Y-%m-%d %H:%M:%S'), 'why': why,
                    'pairs': {str(k): v for k, v in self.pairs.items()}})
        self.pairs = {}
        d['archive'] = arc
        d['pairs'] = {}
        d['frozen'] = [list(f) for f in self.frozen]
        d['snap_stage'] = self.snap_stage
        d['saved'] = time.strftime('%Y-%m-%d %H:%M:%S')
        os.makedirs(os.path.dirname(STATE), exist_ok=True)
        json.dump(d, open(STATE, 'w', encoding='utf-8'),
                  ensure_ascii=False, indent=1)
        print(f"  {n}짝을 보관함으로 옮겼습니다 (보관 {len(arc)}건). 짝 목록은 비었습니다")
        if why:
            print(f"  사유: {why}")

    def mark_bad(self, pose, idxs, why=''):
        """그 자세의 짝 몇 개를 **무효로 표시**한다 (지우지 않는다)."""
        v = self.pairs.get(pose) or []
        hit = []
        for i in idxs:
            if not (0 <= i < len(v)):
                print(f"  {pose}번 짝 {i} 는 범위를 벗어났습니다 (0~{len(v)-1})")
                continue
            v[i]['bad'] = True
            v[i]['bad_why'] = why or '사람이 무효 표시'
            hit.append(i)
        if hit:
            self._save_state()
            print(f"  {pose}번 자세 짝 {hit} 무효 표시 — "
                  f"적합에 쓰는 짝 {len(self.rows(pose))}개")

    def undo(self):
        p = self.pose()
        if p is None or not self.pairs.get(p):
            print("  취소할 짝이 없습니다 (지금 자세 기준)")
            return
        self.pairs[p].pop()
        self._save_state()
        print(f"  {p}번 자세 마지막 짝 취소 — 남은 {len(self.pairs[p])}개")

    def listing(self):
        if not self.pairs:
            print("  기록된 짝이 없습니다")
            return
        for p in sorted(self.pairs):
            rows = self.pairs[p]
            zs = [q['cam'][2] for q in rows]
            ns, no = len(self.stale_idx(p)), len(self.off_plane(p))
            nb = len(self.bad_idx(p))
            tag = ''.join([f" (자세 재정의 {ns}개 제외)" if ns else '',
                           f" (다른 평면 {no}개 제외)" if no else '',
                           f" (무효 표시 {nb}개 제외)" if nb else ''])
            print(f"  {p}번 자세: {len(rows)}개" + tag
                  + (f"  (카메라 z {min(zs):.0f}~{max(zs):.0f}mm)" if rows else ''))
            for i, (c, m) in enumerate(self.rows(p)):
                print(f"     {i} cam({c[0]:+7.1f},{c[1]:+7.1f},{c[2]:7.1f})"
                      f" → stage({m[0]:7.1f},{m[1]:7.1f},{m[2]:7.1f})")

    # ---- 산출 --------------------------------------------------------------
    AXK = {'x': 0, 'y': 1, 'z': 2}

    def _axes_of(self, q):
        """그 짝이 가진 성분 인덱스. 기본은 세 성분 다."""
        return {self.AXK[c] for c in q.get('axes', 'xyz') if c in self.AXK}

    def _solve_axis(self, k, poses, base):
        """성분 k(0=X,1=Y,2=Z) **하나**를 푼다.

        ⚠ 성분마다 **독립**이다 — 미지수가 성분별로 갈린다(a 3개 + b 1개 +
        자세 오프셋). 그래서 **X·Y 만 있는 짝도 섞어 쓸 수 있다.** Z 를 안 내린
        상태에서 X·Y 만 맞춘 점이 그런 경우다 (끝단 시험이 그렇게 나온다 —
        건을 이미 근처에 보낸 뒤 미세 보정만 하므로 정렬 오차가 작아 값이 좋다).
        """
        extra = [p for p in poses if p != base]
        # ⚠⚠ [2026-10-07] **깊이를 X·Y 에서 뺀다. Z 는 상수로 둔다.**
        #   실측이 원인을 정확히 가리켰다 — 1번 자세 3점에서
        #       깊이 401 → X오차 -12.5 / 444 → -14.4 / 534 → -30.8
        #   **X 오차가 깊이에 비례**(기울기 -0.144mm/mm)하는데 Y 오차는 ±2mm 로
        #   깊이와 무관했다. 이유가 행렬에 있다: X 행의 깊이 계수가 +0.0933 인데
        #   Y 행은 -0.0009(~0)다. 즉 **Y 는 이미 깊이를 안 쓰고 있어서 정확했다.**
        #   왜 X 의 깊이 계수가 틀렸나: 교차점이 거의 한 평면(철근 격자)에 있어
        #   깊이가 cx·cy 의 선형함수다 → **깊이 열이 중복**되어 최소제곱이 아무
        #   값이나 고른다. git 로그의 `깊이가 몰려 A 의 z 열이 안 잡혔다` 가 이것.
        #   평면에서는 깊이가 X·Y 결정에 **정보를 더하지 않는다** — 빼는 것이 맞다.
        #   2차년도도 `calibrate_without_depth.py` 로 그렇게 했다.
        # ⚠ Z 는 데크플레이트 위라 **같은 평면**이므로 자세별 상수로 둔다
        #   (사용자 결정). 상수항 + 자세 오프셋만 푼다.
        # ⚠ **yaml 스키마는 그대로다** — X·Y 행의 깊이 열을 0, Z 행을 전부 0 으로
        #   두고 b[2] 에 상수를 넣는다. 그래서 `tying_planner` 는 고칠 것이 없다.
        if k == 2:
            # Z 는 풀지 않는다 (위 `Z_CONST` 주석). 잔차만 재서 보고한다
            e = [q['stage'][2] - Z_CONST
                 for p in poses
                 for i, q in enumerate(self.pairs.get(p, []))
                 if i not in self.skip_idx(p) and 2 in self._axes_of(q)]
            return dict(a=np.zeros(3), b=float(Z_CONST),
                        off={p: 0.0 for p in poses}, n=len(e),
                        e=np.array(e, dtype=float), dof=len(e))
        ncam = 3 if USE_DEPTH else 2
        U = ncam + 1 + len(extra)
        M, y = [], []
        for p in poses:
            bad = self.skip_idx(p)
            for i, q in enumerate(self.pairs.get(p, [])):
                if i in bad or k not in self._axes_of(q):
                    continue
                r = [0.0] * U
                r[0:ncam] = q['cam'][:ncam]
                r[ncam] = 1.0
                if p != base:
                    r[ncam + 1 + extra.index(p)] = 1.0
                M.append(r)
                y.append(q['stage'][k])
        if len(M) < U:
            return None
        M, y = np.array(M), np.array(y)
        sol, _, rank, _ = np.linalg.lstsq(M, y, rcond=None)
        if rank < U:
            return None
        e = M @ sol - y
        off = {base: 0.0}
        off.update({p: float(sol[ncam + 1 + i]) for i, p in enumerate(extra)})
        a3 = np.zeros(3)
        a3[:ncam] = sol[:ncam]
        return dict(a=a3, b=float(sol[ncam]), off=off, n=len(y),
                    e=e, dof=len(y) - U)

    def fit_shared(self):
        """**공통 변환 + 자세별 오프셋.** 성분별로 따로 푼다.

        카메라→세상 변환은 **모든 자세에서 같다** — 카메라가 캐리지에 달려 있지
        않다 (2026-10-04 확인: X 를 240mm 옮겨도 교차점 카메라 좌표가 1mm 만
        변했다). 자세마다 바뀌는 것은 **건 끝 오프셋 하나**뿐이다.

            보낼 위치 = A · P_cam + b + offset(자세)      offset(첫 자세) = 0

        ⚠ **A 의 z 열은 카메라 깊이가 퍼져야 결정된다.** 2026-10-04 에 깊이가
        480~500mm 에 7/11 몰려 있어 z 열이 8% 틀렸다 — 그 깊이에서는 맞고
        (오차 0 지점 534mm) 365mm 에서 16mm 빗나갔다. **잔차는 2.6mm 로 멀쩡해
        보였다.** 그래서 아래에서 깊이 분포를 따로 경고한다.
        """
        # ⚠ 유효 짝이 **0개인 자세는 빼야 한다.** 남겨 두면 그 자세의 오프셋
        #   열이 전부 0 이 되어 랭크가 부족해 전체가 안 풀린다 (2026-10-04 에
        #   4번 자세 짝이 전부 무효가 되자 그렇게 됐다).
        poses = [p for p in sorted(self.pairs) if self.rows(p)]
        if not poses:
            return None
        base = poses[0]
        A = np.zeros((3, 3))
        b = np.zeros(3)
        off = {p: np.zeros(3) for p in poses}
        ns, dofs, errs = [], [], []
        for k in range(3):
            if k == 2:
                # Z 는 상수라 어느 자세에 Z 짝이 있든 상관없다
                pk = poses
            else:
                pk = [p for p in poses
                      if any(k in self._axes_of(q)
                             for i, q in enumerate(self.pairs.get(p, []))
                             if i not in self.skip_idx(p))]
                if base not in pk:
                    return None        # 기준 자세에 그 성분이 없으면 못 푼다
            r = self._solve_axis(k, pk, base)
            if r is None:
                return None
            A[k, :] = r['a']
            b[k] = r['b']
            for p, v in r['off'].items():
                off[p][k] = v
            ns.append(r['n'])
            dofs.append(r['dof'])
            errs.append(r['e'])
        e = np.concatenate(errs)
        return dict(A=A, b=b, off=off, n=sum(ns), dof=min(dofs),
                    pairs=sum(len(v) for v in self.pairs.values()),
                    rms=float(np.sqrt((e ** 2).mean())),
                    worst=float(np.abs(e).max()),
                    per_axis=[(len(x), float(np.sqrt((x ** 2).mean())))
                              for x in errs])

    def fit_one(self, rows):
        """(A 3x3, b 3, rms, worst) 또는 None."""
        if len(rows) < 4:
            return None
        P = np.array([c for c, _ in rows], dtype=float)        # (n,3)
        S = np.array([d for _, d in rows], dtype=float)        # (n,3)
        M = np.hstack([P, np.ones((len(rows), 1))])            # (n,4)
        sol, *_ = np.linalg.lstsq(M, S, rcond=None)            # (4,3)
        A, b = sol[:3, :].T, sol[3, :]
        res = (A @ P.T).T + b - S
        return (A, b,
                float(np.sqrt((res ** 2).sum(axis=1).mean())),
                float(np.abs(res).max()))

    def fit(self, out_path):
        """두 모델을 **다 풀어 비교**하고, 공통 모델을 기준으로 저장한다.

        자세별 독립은 자세당 4점이면 **잔차가 0** 으로 나와 정확도 확인이 안 되고
        과적합하기 쉽다. 공통 모델은 모든 점을 함께 쓰므로 검증 여유가 남는다.
        그래서 **공통 모델을 기준**으로 삼고 독립은 교차검증으로만 쓴다.
        독립이 확실히 더 좋으면(점이 넉넉하고 잔차가 뚜렷이 작으면) 그게 기구에
        공통 모델로 설명 안 되는 것이 있다는 신호다 — 그때 사람이 판단한다.
        """
        bad = self.stale()
        if bad:
            print("\n  ⚠⚠ **자세 각도가 재조정된 뒤의 짝이 섞여 있습니다.**")
            for p, items in sorted(bad.items()):
                for i, g, want in items:
                    print(f"     {p}번 자세 짝 {i}: 기록 당시 건 {g:+.2f}° / "
                          f"지금 설정 {want:+.2f}°")
            print("     그 짝들은 **다른 자세의 데이터**입니다 — "
                  "**피팅에서 자동으로 제외**합니다.")
            print("     파일에는 그대로 남습니다 (어느 각도에서 받았는지 기록이 "
                  "남아야 되살릴 수 있다). 그 자세는 다시 받아야 합니다")
        sh = self.fit_shared()
        print()
        if sh:
            print(f"■ 공통 변환 + 자세별 오프셋 — 짝 {sh['pairs']}개 "
                  f"(식 {sh['n']}개), 성분별 검증 여유 최소 {sh['dof']}")
            print("   성분별 식·잔차: " + "  ".join(
                f"{a}={m}개 {r:.2f}mm" for a, (m, r) in zip('XYZ', sh['per_axis'])))
            print(f"   잔차 RMS {sh['rms']:.2f}mm(성분당)  최대 {sh['worst']:.2f}mm"
                  + ("   ⚠ 미지수=식 이라 0 이 당연 (검증 안 됨)"
                     if sh['dof'] == 0 else ""))
            for p in sorted(sh['off']):
                o = sh['off'][p]
                tag = ' ← 기준' if not o.any() else ''
                print(f"     {p}번 오프셋 ({o[0]:+7.1f}, {o[1]:+7.1f}, {o[2]:+6.1f}) mm{tag}")
            zs = sorted(q['cam'][2] for v in self.pairs.values() for q in v)
            mid = [z for z in zs if 460 <= z <= 520]
            if len(zs) >= 4 and len(mid) > len(zs) * 0.5:
                print(f"   ⚠⚠ 카메라 깊이가 460~520mm 에 {len(mid)}/{len(zs)} 몰려 있습니다 "
                      "— **A 의 z 열이 결정되지 않습니다.**")
                print("      그 깊이에서만 맞고 멀어지면 틀립니다. 잔차로는 안 보입니다 "
                      "(2026-10-04: 365mm 에서 16mm 빗나갔는데 잔차는 2.6mm 였다).")
                print("      깊이 양 끝(348~380, 600~630mm)에서 점을 받으세요")
            zmax = max(abs(o[2]) for o in sh['off'].values())
            if zmax > 10.0:
                print(f"   ⚠ Z 오프셋이 {zmax:.1f}mm 입니다 — yaw 회전은 높이를 바꾸지 "
                      "않아야 합니다. 자세마다 맞추는 기준이 달랐을 수 있습니다")
        else:
            need = 12 + 3 * max(0, len(self.pairs) - 1)
            got = sum(len(v) for v in self.pairs.values()) * 3
            print(f"■ 공통 변환 — 아직 못 풉니다 (식 {got} / 미지수 {need}, "
                  f"또는 점 배치가 겹칩니다)")
        done = {}
        print()
        for p in sorted(self.pairs):
            rows = self.rows(p)
            r = self.fit_one(rows)
            if r is None:
                print(f"■ {p}번 자세 — 짝 {len(rows)}개뿐 (미지수 12개, 최소 4점)")
                continue
            A, b, rms, worst = r
            done[p] = (A, b, rms, worst, len(rows))
            print(f"■ {p}번 자세 — 짝 {len(rows)}개   잔차 RMS {rms:.2f}mm  "
                  f"최대 {worst:.2f}mm")
            zs = [c[2] for c, _ in rows]
            if max(zs) - min(zs) < 30.0:
                print(f"   ⚠ 카메라 z 범위가 {max(zs)-min(zs):.0f}mm 뿐입니다 — "
                      "A 의 z 열이 결정되지 않아 깊이가 다른 점에서 틀립니다")
            if len(rows) == 4:
                print("   ⚠ 4점은 미지수와 같은 수라 잔차가 0 으로 나옵니다 — "
                      "정확도 확인이 안 됩니다")
            elif rms > 10.0:
                print("   ⚠ 잔차가 큽니다. 맞춘 점이 틀렸거나 점이 한 줄로 늘어서 "
                      "있을 수 있습니다")
        if not done and not sh:
            print("  저장할 것이 없습니다")
            return
        usable = set(sh['off']) if sh else set(done)
        missing = [p for p in (1, 2, 3, 4) if p not in usable]
        if missing:
            print(f"\n  ⚠ 아직 쓸 수 없는 자세: {missing} — 공통 모델이라도 그 자세 "
                  "짝이 **1점** 은 있어야 오프셋이 나옵니다")

        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write("# 카메라 좌표(mm) → **그 자세에서 보낼 스테이지 위치(mm)**\n")
            f.write("#   stage_mm = A @ P_cam + b      (A 3x3, b 3)\n")
            f.write("#   [0]=X [1]=Y [2]=Z, 전부 **호밍 원점 기준 mm**\n")
            f.write("# ⚠ 좌표계 변환이 아니다 — 같은 점이라도 자세가 다르면 보낼\n")
            f.write("#   위치가 다르다 (건이 yaw 축에 달려 회전한다). 그래서 자세별이다.\n")
            f.write("# ⚠ mm 는 원점 기준이라 재호밍·전원 재투입을 견딘다. 다만 원점\n")
            f.write("#   반복오차(~0.4mm)는 남는다.\n")
            f.write("# ⚠⚠ **깊이를 X·Y 에서 뺐고 Z 는 상수다** (`USE_DEPTH=False`,\n")
            f.write("#   `Z_CONST`). 교차점이 거의 한 평면이라 깊이가 cx·cy 의\n")
            f.write("#   선형함수고, 깊이 열이 중복되어 최소제곱이 아무 값이나 고른다.\n")
            f.write("#   실측이 그걸 가리켰다 — X 오차가 깊이에 비례(-0.144mm/mm)했고\n")
            f.write("#   Y 오차는 깊이와 무관(±2mm)했는데, 옛 A 의 깊이계수가 X행\n")
            f.write("#   +0.0933 / Y행 -0.0009 였다. **Y 는 이미 깊이를 안 써서 정확했다.**\n")
            f.write("# ⚠⚠ **이 모델은 \"그 평면\" 전용이다.** 데크플레이트 유무나 배근\n")
            f.write(f"#   높이가 바뀌면 다시 받아야 한다. 그래서 {PLANE_SINCE[:10]} 이전\n")
            f.write("#   짝은 적합에서 제외한다 (`PLANE_SINCE`) — 파일에는 남아 있다.\n")
            f.write(f"# ⚠ Z 는 자세와 무관하게 {Z_CONST}mm 고정 (사용자 결정, 적합 안 함).\n")
            if any(XY_OFFSET_MM):
                f.write(f"# ⚠⚠ **손으로 넣은 치우침이 b 에 더해져 있다**: "
                        f"X {XY_OFFSET_MM[0]:+.1f} · Y {XY_OFFSET_MM[1]:+.1f}mm "
                        f"(`XY_OFFSET_MM`). X 는 +가 Xmax 방향.\n")
                f.write("#   아래 잔차는 이 값을 **반영하지 않는다** — 적합이 아니라\n")
                f.write("#   사람이 준 보정이다. 새 짝을 받을 거면 먼저 0 으로 돌릴 것.\n")
            for _p, _o in sorted(XY_OFFSET_BY_POSE_MM.items()):
                if any(_o):
                    f.write(f"# ⚠ {_p}번 자세에는 X {_o[0]:+.1f} · Y {_o[1]:+.1f}mm 가 "
                            f"**추가로** 더해져 있다 (`XY_OFFSET_BY_POSE_MM`).\n")
            f.write(f"# 실측 {time.strftime('%Y-%m-%d %H:%M')}\n")
            # 소비자가 쓰는 값. 모델이 무엇이든 자세별 (A, b) 한 쌍으로 낸다
            f.write("# model: shared = 공통 변환 + 자세별 오프셋 (기준).\n")
            f.write("#   independent = 자세별 독립 (교차검증용, 자세당 4점 이상).\n")
            f.write("poses:\n")
            if sh:
                for p in sorted(sh['off']):
                    # 독립 모델을 기준으로 쓸지 — 그 자세에 독립 적합이 있을 때만
                    use_ind = (MODEL_SOURCE == 'independent' and p in done)
                    if use_ind:
                        A, b = done[p][0], done[p][1]
                    else:
                        A = sh['A']
                        b = sh['b'] + sh['off'][p]
                    # 사람이 준 치우침을 더한다 (위 `XY_OFFSET_MM` 주석 참고)
                    b = b + np.array([XY_OFFSET_MM[0], XY_OFFSET_MM[1], 0.0])
                    # 그 자세에만 더 들어가는 몫 (`XY_OFFSET_BY_POSE_MM`)
                    ex = XY_OFFSET_BY_POSE_MM.get(p)
                    if ex:
                        b = b + np.array([ex[0], ex[1], 0.0])
                    f.write(f"  {p}:\n")
                    f.write(f"    A: {json.dumps([[round(v, 6) for v in r] for r in A.tolist()])}\n")
                    f.write(f"    b: {json.dumps([round(v, 4) for v in b.tolist()])}\n")
                    used = len(self.rows(p))
                    tot = len(self.pairs.get(p, []))
                    f.write(f"    pairs: {used}"
                            + (f"   # 기록 {tot} 중 적합에 쓴 수\n" if used != tot
                               else "\n"))
                    f.write(f"    source: {'independent' if use_ind else 'shared'}\n")
                f.write(f"shared:\n")
                f.write(f"  pairs: {sh['n']}\n")
                f.write(f"  dof: {sh['dof']}\n")
                f.write(f"  rms_mm: {sh['rms']:.3f}\n")
                f.write(f"  max_mm: {sh['worst']:.3f}\n")
                f.write("  offsets:\n")
                for p in sorted(sh['off']):
                    o = sh['off'][p]
                    f.write(f"    {p}: {json.dumps([round(v, 3) for v in o.tolist()])}\n")
            f.write("independent:   # 교차검증용. 기준이 아니다\n")
            for p in sorted(done):
                A, b, rms, worst, n = done[p]
                f.write(f"  {p}:\n")
                f.write(f"    A: {json.dumps([[round(v, 6) for v in r] for r in A.tolist()])}\n")
                f.write(f"    b: {json.dumps([round(v, 4) for v in b.tolist()])}\n")
                f.write(f"    pairs: {n}\n")
                f.write(f"    rms_mm: {rms:.3f}\n")
                f.write(f"    max_mm: {worst:.3f}\n")
            f.write("raw:\n")
            for p in sorted(self.pairs):
                f.write(f"  {p}:\n")
                for c, m in self.rows(p):
                    f.write(f"    - cam: {json.dumps([round(v, 2) for v in c])}\n")
                    f.write(f"      stage: {json.dumps([round(v, 2) for v in m])}\n")
        print(f"\n  저장: {out_path}")

    def committed(self, p):
        """**장비가 실제로 쓰는 모델** (data/calibration/stage_camera.yaml) 을 읽는다.

        ⚠⚠ 왜 세션 적합(`fit_shared`) 을 예측에 쓰지 않는가: pairs.json 에는
        10-04 짝이 그대로 남아 있는데 그 짝들은 **다른 평면**(데크플레이트 전)
        이라 깊이빠진 모델에는 무효다. 그걸 섞어 예측하면 장비가 쓰는 값과
        달라지고(실측 X 386.6 vs yaml 381, 잔차 7.21mm), 그러면 **"실측 − 예측"
        이 보정량이 아니게 된다.** 보정 루프는 운영 모델 기준으로만 뜻이 있다.
        """
        try:
            import yaml as _y
            d = _y.safe_load(open(MODEL, encoding='utf-8'))['poses'][p]
            return np.array(d['A'], dtype=float), np.array(d['b'], dtype=float)
        except Exception:
            return None

    def predict(self, idx, pose=None):
        """고정한 점을 **운영 모델(yaml)** 로 예측한다. yaml 이 없을 때만 세션
        적합으로 내려간다 (독립 모델은 자세당 4점이 필요하고 그때도 과적합이다)."""
        p = self.pose() if pose is None else pose
        if not (0 <= idx < len(self.frozen)):
            print(f"  번호가 범위를 벗어났습니다 (0~{len(self.frozen) - 1})")
            return
        x, y, z = self.frozen[idx][:3]
        cm = self.committed(p)
        if cm is not None:
            A, b = cm
            q = A @ np.array([x, y, z]) + b
            src = f'운영 모델 {os.path.basename(MODEL)}'
        else:
            sh = self.fit_shared()
            if sh is None:
                print("  아직 변환을 못 풉니다 (짝이 모자라거나 배치가 겹칩니다)")
                return
            if p not in sh['off']:
                print(f"  {p}번 자세 오프셋이 없습니다 — "
                      f"그 자세 짝이 1점은 있어야 합니다")
                return
            q = sh['A'] @ np.array([x, y, z]) + sh['b'] + sh['off'][p]
            src = f"세션 적합 · 잔차 RMS {sh['rms']:.2f}mm"
        print(f"  [{idx}] {p}번 자세에서 보낼 위치  "
              f"X {q[0]:.1f}  Y {q[1]:.1f}  Z {q[2]:.1f} mm   ({src})")
        m = self.mm()
        if m:
            d = q - np.array(m)
            print(f"       지금 위치와 차이  X {d[0]:+.1f}  Y {d[1]:+.1f}  Z {d[2]:+.1f} mm")
        return q


    def goto(self, idx):
        """예측 위치로 **X·Y 만** 보낸다. 그다음 사람이 리모콘으로 미세보정하고
        `rxy <n>` 으로 기록하면, 기록값과 예측값의 차이가 곧 보정량이 된다.

        왜 이 단계가 필요한가: 처음부터 손으로 맞추면 자세당 6점이 너무 느리다.
        모델이 이미 있으면 **예측으로 가 놓고 조금만 고치는** 쪽이 훨씬 빠르고,
        남는 차이가 바로 "모델이 얼마나 틀렸나" 라서 눈으로 검증도 된다.

        ⚠ Z 는 보내지 않는다 (NaN). X·Y 가 틀린 채 내리면 철근을 찍는다.
        ⚠ 작업영역 밖이면 `stage_node` 가 거부한다 — 여기서 또 검사하지 않는다
          (두 곳에서 검사하면 규칙이 갈라진다).
        """
        q = self.predict(idx)
        if q is None:
            return
        m = self._Point()
        m.x, m.y, m.z = float(q[0]), float(q[1]), float('nan')
        # ⚠⚠ `--cmd` 는 한 명령만 돌리고 바로 끝난다 — 퍼블리셔를 만든 직후에 쏘면
        #   **디스커버리가 끝나기 전**이라 메시지가 조용히 사라진다. 실제로 `g 6`
        #   이 그렇게 유실됐다 (stage_node 에 `이동 시작` 자체가 안 찍혔다).
        #   그래서 ① 구독자가 붙을 때까지 기다리고 ② 쏜 뒤에도 조금 돌려서
        #   전송을 끝낸 다음 ③ 노드가 정말 받았는지 **움직이기 시작했는지로**
        #   확인한다. 보냈다고 믿고 넘어가면 사람이 허공을 미세조정한다.
        w = time.time() + 5.0
        while rclpy.ok() and self.goal.get_subscription_count() == 0 \
                and time.time() < w:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.goal.get_subscription_count() == 0:
            print("  ⚠ stage_node 가 /stage/goal 을 구독하지 않습니다 — "
                  "보내지 않았습니다")
            return
        self.goal.publish(m)
        self.spin(0.5)
        print(f"  → X {q[0]:.1f}  Y {q[1]:.1f} mm 로 이동 명령 (Z 는 유지). "
              f"맞춘 뒤 `rxy {idx}` 로 기록하세요")
        # 접수 확인: 2초 안에 움직이기 시작하거나 이미 그 자리에 있어야 한다
        t = time.time() + 2.5
        while rclpy.ok() and time.time() < t:
            self.spin(0.1)
            if (self.stage or {}).get('moving'):
                return q
        here = self.mm()
        if here and max(abs(here[0] - q[0]), abs(here[1] - q[1])) <= 2.0:
            print("     (이미 그 위치입니다)")
            return q
        print("  ⚠ 움직이기 시작하지 않았습니다 — 명령이 유실됐거나 "
              "거부됐습니다. 로그를 확인하세요")


def run_one(n, cmd):
    c = cmd[0] if cmd else ''
    if c == 's':
        n.snapshot()
    elif c == 'r' and len(cmd) > 1:
        n.record(int(cmd[1]))
    elif c == 'r!' and len(cmd) > 1:
        n.record(int(cmd[1]), check=False)       # 재확인 건너뛰기
    elif c in ('rxy', 'rxy!') and len(cmd) > 1:
        # X·Y 만 기록한다 (Z 를 안 내린 상태에서 맞춘 점). 값을 주면 그 값을 쓴다 —
        # 끝단 시험처럼 **지난 시점의 위치**를 소급해 넣을 때 필요하다
        st = [float(v) for v in cmd[2:4]] if len(cmd) >= 4 else None
        n.record(int(cmd[1]), check=(c == 'rxy'), axes='xy', stage=st)
    elif c == 'p' and len(cmd) > 1:
        n.predict(int(cmd[1]))
    elif c == 'g' and len(cmd) > 1:
        n.goto(int(cmd[1]))
    elif c == 'bad' and len(cmd) > 2:
        n.mark_bad(int(cmd[1]), [int(v) for v in cmd[2:]])
    elif c == 'l':
        n.listing()
    elif c == 'archive':
        n.archive(' '.join(cmd[1:]))
    elif c == 'u':
        n.undo()
    elif c == 'f':
        n.fit(OUT)
    elif c == '':
        n.show()
    else:
        print("  s / r <n> / r! <n> / rxy <n> [X Y] / p <n> / g <n> / l / u / "
              "archive <사유> / f / q")
        return False
    return True


def main():
    import argparse
    ap = argparse.ArgumentParser()
    # 한 명령만 돌리고 끝낸다. 짝은 STATE 파일에 남아 다음 호출로 이어진다 —
    # 사람이 리모콘으로 맞추는 동안 터미널을 붙잡고 있을 필요가 없다
    ap.add_argument('--cmd',
                    help='비대화 모드: "s" | "g 3" | "r 3" | "r! 3"(재확인 생략) | '
                         '"rxy 3" | "l" | "u" | "f" | "p 3"')
    a = ap.parse_args()

    rclpy.init()
    n = Calib()
    # 상태가 올 때까지 기다린다 — 2.5초로는 모자라 "자세 미확인" 이 잘못 찍혔다
    t0 = time.time()
    while time.time() - t0 < 8.0 and (n.stage is None or n.mm() is None):
        n.spin(0.2)
    if a.cmd is not None:
        n.show()
        run_one(n, a.cmd.strip().split())
        n.destroy_node()
        rclpy.shutdown()
        return 0
    print(__doc__.split('명령:')[0].rstrip())
    print("명령: s=고정  r <n>=기록  p <n>=예측  l=목록  u=취소  f=저장  q=종료\n")
    n.show()
    try:
        while rclpy.ok():
            n.spin(0.3)
            try:
                cmd = input("> ").strip().split()
            except EOFError:
                break
            if not cmd:
                n.show()
                continue
            if cmd[0] == 'q':
                break
            run_one(n, cmd)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
