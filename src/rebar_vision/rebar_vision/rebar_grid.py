#!/usr/bin/env python3
"""철근 격자 기하 → 주행 조향 신호 (ROS 의존 없음).

7클래스 DeepLabV3+ **시맨틱** 마스크에서 heading(정렬오차)을 뽑는다.
정렬돼 있으면 가로철근(rebar_h)이 화면에서 수평 → heading_deg ≈ 0.
로봇이 요(yaw)로 틀어지면 가로철근이 기울어진다 → 그 기울기를 조향 오차로 쓴다.

**왜 가로철근인가** (2026-08-06 RC 주행영상 실측):
  · rebar_v(세로=주행방향) 소실점 방식은 하이브리드 검출기(YOLO-seg **인스턴스**)에선
    되지만, 여기선 시맨틱 마스크라 세로철근이 교차점에서 조각나 엉뚱한 직선이 섞이고
    소실점이 끌려다닌다 (같은 프레임에서 가로 +0.64° ↔ 소실점 -6.27°).
  · 가로철근은 연결성분/행투영 두 독립 방식이 0.2~0.6° 내로 일치 = 신뢰 가능.

⚠ **부호 규약 미확정**: heading_deg 부호가 어느 회전방향인지는 지면 주행으로
   확인해야 한다. 주행노드의 `heading_sign` 파라미터로 뒤집을 수 있게 해뒀다.
"""
import cv2
import numpy as np

# ★ [2026-09-14] 구 7클래스 기준 기본값. 9클래스 모델에서는 6/7 로 밀린다.
#   모델을 로드한 쪽(deck_edge_node)이 set_class_map() 으로 덮어쓴다.
#   하드코딩을 남겨두면 heading 추정이 **obstacle/plate 를 철근으로 착각**한다.
IDX_H, IDX_V = 4, 5


def set_class_map(class_map):
    """활성 모델의 클래스맵으로 rebar_h/rebar_v 인덱스를 재계산."""
    global IDX_H, IDX_V
    name2i = {v: k for k, v in class_map.items()}
    if 'rebar_h' not in name2i or 'rebar_v' not in name2i:
        raise RuntimeError(f'클래스맵에 rebar_h/rebar_v 가 없다: {class_map}')
    IDX_H, IDX_V = name2i['rebar_h'], name2i['rebar_v']
    return IDX_H, IDX_V
MIN_PIXELS = 300           # 인스턴스 최소 픽셀
MIN_WIDTH_FRAC = 0.15      # 화면폭 대비 최소 가로폭 (조각 제거)
CLOSE_KERNEL = (25, 3)     # 교차점에서 끊긴 가로철근 잇기
MAX_ABS_DEG = 30.0         # 이보다 기운 건 가로철근이 아님(오검) → 버림


def _fit_line(mask):
    """마스크 픽셀에 직선 피팅 → 각도(deg, 수평=0)와 범위."""
    ys, xs = np.where(mask)
    if len(xs) < 10:
        return None
    pts = np.column_stack([xs, ys]).astype(np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    ang = float(np.degrees(np.arctan2(vy, vx)))
    if ang > 90:
        ang -= 180
    elif ang < -90:
        ang += 180
    return dict(angle=ang, vx=float(vx), vy=float(vy), x0=float(x0), y0=float(y0),
                xmin=int(xs.min()), xmax=int(xs.max()), cy=float(ys.mean()))


def hbar_lines(mask, min_px=MIN_PIXELS, min_width_frac=MIN_WIDTH_FRAC):
    """가로철근 인스턴스 목록. 가로방향 closing으로 교차점 끊김을 메운 뒤 연결성분."""
    h, w = mask.shape
    mh = (mask == IDX_H).astype(np.uint8)
    if mh.sum() < min_px:
        return []
    k = cv2.getStructuringElement(cv2.MORPH_RECT, CLOSE_KERNEL)
    closed = cv2.morphologyEx(mh, cv2.MORPH_CLOSE, k)
    n, lab = cv2.connectedComponents(closed)
    out = []
    for i in range(1, n):
        m = lab == i
        if m.sum() < min_px:
            continue
        if m.any(axis=0).sum() < w * min_width_frac:      # 폭 좁은 조각 제거
            continue
        line = _fit_line(m & (mh > 0))                    # 원본 픽셀로 피팅
        if line and abs(line['angle']) <= MAX_ABS_DEG:
            out.append(line)
    return out


def weighted_median(vals, weights):
    """가중 중앙값. 누적 가중치가 절반을 넘는 첫 값."""
    order = sorted(zip(vals, weights))
    half = sum(weights) / 2.0
    acc = 0.0
    for v, w in order:
        acc += w
        if acc >= half:
            return float(v)
    return float(order[-1][0])


def heading(mask, max_width=640, weighted=True):
    """★ 조향 신호: 마스크 → (heading_deg, n_bars).

    heading_deg = 가로철근 중심선 각도의 **스팬 가중** 중앙값 (수평=0).
    n_bars=0이면 heading_deg=None (판단 불가 → 조향하지 말 것).

    ★ 왜 가중하는가 (2026-08-19 실측):
        한 프레임에서 검출된 4개가 이랬다.
            cy=243  angle=+1.67°  스팬 640/640   ← 가깝고 화면 전폭
            cy=195  angle=+1.15°  스팬 640/640
            cy=181  angle=+0.13°  스팬 211/640   ← 멀고 부분검출
            cy=171  angle=-0.10°  스팬 313/640
        단순 중앙값 = **0.64°**. 실제 기울기는 가까운 전폭 철근이 말하는 ~1.4°인데
        멀리 있는 조각 둘이 같은 한 표씩 행사해 **절반으로 깎였다.**
        그 상태로 데드밴드(2.0°)를 만나 **보정이 전혀 안 됐다.**

        직선 피팅의 각도 불확실도는 스팬에 반비례한다 — 200px 조각과 640px 전폭을
        같은 표로 세면 안 된다. 스팬으로 가중하면 위 예에서 **1.15°**가 나온다.

    weighted=False면 예전 단순 중앙값(회귀 비교용).

    max_width: 이 폭으로 줄여서 계산(각도는 스케일 불변). 두 가지 이유:
        ① 원본 1920폭에서 모폴로지+연결성분을 매 프레임 돌리면 비싸다
        ② **아래 파라미터들이 640폭 녹화영상으로 튜닝됐다** — CLOSE_KERNEL(25,3)과
           MIN_PIXELS는 픽셀 단위라 해상도가 바뀌면 의미가 달라진다.
        라벨 마스크이므로 NEAREST로 축소.
    """
    if max_width and mask.shape[1] > max_width:
        h = max(1, int(mask.shape[0] * max_width / mask.shape[1]))
        mask = cv2.resize(mask, (max_width, h), interpolation=cv2.INTER_NEAREST)
    lines = hbar_lines(mask)
    if not lines:
        return None, 0
    angles = [l['angle'] for l in lines]
    if not weighted:
        return float(np.median(angles)), len(lines)
    spans = [max(1.0, float(l['xmax'] - l['xmin'])) for l in lines]
    return weighted_median(angles, spans), len(lines)
