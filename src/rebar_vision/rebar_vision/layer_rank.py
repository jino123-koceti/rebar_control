#!/usr/bin/env python3
"""깊이로 **상부근과 하부근을 가른다.** ROS 를 모르는 순수 모듈.

배근은 층이 있다. 상부근 교차점만 묶어야 하는데 검출기는 층을 모르고 전부
낸다 — 하부근 교차점에 건을 내리면 헛친다. 이 모듈이 깊이만으로 층을 가른다.

한국공대 학생 구현(`CROSS_DETECT/monitoring.hpp` 의 `extractLayerRankMap`)을
3차년도 장비에 맞게 옮긴 것이다. 알고리즘은 그대로, **계산량과 입출력만**
우리 쪽에 맞췄다.

## 어떻게 가르는가 — 두 단계

① **바닥 평면 찾기.** 무작위 3점으로 평면 후보를 많이 뽑아 (θ, φ, d) 공간에서
   군집을 짓고 **가장 큰 군집**을 바닥으로 본다. 화면에서 가장 넓은 평면이
   바닥(거푸집·합판)이기 때문이다.
   왜 최소제곱이 아닌가: 철근이 화소의 상당 부분을 차지해서 전체를 한 평면에
   맞추면 철근 쪽으로 끌려간다. 군집은 **다수결**이라 철근에 안 끌린다.

② **그 평면까지의 거리로 층 나누기.** 모든 점의 평면 거리를 1차원으로 군집
   지으면 거리 오름차순이 곧 층이다 — 카메라 쪽이 가까우므로
   **0 = 상부근, 1 = 하부근, 2 = 바닥**.

## 3차년도에 맞춘 것

- **표본 수를 제한한다.** 원본은 ROI 전 화소(최대 42만)를 k-means 에 넣는다.
  젯슨에서 매 프레임 그러면 비싸다. 평면 탐색은 `max_points` 로 솎아낸 표본만
  쓰고, 층 분류는 전 화소에 하되 벡터화한다. 솎아내도 결과가 같은 이유는
  평면이 **다수결로** 정해지기 때문이다.
- **층 수를 미리 정하지 않는다.** 원본은 k-means 로 `LAYER_K` 개를 강제하는데,
  k-means 는 분산을 줄이므로 **점이 많은 층을 쪼개고 적은 층을 뭉갠다** —
  바닥이 전체의 60% 가 넘는 이 장면에서 실제로 레일과 상부근이 한 덩이가
  됐다. 층은 개수가 아니라 **빈 구간**으로 갈린다 (`_bands`).
- 깊이 단위는 **mm** 로 받는다 (Orbbec 이 16UC1 mm 로 준다).

⚠⚠ **ROI 를 반드시 줘야 한다 (2026-10-20 실측).** 전체 영상으로 돌리면
  평면이 흔들린다 — 같은 자세 6회에서 상부근 높이가 209~292mm (폭 83mm)
  로 널뛰고 띠 수도 3~5개로 바뀌었다. 원인은 **데크 밖 배경**(먼 벽)과
  **결속건**이 평면 탐색에 섞여 평면을 끌어당기는 것이다.
  데크만 남기면 **폭 1mm** 로 붙는다:

      전체        상부근 209~292mm · 띠 3~5개   ← 못 쓴다
      ROI_DECK    상부근 160mm 매번 · 띠 4개 매번

  `ROI_DECK` 는 좌우 **프레임 레일까지** 뺀다 — 레일이 상부근과 비슷한
  높이(≈198mm)라 안 빼면 한 띠로 섞여 `pick_rank` 가 레일을 고른다.
  카메라가 프레임에 고정이라 이 창은 자세와 무관하게 유효하다.

⚠ 이 모듈은 **깊이가 맞다고 가정한다.** 깊이가 비어 있는 화소가 40% 가량
  되는 카메라라, 유효 화소가 적으면 평면 자체가 틀어진다. 그래서 유효 화소
  수를 함께 돌려주고 호출부가 적으면 쓰지 않도록 한다.
"""

import numpy as np

INVALID = 255          # 랭크 지도에서 '깊이 없음'

# 기본값은 학생 구현과 같게 둔다 — 바꾼 것은 근거를 남긴다.
HOUGH_ITERS = 10000    # 평면 후보 수
HOUGH_K = 3            # 평면 후보 군집 수 (평면 탐색에만 쓴다)
DEPTH_MIN = 10.0       # 층 분리에 넣는 깊이 범위 [mm]
DEPTH_MAX = 1000.0
MAX_POINTS = 40000     # 평면 탐색에 쓸 표본 상한 (3차년도 추가)
# 층을 가르는 최소 틈. 8~30mm 를 두 장면에서 훑어 **12** 로 정했다 —
# 15 이상이면 상·하부근 사이 20mm 골이 메워져 한 층으로 합쳐진다.
MIN_GAP_MM = 12.0
# 기본 ROI — 데크만 남기고 좌우 레일·위쪽 배경·결속건을 뺀다 (1280x800 기준).
# 이 값으로 6회 반복 재현 확인: 띠 4개(160·79·36·-2), 상부근 높이 폭 1mm.
ROI_DECK = (280, 160, 720, 630)      # x, y, w, h
REFINE_TOL = 40.0      # 평면 정밀화에 쓸 내점 거리 [mm] (3차년도 추가)
REFINE_ITERS = 3       # 정밀화 반복


def _bands(dist, bin_mm=10.0, min_frac=0.02, min_gap_mm=MIN_GAP_MM):
    """평면거리를 **골로** 가른다. (라벨, 띠중심) 또는 (None, None).

    ⚠ **k-means 로 나누면 안 된다** (2026-10-20 실측). k-means 는 분산을
      줄이므로 **점이 많은 층을 쪼개고 적은 층을 뭉갠다.** 이 장면은 바닥이
      전체의 62%(448k)라, k=4 가 바닥을 둘로 쪼개고(−3 / +56) 레일과 상부근을
      한 덩이로 묶었다(−293 / −144). 그 결과 `pick_rank` 가 레일을 상부근으로
      골라 **모든 비율이 0.000** 이 됐다. 1·2차 장면에서 맞았던 것은 층별
      점 개수 비율이 우연히 맞아떨어진 탓이다.

      층은 **개수가 아니라 빈 구간으로** 갈린다 — 철근과 바닥 사이에는 늘
      빈 틈이 있다. 히스토그램에서 그 틈을 찾으면 개수 편차에 안 끌린다.
      덤으로 **층 수를 미리 정할 필요가 없어진다** (`layer_k` 가 사라진다).

    min_frac   : 가장 높은 칸 대비 이 비율 미만이면 '빈 칸'
    min_gap_mm : 이보다 좁은 틈은 같은 층으로 본다 (봉 지름만 한 틈은 층이 아니다)
    """
    lo, hi = float(dist.min()), float(dist.max())
    if hi - lo < bin_mm:
        return None, None
    nb = max(4, int(np.ceil((hi - lo) / bin_mm)))
    hist, edges = np.histogram(dist, bins=nb, range=(lo, hi))
    full = hist >= max(1.0, min_frac * hist.max())
    if not full.any():
        return None, None

    # 찬 칸이 이어진 구간 = 띠. 좁은 틈은 메운다
    gap_bins = max(1, int(round(min_gap_mm / ((hi - lo) / nb))))
    runs, s = [], None
    empty = 0
    for i, f in enumerate(full):
        if f:
            if s is None:
                s = i
            empty = 0
        elif s is not None:
            empty += 1
            if empty >= gap_bins:
                runs.append((s, i - empty + 1))
                s = None
    if s is not None:
        runs.append((s, len(full)))
    if not runs:
        return None, None

    bounds = [(edges[a], edges[b]) for a, b in runs]
    lab = np.full(len(dist), -1, dtype=np.int64)
    cen = []
    for r, (a, b) in enumerate(bounds):
        m = (dist >= a) & (dist <= b)
        if not m.any():
            cen.append(0.5 * (a + b))
            continue
        lab[m] = r
        cen.append(float(dist[m].mean()))
    # 띠에 못 든 점은 가장 가까운 띠로 — 버리면 지도에 구멍이 생긴다
    out = lab < 0
    if out.any():
        c = np.asarray(cen)
        lab[out] = np.argmin(np.abs(dist[out][:, None] - c[None, :]), axis=1)
    return lab, cen


def _refit(pts, normal, d, tol=REFINE_TOL, iters=REFINE_ITERS):
    """군집으로 고른 거친 평면을 **내점 최소제곱**으로 정밀화한다.

    ⚠ 이것이 없으면 쓸 수 없다 (2026-10-20 실측). 허프 군집의 *중심*은
      (θ, φ, d) 공간에서 평균한 값이라 실제 어느 평면과도 다를 수 있다.
      그대로 쓰면 바닥 화소의 평면거리가 **100mm 넘게 퍼지고**, 봉을 따라
      색이 매끄럽게 변하는 기울기로 나타난다 — 실제 층 차이(13~20mm)를
      완전히 덮는다.

    다수결로 고른 평면의 **내점만** 다시 맞추므로 철근에 끌리지 않는 장점은
    유지된다. 거친 선택(robust) + 정밀 맞춤(accurate)의 조합이다.
    """
    for _ in range(iters):
        r = pts @ normal + d
        inl = pts[np.abs(r) < tol]
        if len(inl) < 16:
            break
        c = inl.mean(axis=0)
        # 최소 분산 방향 = 평면 법선 (SVD 의 마지막 특이벡터)
        _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
        n2 = vt[-1]
        if n2[2] < 0:
            n2 = -n2
        normal, d = n2, float(-n2 @ c)
    return normal, d


def _floor_plane(pts, iters=HOUGH_ITERS, k=HOUGH_K, seed=42):
    """무작위 3점 평면 후보를 군집 지어 **바닥 평면**을 고른다.

    반환 (normal, d) — `normal·p + d` 가 그 점의 평면 거리다. 못 구하면 None.
    """
    import cv2
    n = len(pts)
    if n < 3:
        return None
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(iters, 3))
    # 같은 점이 겹친 조합은 법선이 0 이 되어 아래에서 걸러진다
    a, b, c = pts[idx[:, 0]], pts[idx[:, 1]], pts[idx[:, 2]]
    nrm = np.cross(b - a, c - a)
    ln = np.linalg.norm(nrm, axis=1)
    ok = ln > 1e-6
    if ok.sum() < k:
        return None
    nrm = nrm[ok] / ln[ok, None]
    # 카메라 방향(z 양수)으로 통일 — 안 하면 같은 평면이 두 군집으로 갈린다
    flip = nrm[:, 2] < 0
    nrm[flip] = -nrm[flip]
    theta = np.arctan2(nrm[:, 1], nrm[:, 0])
    theta[theta < 0] += np.pi
    phi = np.arccos(np.clip(nrm[:, 2], -1.0, 1.0))
    dd = -np.sum(nrm * a[ok], axis=1)
    h = np.stack([theta, phi, dd], axis=1).astype(np.float32)

    # 세 축의 단위가 달라서(라디안 vs mm) 그대로 넣으면 d 가 지배한다.
    # 0~100 으로 정규화해 세 축을 대등하게 만든다.
    lo, hi = h.min(axis=0), h.max(axis=0)
    rng_ = np.where(hi - lo < 1e-6, 1.0, hi - lo)
    hn = (h - lo) / rng_ * 100.0
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
    _, labels, centers = cv2.kmeans(hn.astype(np.float32), k, None, crit, 10,
                                    cv2.KMEANS_PP_CENTERS)
    labels = labels.reshape(-1)
    best = np.argmax(np.bincount(labels, minlength=k))
    cen = centers[best] / 100.0 * rng_ + lo
    th, ph, d = float(cen[0]), float(cen[1]), float(cen[2])
    normal = np.array([np.cos(th) * np.sin(ph),
                       np.sin(th) * np.sin(ph),
                       np.cos(ph)], dtype=float)
    return _refit(pts, normal, d)


def rank_map(depth_mm, K, roi=None, fit_roi=ROI_DECK,
             depth_min=DEPTH_MIN, depth_max=DEPTH_MAX,
             max_points=MAX_POINTS, seed=42):
    """층 랭크 지도를 만든다.

    depth_mm : 2차원 배열 [mm]. 0 이나 NaN 은 '깊이 없음'.
    K        : (fx, fy, cx, cy)
    roi      : 층을 **매길** 범위. None 이면 전체.
    fit_roi  : 평면을 **맞출** 범위 (기본 `ROI_DECK`). 둘을 나누는 이유는
               아래 ⚠⚠ 참고 — 좁게 맞추고 넓게 매긴다.

    반환 (rank, info)
      rank : depth 와 같은 크기 uint8. 0=상부근 1=하부근 … INVALID(255)=모름
      info : {'valid': 유효화소수, 'plane': (normal, d) 또는 None,
              'centers': 층별 평면거리 중심 [mm]}
    """
    h, w = depth_mm.shape[:2]
    rank = np.full((h, w), INVALID, dtype=np.uint8)
    x0, y0, x1, y1 = (0, 0, w, h)
    if roi is not None:
        rx, ry, rw, rh = roi
        x0, y0 = max(0, int(rx)), max(0, int(ry))
        x1, y1 = min(w, x0 + int(rw)), min(h, y0 + int(rh))
        if x1 <= x0 or y1 <= y0:
            return rank, {'valid': 0, 'plane': None, 'centers': []}

    z = depth_mm[y0:y1, x0:x1].astype(np.float32)
    good = np.isfinite(z) & (z >= depth_min) & (z <= depth_max)
    nvalid = int(good.sum())
    info = {'valid': nvalid, 'plane': None, 'centers': []}
    if nvalid < 3:
        return rank, info

    fx, fy, cx, cy = K
    vv, uu = np.nonzero(good)
    zz = z[vv, uu]
    uu = uu + x0
    vv = vv + y0
    pts = np.stack([(uu - cx) * zz / fx, (vv - cy) * zz / fy, zz], axis=1)

    # ⚠⚠ 평면은 **좁은 데크 영역(fit_roi)에서만** 맞춘다. 층은 **전체**에
    #   매긴다. 둘을 같은 범위로 묶으면 둘 중 하나가 망가진다:
    #     · 넓게 맞추면 데크 밖 배경이 평면을 끌어당겨 흔들린다 (폭 83mm)
    #     · 좁게 매기면 경계 근처 검출의 둘레 원이 잘려 비율이 반 토막 난다
    #       (2026-10-20: v=156 검출이 0.156 — 진짜인데 허위처럼 보였다)
    #   평면만 정확하면 랭크는 어디든 매길 수 있으므로 나누는 것이 맞다.
    fit = np.ones(len(pts), dtype=bool)
    if fit_roi is not None:
        fx0, fy0, fw, fh = fit_roi
        fit = ((uu >= fx0) & (uu < fx0 + fw) & (vv >= fy0) & (vv < fy0 + fh))
    fpts = pts[fit] if fit.any() else pts
    if len(fpts) < 3:
        fpts = pts
    pts_for_plane, pts = fpts, pts
    if len(pts_for_plane) > max_points:
        sel = np.random.default_rng(seed).choice(len(pts_for_plane),
                                                 max_points, replace=False)
        sample = pts_for_plane[sel]
    else:
        sample = pts_for_plane
    plane = _floor_plane(sample, seed=seed)
    if plane is None:
        return rank, info
    normal, d = plane
    info['plane'] = (normal, d)

    dist = pts @ normal + d
    # 띠도 **데크 안에서만** 가른다. 전체로 가르면 좌우 레일이 띠로 살아나
    # (≈206mm) 상부근(≈161mm)보다 높아서 `pick_rank` 가 레일을 고른다.
    # 경계가 정해진 뒤에는 **전체 화소**를 그 경계로 매긴다 — 그래야 가장자리
    # 검출의 둘레 원이 안 잘린다. 레일 화소는 상부근으로 매겨지지만, 레일
    # 위의 검출은 가동범위 밖이라 뒤에서 걸러진다.
    labels, centers = _bands(dist[fit] if fit.any() else dist)
    if labels is None:
        return rank, info
    cen_arr = np.asarray(centers)
    labels = np.argmin(np.abs(dist[:, None] - cen_arr[None, :]), axis=1)
    # `_bands` 는 이미 거리 오름차순으로 띠를 낸다 — 그게 곧 층 번호다
    # (법선이 카메라 반대쪽이라 가까울수록 작다 → 0 이 가장 높은 층)
    if len(centers) > INVALID:
        return rank, info
    rank[vv, uu] = labels.astype(np.uint8)
    info['centers'] = [float(c) for c in centers]
    return rank, info


def layer_ratio(rank, u, v, radius, target=0):
    """(u, v) 둘레 원 안에서 목표 층이 차지하는 비율.

    한 화소만 보면 안 되는 이유: 키포인트가 철근 **사이**에 떨어질 수 있고,
    깊이가 비어 있는 화소도 많다. 둘레를 봐야 "이 점이 그 층에 있다" 가 된다.
    """
    h, w = rank.shape[:2]
    u, v, r = int(round(u)), int(round(v)), int(radius)
    y0, y1 = max(0, v - r), min(h, v + r + 1)
    x0, x1 = max(0, u - r), min(w, u + r + 1)
    if y1 <= y0 or x1 <= x0:
        return 0.0
    yy, xx = np.ogrid[y0 - v:y1 - v, x0 - u:x1 - u]
    disk = (yy * yy + xx * xx) <= r * r
    # 분모는 **원 전체 넓이**다 (화면 밖·깊이 없음도 분모에 든다). 그래야
    # "가장자리라 정보가 적다" 가 낮은 비율로 드러난다 — 원본과 같은 규약.
    total = float(np.pi * r * r)
    hit = float(np.count_nonzero(rank[y0:y1, x0:x1][disk] == target))
    return hit / total if total > 0 else 0.0


def pick_rank(info, lo_mm, hi_mm, prefer='high'):
    """높이 구간으로 목표 랭크를 **고른다.** 없으면 None.

    ⚠ **랭크 번호를 박아 쓰면 안 된다.** 원본(`TARGET_LAYER_RANK = 0`)은 0 을
      상부근으로 못박는데, 우리 장비는 **프레임 레일이 철근보다 카메라에
      가까워서 rank 0 이 레일**이다 (2026-10-20 실측: 레일 −349mm, 상부근
      −186mm, 하부근 −82mm, 바닥 +3mm). 그대로 쓰면 레일을 결속 대상으로
      고른다.

    lo_mm, hi_mm : 바닥 평면 **위쪽** 높이 구간 [mm]. 양수로 준다.
                   (평면거리는 카메라 쪽이 음수이므로 부호를 뒤집어 비교한다)
    prefer       : 구간 안에 여럿이면 'high'(가장 높은) 또는 'low'

    랭크가 층 수에 따라 바뀌므로, 호출부는 늘 이 함수로 고른다.
    """
    cen = info.get('centers') or []
    hits = [(i, -c) for i, c in enumerate(cen) if lo_mm <= -c <= hi_mm]
    if not hits:
        return None
    hits.sort(key=lambda x: x[1], reverse=(prefer == 'high'))
    return hits[0][0]
