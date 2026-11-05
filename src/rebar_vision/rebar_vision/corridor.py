#!/usr/bin/env python3
"""직진 통로 판정 — "지금 자세에서 앞으로 몇 mm 갈 수 있나". ROS 의존 없는 순수 모듈.

세그멘테이션 마스크 + 깊이 → 데크 평면 위 좌표 → **장비 폭만큼의 띠**에 차단물이
들어오는지 본다. `deck_edge.judge` 가 내는 GO/SLOW/STOP 과 역할이 다르다 —
저쪽은 "발밑 배근이 이어지는가"(스칼라), 이쪽은 **"얼마나 갈 수 있나"(거리)** 다.
반응형 자율 고리가 스텝 크기를 정하려면 거리가 필요하다.

## 2026-11-05 실장비 검증

    배치 4가지 + 접근 실험(200mm × 4) 으로 확인했다.
      · 전방 거리   줄자 1170mm vs 계산 1151mm   (1.6%)
      · 접근 추적   명령 800mm 에 판정 782mm     (2.2%)
      · 통과 판정   틈 1300mm 실측 → 1269mm 계산 (2.4%, 보수적)
      · 5프레임 산포 6~13mm
    횡은 보수적으로 90mm 가량 안쪽으로 나오는 경향이 있다 (상자 모서리 기준).
    평판으로 다시 교정할 것 — 지금은 안전한 방향이라 두고 쓴다.

## 설계에서 틀렸다가 고친 것 둘 — 다시 그렇게 하지 말 것

**① 거리는 깊이로 낸다. 광선-평면 교차로 내지 않는다.**
  장애물의 '발'(열마다 가장 아래 차단 화소)을 찾아 평면에 투영하는 방식은,
  접지선이 앞 철근에 가리면 **몸통의 한 점**을 잡는다. 그 점은 데크보다 떠
  있는데(실측 132mm) 광선을 평면까지 연장하므로 **실제보다 멀게** 나온다 —
  1216mm 를 1643mm 로. 안전과 반대 방향이라 쓰면 안 된다.
  평면은 **좌표축(전방·횡)을 정하는 데만** 쓴다.

**② 횡은 '중심선에 가장 가까운 점'이다. '전방 최근접 점의 횡'이 아니다.**
  상자 면이 비스듬하면 둘이 크게 갈린다 (실측 525 vs 281mm, 244mm 차이).
  충돌 여부를 정하는 것은 **얼마나 안으로 들어왔나** 이다.
"""

import numpy as np

# 장비 제원 — 전부 2026-11-05 실측
BODY_W_MM = 1000.0          # 프레임 가로 폭
BODY_L_MM = 1160.0          # 길이 (그림용. 판정에는 안 쓴다)
MARGIN_MM = 60.0            # 좌우 안전 여유. 실여유 8mm 짜리 사례가 실제로 나왔다
# 전방 카메라 좌측 렌즈 광축 → 장비 중심선. + 가 오른쪽.
#   영상(레벨봉) +164 / 핀홀 +181 / 줄자 +160 세 값이 20mm 안에서 일치했다.
CENTER_OFFSET_MM = 160.0
# 렌즈 바로 아래 지점 → 장비 앞단. 카메라가 프레임에 직결돼 0 이다.
FRONT_OFFSET_MM = 0.0

MIN_OBSTACLE_H_MM = 40.0    # 데크 위 이만큼 넘어야 '부딪히는 것'. 납작한 건 밟고 간다
NEAR_PCT = 1.0              # 가장 가까운 하위 N% 의 중앙값을 쓴다 (단일 화소 잡음 방어)
LAT_SPAN_MM = 300.0         # '가장 가까운 것' 으로 칠 전방 구간 폭
RANGE_MM = (150.0, 6000.0)  # 판정에 쓸 전방 범위
PLANE_FIT_MM = (400.0, 1500.0)   # 평면 적합에 쓸 깊이 창 (근거리일수록 깊이가 좋다)


class Plane:
    """데크 평면과 그 위의 좌표축.

    `fwd` = 광축을 평면에 투영한 방향(전방), `lat` = 평면 안에서 그에 수직(+가 오른쪽),
    `n` = 평면 법선(아래), `D` = 카메라에서 평면까지 수직 거리(= 카메라 높이).

    ⚠ **정차마다 다시 맞춰야 한다.** 장비가 배근 위에서 기울어 카메라 높이가
      429~473mm 로 흔들렸다(2026-11-05 접근 실험). 적합 잔차가 8~16mm 인데
      평면 자체가 ±25mm 움직이니 "한 번 맞추고 캐시" 는 안 된다.
      주행 중에는 캐시하고 싼 잔차 검증만 하는 정도가 맞다.
    """

    __slots__ = ('n', 'D', 'fwd', 'lat', 'residual_mm', 'points')

    def __init__(self, n, D, residual_mm, points):
        if n[1] < 0:                      # +y(영상 아래)를 향하게 통일
            n = -n
        axis = np.array([0.0, 0.0, 1.0])
        fwd = axis - (axis @ n) * n
        fwd /= np.linalg.norm(fwd)
        lat = np.cross(n, fwd)
        lat /= np.linalg.norm(lat)
        if lat[0] < 0:                    # +lat 이 영상 오른쪽
            lat = -lat
        self.n, self.D, self.fwd, self.lat = n, float(D), fwd, lat
        self.residual_mm = float(residual_mm)
        self.points = int(points)

    @property
    def camera_height_mm(self):
        return abs(self.D)

    def blind_mm(self, cy, fy, height):
        """영상 맨 아랫줄이 데크에 닿는 거리 — **이 안쪽 바닥은 안 보인다.**

        2026-11-05 실측 365mm. 반응형 고리는 이 거리 전에 판단을 끝내야 한다.
        """
        down = np.degrees(np.arcsin(float(np.clip(self.n[2], -1, 1))))
        half = np.degrees(np.arctan((height - cy) / fy))
        a = np.radians(down + half)
        return float('inf') if a <= 0 else float(self.camera_height_mm / np.tan(a))

    def project(self, u, v, z, fx, fy, cx, cy):
        """화소 + **실측 깊이** → (전방 mm, 횡 mm, 데크 위 높이 mm).

        깊이를 쓰는 이유는 모듈 머리말 ① 참고.
        """
        P = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], 1)
        return P @ self.fwd, P @ self.lat, -(P @ self.n - self.D)


def fit_plane(sel, depth_mm, K, window=PLANE_FIT_MM, max_points=150000,
              iters=8, tol_mm=15.0, seed=0):
    """데크 평면을 맞춘다. `sel` 은 평면 위에 있다고 보는 화소의 불리언 마스크.

    ⚠ **무엇을 `sel` 로 줄지가 이 함수의 전부다.** 테스트베드에서는 방수포
      (`background`)가 가장 깨끗했다(잔차 14.7mm). 철근으로 맞추면 28.4mm 로
      나빠진다 — 가는 구조라 혼합화소가 많다.
      ⚠ **현장 데크는 방수포가 아니라 아연도금 강판이다** (2026-11-05 현장
        사진 확인). `background` 전제가 현장에서는 성립하지 않으니, 호출부가
        그때의 '바닥' 클래스를 골라 넘겨야 한다.
    """
    fx, fy, cx, cy = K
    h, w = depth_mm.shape[:2]
    ys, xs = np.mgrid[0:h, 0:w]
    q = sel & (depth_mm > window[0]) & (depth_mm < window[1])
    if q.sum() < 5000:
        return None
    z = depth_mm[q]
    P = np.stack([(xs[q] - cx) * z / fx, (ys[q] - cy) * z / fy, z], 1).astype(np.float64)
    if len(P) > max_points:
        P = P[np.random.default_rng(seed).choice(len(P), max_points, replace=False)]
    c = P.mean(0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    n = vt[-1]
    for _ in range(iters):
        r = (P - c) @ n
        keep = np.abs(r) < max(tol_mm, 2.0 * r.std())
        if keep.sum() < 1000:
            break
        c = P[keep].mean(0)
        _, _, vt = np.linalg.svd(P[keep] - c, full_matrices=False)
        n = vt[-1]
    r = (P - c) @ n
    inl = np.abs(r) < 50.0
    return Plane(n, c @ n, r[inl].std() if inl.any() else np.nan, len(P))


def _cloud(mask, depth_mm, plane, K):
    v, u = np.nonzero(mask)
    if len(u) == 0:
        return (np.empty(0),) * 3
    z = depth_mm[v, u]
    g = z > 300.0                      # ZED X 최소 깊이. 그 아래는 값이 없다
    if g.sum() == 0:
        return (np.empty(0),) * 3
    return plane.project(u[g].astype(float), v[g].astype(float), z[g], *K)


def judge(block_mask, drive_mask, depth_mm, plane, K,
          center_mm=CENTER_OFFSET_MM, body_w_mm=BODY_W_MM, margin_mm=MARGIN_MM,
          front_offset_mm=FRONT_OFFSET_MM, min_h_mm=MIN_OBSTACLE_H_MM):
    """직진 통로 판정 한 장.

    block_mask : 부딪히는 것 (obstacle·human·wall).
                 ⚠ `plate` 는 넣지 말 것 — 바닥 철판이고, 맨 합판을 그렇게
                   오검출한다. 납작한 것은 높이 필터가 거른다.
    drive_mask : 밟을 수 있는 것 (rebar_h·rebar_v)

    반환 dict:
        go_mm        **장애물까지** 갈 수 있는 거리 (장비 앞단 기준).
                     장애물이 없으면 None — '무한' 이 아니라 '이 판정으로는
                     제한 없음' 이다. 데크끝은 rebar_frac 이 따로 본다.
        limit        'obstacle' | 'clear'
        obs_mm       통로 안 차단물까지 (없으면 None)
        obs_lat_mm   그 차단물이 중심선에서 얼마나 들어왔나 (+가 오른쪽)
        deck_mm      통로 안 배근 끝 — **참고용. 제한으로 쓰지 말 것** (아래 ⚠⚠)
    """
    half = body_w_mm / 2.0 + margin_mm
    lo, hi = center_mm - half, center_mm + half
    out = {'go_mm': None, 'limit': 'none', 'obs_mm': None, 'obs_lat_mm': None,
           'deck_mm': None, 'corridor': (lo, hi)}

    F, L, H = _cloud(block_mask, depth_mm, plane, K)
    obs = np.inf
    if len(F):
        real = (H > min_h_mm) & (F > RANGE_MM[0]) & (F < RANGE_MM[1])
        ins = real & (L >= lo) & (L <= hi)
        if ins.sum() > 50:
            k = np.argsort(F[ins])[:max(1, int(ins.sum() * NEAR_PCT / 100.0))]
            obs = float(np.median(F[ins][k]))
            # 횡은 **중심선에 가장 가까운 점** (모듈 머리말 ② 참고)
            near = ins & (F < obs + LAT_SPAN_MM)
            if near.any():
                j = np.argmin(np.abs(L[near] - center_mm))
                out['obs_lat_mm'] = float(L[near][j] - center_mm)
            out['obs_mm'] = obs - front_offset_mm

    Fr, Lr, Hr = _cloud(drive_mask, depth_mm, plane, K)
    deck = 0.0
    if len(Fr):
        rin = ((Lr >= lo) & (Lr <= hi) & (Fr > RANGE_MM[0]) & (Fr < RANGE_MM[1])
               & (np.abs(Hr) < 120.0))
        if rin.sum() > 500:
            deck = float(np.percentile(Fr[rin], 99))
    # ⚠⚠ **`deck_mm` 을 거리 제한으로 쓰지 않는다. 참고용으로만 싣는다.**
    #   "통로 안 철근 화소의 99백분위" 는 원거리 철근 검출이 프레임마다 달라
    #   못 믿는다 — 900mm 주행 실험(2026-11-05)에서 1309~1823 을 오가고 절반은
    #   아예 결측이었다. 같은 주행에서 `deck_edge.judge` 의 `rebar_frac` 은
    #   0.582 → 0.497 로 **한 번도 역전 없이** 떨어졌다.
    #   → **데크끝은 rebar_frac 이, 장애물은 여기가 맡는다.** 각자 설계된 대로다.
    #   (장애물 거리는 줄자 대비 1.6~2.4%, 5프레임 산포 6~13mm 로 믿을 만하다.)
    out['deck_mm'] = (deck - front_offset_mm) if deck > 0.0 else None

    out['limit'] = 'obstacle' if np.isfinite(obs) else 'clear'
    out['go_mm'] = (None if not np.isfinite(obs)
                    else obs - margin_mm - front_offset_mm)
    return out


def lateral_options(block_mask, drive_mask, depth_mm, plane, K, lat_step_mm=50.0,
                    turns=8, **kw):
    """횡으로 한 칸씩 비켜봤을 때 각각 얼마나 갈 수 있나.

    "막혔으니 비킨다" 를 **얼마나** 비킬지까지 정하려면 이 표가 필요하다.
    횡 1회전이 7.2초라 아무 데나 비키면 안 된다.

    반환 [(회전수, 중심 이동 mm, go_mm, limit)] — 회전수 오름차순.
    """
    base = kw.pop('center_mm', CENTER_OFFSET_MM)
    rows = []
    for t in range(-turns, turns + 1):
        r = judge(block_mask, drive_mask, depth_mm, plane, K,
                  center_mm=base + t * lat_step_mm, **kw)
        rows.append((t, t * lat_step_mm, r['go_mm'], r['limit']))
    return rows


def gap_between(block_mask, depth_mm, plane, K, center_mm=CENTER_OFFSET_MM,
                within_mm=2000.0, min_h_mm=MIN_OBSTACLE_H_MM,
                body_w_mm=BODY_W_MM, margin_mm=MARGIN_MM):
    """좌우 차단물 사이 틈과 통과 가능 여부.

    반환 (틈 mm, 좌측 안쪽끝, 우측 안쪽끝, 통과가능?) — 한쪽이 없으면 None.
    """
    F, L, H = _cloud(block_mask, depth_mm, plane, K)
    if not len(F):
        return None
    near = (H > min_h_mm) & (F > RANGE_MM[0]) & (F < within_mm)
    d = L[near] - center_mm
    left, right = d[d < 0], d[d > 0]
    if not len(left) or not len(right):
        return None
    gl, gr = float(left.max()), float(right.min())
    need = body_w_mm + 2 * margin_mm
    return (gr - gl, gl, gr, (gr - gl) >= need)
