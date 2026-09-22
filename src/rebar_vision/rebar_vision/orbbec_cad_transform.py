#!/usr/bin/env python3
"""CAD 기반 camera→EEF(결속건) 강체변환 + 자세별 touch 오프셋 → 스테이지 좌표.

기존 호모그래피 캘리브레이션을 대체하는 로컬라이즈 코어.
- 입력: 컬러 픽셀(u,v) + 정합 depth(mm) + 컬러 intrinsic(fx,fy,cx,cy) + 자세(r/l)
- 출력: 스테이지 명령 XY(mm)  (Z는 결속 시퀀스가 별도 하강)

파이프라인:
  (u,v,depth) --intrinsic 역투영--> P_cam(카메라 광학3D)
  --CAD 강체변환(R,t)--> P_eef(결속건 프레임, X=근/원, Y=좌/우, +Z=하강)
  --자세별 XY 오프셋--> 스테이지 XY

좌표계: CAD 글로벌 Y-up, 장비 스테이지 −Z-up(+Z=하강). 축정렬은 EEF_AXIS_FIX_DEG로 맞춤.
검증: 2026-08-05 4점 실측(P1/P6=r, P3/P4=l), XY 잔차 ~2mm, 스케일≈1.0.
⚠️ 상수는 tools/calibration/test_cad_transform_plot.py 와 반드시 동일하게 유지할 것.
"""
import numpy as np

# ── CAD 좌표계 (XYZ 스테이지 홈 글로벌 기준, Y-up) ──
CAM_AXANG = ([0.66, -0.38, 0.65], 138.03)
CAM_POS = [-221.43, -18.07, -25.38]          # LCS002 카메라(광학)
EEF_AXANG = ([0.58, -0.58, 0.58], 120.0)
EEF_POS = [267.16, -266.50, -4.98]           # LCS001 결속건 EEF
EEF_AXIS_FIX_DEG = 90.0                       # 하강축 중심 축보정(near-far가 +X)

# ── 자세별 touch 오프셋 (2026-08-05 4점 실측) : stage_XY = P_eef_XY + (dx,dy) ──
# 자세별 오프셋(mm). 변환행렬(R,t)은 자세 공통이고 여기 병진만 자세별로 다르다.
# l의 Y: 159.4 → 164.4 (+5mm, 2026-08-06) → 167.4 (+3mm, 2026-08-19)
#        → 162.4 (2026-09-02: 08-06의 **+5mm를 되돌림**)
# r의 Y: 146.8 → 141.8 (2026-09-02: -5mm) → 138.8 (2026-09-02: **-3mm 추가**)
# X: r -50.8 → -53.8, l -63.8 → -66.8 (2026-09-02: **양 자세 -3mm**)
#    → r -53.8 → -58.8 (2026-09-08: **우측만 -5mm 추가**)
#   ⚠ 이번엔 한쪽만 옮겼다 = 우측 자세에서만 X가 치우쳐 있었다는 뜻.
#     그래서 자세별 X 차이(l-r)가 -13.0 → -8.0mm 로 바뀐다.
#   ⚠ 두 자세를 같은 방향(-5mm)으로 옮긴 것 = 결속점이 Y+ 쪽으로 치우쳐 있었다는 뜻.
#     자세별 차이(l-r)는 20.6mm로 그대로 유지된다.
# yaml과 반드시 같이 유지할 것 —
# yaml이 실제 로드값이고, save_calibration()은 여기 상수로 yaml을 다시 쓴다.
POSE_OFFSET = {'r': (-58.8, 138.8), 'l': (-66.8, 162.4)}
DECK_Z_MM = 420.0            # 단층 덱 하강 스테이지 절대 Z (참고값; 결속 시퀀스가 하강)
DEPTH_Z_OFFSET_MM = 317.5    # 다층 시 stage_Z = P_eef_Z + 이 값 (자세 무관)


def _axang(ax, deg):
    a = np.array(ax, float); a /= np.linalg.norm(a); th = np.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def _Rz(deg):
    t = np.radians(deg); c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def cad_transform():
    """CAD 배치값 → camera→EEF 강체변환 (P_eef = R·P_cam + t)."""
    Rc = _axang(*CAM_AXANG); pc = np.array(CAM_POS)
    Re = _axang(*EEF_AXANG) @ _Rz(EEF_AXIS_FIX_DEG); pe = np.array(EEF_POS)
    R = Re.T @ Rc
    t = Re.T @ (pc - pe)
    return R, t


def backproject(u, v, z_mm, fx, fy, cx, cy):
    """컬러 픽셀(u,v)+depth(mm) → 카메라 광학 3D P_cam(mm)."""
    return np.array([(u - cx) * z_mm / fx, (v - cy) * z_mm / fy, z_mm])


def to_stage_xy(peef, pose, pose_offset=None):
    """P_eef → 자세별 스테이지 XY(mm). pose_offset 미지정 시 모듈 기본값."""
    dx, dy = (pose_offset or POSE_OFFSET)[pose]
    return float(peef[0]) + dx, float(peef[1]) + dy


# 캘리브레이션 아티팩트 기본 경로 (호모그래피 대체)
CALIB_PATH = '/home/koceti/ros2_ws/data/calibration/cad_transform_orbbec.yaml'


def save_calibration(path=CALIB_PATH):
    """현재 모듈 상수로 R,t 계산 → 자세별 오프셋과 함께 yaml 저장(출처·검증 포함)."""
    import yaml
    R, t = cad_transform()
    data = {
        'transform': {'R': [[float(x) for x in row] for row in R],
                      't': [float(x) for x in t]},
        'pose_offset': {p: [float(POSE_OFFSET[p][0]), float(POSE_OFFSET[p][1])]
                        for p in POSE_OFFSET},
        'deck_z_mm': float(DECK_Z_MM),
        'depth_z_offset_mm': float(DEPTH_Z_OFFSET_MM),
        'source_cad': {
            'cam_axang': {'axis': list(CAM_AXANG[0]), 'deg': float(CAM_AXANG[1])},
            'cam_pos': list(CAM_POS),
            'eef_axang': {'axis': list(EEF_AXANG[0]), 'deg': float(EEF_AXANG[1])},
            'eef_pos': list(EEF_POS),
            'eef_axis_fix_deg': float(EEF_AXIS_FIX_DEG),
        },
        'validation': {
            'date': '2026-08-05',
            'points': 'P1/P6=r, P3/P4=l (4점 실측)',
            'xy_residual_mm': 2.0,
            'scale': 1.0,
            'note': 'CAD 글로벌 Y-up, 스테이지 -Z-up(+Z=하강). X=근/원 Y=좌/우.',
        },
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
    return path


def load_calibration(path=CALIB_PATH):
    """yaml → dict(R, t, pose_offset, deck_z_mm, depth_z_offset_mm).
    파일 없거나 파싱 실패 시 모듈 상수로 계산한 값 반환(폴백)."""
    R, t = cad_transform()
    out = {'R': R, 't': t, 'pose_offset': dict(POSE_OFFSET),
           'deck_z_mm': DECK_Z_MM, 'depth_z_offset_mm': DEPTH_Z_OFFSET_MM,
           'source': 'module_constants'}
    try:
        import yaml
        d = yaml.safe_load(open(path))
        out['R'] = np.array(d['transform']['R'], float)
        out['t'] = np.array(d['transform']['t'], float)
        out['pose_offset'] = {p: tuple(v) for p, v in d['pose_offset'].items()}
        out['deck_z_mm'] = float(d.get('deck_z_mm', DECK_Z_MM))
        out['depth_z_offset_mm'] = float(d.get('depth_z_offset_mm', DEPTH_Z_OFFSET_MM))
        out['source'] = path
    except Exception:
        pass
    return out


if __name__ == '__main__':
    p = save_calibration()
    print(f'저장: {p}')
    import numpy as _np
    c = load_calibration()
    print('로드 확인:', c['source'])
    print('R=\n', _np.array2string(c['R'], precision=4, suppress_small=True))
    print('t=', _np.round(c['t'], 1))
    print('pose_offset=', c['pose_offset'])


def _near_band(vals, band=25.0, min_n=10):
    """표본 무리에서 **가장 가까운 밴드**의 평균(mm). 표본 부족이면 None.
    철근은 그 자리에서 가장 가까운 구조물이라, 근접 5%를 기준으로 잡으면
    뒤의 데크·바닥이 섞이지 않는다."""
    if len(vals) < min_n:
        return None
    a = np.asarray(vals, float)
    lo = np.percentile(a, 5)
    b = a[a < lo + band]
    return float(b.mean() if b.size >= 3 else a.mean())


def _kmeans1d(x, k, iters=25, seed=0):
    """1차원 k-means. sklearn 없이 쓰려고 직접 둔다. → (centers, labels, wcss)"""
    x = np.asarray(x, float)
    rng = np.random.default_rng(seed)
    q = np.linspace(0, 100, k + 2)[1:-1]
    c = np.percentile(x, q) if k > 1 else np.array([x.mean()])
    for _ in range(iters):
        lab = np.argmin(np.abs(x[:, None] - c[None, :]), axis=1)
        nc = np.array([x[lab == j].mean() if (lab == j).any() else c[j]
                       for j in range(k)])
        if np.allclose(nc, c):
            c = nc
            break
        c = nc
    lab = np.argmin(np.abs(x[:, None] - c[None, :]), axis=1)
    wcss = float(((x - c[lab]) ** 2).sum())
    return c, lab, wcss


def extract_layers(depth_frame, fx, fy, cx, cy, step=6, zmin=100.0, zmax=1500.0,
                   k_min=1, k_max=6, min_pts=600, min_frac=0.03, seed=0,
                   roi=None):
    """depth 한 장 → **층 목록**. (normal, origin, centers, counts) 또는 None.

    ## 방법 (학생 알고리즘 `rebar_crossing_networked.cpp` 구조를 옮김)
      1) depth → 3D 점군
      2) 지배 평면의 **방향**을 잡는다. 상단근·하단근·바닥이 **전부 평행**이므로
         방향은 하나로 모인다 — 어느 평면이 잡히든 법선은 같다.
         (⚠ 여기서 RANSAC이 면적 큰 바닥을 잡아도 무방하다. 예전에 그 평면의
           **위치**를 배근 평면으로 오해해 실패했다 — 위치는 아래 3)이 정한다.)
      3) 모든 점의 부호거리를 **1차원 k-means**로 나눈다. k는 **엘보법**
         (log(wcss) 곡률 최대)으로 자동 선택 → 층이 2개든 3개(바닥 포함)든 알아서.
      4) 카메라에 가까운 순으로 rank: **0 = 상단근, 1 = 하단근, 2 = 바닥**

    centers는 origin 기준 높이(mm, 카메라 쪽이 +)를 rank 순으로 담는다.
    """
    h, w = depth_frame.shape[:2]
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth_frame[::step, ::step].astype(np.float32)
    m = (z > zmin) & (z < zmax)
    # ★ ROI (u0,v0,u1,v1) — **배근이 점군을 지배해야** 층 군집이 배근에 붙는다.
    #   화면 전체를 쓰면 바닥(55%)·장비가 지배해 철근이 자기 군집을 못 얻는다
    #   (2026-09-08 실측: 교차점이 -80~-28mm인데 군집 중심은 -248/-100/-11/+86).
    #   학생 코드도 ROI를 쓴다(docs/student_networked/6_ROIimage).
    if roi is not None:
        u0, v0, u1, v1 = roi
        m &= (us >= u0) & (us < u1) & (vs >= v0) & (vs < v1)
    if m.sum() < min_pts:
        return None
    us, vs, z = us[m].astype(np.float32), vs[m].astype(np.float32), z[m]
    P = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], 1)

    # 2) 방향: RANSAC으로 지배 평면 → 법선만 쓴다
    rng = np.random.default_rng(seed)
    best_n, best_cnt = None, 0
    for _ in range(80):
        i3 = rng.choice(len(P), 3, replace=False)
        a, b, c = P[i3]
        nv = np.cross(b - a, c - a)
        ln = np.linalg.norm(nv)
        if ln < 1e-6:
            continue
        nv = nv / ln
        cnt = int((np.abs((P - a) @ nv) < 20.0).sum())
        if cnt > best_cnt:
            best_n, best_cnt = nv, cnt
    if best_n is None:
        return None
    if best_n[2] > 0:
        best_n = -best_n
    origin = P.mean(0)
    # ⚠ 부호 주의 (2026-09-08 실측으로 확정): best_n은 z성분이 음수(카메라 쪽)라
    #   `-(P-origin)@n` 은 **멀수록 커진다**. 상단근을 rank0으로 두려면
    #   **작은 쪽부터** 정렬해야 한다. 처음에 반대로 해서 바닥(54.6%)이
    #   '상단근'으로 뽑혔고 교차점이 0개였다.
    d = -((P - origin) @ best_n)            # 값이 클수록 **먼 쪽**

    # 3) 거리 1차원 k-means + 엘보법
    ks, wc, res = [], [], {}
    for k in range(k_min, k_max + 1):
        c_, lab_, w_ = _kmeans1d(d, k, seed=seed)
        ks.append(k); wc.append(max(w_, 1e-9)); res[k] = (c_, lab_)
    best_k = k_min
    if len(wc) >= 3:
        lw = np.log(np.asarray(wc))
        kap = []
        for i in range(1, len(lw) - 1):
            dy = (lw[i + 1] - lw[i - 1]) / 2.0
            d2y = lw[i - 1] - 2 * lw[i] + lw[i + 1]
            kap.append(abs(d2y) / (1.0 + dy * dy) ** 1.5)
        best_k = ks[1 + int(np.argmax(kap))]
    c_, lab_ = res[best_k]
    order = np.argsort(c_)                  # 가까운(작은 d) 순 = 상단근부터
    centers = c_[order]
    counts = np.array([int((lab_ == j).sum()) for j in order])
    # ★ 점이 너무 적은 군집은 버린다 — 실측에서 46점(0.3%)짜리 잡음 군집이
    #   rank0을 차지해 '상단근'으로 뽑힐 뻔했다 (2026-09-08).
    keep = counts >= max(1, int(min_frac * counts.sum()))
    if keep.sum() == 0:
        return None
    return best_n, origin, centers[keep], counts[keep]


def layer_mask(depth_frame, fx, fy, cx, cy, normal, origin, center, tol_mm=40.0):
    """선택한 층만 남기는 **픽셀 마스크**(uint8 0/255).

    이 마스크 위에서 직선을 뽑으면, 다른 층 철근은 지워져 선이 안 나온다 →
    `상단 가로 × 하단 세로` 같은 가짜 교차점은 **선이 하나뿐**이라 교차점이
    안 생긴다. (paper/src/methods/hough_detector.py `_top_layer_mask` 발전형 —
    원본은 raw depth 중앙값 기준이라 카메라가 기울면 못 쓴다.)
    """
    h, w = depth_frame.shape[:2]
    vs, us = np.mgrid[0:h, 0:w]
    z = depth_frame.astype(np.float32)
    ok = (z > 50) & (z < 3000)
    P = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], -1)
    d = -((P - origin) @ normal)
    return (ok & (np.abs(d - center) <= tol_mm)).astype(np.uint8) * 255


def ring_bar_heights(depth_frames, u, v, z_mm, plane_n, plane_c, fx, fy, cx, cy,
                     r_in_mm=18.0, r_out_mm=34.0, n_ang=72, band_mm=(-320.0, 90.0),
                     min_per_sector=3):
    """교차점 둘레를 **링으로 훑어** 만나는 철근들의 높이를 각도별로 낸다.

    ## 왜 링인가 (2026-09-08, 두 번의 실패 뒤)
    ① 화면축 팔 → 카메라가 28° 기울고 철근이 대각선이라 **팔이 철근을 벗어난다**.
       사용자 라벨 대조에서 진짜 259mm / 가짜 0mm로 **상관이 없었다**.
    ② 교차점 중심 depth 하나 → 위에 얹힌 철근만 보여서 `상단×하단`을
       `상단×상단`과 구분 못 한다(사용자 지적).

    → 둘레를 한 바퀴 돌면 **네 갈래(가로 2 + 세로 2)** 가 각도로 나타난다.
      방향을 몰라도 되고, 갈래마다 높이를 따로 얻는다.
      depth가 아니라 **평면 위 높이**로 보므로 원근·기울기가 상쇄된다.

    반환 (angles_deg, heights_mm) — 철근으로 보이는 각도와 그 높이. 없으면 (빈,빈).
    band_mm: 평면 기준 이 높이 범위 밖(바닥·장비)은 버린다.
    """
    if not depth_frames or not z_mm or fx is None:
        return np.empty(0), np.empty(0)
    ppm = fx / float(z_mm)
    r0, r1 = r_in_mm * ppm, r_out_mm * ppm
    h, w = depth_frames[0].shape[:2]
    rs = np.linspace(r0, r1, max(3, int(round(r1 - r0))))
    angs, hts = [], []
    for k in range(n_ang):
        a = 2.0 * np.pi * k / n_ang
        ca, sa = np.cos(a), np.sin(a)
        vals = []
        for rr in rs:
            x, y = int(round(u + ca * rr)), int(round(v + sa * rr))
            if not (0 <= x < w and 0 <= y < h):
                continue
            for d in depth_frames:
                z = float(d[y, x])
                if not (50.0 < z < 3000.0):
                    continue
                P = np.array([(x - cx) * z / fx, (y - cy) * z / fy, z])
                hh = -((P - plane_c) @ plane_n)
                if band_mm[0] <= hh <= band_mm[1]:
                    vals.append(hh)
        if len(vals) >= min_per_sector:
            angs.append(np.degrees(a))
            hts.append(float(np.median(vals)))       # 중앙값 — 가장자리 튐에 둔감
    return np.asarray(angs), np.asarray(hts)


def crossing_bar_layers(depth_frames, u, v, z_mm, plane_n, plane_c, fx, fy, cx, cy,
                        layer_gap_mm=75.0, **kw):
    """교차점의 **두 철근이 같은 층인지** 판정 → (ok, spread_mm, n_sector).

    링에서 얻은 각도별 높이를 **대향 각도쌍으로 평균**한 뒤(같은 철근의 양쪽 갈래를
    묶는다 — 한 갈래만 튀는 잡음에 둔감해진다) 그 폭이 layer_gap_mm 이상이면
    서로 다른 층이 섞인 것 → ok=False.

    ⚠ 갈래가 6개 미만이면 판정 불가(None). 화면 가장자리·가림에서 자주 난다.
    ⚠ 층간이 층내 산포와 비슷하면 못 가린다. RC 목업(40mm)에서 실측 분리도가
      1.36σ에 그쳤다 — 그런 곳에서는 어차피 못 가리므로 layer_mode='off'로 둔다.

    ★ 임계 75mm의 근거 (2026-09-08 현장 실측):
        하단근 바닥+30mm / 상단근 바닥+180mm → **층간 150mm**
      같은 층이면 spread≈0(+노이즈), 층이 섞이면 spread≈150.
      임계는 그 **중간**이 양쪽 여유가 같다 → 150/2 = 75mm.
      (100mm로 두면 층 혼합 쪽 여유가 50mm뿐이라, 측정오차가 실제 교차점을
       살려주는 게 아니라 가짜를 통과시키는 방향으로만 작용한다.)
    """
    angs, hts = ring_bar_heights(depth_frames, u, v, z_mm,
                                 plane_n, plane_c, fx, fy, cx, cy, **kw)
    if hts.size < 6:
        return None, None, int(hts.size)
    pair = []
    for k in range(len(angs)):
        opp = (angs[k] + 180.0) % 360.0
        j = int(np.argmin(np.abs(((angs - opp + 180.0) % 360.0) - 180.0)))
        pair.append((hts[k] + hts[j]) / 2.0)
    pair = np.asarray(pair)
    spread = float(pair.max() - pair.min())
    return (spread < layer_gap_mm), spread, int(hts.size)


def split_layers(pts3d, min_gap_mm=80.0, min_side=2):
    """교차점 3D 점들을 **두 층으로 가른다** → (heights, thr, n_upper, n_lower).

    ## 왜 이 방식인가 (2026-09-08)
    앞서 시도한 두 가지가 다 실패했다.
      · 교차점의 두 철근 depth를 축별로 재기 → 카메라가 28° 기울고 철근이 화면에서
        대각선이라 **팔이 철근을 벗어난다**. 사용자 라벨 대조 결과 진짜가 259mm,
        가짜가 0mm를 내 **상관이 없었다**.
      · 화면 전체 평면 적합 → RANSAC이 **바닥(면적 54%)** 을 잡아 철근이 아니었다.

    → 가장 믿을 수 있는 값만 쓴다: **교차점 중심 depth**(반복측정 σ≈0).
      교차점들끼리 주성분 평면을 맞추고 그 위 높이로 정렬한 뒤,
      **가장 큰 틈**에서 자른다. 층이 실제로 갈라져 있으면 그 틈이 층간이다.
      틈이 min_gap_mm보다 작으면 **단층으로 보고 자르지 않는다**(None 반환) —
      없는 경계를 억지로 만들면 진짜 교차점을 버린다.

    pts3d: [(X,Y,Z)mm, ...] 카메라 좌표. 반환 heights는 평면 위 높이(양수=카메라 쪽).
    """
    P = np.asarray(pts3d, float)
    if len(P) < min_side * 2:
        return None
    c0 = P.mean(0)
    _, _, vt = np.linalg.svd(P - c0, full_matrices=False)
    nv = vt[2] / np.linalg.norm(vt[2])
    if nv[2] > 0:
        nv = -nv
    hts = -((P - c0) @ nv)                      # 평면 위 높이(카메라 쪽 +)
    order = np.argsort(hts)
    sh = hts[order]
    # 양쪽에 min_side개 이상 남는 위치 중 **가장 큰 틈**
    best_i, best_gap = -1, 0.0
    for i in range(min_side - 1, len(sh) - min_side):
        g = sh[i + 1] - sh[i]
        if g > best_gap:
            best_i, best_gap = i, g
    if best_i < 0 or best_gap < min_gap_mm:
        return hts, None, 0, len(P)             # 단층으로 판단
    thr = (sh[best_i] + sh[best_i + 1]) / 2.0
    return hts, float(thr), int((hts > thr).sum()), int((hts <= thr).sum())


def fit_rebar_plane(depth_frame, fx, fy, cx, cy, step=8,
                    zmin=50.0, zmax=3000.0, iters=60, tol_mm=25.0, seed=0):
    """depth 한 장 → **배근 평면** (n, d): n·P + d = 0, |n|=1. 실패 시 None.

    ## 왜 평면인가 (2026-09-08)
    카메라가 수직에서 **28° 기울어** 달려 있어, 평평한 배근도 화면 위치마다
    depth가 크게 달라진다(실측 319~689mm). 그래서 depth 값 자체로는
    "위 철근이냐 아래 철근이냐"를 못 가린다.

    → 평면을 맞추고 **평면 위 높이**로 보면 기울기가 사라진다.
      하단근 = 높이 0 근처, 상단근 = +층간(200mm급).

    RANSAC으로 **가장 표본이 많은 평면**을 잡는다. 하단근이 시야를 지배하면
    하단 평면이 잡히고, 상단근은 양(+)의 높이로 뜬다.
    """
    h, w = depth_frame.shape[:2]
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth_frame[::step, ::step].astype(np.float32)
    m = (z > zmin) & (z < zmax)
    if m.sum() < 200:
        return None
    us, vs, z = us[m].astype(np.float32), vs[m].astype(np.float32), z[m]
    P = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], 1)
    rng = np.random.default_rng(seed)
    best_n, best_d, best_cnt = None, None, 0
    for _ in range(iters):
        idx = rng.choice(len(P), 3, replace=False)
        a, b, c = P[idx]
        nv = np.cross(b - a, c - a)
        ln = np.linalg.norm(nv)
        if ln < 1e-6:
            continue
        nv = nv / ln
        dd = -float(nv @ a)
        cnt = int((np.abs(P @ nv + dd) < tol_mm).sum())
        if cnt > best_cnt:
            best_n, best_d, best_cnt = nv, dd, cnt
    if best_n is None or best_cnt < 200:
        return None
    inl = np.abs(P @ best_n + best_d) < tol_mm          # 인라이어로 재적합
    Q = P[inl]
    c0 = Q.mean(0)
    _, _, vt = np.linalg.svd(Q - c0, full_matrices=False)
    nv = vt[2] / np.linalg.norm(vt[2])
    if nv[2] > 0:                                       # 카메라 쪽(−z)을 향하게
        nv = -nv
    return nv, -float(nv @ c0)


def height_above_plane(u, v, z_mm, plane, fx, fy, cx, cy):
    """(u,v,z) 점이 평면보다 **얼마나 위(카메라 쪽)** 인지 mm. 양수=위."""
    if plane is None or not z_mm:
        return None
    nv, d = plane
    P = np.array([(u - cx) * z_mm / fx, (v - cy) * z_mm / fy, z_mm], float)
    return float(-(nv @ P + d))          # 카메라 쪽이 양수가 되도록 부호


def estimate_grid_dirs(points, kmax=6, min_pairs=4):
    """검출된 교차점들로 **철근 두 방향**을 추정한다 → ((dx,dy),(dx,dy)) 단위벡터.

    ## 왜 필요한가 (2026-09-08 실측)
    Orbbec은 **비스듬히** 달려 있어 철근이 화면에서 대각선으로 흐른다.
    팔을 화면 수평·수직으로 뻗으면 몇 px 만에 철근을 벗어나 바닥을 짚는다
    — 단층 배근인데 262mm 차이가 나와 '다른 층'으로 오판했다.

    교차점들은 격자를 이루므로, **가까운 이웃까지의 벡터**들이 곧 철근 방향이다.
    원근 때문에 화면 위치마다 방향이 조금씩 달라서, 전역 하나로 뭉치지 않고
    **점마다 자기 이웃으로** 낸다(호출자가 점별로 부른다).

    반환 방향은 부호를 정규화(위쪽 반평면)해 ±가 섞이지 않게 한다.
    추정 실패 시 None — 호출자가 화면축 폴백을 쓰면 된다.
    """
    if len(points) < 3:
        return None
    P = np.asarray(points, float)
    vecs = []
    for i in range(len(P)):
        d = np.linalg.norm(P - P[i], axis=1)
        idx = np.argsort(d)[1:kmax + 1]           # 자기 자신 제외 최근접 k개
        for j in idx:
            if d[j] > 1e-6:
                vecs.append((P[j] - P[i]) / d[j])
    if len(vecs) < min_pairs:
        return None
    V = np.asarray(vecs)
    V[V[:, 1] < 0] *= -1.0                        # 위쪽 반평면으로 부호 통일
    ang = np.arctan2(V[:, 1], V[:, 0])            # 0~pi
    # 각도를 2배로 올려 원형 평균 → 방향(선)의 군집을 잡는다
    h, edges = np.histogram(ang, bins=18, range=(0.0, np.pi))
    order = np.argsort(h)[::-1]
    picked = []
    for b in order:
        c = (edges[b] + edges[b + 1]) / 2.0
        if all(min(abs(c - q), np.pi - abs(c - q)) > np.deg2rad(25) for q in picked):
            picked.append(c)
        if len(picked) == 2:
            break
    if len(picked) < 2:
        return None
    return tuple((float(np.cos(a)), float(np.sin(a))) for a in picked)


def bar_depths_dir(depth_frames, u, v, fx, z_hint_mm, dirs,
                   inner_mm=12.0, outer_mm=40.0, halfw_mm=5.0, min_n=10,
                   max_dev_mm=None):
    """`bar_depths_at`의 **방향 지정판** — 팔을 실제 철근 방향으로 뻗는다.

    dirs: ((dx,dy),(dx,dy)) 화면상 단위벡터 두 개.
    max_dev_mm: 중심 depth에서 이만큼 넘게 벗어난 표본은 **버린다**.
        팔이 철근을 벗어나 바닥·장비를 짚으면 엉뚱한 값이 나오는데,
        그건 '다른 층'이 아니라 **측정 실패**다. 층간 간격(200mm급)은 살리고
        바닥(수백~1000mm)은 빼려면 이 창을 층간보다 넉넉히 잡는다.
    반환 (d_a, d_b) — 각 방향 철근의 depth(mm). 표본 부족이면 None.
    """
    if fx is None or not depth_frames or not z_hint_mm or z_hint_mm <= 0 or not dirs:
        return None, None
    ppm = fx / float(z_hint_mm)
    ri, ro = inner_mm * ppm, outer_mm * ppm
    hw = max(1.0, halfw_mm * ppm)
    if ro <= ri:
        return None, None
    h, w = depth_frames[0].shape[:2]
    # 팔을 따라 등간격 점을 찍고, 각 점에서 수직으로 halfw 만큼 훑는다
    n_along = max(4, int(round(ro - ri)))
    out = []
    for (dx, dy) in dirs[:2]:
        px, py = -dy, dx                          # 수직 방향
        vals = []
        for sgn in (1.0, -1.0):
            for t in np.linspace(ri, ro, n_along):
                for o in np.linspace(-hw, hw, max(3, int(2 * hw))):
                    x = int(round(u + sgn * dx * t + px * o))
                    y = int(round(v + sgn * dy * t + py * o))
                    if 0 <= x < w and 0 <= y < h:
                        for d in depth_frames:
                            z = float(d[y, x])
                            if 50.0 < z < 3000.0:
                                if max_dev_mm is None or abs(z - z_hint_mm) <= max_dev_mm:
                                    vals.append(z)
        out.append(_near_band(vals, min_n=min_n))
    return out[0], out[1]


def bar_depths_at(depth_frames, u, v, fx, z_hint_mm,
                  inner_mm=12.0, outer_mm=40.0, halfw_mm=5.0, min_n=10):
    """교차점에서 만나는 **두 철근의 depth를 축별로 따로** 잰다 → (d_h, d_v) mm.

    ## 왜 필요한가 (2026-09-03 현장: 상·하단근 2층 배근)
    하단근 세로 × 상단근 가로는 **탑뷰에서 교차점처럼 보이지만 실제로는 아니다**
    (높이차 200mm+). 그런데 교차점 한 점의 depth만 보면 위에 얹힌 상단근 값이
    잡혀서 "상단 교차점"으로 통과해버린다 — 단일 depth로는 못 거른다.

    → 중심을 **건너뛰고** 상하·좌우로 팔을 뻗어 각 철근을 따로 잰다.
      진짜 교차점이면 두 값 차이가 철근 지름 정도(13~25mm),
      다른 층이면 층간 간격(200mm+)이 그대로 나온다.

    중심을 건너뛰는 이유: 교차점 한가운데는 **위에 얹힌 쪽만 보인다.**
    아래 철근은 가려져서 그 픽셀에는 없다.

    치수는 mm로 받고 depth·fx로 픽셀 환산한다(높이가 바뀌어도 같은 실치수).
    """
    if fx is None or not depth_frames or z_hint_mm is None or z_hint_mm <= 0:
        return None, None
    ppm = fx / float(z_hint_mm)                    # px per mm
    ri, ro = int(round(inner_mm * ppm)), int(round(outer_mm * ppm))
    hw = max(1, int(round(halfw_mm * ppm)))
    if ro <= ri:
        return None, None

    def collect(boxes):
        vals = []
        for d in depth_frames:
            h, w = d.shape[:2]
            for (x0, x1, y0, y1) in boxes:
                x0, x1 = max(0, x0), min(w, x1)
                y0, y1 = max(0, y0), min(h, y1)
                if x1 <= x0 or y1 <= y0:
                    continue
                sub = d[y0:y1, x0:x1]
                m = (sub > 50) & (sub < 3000)
                vals.extend(sub[m].tolist())
        return vals

    # 좌우 팔(가로 철근) / 상하 팔(세로 철근) — 중심 ±ri 는 건너뛴다
    h_boxes = [(u + ri, u + ro, v - hw, v + hw + 1),
               (u - ro, u - ri, v - hw, v + hw + 1)]
    v_boxes = [(u - hw, u + hw + 1, v + ri, v + ro),
               (u - hw, u + hw + 1, v - ro, v - ri)]
    return (_near_band(collect(h_boxes), min_n=min_n),
            _near_band(collect(v_boxes), min_n=min_n))


def rebar_depth_mm(depth_frames, u, v, r=4):
    """(u,v) 주변 근접밴드 depth(mm) — 철근 top. depth_frames: float32(mm) 프레임 리스트.
    유효 표본 부족 시 None."""
    vals = []
    for d in depth_frames:
        h, w = d.shape[:2]
        sub = d[max(0, v - r):min(h, v + r + 1), max(0, u - r):min(w, u + r + 1)]
        m = (sub > 50) & (sub < 3000)
        vals.extend(sub[m].tolist())
    if len(vals) < 10:
        return None
    near = np.array(vals); lo = np.percentile(near, 5)
    band = near[near < lo + 25.0]
    return float(band.mean() if band.size >= 3 else near.mean())
