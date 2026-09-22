#!/usr/bin/env python3
"""배근 간격(pitch) 추정 + 다음 이동거리 계산 (ROS 무관 순수함수).

⚠ 정본은 이 패키지 파일이다. tools/drive/coverage_planner.py 는 자가테스트용 사본.
   (ROS 노드는 tools/를 import할 수 없다 — 설치 경로 밖)
사용처: rebar_drive_node(스텝 주행 거리 산출), tools/drive/test_orbbec_coverage.py

순수 함수(ROS 무관). orbbec 결속검출이 준 교차점(로봇XY, mm)에서:
  · X 클러스터 = 열(column),  Y 클러스터 = 행(row)
  · pitch_x/pitch_y = 인접 클러스터 간격 (결측 열/행 = 2×간격 자동 보정)
  · 다음 전진/후진 거리 = 열수 × pitch_x
  · 측면 이동 거리       = 행수 × pitch_y

원리: 도달범위 내 열/행을 모두 결속했으니, 다음 미결속 열은 (결속한 열 수)×pitch
      만큼 떨어져 있음 → 그만큼 전/후진. 측면도 동일(행 기준).

테스트:  python3 tools/drive/coverage_planner.py
"""
import numpy as np


def cluster_1d(vals, tol):
    """1D 값들을 tol(mm) 이내로 그룹핑 → [(center, members), ...] center 오름차순."""
    if len(vals) == 0:
        return []
    s = sorted(float(v) for v in vals)
    groups = [[s[0]]]
    for v in s[1:]:
        if v - groups[-1][-1] <= tol:
            groups[-1].append(v)
        else:
            groups.append([v])
    return [(float(np.mean(g)), g) for g in groups]


def estimate_pitch(centers, fallback):
    """정렬된 클러스터 중심 → pitch(mm).
    결측 열/행으로 2×·3× 간격이 섞여도, 최소간격의 정수배로 정규화해 1×로 복원."""
    if len(centers) < 2:
        return float(fallback), 'fallback'
    diffs = np.diff(sorted(centers))
    base = float(np.min(diffs))
    if base < 1e-3:
        return float(fallback), 'fallback'
    norm = [d / max(1, round(d / base)) for d in diffs]   # 2×간격 → 1×
    return float(np.median(norm)), 'measured'


def compute_next_move(points_xy, prev_pitch=(None, None),
                      cluster_tol_mm=50.0, default_pitch_mm=195.0,
                      tie_x_range=None, tie_y_range=None, clear_margin_mm=30.0):
    """결속 교차점(로봇XY mm) 리스트 → pitch/열/행 + 다음 전진·측면 거리.

    points_xy : [(x,y), ...]  검출된 교차점 전부 (r+l 자세 합쳐서)
    prev_pitch: (pitch_x, pitch_y) 이전 추정값 (열/행 1개뿐일 때 폴백)
    tie_x_range/tie_y_range: (min, max) **결속 가능(스테이지 도달) 범위** mm.
        주면 **이 범위 안 교차점만으로** 간격과 열/행 수를 구한다.
        None이면 검출된 전체를 쓴다.

    ★ 원리: "도달범위 안 교차점은 결속했다"고 보고, **결속한 열들을 지나칠 만큼** 전진하면
       다음 미결속 열에 선다. 이동거리는 **마지막 결속 열 기준**으로 잡는다:

           d = (마지막 결속 열 X) - (도달범위 시작) + clear_margin_mm

       유효 구간은 `마지막 열 < d ≤ 마지막 열 + pitch` (아래는 재결속, 위는 열 누락).
       ⚠ 예전엔 `열수 × pitch`를 썼는데 이건 **첫 열이 도달범위 시작에 있을 때만** 맞다.
          실측(2026-08-06): 열 [154.7, 311.6], pitch 156.9 → 313.8mm 지령했으나 유효
          하한(311.6)보다 2.2mm 큰 값이라, 도달오차 14mm가 겹쳐 300mm만 가서
          311.6 열이 11.6mm 지점에 남아 **재결속**됐다. 마진은 도달오차(tolerance_mm)를
          덮을 만큼 줘야 한다.
       카메라는 스테이지보다 멀리 보므로, 검출 전체를 쓰면 도달 못 한 열까지 세어
       **결속 안 된 열을 건너뛴다**.

    폴백: 범위 안 열이 1개뿐이라 간격을 못 재면 → 전체 검출로 추정 → prev_pitch →
          default 순으로 내려간다 (제자리걸음 방지).

    반환 dict: pitch_x/y, n_cols/n_rows(=범위 안), n_cols_all/n_rows_all(=검출 전체),
               cols_x/rows_y(범위 안), next_forward_mm, next_lateral_mm,
               pitch_src, confidence
    """
    pts = np.asarray(points_xy, float)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return {'error': 'no points', 'n_cols': 0, 'n_rows': 0}
    all_x, all_y = pts[:, 0].tolist(), pts[:, 1].tolist()

    def _keep(v, rng):
        return True if rng is None else (rng[0] <= v <= rng[1])

    # ★ 결속 가능 범위 안 교차점만 남긴다 (범위 밖은 '결속 안 됨' → 세면 안 됨)
    tie = [(x, y) for x, y in zip(all_x, all_y)
           if _keep(x, tie_x_range) and _keep(y, tie_y_range)]
    # ⚠ 비었다고 전체로 폴백하면 안 된다 — 도달 못 하는 열을 세어 건너뛰게 된다.
    #   열 수는 0이 되고(→ 아래에서 최소 1피치), 간격만 전체 검출로 폴백한다.
    txs = [x for x, _ in tie]
    tys = [y for _, y in tie]

    cols_x = [c for c, _ in cluster_1d(txs, cluster_tol_mm)]      # 열 (X)
    rows_y = [c for c, _ in cluster_1d(tys, cluster_tol_mm)]      # 행 (Y)
    all_cols = [c for c, _ in cluster_1d(all_x, cluster_tol_mm)]
    all_rows = [c for c, _ in cluster_1d(all_y, cluster_tol_mm)]
    ppx, ppy = prev_pitch

    def _pitch(centers, all_centers, prev):
        """범위 안 → 전체 → prev → default 순 폴백.
        (estimate_pitch의 fallback 값은 여기서 안 쓰므로 0.0을 넘기고 src로만 판단)"""
        p, src = estimate_pitch(centers, 0.0)
        if src == 'measured':
            return p, 'measured'
        p, src = estimate_pitch(all_centers, 0.0)
        if src == 'measured':
            return p, 'measured(all)'
        if prev:
            return float(prev), 'prev'
        return float(default_pitch_mm), 'fallback'

    pitch_x, srcx = _pitch(cols_x, all_cols, ppx)
    pitch_y, srcy = _pitch(rows_y, all_rows, ppy)
    n_cols, n_rows = len(cols_x), len(rows_y)

    def _advance(centers, rng, pitch):
        """마지막 결속 열/행을 지나칠 거리. 범위 안이 비면 1피치(제자리걸음 방지).
        최소 1피치는 보장 — 검출이 열을 놓쳐 last가 작게 잡혀도 진행은 하도록."""
        if not centers:
            return pitch
        start = rng[0] if rng else 0.0
        return max((max(centers) - start) + clear_margin_mm, pitch)

    def _lat_window(centers, rng, pitch):
        """다음 n행이 결속범위에 다 들어오는 이동거리 구간 [lo, hi]. 못 구하면 None.

        이번에 n행을 결속했으므로 다음 미결속 행은 **+n피치** 위치에 있다.
        그 첫 행이 범위 안 [y0, y1-(n-1)피치]에 놓이도록 이동하면 n행이 다 들어온다.
        """
        n = len(centers)
        if n < 1 or not rng or pitch <= 0:
            return None
        y0, y1 = float(rng[0]), float(rng[1])
        span = (n - 1) * pitch                 # 첫 행~마지막 행 거리
        if y1 - y0 < span:                     # 범위가 좁아 n행을 못 담는다
            return None
        nxt_first = min(centers) + n * pitch   # 다음 미결속 첫 행 (현재 좌표계)
        lo = nxt_first - (y1 - span)           # 첫 행을 범위 위끝에 두는 이동
        hi = nxt_first - y0                    # 첫 행을 범위 아래끝에 두는 이동
        if hi <= 0:
            return None
        return [round(max(0.0, lo), 1), round(hi, 1)]

    conf = 'high' if (srcx == 'measured' and srcy == 'measured') else 'low'
    return {
        'pitch_x': round(pitch_x, 1), 'pitch_y': round(pitch_y, 1),
        'n_cols': n_cols, 'n_rows': n_rows,
        'n_cols_all': len(all_cols), 'n_rows_all': len(all_rows),
        'cols_x': [round(c, 1) for c in cols_x],
        'rows_y': [round(c, 1) for c in rows_y],
        'last_col_x': round(max(cols_x), 1) if cols_x else None,
        'clear_margin_mm': clear_margin_mm,
        'next_forward_mm': round(_advance(cols_x, tie_x_range, pitch_x), 1),
        # ★ 횡이동 유효 구간 [lo, hi] — 다음 n행이 결속범위에 **둘 다** 들어오는 거리.
        #   회전 크기(70mm)는 여기서 모르므로 구간만 주고, 주행노드가 그 안에서
        #   **최소 회전수**를 고른다. 올림으로 하나만 정하면 회전을 한 번 더 하게 된다
        #   (실측: 구간 345~428mm인데 올림값 411 → 6회전. 5회전 350도 유효한데 낭비).
        'lateral_window_mm': _lat_window(rows_y, tie_y_range, pitch_y),
        # ★ 횡이동은 **결속한 행 수 × 피치**다 (2026-09-02 수정).
        #   전후진과 계산이 다른 이유: 한 번 서면 **행 2개를 동시에 결속**하므로,
        #   "마지막 행을 지나칠 거리"(전후진 방식)만 가면 다음 레인엔 1행만
        #   범위에 들어와 레인 전환이 두 배로 필요해진다.
        #     실측(배근 200mm): rows_y=[16, 222] → 옛 계산 252mm (다음 레인 1행)
        #                                       → 새 계산 410mm (다음 2행이 16/221로 안착)
        #   n_rows=0이면 제자리걸음 방지로 1피치.
        'next_lateral_mm': round(max(n_rows, 1) * pitch_y, 1),
        'pitch_src': (srcx, srcy), 'confidence': conf,
    }


def _fmt(r):
    if 'error' in r:
        return f"  {r['error']}"
    return (f"  pitch=({r['pitch_x']},{r['pitch_y']})mm  "
            f"열{r['n_cols']}×행{r['n_rows']}  "
            f"cols_x={r['cols_x']} rows_y={r['rows_y']}\n"
            f"  → 다음 전/후진 {r['next_forward_mm']}mm, 측면 {r['next_lateral_mm']}mm "
            f"[{r['confidence']}, src={r['pitch_src']}]")
