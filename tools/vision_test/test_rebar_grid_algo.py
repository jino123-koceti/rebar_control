#!/usr/bin/env python3
"""
철근 격자 검출 알고리즘 테스트 (headless 배치)

목적: 노드로 만들기 전에 순수 CV 알고리즘을 저장된 전방 카메라 이미지로 검증한다.
주행제어 보조 4대 목표:
  1) 직진/정렬 가이드  - 가로철근 평균 기울기 → heading 오차
  2) 접근 정렬        - 세로 방향(트러스 중심선) → lateral 오프셋
  3) 이동거리 산출     - 가로철근 간격/카운트 × 배근 피치
  4) 배근 철근 누적 카운팅 - 가로철근 개수

이 장면은 "트러스거더 데크플레이트"라 세로는 트러스 대각재/데크 리브와
섞여 어렵고, **가로철근(횡방향)이 가장 신뢰도 높은 신호**다. 따라서
1차 알고리즘은 가로철근 검출/카운팅을 핵심으로 한다.

사용:
    python3 test_rebar_grid_algo.py                     # 기본 폴더 전체 처리
    python3 test_rebar_grid_algo.py -i img1.png img2.png
    python3 test_rebar_grid_algo.py --outdir /tmp/out

결과: <outdir>/<name>_montage.png (원본+검출 | 엣지) 저장 + 콘솔 요약.
GUI 불필요(imshow 없음) → 서버/헤드리스에서 동작.
"""

import argparse
import glob
import json
import os
import math

import cv2
import numpy as np

# ==========================================================================
#  튜닝 파라미터 (여기만 고치면서 반복)
# ==========================================================================
PARAMS = {
    # ROI: 전방 지면 격자 영역만 (상단=지평선/장비, 비율 0~1)
    "roi_top": 0.18,
    "roi_bottom": 0.98,

    # 전처리
    "clahe_clip": 2.0,
    "clahe_grid": 8,
    "blur": 5,
    "canny_low": 40,
    "canny_high": 120,
    "close_ksize": 3,          # 가로선 연결용 수평 모폴로지 (0=off)

    # Hough
    "hough_thresh": 50,
    "min_len_ratio": 0.18,     # 최소 선분 길이 = ROI폭 * 이 비율
    "max_gap": 25,

    # 가로철근(rung) 판정
    "horiz_angle_tol": 18.0,   # |angle| < tol → 가로
    "rung_merge_px": 22,       # 같은 rung으로 병합할 y 간격
    "rung_min_span": 0.35,     # rung이 ROI폭에서 차지할 최소 비율

    # 세로/트러스(rail) 판정 (참고용)
    "vert_angle_min": 55.0,    # |angle| > 이 값 → 세로(급경사)

    # v2 방향성 필터
    "v2_open_kernel_ratio": 0.05,   # open 커널(대각/수직 제거), ROI폭 * 값
    "v2_close_kernel_ratio": 0.15,  # close 커널(끊긴 철근 잇기)
    "v2_bin_thresh": 35,            # 수평맵 이진화 임계
    "v2_row_smooth": 5,             # 행 투영 스무딩 창
    "v2_peak_min_ratio": 0.15,      # 피크 최소 폭 = ROI폭 * 값
    "v2_ignore_bottom_ratio": 0.05, # 하단 데크모서리 밴드 제외 비율
    "method": "v2",                 # "v1"(Hough) 또는 "v2"(방향성)
}


def preprocess(roi, p):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=p["clahe_clip"],
                            tileGridSize=(p["clahe_grid"], p["clahe_grid"]))
    gray = clahe.apply(gray)
    k = max(1, p["blur"] | 1)
    gray = cv2.GaussianBlur(gray, (k, k), 0)
    edges = cv2.Canny(gray, p["canny_low"], p["canny_high"])
    if p["close_ksize"] > 0:
        # 가로선 연결을 돕는 수평 방향 모폴로지 close
        hk = cv2.getStructuringElement(cv2.MORPH_RECT, (p["close_ksize"] * 3, 1))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, hk)
    return gray, edges


# ==========================================================================
#  v2: 방향성 필터 기반 가로철근 검출 (대각재 노이즈에 강건)
#  원리: 수평 철근은 "수직 그래디언트(Sobel_y)"가 강함. 이를 긴 수평
#        커널로 강조 → 행 투영(row projection) 피크 = 가로철근.
#        대각 트러스는 행마다 흩어져 피크를 만들지 않음.
# ==========================================================================
def detect_v2(roi, p):
    rh, rw = roi.shape[:2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=p["clahe_clip"],
                            tileGridSize=(p["clahe_grid"], p["clahe_grid"]))
    gray = clahe.apply(gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # 1) 수직 그래디언트(수평 엣지 강조)
    sob = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.abs(sob)
    mag = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # 2) open: 짧은 수평 커널로 대각/수직 구조 제거 (가로철근은 보존)
    klen = max(9, int(rw * p["v2_open_kernel_ratio"]) | 1)
    hkernel = cv2.getStructuringElement(cv2.MORPH_RECT, (klen, 1))
    horiz_map = cv2.morphologyEx(mag, cv2.MORPH_OPEN, hkernel)

    # 3) 이진화
    _, binmap = cv2.threshold(horiz_map, p["v2_bin_thresh"], 255,
                              cv2.THRESH_BINARY)

    # 4) close: 긴 수평 커널로 끊긴 철근 조각을 이음
    clen = max(11, int(rw * p["v2_close_kernel_ratio"]) | 1)
    ckernel = cv2.getStructuringElement(cv2.MORPH_RECT, (clen, 1))
    binmap = cv2.morphologyEx(binmap, cv2.MORPH_CLOSE, ckernel)

    # 5) 행 투영: 각 행의 수평구조 픽셀 수
    row_score = binmap.sum(axis=1) / 255.0
    ksm = np.ones(p["v2_row_smooth"]) / p["v2_row_smooth"]
    row_score_s = np.convolve(row_score, ksm, mode="same")

    # 6) 피크 검출 (국소 최대 + 최소 폭). 하단 데크모서리 밴드는 제외
    min_pix = rw * p["v2_peak_min_ratio"]
    ignore_y = rh * (1.0 - p["v2_ignore_bottom_ratio"])
    peaks = []
    md = p["rung_merge_px"]
    for y in range(rh):
        if y >= ignore_y:
            continue
        s = row_score_s[y]
        if s < min_pix:
            continue
        lo, hi = max(0, y - md), min(rh, y + md + 1)
        if s >= row_score_s[lo:hi].max() - 1e-6:
            if not peaks or (y - peaks[-1][0]) > md:
                peaks.append((y, s))
            elif s > peaks[-1][1]:
                peaks[-1] = (y, s)

    # 6) 각 피크의 수평 span/각도 산출
    rungs = []
    for y, s in peaks:
        band = binmap[max(0, y - md // 2):y + md // 2 + 1, :]
        cols = np.where(band.any(axis=0))[0]
        if len(cols) < 2:
            continue
        x1, x2 = int(cols.min()), int(cols.max())
        if (x2 - x1) / rw < p["rung_min_span"]:
            continue
        # 밴드 내 픽셀로 각도(기울기) 추정
        ys_p, xs_p = np.where(band > 0)
        angle = 0.0
        if len(xs_p) > 10 and xs_p.ptp() > 5:
            a = np.polyfit(xs_p, ys_p, 1)[0]      # dy/dx
            angle = math.degrees(math.atan(a))
        rungs.append(dict(y=float(y), x1=x1, x2=x2, angle=angle,
                          score=float(s)))
    return dict(edges=binmap, rungs=rungs, horiz=[], vert=[], diag=[],
                row_score=row_score_s)


def line_angle(x1, y1, x2, y2):
    """수평=0, 수직=±90 으로 정규화된 각도(deg)."""
    ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
    if ang > 90:
        ang -= 180
    elif ang < -90:
        ang += 180
    return ang


def detect(roi, p):
    rh, rw = roi.shape[:2]
    gray, edges = preprocess(roi, p)
    min_len = int(rw * p["min_len_ratio"])
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, p["hough_thresh"],
                            minLineLength=min_len, maxLineGap=p["max_gap"])

    horiz, vert, diag = [], [], []
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0, :]:
            ang = line_angle(x1, y1, x2, y2)
            length = math.hypot(x2 - x1, y2 - y1)
            info = dict(pts=(int(x1), int(y1), int(x2), int(y2)),
                        angle=ang, length=length,
                        cy=(y1 + y2) / 2.0, cx=(x1 + x2) / 2.0,
                        span=abs(x2 - x1) / max(1, rw))
            if abs(ang) <= p["horiz_angle_tol"]:
                if info["span"] >= p["rung_min_span"]:
                    horiz.append(info)
            elif abs(ang) >= p["vert_angle_min"]:
                vert.append(info)
            else:
                diag.append(info)

    rungs = cluster_rungs(horiz, p)
    return dict(edges=edges, rungs=rungs, horiz=horiz, vert=vert, diag=diag)


def cluster_rungs(horiz, p):
    """가로 선분을 y로 병합 → rung 리스트 (y오름차순 = 먼→가까운)."""
    if not horiz:
        return []
    hs = sorted(horiz, key=lambda l: l["cy"])
    groups, cur = [], [hs[0]]
    for l in hs[1:]:
        if l["cy"] - cur[-1]["cy"] > p["rung_merge_px"]:
            groups.append(cur)
            cur = []
        cur.append(l)
    groups.append(cur)

    rungs = []
    for g in groups:
        ys = [l["cy"] for l in g]
        xs1 = [min(l["pts"][0], l["pts"][2]) for l in g]
        xs2 = [max(l["pts"][0], l["pts"][2]) for l in g]
        angs = [l["angle"] for l in g]
        rungs.append(dict(y=float(np.mean(ys)),
                          x1=int(min(xs1)), x2=int(max(xs2)),
                          angle=float(np.mean(angs)),
                          n=len(g)))
    return rungs


def analyze(rungs, rw):
    """4대 목표용 지표 산출."""
    out = dict(count=len(rungs), heading_deg=0.0,
               spacings=[], lateral_px=0.0)
    if rungs:
        out["heading_deg"] = float(np.median([r["angle"] for r in rungs]))
        ys = sorted(r["y"] for r in rungs)
        out["spacings"] = [round(ys[i + 1] - ys[i], 1)
                           for i in range(len(ys) - 1)]
        # rung 중심 x의 중앙값 vs ROI 중심 → 대략적 lateral
        cxs = [(r["x1"] + r["x2"]) / 2.0 for r in rungs]
        out["lateral_px"] = float(np.median(cxs) - rw / 2.0)
    return out


def draw(img, x0, y0, res, metric, p):
    vis = img.copy()
    rh, rw = res["edges"].shape[:2]
    cv2.rectangle(vis, (x0, y0), (x0 + rw, y0 + rh), (90, 90, 90), 1)

    # 트러스 대각재(참고) - 얇은 회색
    for l in res["diag"]:
        a, b, c, d = l["pts"]
        cv2.line(vis, (x0 + a, y0 + b), (x0 + c, y0 + d), (110, 110, 110), 1)
    # 세로/트러스 급경사 - 파랑
    for l in res["vert"]:
        a, b, c, d = l["pts"]
        cv2.line(vis, (x0 + a, y0 + b), (x0 + c, y0 + d), (255, 160, 0), 1)
    # 가로철근(rung) - 초록 굵게 + 번호
    for i, r in enumerate(res["rungs"]):
        yy = int(y0 + r["y"])
        cv2.line(vis, (x0 + r["x1"], yy), (x0 + r["x2"], yy), (0, 230, 0), 3)
        cv2.putText(vis, f"{i}", (x0 + r["x1"] - 22, yy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 0), 2)

    # HUD
    hud = [
        f"rungs(count):{metric['count']}  heading:{metric['heading_deg']:+.1f}deg",
        f"spacing_px:{metric['spacings']}",
        f"lateral_px:{metric['lateral_px']:+.0f}  "
        f"vert:{len(res['vert'])} diag:{len(res['diag'])}",
    ]
    cv2.rectangle(vis, (0, 0), (vis.shape[1], 82), (0, 0, 0), -1)
    for i, t in enumerate(hud):
        cv2.putText(vis, t, (10, 24 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2)
    return vis


def montage(vis, edges, x0, y0):
    """원본+검출 위, 엣지(ROI) 아래로 세로 몽타주."""
    h, w = vis.shape[:2]
    edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    # ROI 엣지를 전체폭 캔버스에 배치
    canvas = np.zeros((edges.shape[0], w, 3), np.uint8)
    canvas[:, x0:x0 + edges.shape[1]] = edges_bgr
    return np.vstack([vis, canvas])


def process_one(path, outdir, p):
    img = cv2.imread(path)
    if img is None:
        print(f"  ! 읽기 실패: {path}")
        return None
    h, w = img.shape[:2]
    y0, y1 = int(h * p["roi_top"]), int(h * p["roi_bottom"])
    x0, x1 = 0, w
    roi = img[y0:y1, x0:x1]

    res = detect_v2(roi, p) if p.get("method") == "v2" else detect(roi, p)
    metric = analyze(res["rungs"], roi.shape[1])
    vis = draw(img, x0, y0, res, metric, p)
    mont = montage(vis, res["edges"], x0, y0)

    name = os.path.splitext(os.path.basename(path))[0]
    os.makedirs(outdir, exist_ok=True)
    out_png = os.path.join(outdir, f"{name}_montage.png")
    cv2.imwrite(out_png, mont)

    print(f"  {name}: count={metric['count']} "
          f"heading={metric['heading_deg']:+.1f} "
          f"spacing={metric['spacings']} lat={metric['lateral_px']:+.0f} "
          f"(vert={len(res['vert'])},diag={len(res['diag'])}) -> {out_png}")
    return metric


def main():
    ap = argparse.ArgumentParser()
    default_glob = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "data", "vision_test",
                                "rebar_lines", "*original*.png")
    ap.add_argument("-i", "--images", nargs="+", default=None,
                    help="입력 이미지들 (기본: rebar_lines/*original*.png)")
    ap.add_argument("--outdir", default="/tmp/rebar_grid_test",
                    help="결과 저장 폴더")
    args = ap.parse_args()

    images = args.images if args.images else sorted(glob.glob(default_glob))
    if not images:
        print("입력 이미지 없음.")
        return

    print(f"파라미터: {json.dumps(PARAMS)}")
    print(f"이미지 {len(images)}장 처리 → {args.outdir}\n")
    metrics = []
    for path in images:
        m = process_one(path, args.outdir, PARAMS)
        if m:
            metrics.append(m)

    if metrics:
        counts = [m["count"] for m in metrics]
        print(f"\n요약: rung count 평균={np.mean(counts):.1f} "
              f"min={min(counts)} max={max(counts)}")


if __name__ == "__main__":
    main()
