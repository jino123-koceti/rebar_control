#!/usr/bin/env python3
"""
주행 시퀀스 rung 추적/카운팅 검증 (YOLO 하이브리드 + odom 정답 대조)

capture_drive_frames.py로 저장한 주행 프레임 시퀀스에 하이브리드 검출기를
프레임순으로 적용하여:
  - rebar_h(가로철근) 중심선을 프레임 간 추적
  - 카운트 라인 통과 = 누적 카운팅(④)
  - 카운트 × 피치 = 이동거리(③)
  - **odom_x(정답)와 대조**해 정확도 검증

피치를 모르므로 odom으로 역산: (관측 통과수) vs (odom 이동거리) 선형회귀 →
기울기 = rungs/m = 1/피치. R²이 높으면 "비전 카운팅 ∝ 실제 거리" 검증 성공.

사용:
  python3 track_drive_sequence.py --dir data/vision_test/drive_20260714_115833 \
        --start 1 --end 105
결과: <dir>/track/  (annotated .mp4, validation_plot.png, track_log.csv)
"""

import argparse
import csv
import os
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from ultralytics import YOLO

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hybrid_rebar_detector import (fit_line, vanishing_point,
                                   line_x_at_y, line_y_at_x)

# --- 파라미터 ---
CONF = 0.25
MIN_MASK_PX = 200
ROI_TOP = 0.15           # 상단(실내/벽) 잘라내기
COUNT_LINE = 0.70        # 전체 높이 대비 카운트 라인 (0.55→0.70: up노이즈 제거, R²0.86→0.97)
MATCH_TOL = 60           # 프레임 간 rung 매칭 허용 y(px)
MAX_MISS = 6
HMERGE_PX = 20           # 같은 rung 중복 마스크 병합


def _f(v):
    return f"{v:+.1f}" if isinstance(v, (int, float)) else "None"


def detect_frame(model, roi):
    """YOLO-seg 1회 → rebar_h/rebar_v 인스턴스 (ROI 좌표계).

    반환: h_lines, v_lines (fit_line dict 리스트), h_mask, v_mask
    """
    rh, rw = roi.shape[:2]
    res = model.predict(roi, conf=CONF, verbose=False)[0]
    h_lines, v_lines = [], []
    h_mask = np.zeros((rh, rw), bool)
    v_mask = np.zeros((rh, rw), bool)
    names = model.names
    if res.masks is not None:
        for m, cls in zip(res.masks.data.cpu().numpy(),
                          res.boxes.cls.cpu().numpy().astype(int)):
            cname = names[cls]
            if cname not in ("rebar_h", "rebar_v"):
                continue
            mask = cv2.resize(m, (rw, rh)) > 0.5
            if mask.sum() < MIN_MASK_PX:
                continue
            l = fit_line(mask)
            if l is None:
                continue
            if cname == "rebar_h":
                h_lines.append(l); h_mask |= mask
            else:
                v_lines.append(l); v_mask |= mask
    h_lines.sort(key=lambda l: l["cy"])
    return h_lines, v_lines, h_mask, v_mask


def merge_h(h_lines):
    """같은 가로철근 중복 인스턴스를 cy로 병합 → 대표 y 리스트(ROI좌표)."""
    ys = []
    for l in h_lines:
        if ys and abs(l["cy"] - ys[-1]) <= HMERGE_PX:
            ys[-1] = (ys[-1] + l["cy"]) / 2.0
        else:
            ys.append(l["cy"])
    return ys


def analyze_v(v_lines, rw, rh):
    """세로철근 → 소실점, heading_vp, lateral_px (ROI좌표)."""
    vp = vanishing_point(v_lines) if len(v_lines) >= 2 else None
    heading_vp = None
    if vp:
        heading_vp = float(np.degrees(np.arctan2(vp[0] - rw / 2.0, rh)))
    lateral = None
    if v_lines:
        ref_y = rh * 0.9
        xs = sorted(line_x_at_y(l, ref_y) for l in v_lines)
        cx = rw / 2.0
        left = [x for x in xs if x <= cx]; right = [x for x in xs if x > cx]
        if left and right:
            lateral = float((max(left) + min(right)) / 2.0 - cx)
        else:
            lateral = float(np.median(xs) - cx)
    return vp, heading_vp, lateral


class Tracker:
    """rung을 프레임간 추적, 카운트라인 통과를 부호까지 카운팅."""
    def __init__(self, line_y):
        self.line_y = line_y
        self.tracks = []   # {'y','miss'}
        self.down = 0      # 아래로 통과(+)
        self.up = 0        # 위로 통과(-)

    def update(self, ys):
        used = [False] * len(ys)
        for tr in self.tracks:
            bi, bd = -1, MATCH_TOL + 1
            for i, y in enumerate(ys):
                if used[i]:
                    continue
                if abs(y - tr["y"]) < bd:
                    bd, bi = abs(y - tr["y"]), i
            if bi >= 0:
                py, ny = tr["y"], ys[bi]
                used[bi] = True
                tr["miss"] = 0
                if py < self.line_y <= ny:
                    self.down += 1
                elif py > self.line_y >= ny:
                    self.up += 1
                tr["y"] = ny
            else:
                tr["miss"] += 1
        for i, y in enumerate(ys):
            if not used[i]:
                self.tracks.append({"y": y, "miss": 0})
        self.tracks = [t for t in self.tracks if t["miss"] <= MAX_MISS]

    @property
    def net(self):
        return self.down - self.up


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--model", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..",
        "captured_training", "rebar_seg_proto.pt"))
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--end", type=int, default=0)   # 0=끝까지
    ap.add_argument("--count-line", type=float, default=COUNT_LINE,
                    help="카운트라인 위치(0=위,1=아래). 기본 0.55")
    ap.add_argument("--no-video", action="store_true",
                    help="mp4 생략(라인 위치 비교 시 빠르게)")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(os.path.join(args.dir, "manifest.csv"))))
    end = args.end or len(rows)
    rows = [r for r in rows if args.start <= int(r["idx"]) <= end]

    model = YOLO(args.model)
    outdir = os.path.join(args.dir, "track")
    os.makedirs(outdir, exist_ok=True)

    # 첫 프레임으로 크기/라인
    f0 = cv2.imread(os.path.join(args.dir, rows[0]["filename"]))
    h, w = f0.shape[:2]
    line_y = int(h * args.count_line)
    tracker = Tracker(line_y)
    vw = None
    if not args.no_video:
        vw = cv2.VideoWriter(os.path.join(outdir, "annotated.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), 10, (w, h))

    y0 = int(h * ROI_TOP)
    rh, rw = h - y0, w
    log = []
    for r in rows:
        img = cv2.imread(os.path.join(args.dir, r["filename"]))
        if img is None:
            continue
        roi = img[y0:]
        h_lines, v_lines, h_mask, v_mask = detect_frame(model, roi)

        # --- 가로철근: 카운팅 (ROI→전체좌표) ---
        ys_roi = merge_h(h_lines)
        ys = [y + y0 for y in ys_roi]
        tracker.update(ys)
        heading_h = float(np.median([l["angle"] for l in h_lines])) \
            if h_lines else None

        # --- 세로철근: heading/lateral ---
        vp, heading_vp, lateral = analyze_v(v_lines, rw, rh)

        odx = float(r["odom_x"]); odxs = float(r["dx_from_start_m"])
        log.append(dict(idx=int(r["idx"]), odom_x=odx, dx=odxs,
                        n_rungs=len(ys), net=tracker.net,
                        down=tracker.down, up=tracker.up,
                        n_vert=len(v_lines),
                        heading_h=round(heading_h, 2) if heading_h is not None else "",
                        heading_vp=round(heading_vp, 2) if heading_vp is not None else "",
                        lateral_px=round(lateral, 1) if lateral is not None else ""))

        # --- 시각화 ---
        vis = img.copy()
        ov = np.zeros_like(vis)
        ov[y0:][v_mask] = (255, 90, 0)     # 세로 파랑
        ov[y0:][h_mask] = (0, 230, 0)      # 가로 초록
        vis = cv2.addWeighted(vis, 1.0, ov, 0.45, 0)
        # 세로철근 피팅선(노랑) + 소실점(빨강)
        for l in v_lines:
            p1 = (int(line_x_at_y(l, l["ymin"])), int(l["ymin"] + y0))
            p2 = (int(line_x_at_y(l, l["ymax"])), int(l["ymax"] + y0))
            cv2.line(vis, p1, p2, (0, 255, 255), 1)
        if vp and 0 <= vp[0] < w:
            cv2.circle(vis, (int(vp[0]), int(np.clip(vp[1] + y0, 0, h - 1))),
                       7, (0, 0, 255), -1)
        # 카운트라인(주황) + 가로철근 중심선(흰)
        cv2.line(vis, (0, line_y), (w, line_y), (0, 200, 255), 2)
        for y in ys:
            cv2.line(vis, (0, int(y)), (w, int(y)), (255, 255, 255), 2)
        # 이미지 중심선 + lateral 화살표
        cv2.line(vis, (w // 2, y0), (w // 2, h), (150, 150, 150), 1)
        if lateral is not None:
            ry = int(y0 + rh * 0.9)
            cv2.arrowedLine(vis, (w // 2, ry), (int(w / 2 + lateral), ry),
                            (0, 0, 255), 2, tipLength=0.3)

        # HUD
        cv2.rectangle(vis, (0, 0), (w, 100), (0, 0, 0), -1)
        cv2.putText(vis, f"#{r['idx']} odom_x={odx:.3f}m dx={odxs:.3f}m",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(vis, f"[count] rungs={len(ys)} net={tracker.net} "
                    f"(dn={tracker.down} up={tracker.up})",
                    (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.putText(vis, f"[align] head_h={_f(heading_h)} head_vp={_f(heading_vp)} "
                    f"lat={_f(lateral)}px  vert={len(v_lines)}",
                    (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 170, 0), 2)
        if vw is not None:
            vw.write(vis)
    if vw is not None:
        vw.release()

    # 로그 저장
    with open(os.path.join(outdir, "track_log.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(log[0].keys()))
        wr.writeheader(); wr.writerows(log)

    # === 검증 플롯: net_cross vs odom 이동거리 ===
    dxs = np.array([l["dx"] for l in log])
    nets = np.array([l["net"] for l in log])
    odom_x = np.array([l["odom_x"] for l in log])

    # 선형회귀 (net = a*dx + b)
    fit = np.polyfit(dxs, nets, 1) if len(set(nets)) > 1 else [0, 0]
    pred = np.polyval(fit, dxs)
    ss_res = np.sum((nets - pred) ** 2)
    ss_tot = np.sum((nets - nets.mean()) ** 2) + 1e-9
    r2 = 1 - ss_res / ss_tot
    pitch = 1.0 / fit[0] if fit[0] != 0 else float("nan")

    fig, ax = plt.subplots(1, 2, figsize=(14, 5))
    ax[0].plot([l["idx"] for l in log], odom_x, "b-", label="odom_x (m)")
    ax[0].plot([l["idx"] for l in log], dxs, "c--", label="dx_from_start (m)")
    ax[0].set_xlabel("frame"); ax[0].set_ylabel("m"); ax[0].legend()
    ax[0].set_title("odom motion")

    ax[1].plot(dxs, nets, "go", ms=4, label="net crossings")
    ax[1].plot(dxs, pred, "r-",
               label=f"fit: {fit[0]:.2f}/m  R2={r2:.3f}\npitch~{pitch*1000:.0f}mm")
    ax[1].set_xlabel("odom distance dx (m)")
    ax[1].set_ylabel("cumulative net rung crossings")
    ax[1].legend(); ax[1].set_title("vision count vs odom (validation)")
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "validation_plot.png"), dpi=90)

    print(f"프레임 {len(log)}개 처리")
    print(f"총 net 통과: {tracker.net} (down={tracker.down}, up={tracker.up})")
    print(f"odom 이동거리: {dxs.max():.3f}m")
    print(f"선형회귀: {fit[0]:.2f} rungs/m, R2={r2:.3f}, 추정피치={pitch*1000:.0f}mm")
    print(f"결과: {outdir}/  (annotated.mp4, validation_plot.png, track_log.csv)")


if __name__ == "__main__":
    main()
