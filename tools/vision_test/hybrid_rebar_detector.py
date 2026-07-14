#!/usr/bin/env python3
"""
하이브리드 철근 격자 검출기 프로토타입 (YOLO-seg + OpenCV 후처리)

파이프라인:
  1) YOLO-seg (rebar_seg_proto.pt) → rebar_h / rebar_v / floor 마스크
  2) rebar_h 마스크 → 인스턴스별 중심선 피팅 → 카운트/간격/heading
  3) rebar_v 마스크 → 인스턴스별 직선 피팅 → 소실점 → heading/lateral
  4) floor 마스크 → 주행가능 영역(참고)

주행제어 4대 지표:
  ① heading_deg   : 정렬(직진) 오차. rebar_h 평균 기울기 + rebar_v 소실점 x
  ② lateral_px    : 접근 정렬. rebar_v 하단 레일 중심 - 이미지 중심
  ③ spacing_px    : 이동거리 산출용 rebar_h 간격 (주행 영상에서 프레임간 추적으로 확장)
  ④ count_h       : 배근 가로철근 개수

이 파일은 ROS 비의존 headless 테스트. 결과 PNG 저장 (imshow 없음).

사용:
  python3 hybrid_rebar_detector.py                 # 기본 2개 고유 프레임
  python3 hybrid_rebar_detector.py -i a.png b.png
  python3 hybrid_rebar_detector.py --model captured_training/rebar_seg_proto.pt
"""

import argparse
import glob
import os

import cv2
import numpy as np
from ultralytics import YOLO

# 클래스 색 (BGR)
COL = {"rebar": (0, 200, 200), "floor": (70, 70, 70),
       "rebar_h": (0, 230, 0), "rebar_v": (255, 90, 0)}

PARAMS = {
    "conf": 0.25,
    "min_mask_pixels": 200,     # 이보다 작은 인스턴스 무시
    "hmerge_px": 18,            # rebar_h 중복 인스턴스 병합 y 임계
    "roi_top": 0.15,            # lateral 기준행(하단) 참고용
}


# --------------------------------------------------------------------------
#  기하 유틸
# --------------------------------------------------------------------------
def fit_line(mask):
    """마스크 픽셀에 직선 피팅 → (vx,vy,x0,y0), 각도(deg,수평=0)."""
    ys, xs = np.where(mask)
    if len(xs) < 10:
        return None
    pts = np.column_stack([xs, ys]).astype(np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    ang = np.degrees(np.arctan2(vy, vx))
    if ang > 90:
        ang -= 180
    elif ang < -90:
        ang += 180
    return dict(vx=float(vx), vy=float(vy), x0=float(x0), y0=float(y0),
                angle=float(ang), xs=xs, ys=ys,
                xmin=int(xs.min()), xmax=int(xs.max()),
                ymin=int(ys.min()), ymax=int(ys.max()),
                cx=float(xs.mean()), cy=float(ys.mean()))


def _merge_close(hlines, tol):
    """cy가 tol 이내로 겹치는 rebar_h 인스턴스를 하나로 병합(더 넓은 것 우선)."""
    merged = []
    for l in hlines:
        if merged and abs(l["cy"] - merged[-1]["cy"]) <= tol:
            # span이 더 넓은 쪽을 대표로
            if (l["xmax"] - l["xmin"]) > (merged[-1]["xmax"] - merged[-1]["xmin"]):
                merged[-1] = l
        else:
            merged.append(l)
    return merged


def line_x_at_y(l, y):
    """직선 위 주어진 y에서의 x."""
    if abs(l["vy"]) < 1e-6:
        return l["x0"]
    t = (y - l["y0"]) / l["vy"]
    return l["x0"] + t * l["vx"]


def line_y_at_x(l, x):
    if abs(l["vx"]) < 1e-6:
        return l["y0"]
    t = (x - l["x0"]) / l["vx"]
    return l["y0"] + t * l["vy"]


def vanishing_point(lines):
    """여러 직선의 최소제곱 교점(소실점)."""
    if len(lines) < 2:
        return None
    A, b = [], []
    for l in lines:
        # 단위 법선 n=(-vy,vx); n·X = n·p
        nx, ny = -l["vy"], l["vx"]
        norm = np.hypot(nx, ny)
        if norm < 1e-6:
            continue
        nx, ny = nx / norm, ny / norm
        A.append([nx, ny])
        b.append(nx * l["x0"] + ny * l["y0"])
    if len(A) < 2:
        return None
    A, b = np.array(A), np.array(b)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return float(sol[0]), float(sol[1])


# --------------------------------------------------------------------------
#  검출
# --------------------------------------------------------------------------
def detect(model, img, p):
    h, w = img.shape[:2]
    res = model.predict(img, conf=p["conf"], verbose=False)[0]

    inst = {"rebar_h": [], "rebar_v": [], "rebar": [], "floor": []}
    masks_by_cls = {"rebar_h": None, "rebar_v": None, "floor": None, "rebar": None}
    names = model.names

    if res.masks is not None:
        for m, cls in zip(res.masks.data.cpu().numpy(),
                          res.boxes.cls.cpu().numpy().astype(int)):
            cname = names[cls]
            mask = cv2.resize(m, (w, h)) > 0.5
            if mask.sum() < p["min_mask_pixels"]:
                continue
            # 병합 마스크(시각화용)
            if masks_by_cls[cname] is None:
                masks_by_cls[cname] = np.zeros((h, w), bool)
            masks_by_cls[cname] |= mask
            if cname in ("rebar_h", "rebar_v"):
                l = fit_line(mask)
                if l:
                    inst[cname].append(l)

    # --- rebar_h 분석: 중복(같은 철근 겹친 마스크) 병합 후 y 정렬 ---
    hlines = _merge_close(sorted(inst["rebar_h"], key=lambda l: l["cy"]),
                          p["hmerge_px"])
    inst["rebar_h"] = hlines
    heading_h = float(np.median([l["angle"] for l in hlines])) if hlines else None
    ys = [l["cy"] for l in hlines]
    spacing = [round(ys[i + 1] - ys[i], 1) for i in range(len(ys) - 1)]

    # --- rebar_v 분석: 소실점 + 하단 레일 lateral ---
    vlines = inst["rebar_v"]
    vp = vanishing_point(vlines)
    # 소실점 x가 이미지 중심에서 벗어난 정도 = yaw 지표
    heading_vp = None
    if vp:
        heading_vp = float(np.degrees(np.arctan2(vp[0] - w / 2.0, h)))  # 근사
    # 하단 기준행에서 레일 x → lateral (로봇 중심 대비)
    ref_y = h * 0.9
    lateral_px = None
    if vlines:
        rail_xs = sorted(line_x_at_y(l, ref_y) for l in vlines)
        # 이미지 중심에 가장 가까운 좌우 레일쌍의 중점
        cx = w / 2.0
        left = [x for x in rail_xs if x <= cx]
        right = [x for x in rail_xs if x > cx]
        if left and right:
            mid = (max(left) + min(right)) / 2.0
            lateral_px = float(mid - cx)
        else:
            lateral_px = float(np.median(rail_xs) - cx)

    return dict(
        inst=inst, masks=masks_by_cls,
        count_h=len(hlines), heading_h=heading_h, spacing=spacing,
        count_v=len(vlines), vp=vp, heading_vp=heading_vp,
        lateral_px=lateral_px, ref_y=ref_y,
    )


# --------------------------------------------------------------------------
#  시각화
# --------------------------------------------------------------------------
def draw(img, r, p):
    h, w = img.shape[:2]
    vis = img.copy()
    # 마스크 오버레이
    for cname in ("floor", "rebar_v", "rebar_h"):
        m = r["masks"].get(cname)
        if m is not None:
            vis[m] = (0.5 * vis[m] + 0.5 * np.array(COL[cname])).astype(np.uint8)

    # rebar_h 중심선 (흰색) + 번호
    for i, l in enumerate(sorted(r["inst"]["rebar_h"], key=lambda x: x["cy"])):
        x1, x2 = l["xmin"], l["xmax"]
        y1 = int(line_y_at_x(l, x1)); y2 = int(line_y_at_x(l, x2))
        cv2.line(vis, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(vis, str(i), (x1 - 22, (y1 + y2) // 2 + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # rebar_v 중심선 (노랑, 소실점까지 연장)
    for l in r["inst"]["rebar_v"]:
        y1, y2 = l["ymin"], l["ymax"]
        x1 = int(line_x_at_y(l, y1)); x2 = int(line_x_at_y(l, y2))
        cv2.line(vis, (x1, y1), (x2, y2), (0, 255, 255), 1)

    # 소실점
    if r["vp"]:
        vx, vy = int(r["vp"][0]), int(r["vp"][1])
        if 0 <= vx < w and -h < vy < 2 * h:
            cv2.circle(vis, (vx, max(0, min(h - 1, vy))), 8, (0, 0, 255), -1)
    # 이미지 중심선
    cv2.line(vis, (w // 2, 0), (w // 2, h), (150, 150, 150), 1)
    # lateral 기준행 + 오프셋
    ry = int(r["ref_y"])
    if r["lateral_px"] is not None:
        mx = int(w / 2 + r["lateral_px"])
        cv2.arrowedLine(vis, (w // 2, ry), (mx, ry), (0, 0, 255), 2, tipLength=0.3)

    # HUD
    hud = [
        f"count_h:{r['count_h']}  spacing_px:{r['spacing']}",
        f"heading_h:{_f(r['heading_h'])}deg  heading_vp:{_f(r['heading_vp'])}deg",
        f"count_v:{r['count_v']}  lateral_px:{_f(r['lateral_px'])}",
    ]
    cv2.rectangle(vis, (0, 0), (w, 84), (0, 0, 0), -1)
    for i, t in enumerate(hud):
        cv2.putText(vis, t, (10, 25 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2)
    return vis


def _f(v):
    return f"{v:+.1f}" if isinstance(v, (int, float)) else "None"


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    ap.add_argument("--model",
                    default=os.path.join(root, "captured_training",
                                         "rebar_seg_proto.pt"))
    ap.add_argument("-i", "--images", nargs="+", default=None)
    ap.add_argument("--outdir",
                    default=os.path.join(root, "data", "vision_test",
                                         "rebar_grid_results"))
    args = ap.parse_args()

    default = [os.path.join(root, "data", "vision_test", "rebar_lines", f)
               for f in ("20260324_182117_auto_first_original.png",
                         "20260324_183652_vert_test_original.png")]
    images = args.images if args.images else default

    print(f"모델 로드: {args.model}")
    model = YOLO(args.model)
    os.makedirs(args.outdir, exist_ok=True)

    for path in images:
        img = cv2.imread(path)
        if img is None:
            print(f"  ! 읽기 실패: {path}")
            continue
        r = detect(model, img, PARAMS)
        vis = draw(img, r, PARAMS)
        name = os.path.splitext(os.path.basename(path))[0]
        out = os.path.join(args.outdir, f"{name}_hybrid.png")
        cv2.imwrite(out, vis)
        print(f"  {name}: count_h={r['count_h']} heading_h={_f(r['heading_h'])} "
              f"count_v={r['count_v']} heading_vp={_f(r['heading_vp'])} "
              f"lateral_px={_f(r['lateral_px'])} spacing={r['spacing']}")
        print(f"      -> {out}")


if __name__ == "__main__":
    main()
