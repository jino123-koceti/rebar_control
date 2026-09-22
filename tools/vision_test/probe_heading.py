#!/usr/bin/env python3
"""7클래스 DeepLabV3+ 시맨틱 마스크에서 heading 추출 방식 실측 비교.

하이브리드 검출기(hybrid_rebar_detector.py)는 YOLO-seg **인스턴스** 마스크 전제라
철근 1개씩 fitLine이 가능했다. 7클래스 모델은 **시맨틱**이라 세로철근이 한 덩어리다.
게다가 교차점 픽셀이 h/v 중 하나로만 라벨되어 서로를 끊어먹는다.

여기서 세 가지를 비교한다:
  A) rebar_h 연결성분 + 모폴로지 closing → 밴드별 fitLine → 각도 중앙값
  B) rebar_h 행투영 피크 → 밴드별 fitLine → 각도 중앙값
  C) rebar_v 열클러스터 → 직선피팅 → 소실점 x → yaw 근사
출력: 프레임별 수치 + 오버레이 PNG.
"""
import os
import sys

import cv2
import numpy as np

ROOT = '/home/koceti/ros2_ws'
sys.path.insert(0, os.path.join(ROOT, 'src/rebar_vision'))
sys.path.insert(0, os.path.join(ROOT, 'tools/vision_test'))
from rebar_vision.rebar_seg import RebarSegmenter                      # noqa: E402
from hybrid_rebar_detector import fit_line, vanishing_point            # noqa: E402

WEIGHTS = os.path.join(ROOT, 'src/rebar_vision/model/retrain_best_260804.pt')
REC = os.path.join(ROOT, 'data/drive_rec/20260805_141630')
OUT = os.path.join(ROOT, 'data/vision_test/heading_probe')
IDX_H, IDX_V = 4, 5
SAMPLES = [('front', t) for t in (6.0, 12.0, 20.0, 26.0, 50.0, 60.0)] + \
          [('back', t) for t in (20.0, 30.0, 45.0)]


def instances_h(mask_h, min_px=300):
    """A) 가로철근 인스턴스: 가로방향 closing으로 교차점 끊김 메운 뒤 연결성분."""
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 3))
    closed = cv2.morphologyEx(mask_h.astype(np.uint8), cv2.MORPH_CLOSE, k)
    n, lab = cv2.connectedComponents(closed)
    out = []
    for i in range(1, n):
        m = lab == i
        if m.sum() < min_px:
            continue
        w = m.any(axis=0).sum()
        if w < mask_h.shape[1] * 0.15:      # 화면폭 15% 미만이면 조각
            continue
        l = fit_line(m)
        if l:
            out.append(l)
    return out


def instances_h_proj(mask_h, min_frac=0.05):
    """B) 행투영 피크로 밴드를 나눈 뒤 밴드별 fitLine."""
    prof = mask_h.mean(axis=1)
    thr = max(prof.max() * 0.25, min_frac)
    above = prof > thr
    bands, s = [], None
    for y, a in enumerate(above):
        if a and s is None:
            s = y
        elif not a and s is not None:
            bands.append((s, y)); s = None
    if s is not None:
        bands.append((s, len(above)))
    out = []
    for y0, y1 in bands:
        if y1 - y0 < 3:
            continue
        m = np.zeros_like(mask_h)
        m[y0:y1] = mask_h[y0:y1]
        if m.sum() < 200:
            continue
        l = fit_line(m)
        if l:
            out.append(l)
    return out


def instances_v(mask_v, min_px=200):
    """C) 세로철근 인스턴스: 세로방향 closing 후 연결성분."""
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 25))
    closed = cv2.morphologyEx(mask_v.astype(np.uint8), cv2.MORPH_CLOSE, k)
    n, lab = cv2.connectedComponents(closed)
    out = []
    for i in range(1, n):
        m = lab == i
        if m.sum() < min_px:
            continue
        h = m.any(axis=1).sum()
        if h < mask_v.shape[0] * 0.10:
            continue
        l = fit_line(m)
        if l:
            out.append(l)
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    import torch
    seg = RebarSegmenter(WEIGHTS, 'cuda' if torch.cuda.is_available() else 'cpu', 512)
    caps = {}
    print(f"{'frame':<14}{'A:h_cc':>18}{'B:h_proj':>18}{'C:vp_x/yaw':>20}")
    for cam, t in SAMPLES:
        if cam not in caps:
            caps[cam] = cv2.VideoCapture(os.path.join(REC, f'{cam}.mp4'))
        cap = caps[cam]
        fps = cap.get(cv2.CAP_PROP_FPS) or 15
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(t * fps))
        ok, fr = cap.read()
        if not ok:
            continue
        mask = seg.predict(fr)
        h, w = mask.shape
        mh, mv = (mask == IDX_H), (mask == IDX_V)

        A = instances_h(mh)
        B = instances_h_proj(mh)
        C = instances_v(mv)
        angA = np.median([l['angle'] for l in A]) if A else float('nan')
        angB = np.median([l['angle'] for l in B]) if B else float('nan')
        vp = vanishing_point(C) if len(C) >= 2 else None
        yaw = np.degrees(np.arctan2(vp[0] - w / 2.0, h)) if vp else float('nan')
        vpx = vp[0] if vp else float('nan')
        print(f'{cam}@{t:<8.0f}{len(A):>3}개 {angA:>+8.2f}°{len(B):>5}개 {angB:>+8.2f}°'
              f'{len(C):>5}개 x={vpx:>7.0f} {yaw:>+6.2f}°')

        vis = fr.copy()
        vis[mh] = (0.4 * vis[mh] + 0.6 * np.array([0, 230, 0])).astype(np.uint8)
        vis[mv] = (0.4 * vis[mv] + 0.6 * np.array([255, 90, 0])).astype(np.uint8)
        for l in A:                                    # 가로철근 중심선(흰)
            p0 = (int(l['xmin']), int(l['y0'] + (l['xmin'] - l['x0']) * l['vy'] / (l['vx'] or 1e-6)))
            p1 = (int(l['xmax']), int(l['y0'] + (l['xmax'] - l['x0']) * l['vy'] / (l['vx'] or 1e-6)))
            cv2.line(vis, p0, p1, (255, 255, 255), 2)
        for l in C:                                    # 세로철근 연장선(노랑)
            if abs(l['vy']) < 1e-6:
                continue
            for yy in (0, h):
                pass
            x_top = l['x0'] + (0 - l['y0']) * l['vx'] / l['vy']
            x_bot = l['x0'] + (h - l['y0']) * l['vx'] / l['vy']
            cv2.line(vis, (int(x_top), 0), (int(x_bot), h), (0, 255, 255), 1)
        if vp:
            cv2.circle(vis, (int(vp[0]), int(vp[1])), 8, (0, 0, 255), -1)
        cv2.line(vis, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
        cv2.putText(vis, f'A={angA:+.2f} B={angB:+.2f} yaw={yaw:+.2f}', (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imwrite(f'{OUT}/{cam}_{int(t)}.jpg', vis)
    for c in caps.values():
        c.release()
    print(f'\n오버레이: {OUT}/')


if __name__ == '__main__':
    main()
