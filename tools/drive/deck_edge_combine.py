#!/usr/bin/env python3
"""front/back 주석영상 합본 + rebar_frac 타임라인 그래프 스트립.

deck_edge_analyze.py를 front.mp4/back.mp4에 각각 돌린 뒤 실행하면
두 카메라를 나란히 보면서 판정 궤적을 한눈에 확인할 수 있다.

  python3 tools/drive/deck_edge_combine.py --rec data/drive_rec/<ts>
저장: <rec>/combined_edge.mp4
"""
import os
import csv
import argparse

import cv2
import numpy as np

GH = 150                                    # 그래프 스트립 높이
COLOR = {'GO': (0, 200, 0), 'SLOW': (0, 165, 255), 'STOP': (0, 0, 255)}


def graph(rows, upto, width, stop, slow):
    """rebar_frac 타임라인 (0~0.8 스케일) + 임계선 + 현재 커서."""
    g = np.full((GH, width, 3), 32, np.uint8)

    def y_of(v):
        return int(GH - 12 - (v / 0.8) * (GH - 24))

    for v, c, lbl in ((slow, (0, 165, 255), f'slow {slow}'),
                      (stop, (0, 0, 255), f'stop {stop}')):
        y = y_of(v)
        cv2.line(g, (0, y), (width, y), c, 1, cv2.LINE_AA)
        cv2.putText(g, lbl, (width - 70, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.32, c, 1)

    pts = []
    for i, r in enumerate(rows):
        x = int(i / max(1, len(rows) - 1) * (width - 1))
        pts.append((x, y_of(float(r['rebar_frac_sm'])), COLOR[r['verdict']]))
    for i in range(1, min(upto + 1, len(pts))):          # 지나온 구간 = 판정색
        cv2.line(g, pts[i - 1][:2], pts[i][:2], pts[i][2], 2, cv2.LINE_AA)
    if upto < len(pts):                                  # 앞으로 올 구간 = 흐리게
        for i in range(upto + 1, len(pts)):
            cv2.line(g, pts[i - 1][:2], pts[i][:2], (90, 90, 90), 1, cv2.LINE_AA)
        cv2.line(g, (pts[upto][0], 0), (pts[upto][0], GH), (255, 255, 255), 1)
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rec', required=True, help='data/drive_rec/<ts> 디렉터리')
    ap.add_argument('--stop', type=float, default=0.45)
    ap.add_argument('--slow', type=float, default=0.55)
    args = ap.parse_args()

    rec = args.rec.rstrip('/')
    out = os.path.join(rec, 'combined_edge.mp4')
    data, caps = {}, {}
    for cam in ('front', 'back'):
        csv_path = os.path.join(rec, f'{cam}_edge.csv')
        mp4_path = os.path.join(rec, f'{cam}_edge.mp4')
        if not (os.path.exists(csv_path) and os.path.exists(mp4_path)):
            print(f'⚠ {cam}_edge.csv/mp4 없음 → deck_edge_analyze.py 먼저 실행'); return
        data[cam] = list(csv.DictReader(open(csv_path)))
        caps[cam] = cv2.VideoCapture(mp4_path)

    n = min(len(data['front']), len(data['back']))
    w = int(caps['front'].get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(caps['front'].get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = caps['front'].get(cv2.CAP_PROP_FPS) or 5
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w * 2, h + GH))

    for i in range(n):
        tiles, strips = [], []
        for cam in ('front', 'back'):
            ok, fr = caps[cam].read()
            if not ok:
                fr = np.zeros((h, w, 3), np.uint8)
            r = data[cam][i]
            cv2.putText(fr, f"{cam.upper()}  t={float(r['t_s']):.1f}s",
                        (w - 190, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            tiles.append(fr)
            strips.append(graph(data[cam], i, w, args.stop, args.slow))
        frame = np.vstack([np.hstack(tiles), np.hstack(strips)])
        cv2.line(frame, (w, 0), (w, h + GH), (255, 255, 255), 1)
        vw.write(frame)

    for c in caps.values():
        c.release()
    vw.release()
    print(f'합본 저장: {out}  ({n}프레임 {fps:.0f}fps {w * 2}x{h + GH})')


if __name__ == '__main__':
    main()
