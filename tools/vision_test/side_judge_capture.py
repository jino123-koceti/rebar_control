#!/usr/bin/env python3
"""측면(좌·우) 카메라 판정을 **deck_edge와 똑같이** 재현해 이미지로 남긴다.

## 왜
횡이동 직전 측면 판정이 '불가'를 내면 'ㄹ'자 커버리지가 그 자리에서 끝난다
(2026-09-22: left 0.345 / right 0.0 → "좌·우 모두 주행 불가 → 커버리지 종료").
판정값만으로는 **실제로 배근이 끝난 건지, 카메라가 엉뚱한 데를 보는 건지** 못 가른다.
화면을 봐야 한다.

## 판정 규칙 (deck_edge.rebar_edge 와 동일)
    발밑(화면 하단 ANCHOR_FRAC)에서 위로 스캔 → 철근 행이 이어지는 최상단 = edge_row
    rebar_frac = (H - edge_row) / H          클수록 옆으로 갈 여지가 많다
    on_rebar   = 발밑에 철근 행이 있는가       False 면 rebar_frac = 0.0
    가능 조건  : on_rebar AND rebar_frac >= side_thr(0.45)

## 사용
    python3 tools/vision_test/side_judge_capture.py
    python3 tools/vision_test/side_judge_capture.py --n 5 --out data/debug/side_judge

저장물 (카메라별): raw.png · mask.png · judge.png(판정선·수치 오버레이)
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

TOPIC = {
    'left':  '/zedxone_left/zed_node/rgb/color/rect/image/compressed',
    'right': '/zedxone/zed_node/rgb/color/rect/image/compressed',
}
WEIGHTS = os.path.expanduser('~/ros2_ws/src/rebar_vision/model/rebar_seg_260914.pt')


class Grab(Node):
    def __init__(self, n):
        super().__init__('side_judge_capture')
        self.n = n
        self.frames = {k: [] for k in TOPIC}
        for k, t in TOPIC.items():
            self.create_subscription(CompressedImage, t,
                                     lambda m, k=k: self._cb(k, m), qos_profile_sensor_data)

    def _cb(self, k, m):
        if len(self.frames[k]) >= self.n:
            return
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.frames[k].append(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=3, help='카메라당 프레임 수(판정은 중앙값)')
    ap.add_argument('--thr', type=float, default=0.45, help='측면 판정 임계(side_thr)')
    ap.add_argument('--out', default='data/debug/side_judge')
    ap.add_argument('--timeout', type=float, default=30.0)
    a = ap.parse_args()

    rclpy.init()
    g = Grab(a.n)
    print(f'■ 측면 카메라 {a.n}장씩 수집 중…')
    t0 = time.time()
    while time.time() - t0 < a.timeout and any(len(v) < a.n for v in g.frames.values()):
        rclpy.spin_once(g, timeout_sec=0.1)
    g.destroy_node(); rclpy.shutdown()

    from rebar_vision.rebar_seg import RebarSegmenter, colorize
    from rebar_vision import deck_edge as de
    import torch
    seg = RebarSegmenter(WEIGHTS, 'cuda' if torch.cuda.is_available() else 'cpu', 512)
    de.set_class_map(seg.class_map)                # ★ 인덱스는 이름에서 유도 (필수)

    stamp = time.strftime('%Y%m%d_%H%M%S')
    root = os.path.join(a.out, stamp)
    print(f'\n저장 위치: {root}/\n')
    for side, frames in g.frames.items():
        if not frames:
            print(f'[{side:5s}] ❌ 프레임 없음 — 카메라/토픽 확인'); continue
        res = []
        for img in frames:
            mask = seg.predict(img)
            edge, frac, on = de.rebar_edge(mask)
            res.append((frac, edge, on, img, mask))
        res.sort(key=lambda r: r[0])
        frac, edge, on, img, mask = res[len(res) // 2]          # 중앙값 프레임
        ok = bool(on and frac >= a.thr)
        H, W = img.shape[:2]

        d = os.path.join(root, side); os.makedirs(d, exist_ok=True)
        cv2.imwrite(os.path.join(d, 'raw.png'), img)
        cv2.imwrite(os.path.join(d, 'mask.png'), colorize(mask, seg.class_map))

        ov = cv2.addWeighted(img, 0.55, colorize(mask, seg.class_map), 0.45, 0)
        anchor = int(H * (1.0 - de.ANCHOR_FRAC))
        need = int(H * (1.0 - a.thr))
        # 발밑 판정 구역 (이 안에 철근이 있어야 on_rebar)
        cv2.rectangle(ov, (0, anchor), (W - 1, H - 1), (255, 200, 0), 2)
        cv2.putText(ov, f'foot zone (bottom {de.ANCHOR_FRAC:.0%})', (8, anchor + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        # 임계선: 철근이 이 선 위까지 이어져야 '가능'
        cv2.line(ov, (0, need), (W, need), (0, 255, 255), 2)
        cv2.putText(ov, f'need up to here (frac {a.thr:.2f})', (8, need - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        # 실제 edge_row
        if on:
            cv2.line(ov, (0, edge), (W, edge), (0, 0, 255), 3)
            cv2.putText(ov, f'rebar ends here (frac {frac:.3f})', (8, max(20, edge - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        verdict = '가능' if ok else '불가'
        txt = f'{side.upper()}  {"OK" if ok else "BLOCKED"}  frac={frac:.3f}  on_rebar={on}'
        cv2.rectangle(ov, (0, 0), (W, 40), (0, 0, 0), -1)
        cv2.putText(ov, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0) if ok else (0, 0, 255), 2)
        cv2.imwrite(os.path.join(d, 'judge.png'), ov)

        fr = [r[0] for r in res]
        reason = '' if ok else ('배근이탈(발밑에 철근 없음)' if not on else f'배근부족 {frac:.2f} < {a.thr}')
        print(f'[{side:5s}] {verdict}  rebar_frac={frac:.3f}  on_rebar={on}  '
              f'({len(res)}장 {min(fr):.3f}~{max(fr):.3f}){"  ← " + reason if reason else ""}')
    print('\n  judge.png: 노랑=임계선(여기까지 철근이 있어야 가능) · 빨강=실제 철근 끝 · 하늘색=발밑 구역')


if __name__ == '__main__':
    main()
