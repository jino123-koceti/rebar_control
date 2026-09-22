#!/usr/bin/env python3
"""측면 카메라(Orbbec 305 좌측) 영상에 7클래스 seg 적용 — 횡이동 주행가능 판정 가능성 확인.

seg 모델은 **전후방 주행카메라 시점**으로 학습됐다. 측면 뷰에서도 철근/바닥을
가르는지 실제로 보고 판단하려고 만든 도구.

  python3 tools/vision_test/side_cam_seg_test.py
     [--topic /camera_left/color/image_rotated] [--n 3] [--out data/vision_test/side_seg]
저장: <out>/side_<i>.jpg (원본+오버레이 나란히)
"""
import os
import sys
import argparse

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

ROOT = '/home/koceti/ros2_ws'
sys.path.insert(0, os.path.join(ROOT, 'src/rebar_vision'))
from rebar_vision.rebar_seg import RebarSegmenter, colorize, CLASS_MAP   # noqa: E402
from rebar_vision import deck_edge as de                                 # noqa: E402

WEIGHTS = os.path.join(ROOT, 'src/rebar_vision/model/retrain_best_260804.pt')


class Grab(Node):
    def __init__(self, topic, n):
        super().__init__('side_cam_seg_test')
        self.imgs = []
        self.n = n
        self.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        if len(self.imgs) >= self.n:
            return
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, np.uint8)
        enc = msg.encoding.lower()
        if enc in ('rgb8', 'bgr8'):
            img = arr.reshape(h, w, 3)
            if enc == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif enc in ('bgra8', 'rgba8'):
            img = arr.reshape(h, w, 4)[:, :, :3]
            if enc == 'rgba8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            self.get_logger().error(f'미지원 encoding: {msg.encoding}')
            return
        self.imgs.append(img.copy())


def main():
    ap = argparse.ArgumentParser()
    # ⚠ 305는 물리적으로 180° 뒤집혀 장착됐다. 예전엔 image_rotate 노드가 상시 회전·재발행
    #   했으나 CPU 24%를 먹어 제거했다(2026-08-13). 이제 **소비자가 판정 순간에 한 번** 돌린다.
    ap.add_argument('--topic', default='/camera_left/color/image_raw')
    ap.add_argument('--rotate', type=int, default=180, choices=[0, 90, 180, 270],
                    help='입력 영상 회전(도). 305 좌측은 180')
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--out', default=os.path.join(ROOT, 'data/vision_test/side_seg'))
    ap.add_argument('--label', default='', help='보고서 제목에 넣을 카메라 설명(미지정 시 토픽에서 추정)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    node = Grab(a.topic, a.n)
    import time
    t0 = time.time()
    while rclpy.ok() and len(node.imgs) < a.n and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not node.imgs:
        print(f'⚠ {a.topic} 에서 영상을 못 받음'); return

    # 상시 회전 노드 대신 여기서 1회 회전 (수 ms). ROT_MAP 없으면 그대로.
    ROT = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
           270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    if a.rotate in ROT:
        node.imgs = [cv2.rotate(im, ROT[a.rotate]) for im in node.imgs]

    # 제목 라벨: 미지정 시 토픽으로 좌/우 카메라를 추정 (하드코딩 방지)
    label = a.label or ('Side camera (Orbbec Gemini 305, LEFT)' if 'camera_left' in a.topic
                        else 'Side camera (ZED X One UHD, RIGHT)' if 'zedxone' in a.topic
                        else f'Camera: {a.topic}')

    import torch
    seg = RebarSegmenter(WEIGHTS, 'cuda' if torch.cuda.is_available() else 'cpu', 512)
    for i, img in enumerate(node.imgs):
        mask = seg.predict(img)
        u, c = np.unique(mask, return_counts=True)
        dist = {CLASS_MAP[int(k)]: v / mask.size for k, v in zip(u, c)}
        edge_row, frac, on_rebar = de.rebar_edge(mask)
        hd, bars = None, 0
        try:
            from rebar_vision.rebar_grid import heading
            hd, bars = heading(mask, 640)
        except Exception:
            pass
        print(f'\n[{i}] {img.shape[1]}x{img.shape[0]}')
        print('   클래스 비율: ' + '  '.join(f'{k}={v:.3f}' for k, v in
                                          sorted(dist.items(), key=lambda x: -x[1])))
        print(f'   rebar_edge → rebar_frac={frac:.3f} on_rebar={on_rebar} edge_row={edge_row}')
        print(f'   heading    → {hd}° (가로철근 {bars}개)')
        over = cv2.addWeighted(img, 0.55, colorize(mask), 0.45, 0)
        cv2.imwrite(f'{a.out}/side_{i}.jpg', np.hstack([img, over]))
        cv2.imwrite(f'{a.out}/side_{i}_report.jpg',
                    _annotate(img, over, dist, frac, on_rebar, hd, bars, label))
    print(f'\n저장: {a.out}/side_*.jpg          (좌=원본, 우=오버레이)')
    print(f'      {a.out}/side_*_report.jpg   (지표·범례 포함, 보고서용)')


# 오버레이 팔레트(BGR)와 같은 색을 범례에 쓴다 — rebar_seg.PALETTE 기준.
# ⚠ OpenCV Hershey 폰트는 한글을 못 그린다(?로 깨짐) → 범례는 영문만 쓸 것.
LEGEND = [('rebar_h (cross)', (0, 200, 0)), ('rebar_v (drive dir)', (0, 200, 120)),
          ('background (sheet)', (0, 0, 0)), ('floor', (0, 140, 255)),
          ('obstacle', (0, 0, 160)), ('wall', (150, 150, 150))]


def _annotate(img, over, dist, frac, on_rebar, hd, bars, label='side camera'):
    """보고서용: 원본|오버레이 + 상단 제목/지표 + 하단 범례."""
    h, w = img.shape[:2]
    pair = np.hstack([img, over])
    head, foot = 78, 44
    canvas = np.full((h + head + foot, w * 2, 3), 28, np.uint8)
    canvas[head:head + h] = pair
    f, F = cv2.FONT_HERSHEY_SIMPLEX, cv2.LINE_AA
    cv2.putText(canvas, f'{label} - rebar segmentation',
                (12, 26), f, 0.62, (255, 255, 255), 1, F)
    # heading은 가로철근이 0개면 None으로 온다 (주행불가 구간에서 정상적으로 발생)
    hd_s = 'n/a' if hd is None else f'{hd:+.2f}deg'
    ok = frac >= de.STOP_THR and on_rebar
    cv2.putText(canvas, f'rebar_frac={frac:.3f}  on_rebar={on_rebar}  '
                        f'heading={hd_s} (bars {bars})',
                (12, 50), f, 0.55, (120, 230, 120) if ok else (110, 110, 255), 1, F)
    cv2.putText(canvas, 'DRIVABLE' if ok else 'NOT DRIVABLE',
                (w * 2 - 250, 30), f, 0.8, (120, 230, 120) if ok else (110, 110, 255), 2, F)
    cv2.putText(canvas, '  '.join(f'{k}={v:.3f}' for k, v in
                                  sorted(dist.items(), key=lambda x: -x[1])[:4]),
                (12, 70), f, 0.46, (190, 190, 190), 1, F)
    cv2.putText(canvas, 'RAW', (12, head + 24), f, 0.7, (255, 255, 255), 2, F)
    cv2.putText(canvas, 'SEGMENTATION', (w + 12, head + 24), f, 0.7, (255, 255, 255), 2, F)
    x = 12
    for name, col in LEGEND:
        cv2.rectangle(canvas, (x, h + head + 14), (x + 20, h + head + 30), col, -1)
        cv2.rectangle(canvas, (x, h + head + 14), (x + 20, h + head + 30), (90, 90, 90), 1)
        cv2.putText(canvas, name, (x + 26, h + head + 27), f, 0.45, (230, 230, 230), 1, F)
        # 실제 렌더 폭으로 다음 항목 위치를 잡는다(글자수 추정은 겹침 발생)
        x += 26 + cv2.getTextSize(name, f, 0.45, 1)[0][0] + 24
    return canvas


if __name__ == '__main__':
    main()
