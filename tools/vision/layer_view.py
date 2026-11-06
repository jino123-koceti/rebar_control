#!/usr/bin/env python3
"""상부근/하부근 **층 판정**을 눈으로 본다. 검출이 왜 버려졌는지 보려는 것.

    python3 tools/vision/layer_view.py --out /tmp/layer.png

`crossing_detector._layer_filter` 와 **같은 호출**을 한다 (rank_map → pick_rank
→ layer_ratio). 다르게 부르면 보는 것과 쓰는 것이 어긋나 아무 의미가 없다.

    빨강 = 상부근 띠(rank=up)   파랑 = 그 외 층   회색 = 깊이 없음
    원   = 검출점, 숫자 = 그 점의 상부근 비율 (임계 0.35)
"""
import argparse
import os
import sys

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src', 'rebar_vision'))
from rebar_vision.layer_rank import rank_map, layer_ratio, pick_rank  # noqa: E402


class LayerView(Node):
    def __init__(self, out, pts):
        super().__init__('layer_view')
        self.out, self.pts = out, pts
        self.color = self.depth = self.K = None
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._c, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._d, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 self._k, qos_profile_sensor_data)
        self.create_timer(0.5, self.tick)
        self.done = False

    def _c(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        self.color = a[:, :, :3].copy() if m.encoding in ('bgr8', 'bgra8') \
            else cv2.cvtColor(a[:, :, :3], cv2.COLOR_RGB2BGR)

    def _d(self, m):
        if m.encoding == '16UC1':
            a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            self.depth = a.astype(np.float32)
        else:
            a = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
            self.depth = np.nan_to_num(a * 1000.0, nan=0.0)

    def _k(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def tick(self):
        if self.done or self.color is None or self.depth is None or self.K is None:
            return
        self.done = True
        rank, info = rank_map(self.depth, self.K)
        up = pick_rank(info, 120, 300, 'high')
        centers = [round(-c) for c in info['centers']]
        print(f"  유효화소 {info['valid']}  ·  층 중심 {centers} mm  ·  상부근 rank={up}")
        if up is None:
            print('  ★ 상부근 띠를 못 골랐다 (120~300mm 에 층이 없다)')

        h, w = rank.shape
        im = cv2.resize(self.color, (w, h))
        ov = im.copy()
        if up is not None:
            ov[rank == up] = (0, 0, 255)                 # 상부근 = 빨강
        other = (rank != 255) & (rank != (up if up is not None else 254))
        ov[other] = (255, 128, 0)                        # 다른 층 = 파랑
        ov[rank == 255] = (110, 110, 110)                # 깊이 없음 = 회색
        im = cv2.addWeighted(im, 0.45, ov, 0.55, 0)

        for i, (u, v) in enumerate(self.pts):
            r = layer_ratio(rank, u, v, 50, up) if up is not None else 0.0
            win = self.depth[max(0, int(v) - 4):int(v) + 5,
                             max(0, int(u) - 4):int(u) + 5]
            win = win[win > 0]
            z = float(np.median(win)) if win.size else 0.0
            inwin = 330.0 <= z <= 660.0        # 캘리가 유효한 깊이 구간
            col = (0, 255, 0) if r >= 0.35 else (
                (0, 200, 255) if inwin else (0, 0, 255))
            cv2.circle(im, (int(u), int(v)), 50, col, 2)
            txt = f'#{i} {r:.2f} z{z:.0f}'
            for c, th in (((0, 0, 0), 5), (col, 2)):
                cv2.putText(im, txt, (int(u) - 54, int(v) - 58),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58, c, th)
        cv2.rectangle(im, (0, 0), (w, 32), (0, 0, 0), -1)
        cv2.putText(im, f'upper band rank {up} {centers}mm | circle: '
                        f'GREEN pass  ORANGE in-depth-window(330-660) but fail  '
                        f'RED out of window | ratio, z=depth mm',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
        cv2.imwrite(self.out, im)
        print(f'  저장: {self.out}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/layer.png')
    ap.add_argument('--pts', default='', help='"u,v u,v ..." 검출점')
    a = ap.parse_args()
    pts = [tuple(float(x) for x in p.split(',')) for p in a.pts.split() if ',' in p]
    rclpy.init()
    n = LayerView(a.out, pts)
    for _ in range(80):
        rclpy.spin_once(n, timeout_sec=0.25)
        if n.done:
            break
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
