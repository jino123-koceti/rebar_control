#!/usr/bin/env python3
"""배근 depth **층 분포** 플롯 — 2층이 실제로 갈라지는지 눈으로 본다.

## 왜 필요한가
"층이 200mm 떨어져 있으니 depth로 가리면 된다"는 가정을 **그림으로 검증**한다.
갈라지면 봉우리가 둘, 안 갈라지면 하나다. 임계값을 고르기 전에 이걸 먼저 봐야
한다 (2026-09-08: RC 목업은 30~40mm라 봉우리가 하나뿐이었다).

기준면은 **교차점들로 맞춘 평면**이다 — 화면 전체로 맞추면 면적이 넓은
바닥(54%)이 잡혀 배근이 아니게 된다.

## 사용
    python3 tools/vision_test/plot_layer_depth.py
    python3 tools/vision_test/plot_layer_depth.py --out /path/fig.png
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

sys.path.insert(0, '/home/koceti/ros2_ws/src/rebar_vision')
from rebar_vision.orbbec_cad_transform import (          # noqa: E402
    rebar_depth_mm, backproject, split_layers)

for _c in ('NanumGothic', 'NanumSquareRound', 'Noto Sans CJK KR'):
    if any(_c in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = _c
        break
plt.rcParams['axes.unicode_minus'] = False

BLUE, ORANGE = '#2a78d6', '#eb6834'
INK, INK2, MUTED, SURFACE = '#0b0b0b', '#52514e', '#b8b7b0', '#fcfcfb'
MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt'


class Cap(Node):
    def __init__(self, a):
        super().__init__('layer_depth_plot')
        from cv_bridge import CvBridge
        from ultralytics import YOLO
        self.a, self.br = a, CvBridge()
        self.model = YOLO(a.model)
        self.color, self.depths, self.K = None, [], None
        self.create_subscription(Image, a.color, self._c, qos_profile_sensor_data)
        self.create_subscription(Image, a.depth, self._d, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, a.info, self._i, qos_profile_sensor_data)

    def _c(self, m):
        self.color = self.br.imgmsg_to_cv2(m, 'bgr8')

    def _d(self, m):
        self.depths.append(self.br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32))
        if len(self.depths) > 5:
            self.depths.pop(0)

    def _i(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def ready(self):
        return self.color is not None and self.depths and self.K is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--color', default='/camera/color/image_raw')
    ap.add_argument('--depth', default='/camera/depth/image_raw')
    ap.add_argument('--info', default='/camera/color/camera_info')
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--min-gap', type=float, default=80.0)
    ap.add_argument('--out', default='/home/koceti/ros2_ws/docs/reports/figures/'
                                     'layer_depth_dist.png')
    a = ap.parse_args()

    rclpy.init()
    n = Cap(a)
    t0 = time.time()
    while rclpy.ok() and not n.ready() and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not n.ready():
        print('❌ color/depth/info 수신 실패'); return
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)
    fx, fy, cx, cy = n.K

    # 1) 교차점 검출 → 3D
    r = n.model(n.color, conf=a.conf, verbose=False)[0]
    pts, P = [], []
    for b in r.boxes:
        u = int((float(b.xyxy[0][0]) + float(b.xyxy[0][2])) / 2)
        v = int((float(b.xyxy[0][1]) + float(b.xyxy[0][3])) / 2)
        z = rebar_depth_mm(n.depths, u, v)
        if z:
            pts.append((u, v, z))
            P.append(backproject(u, v, z, fx, fy, cx, cy))
    if len(P) < 4:
        print(f'❌ 교차점 부족 ({len(P)}개)'); return
    res = split_layers(P, min_gap_mm=a.min_gap)
    hts, thr, n_up, n_lo = res
    c0 = np.asarray(P).mean(0)
    _, _, vt = np.linalg.svd(np.asarray(P) - c0, full_matrices=False)
    nv = vt[2] / np.linalg.norm(vt[2])
    if nv[2] > 0:
        nv = -nv

    # 2) 화면 전체 픽셀의 같은 평면 기준 높이
    D = n.depths[-1]
    h, w = D.shape[:2]
    vs, us = np.mgrid[0:h:3, 0:w:3]
    z = D[::3, ::3].astype(np.float32)
    m = (z > 50) & (z < 2000)
    us, vs, z = us[m].astype(np.float32), vs[m].astype(np.float32), z[m]
    Q = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], 1)
    H = -((Q - c0) @ nv)

    print(f'교차점 {len(P)}개 · 전체 픽셀 {len(H)}개')
    print(f'교차점 높이 {hts.min():+.0f}~{hts.max():+.0f}mm  '
          f'(표준편차 {hts.std():.0f})')
    print(f'층 분리: {"없음(단층)" if thr is None else f"임계 {thr:+.0f}mm, 상단{n_up} 하단{n_lo}"}')

    # ── 그림 ──────────────────────────────────────────────────────
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(10.5, 7.6), facecolor=SURFACE,
                                  gridspec_kw={'height_ratios': [2, 1]})
    fig.subplots_adjust(hspace=0.42, top=0.855, bottom=0.11, left=0.10, right=0.97)

    lo, hi = np.percentile(H, 1), np.percentile(H, 99)
    lo, hi = min(lo, hts.min() - 40), max(hi, hts.max() + 40)
    bins = np.linspace(lo, hi, 70)
    ax.hist(H, bins=bins, color=MUTED, alpha=0.55, label='화면 전체 픽셀')
    ax.set_ylabel('픽셀 수', fontsize=10.5, color=INK2)
    ax.set_xlabel('교차점 평면 기준 높이 (mm, + = 카메라 쪽)', fontsize=10.5, color=INK2)

    axr = ax.twinx()
    axr.hist(hts, bins=np.linspace(lo, hi, 26), color=BLUE, alpha=0.9,
             label='교차점')
    axr.set_ylabel('교차점 수', fontsize=10.5, color=BLUE)
    axr.tick_params(axis='y', colors=BLUE)
    for x in hts:
        axr.plot([x], [0.06], marker='|', ms=13, color=BLUE, clip_on=False)

    if thr is not None:
        for A in (ax, axr):
            A.axvline(thr, color=ORANGE, lw=2.0, ls='--', zorder=6)
        ax.text(thr, ax.get_ylim()[1] * 0.94, f'  층 경계 {thr:+.0f}mm',
                color=ORANGE, fontsize=10, fontweight='bold', va='top')
    else:
        ax.text(0.015, 0.94, f'층 분리 없음 — 최대 틈 < {a.min_gap:.0f}mm',
                transform=ax.transAxes, color=ORANGE, fontsize=10.5,
                fontweight='bold', va='top')

    ax.set_title('배근 depth 층 분포', fontsize=14.5, color=INK,
                 pad=28, loc='left', fontweight='bold')
    ax.text(0, 1.06,
            f'교차점 {len(P)}개(파랑) · 화면 전체 픽셀(회색) — 2층이면 봉우리가 둘이어야 한다   '
            f'| {time.strftime("%Y-%m-%d %H:%M")}',
            transform=ax.transAxes, fontsize=9.5, color=INK2, va='bottom')
    ax.grid(axis='y', color=MUTED, alpha=0.3, lw=0.7)
    for sp in ('top',):
        ax.spines[sp].set_visible(False); axr.spines[sp].set_visible(False)
    ax.set_facecolor(SURFACE)

    # 하단: 화면 위치 vs 높이 (원근 영향 확인용)
    uu = [p[0] for p in pts]
    ax2.scatter(uu, hts, s=46, color=BLUE, zorder=4)
    if thr is not None:
        ax2.axhline(thr, color=ORANGE, lw=1.8, ls='--', zorder=3)
    ax2.axhline(0, color=MUTED, lw=1.0, zorder=2)
    ax2.set_xlabel('화면 가로 위치 u (px)', fontsize=10.5, color=INK2)
    ax2.set_ylabel('높이 (mm)', fontsize=10.5, color=INK2)
    ax2.text(0.0, 1.06, '화면 위치별 높이 — 한쪽으로 기울면 평면 적합이 덜 맞은 것',
             transform=ax2.transAxes, fontsize=9, color=MUTED, va='bottom')
    ax2.grid(color=MUTED, alpha=0.3, lw=0.7)
    for sp in ('top', 'right'):
        ax2.spines[sp].set_visible(False)
    ax2.set_facecolor(SURFACE)

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fig.savefig(a.out, dpi=165, facecolor=SURFACE)
    print(f'저장: {a.out}')
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


main()
