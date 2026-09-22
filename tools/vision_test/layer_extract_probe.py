#!/usr/bin/env python3
"""층 추출 확인 — **선택한 층만 화면에 칠해서** 눈으로 검증한다.

## 방법 (학생 알고리즘 `docs/student_networked/rebar_crossing_networked.cpp` 이식
   + `paper/src/methods/hough_detector.py` 의 층 마스크 게이팅)
  1) depth → 3D 점군
  2) 지배 평면의 **방향**만 취한다 (상단근·하단근·바닥이 전부 평행 → 법선 동일).
     ⚠ 이 평면의 **위치**를 배근으로 오해하면 안 된다 — 면적 큰 바닥이 잡힌다.
  3) 부호거리를 1차원 k-means + **엘보법**으로 나눠 층을 찾는다(k 자동).
  4) 카메라에 가까운 순 rank: 0=상단근, 1=하단근, 2=바닥
  5) rank의 밴드(±tol)만 마스크로 남기고, **그 위에서만** 교차점을 인정한다.

## 사용
    python3 tools/vision_test/layer_extract_probe.py                # 층 목록 + rank0 칠하기
    python3 tools/vision_test/layer_extract_probe.py --rank 1       # 하단근
    python3 tools/vision_test/layer_extract_probe.py --tol 30       # 밴드 폭
    python3 tools/vision_test/layer_extract_probe.py --watch
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

sys.path.insert(0, '/home/koceti/ros2_ws/src/rebar_vision')
from rebar_vision.orbbec_cad_transform import (          # noqa: E402
    extract_layers, layer_mask, rebar_depth_mm)

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt'
OUT = '/home/koceti/ros2_ws/data/vision_test/layer_extract'
NAMES = {0: '상단근', 1: '하단근', 2: '바닥'}


class Probe(Node):
    def __init__(self, a):
        super().__init__('layer_extract_probe')
        from cv_bridge import CvBridge
        from ultralytics import YOLO
        self.a, self.br = a, CvBridge()
        self.model = YOLO(a.model)
        self.color, self.depth, self.K = None, None, None
        self.create_subscription(Image, a.color, self._c, qos_profile_sensor_data)
        self.create_subscription(Image, a.depth, self._d, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, a.info, self._i, qos_profile_sensor_data)

    def _c(self, m):
        self.color = self.br.imgmsg_to_cv2(m, 'bgr8')

    def _d(self, m):
        self.depth = self.br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)

    def _i(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def ready(self):
        return self.color is not None and self.depth is not None and self.K is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--color', default='/camera/color/image_raw')
    ap.add_argument('--depth', default='/camera/depth/image_raw')
    ap.add_argument('--info', default='/camera/color/camera_info')
    ap.add_argument('--rank', type=int, default=0, help='0=상단근 1=하단근 2=바닥')
    ap.add_argument('--tol', type=float, default=40.0, help='층 밴드 반폭(mm)')
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--watch', action='store_true')
    ap.add_argument('--period', type=float, default=2.5)
    a = ap.parse_args()

    rclpy.init()
    n = Probe(a)
    print('▶ 영상 대기…')
    t0 = time.time()
    while rclpy.ok() and not n.ready() and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not n.ready():
        print('❌ 수신 실패'); return
    fx, fy, cx, cy = n.K
    try:
        while True:
            for _ in range(12):
                rclpy.spin_once(n, timeout_sec=0.05)
            t = time.time()
            res = extract_layers(n.depth, fx, fy, cx, cy)
            el = (time.time() - t) * 1000
            if res is None:
                print('  층 추출 실패');
                if not a.watch: break
                time.sleep(a.period); continue
            nv, org, cen, cnt = res
            tot = cnt.sum()
            print(f'\n층 {len(cen)}개  ({el:.0f}ms)   법선 {nv.round(3)}')
            for i, (c, k) in enumerate(zip(cen, cnt)):
                mark = ' ←선택' if i == a.rank else ''
                print(f'  rank{i}  높이 {c:+8.1f}mm  점 {k:>6} ({k/tot*100:4.1f}%)  '
                      f'{NAMES.get(i, ""):<6}{mark}')
            if len(cen) > 1:
                print('  층 간격:', [f'{cen[i]-cen[i+1]:.0f}' for i in range(len(cen)-1)], 'mm')
            if a.rank >= len(cen):
                print(f'  ⚠ rank {a.rank} 없음 (층 {len(cen)}개)')
                if not a.watch: break
                time.sleep(a.period); continue

            mask = layer_mask(n.depth, fx, fy, cx, cy, nv, org, cen[a.rank], a.tol)
            k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5)

            # 교차점 검출 → 마스크 위/밖 구분
            r = n.model(n.color, conf=a.conf, verbose=False)[0]
            on = off = 0
            im = n.color.copy()
            tint = im.copy(); tint[mask > 0] = (0, 200, 0)
            im = cv2.addWeighted(im, 0.62, tint, 0.38, 0)
            for b in r.boxes:
                u = int((float(b.xyxy[0][0]) + float(b.xyxy[0][2])) / 2)
                v = int((float(b.xyxy[0][1]) + float(b.xyxy[0][3])) / 2)
                hit = (0 <= u < mask.shape[1] and 0 <= v < mask.shape[0]
                       and mask[v, u] > 0)
                col = (0, 220, 0) if hit else (0, 0, 255)
                on, off = on + int(hit), off + int(not hit)
                cv2.circle(im, (u, v), 10, col, 2)
            print(f'  교차점: 선택층 위 {on}개 / 밖 {off}개  '
                  f'(마스크 픽셀 {int((mask>0).sum()/mask.size*100)}%)')
            cv2.putText(im, f'rank{a.rank} ({NAMES.get(a.rank,"")}) '
                            f'h={cen[a.rank]:+.0f}mm +-{a.tol:.0f}  '
                            f'green=on layer({on})  red=off({off})',
                        (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
            os.makedirs(OUT, exist_ok=True)
            p = os.path.join(OUT, f'rank{a.rank}_{time.strftime("%H%M%S")}.jpg')
            cv2.imwrite(p, im)
            print(f'  저장: {p}')
            if not a.watch:
                break
            time.sleep(a.period)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
