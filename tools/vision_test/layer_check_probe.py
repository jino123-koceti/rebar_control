#!/usr/bin/env python3
"""2층 배근 — 교차점의 **두 철근이 같은 층인지** 실측 (현장에서 임계 정하는 도구).

## 무엇을 재나
하단근 세로 × 상단근 가로는 **탑뷰에서 교차점처럼 보이지만 실제가 아니다**.
교차점 한 점의 depth만 보면 위에 얹힌 철근만 보여서 `상단×상단(진짜)`와
`상단×하단(가짜)`를 구분 못 한다.

→ 교차점 **둘레를 링으로 훑어** 갈래(가로 2 + 세로 2)의 높이를 각도별로 잰다.
  depth가 아니라 **교차점들로 맞춘 평면 위 높이**로 보므로 카메라 기울기(28°)와
  원근이 상쇄된다. 대향 각도쌍을 묶어(같은 철근의 양쪽) 그 폭이 층간이면 가짜다.

## 폐기한 방법 (같은 길 다시 가지 말 것)
· 화면축 팔로 d_h/d_v 비교 → 철근이 화면에서 대각선이라 팔이 벗어난다.
  라벨 대조에서 진짜 259mm / 가짜 0mm로 **상관이 없었다** (2026-09-08).
· 중심 depth 단독 → 위 철근만 보여 `상단×상단`과 `상단×하단`이 같은 값.

## 사용
    python3 tools/vision_test/layer_check_probe.py                 # 1회
    python3 tools/vision_test/layer_check_probe.py --watch         # 반복
    python3 tools/vision_test/layer_check_probe.py --gap 60        # 임계 바꿔보기
    python3 tools/vision_test/layer_check_probe.py --watch --save-bad
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
    rebar_depth_mm, backproject, ring_bar_heights)

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt'
OUT = '/home/koceti/ros2_ws/data/vision_test/layer_check'


class Probe(Node):
    def __init__(self, a):
        super().__init__('layer_check_probe')
        from cv_bridge import CvBridge
        from ultralytics import YOLO
        self.a, self.br = a, CvBridge()
        self.model = YOLO(a.model)
        self.names = getattr(self.model, 'names', {}) or {}
        self.color, self.depths, self.K = None, [], None
        self.create_subscription(Image, a.color, self._c, qos_profile_sensor_data)
        self.create_subscription(Image, a.depth, self._d, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, a.info, self._i, qos_profile_sensor_data)

    def _c(self, m):
        self.color = self.br.imgmsg_to_cv2(m, 'bgr8')

    def _d(self, m):
        self.depths.append(self.br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32))
        if len(self.depths) > self.a.n_depth:
            self.depths.pop(0)

    def _i(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def ready(self):
        return self.color is not None and self.depths and self.K is not None

    def measure(self):
        """검출 → 교차점별 (u,v,cls,conf,z,중심높이,폭,갈래수,섹터요약)"""
        fx, fy, cx, cy = self.K
        r = self.model(self.color, conf=self.a.conf, verbose=False)[0]
        pts, P = [], []
        for b in r.boxes:
            u = int((float(b.xyxy[0][0]) + float(b.xyxy[0][2])) / 2)
            v = int((float(b.xyxy[0][1]) + float(b.xyxy[0][3])) / 2)
            ci = int(b.cls[0]) if b.cls is not None and len(b.cls) else 0
            cf = float(b.conf[0]) if b.conf is not None and len(b.conf) else 0.0
            z = rebar_depth_mm(self.depths, u, v)
            if z:
                pts.append((u, v, self.names.get(ci, str(ci)), cf, z))
                P.append(backproject(u, v, z, fx, fy, cx, cy))
        if len(P) < 4:
            return [], None
        A = np.asarray(P); c0 = A.mean(0)
        _, _, vt = np.linalg.svd(A - c0, full_matrices=False)
        nv = vt[2] / np.linalg.norm(vt[2])
        if nv[2] > 0:
            nv = -nv
        rows = []
        for (u, v, cls, cf, z), Pi in zip(pts, P):
            ch = float(-((np.asarray(Pi) - c0) @ nv))
            angs, hts = ring_bar_heights(
                self.depths, u, v, z, nv, c0, fx, fy, cx, cy,
                r_in_mm=self.a.r_in, r_out_mm=self.a.r_out)
            if hts.size < 6:
                rows.append((u, v, cls, cf, z, ch, None, int(hts.size), None))
                continue
            pair = []
            for k in range(len(angs)):
                opp = (angs[k] + 180.0) % 360.0
                j = int(np.argmin(np.abs(((angs - opp + 180.0) % 360.0) - 180.0)))
                pair.append((hts[k] + hts[j]) / 2.0)
            pair = np.asarray(pair)
            sect = []
            for s0 in range(0, 360, 45):
                m = (angs >= s0) & (angs < s0 + 45)
                sect.append(f'{np.median(hts[m]):+.0f}' if m.sum() else '   .')
            rows.append((u, v, cls, cf, z, ch,
                         float(pair.max() - pair.min()), int(hts.size), sect))
        rows.sort(key=lambda r_: (r_[6] is None, -(r_[6] or 0)))
        return rows, (nv, c0)


def report(rows, thr):
    print(f'\n{"u":>5}{"v":>5}{"conf":>6}{"z":>7}{"중심높이":>9}{"철근높이차":>11}'
          f'{"갈래":>5}  판정   갈래 높이 (0° 45° 90° …)')
    print('-' * 104)
    ok = bad = unk = 0
    for u, v, cls, cf, z, ch, sp, ns, sect in rows:
        if sp is None:
            mark, unk = '?    ', unk + 1
            sp_s = '    -'
        elif sp > thr:
            mark, bad = '❌가짜', bad + 1
            sp_s = f'{sp:5.0f}'
        else:
            mark, ok = '✅진짜', ok + 1
            sp_s = f'{sp:5.0f}'
        ss = ' '.join(f'{x:>5}' for x in sect) if sect else '갈래 부족'
        print(f'{u:>5}{v:>5}{cf:>6.2f}{z:>7.0f}{ch:>+9.1f}{sp_s:>11}{ns:>5}  {mark}  {ss}')
    print('-' * 104)
    print(f'  같은 층 {ok} · 층 혼합 {bad} · 판정불가 {unk}   (임계 {thr:.0f}mm)')
    g = sorted(r[6] for r in rows if r[6] is not None)
    if len(g) >= 3:
        print(f'  철근 높이차: 최소 {g[0]:.0f}  중앙 {g[len(g)//2]:.0f}  최대 {g[-1]:.0f}mm')
        hi = max(120.0, g[-1]); nb = 12; w = hi / nb
        cnt = [0] * nb
        for x in g:
            cnt[min(nb - 1, int(x / w))] += 1
        mx = max(cnt) or 1
        for i, c in enumerate(cnt):
            if c:
                print(f'    {i*w:4.0f}~{(i+1)*w:4.0f}mm |{"█"*int(c*22/mx)} {c}')
        print('  → 두 무리 사이 골짜기를 layer_gap_mm 으로.')
    return ok, bad, unk


def overlay(img, rows, thr, a, path):
    im = img.copy()
    for u, v, cls, cf, z, ch, sp, ns, sect in rows:
        if sp is None:
            col, tag = (0, 165, 255), '?'
        elif sp > thr:
            col, tag = (0, 0, 255), f'{sp:.0f}'
        else:
            col, tag = (0, 200, 0), f'{sp:.0f}'
        ppm = a_fx / float(z) if z else 1.0
        cv2.circle(im, (u, v), max(3, int(a.r_in * ppm)), (255, 160, 0), 1)
        cv2.circle(im, (u, v), max(4, int(a.r_out * ppm)), (255, 160, 0), 1)
        cv2.circle(im, (u, v), 5, col, -1)
        cv2.putText(im, tag, (u + 12, v - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    cv2.putText(im, f'green=same layer  red=mixed(>{thr:.0f}mm)  label=bar height spread',
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(im, 'orange rings = sampling annulus (must straddle the rebar arms)',
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, im)
    print(f'  오버레이 저장: {path}')


def main():
    global a_fx
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--color', default='/camera/color/image_raw')
    ap.add_argument('--depth', default='/camera/depth/image_raw')
    ap.add_argument('--info', default='/camera/color/camera_info')
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--n-depth', type=int, default=5)
    ap.add_argument('--gap', type=float, default=75.0,
                    help='층 혼합 판정 임계(mm). 현장 층간 150mm의 절반')
    ap.add_argument('--r-in', type=float, default=18.0, help='링 안쪽 반지름(mm)')
    ap.add_argument('--r-out', type=float, default=34.0, help='링 바깥 반지름(mm)')
    ap.add_argument('--watch', action='store_true')
    ap.add_argument('--save-every', type=int, default=1, help='0=저장 안 함')
    ap.add_argument('--save-bad', action='store_true', help="'가짜' 있는 프레임만")
    ap.add_argument('--period', type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    n = Probe(a)
    print('▶ 영상 대기…')
    t0 = time.time()
    while rclpy.ok() and not n.ready() and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not n.ready():
        print('❌ color/depth/camera_info 수신 실패'); return
    a_fx = n.K[0]
    print(f'  fx={a_fx:.1f}  링 {a.r_in:.0f}~{a.r_out:.0f}mm  임계 {a.gap:.0f}mm')
    i = saved = 0
    try:
        while True:
            for _ in range(a.n_depth * 3):
                rclpy.spin_once(n, timeout_sec=0.05)
            rows, pl = n.measure()
            if not rows:
                print('  교차점 부족 (4개 미만)')
            else:
                i += 1
                _, bad, _ = report(rows, a.gap)
                want = (a.save_every > 0 and i % a.save_every == 0)
                if a.save_bad:
                    want = bad > 0
                if want:
                    overlay(n.color, rows, a.gap, a,
                            os.path.join(OUT, f'layer_{time.strftime("%Y%m%d_%H%M%S")}.jpg'))
                    saved += 1
            if not a.watch:
                break
            time.sleep(a.period)
    except KeyboardInterrupt:
        pass
    finally:
        if a.watch:
            print(f'\n■ 종료 — 측정 {i}회, 저장 {saved}장  ({OUT})')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
