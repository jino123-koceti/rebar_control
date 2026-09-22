#!/usr/bin/env python3
"""레벨봉 오검출 진단 — 실제 카메라 프레임에 seg를 돌려 `level_rod`가 뭘 잡는지 본다.

## 왜
색 기반과 seg가 `max()`로 OR 되어 있어, 합쳐진 값만 보면 **어느 쪽이 왜 틀렸는지**
알 수 없다. 2026-09-15 헤딩 영점 시험이 3번 연속 중단됐고 전부 `색0.00/seg0.66~0.80`
— seg 단독 오검출이었다. 그 seg가 **화면 어디를** 레벨봉으로 보는지 눈으로 봐야
고칠 방향이 정해진다(임계? 최소픽셀? 재학습?).

⚠ 오검출은 **주행 중에** 났다. 정지 상태 한 장으로는 재현이 안 될 수 있어
   여러 프레임을 받아 분포를 본다. 임계 미만이어도 `level_rod` 픽셀이 어디에
   붙는지가 단서다.

## 사용
    python3 tools/vision_test/level_rod_debug.py --cam back --n 20
    python3 tools/vision_test/level_rod_debug.py --cam back --n 60 --out /tmp/rod

저장물: 원본 / 클래스 컬러맵 / level_rod 오버레이 (값이 가장 큰 프레임 + 첫 프레임)
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
    'back':  '/zedxmini1/zed_node/rgb/color/rect/image/compressed',
    'front': '/zedxmini2/zed_node/rgb/color/rect/image/compressed',
    'left':  '/zedxone_left/zed_node/rgb/color/rect/image/compressed',
    'right': '/zedxone/zed_node/rgb/color/rect/image/compressed',
}
DEFAULT_WEIGHTS = os.path.expanduser(
    '~/ros2_ws/src/rebar_vision/model/rebar_seg_260914.pt')


class Grab(Node):
    def __init__(self, topic, n):
        super().__init__('level_rod_debug')
        self.frames = []
        self.n = n
        self.create_subscription(CompressedImage, topic, self._cb,
                                 qos_profile_sensor_data)

    def _cb(self, m):
        if len(self.frames) >= self.n:
            return
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.frames.append(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', default='back', choices=list(TOPIC))
    ap.add_argument('--n', type=int, default=20, help='수집 프레임 수')
    ap.add_argument('--weights', default=DEFAULT_WEIGHTS)
    ap.add_argument('--imgsz', type=int, default=512)
    ap.add_argument('--out', default='/tmp/level_rod_debug')
    ap.add_argument('--timeout', type=float, default=40.0)
    a = ap.parse_args()

    rclpy.init()
    g = Grab(TOPIC[a.cam], a.n)
    print(f'■ {a.cam} 프레임 {a.n}장 수집 중… ({TOPIC[a.cam]})')
    t0 = time.time()
    while len(g.frames) < a.n and time.time() - t0 < a.timeout:
        rclpy.spin_once(g, timeout_sec=0.1)
    g.destroy_node()
    rclpy.shutdown()
    if not g.frames:
        print('❌ 프레임 수신 없음 — 카메라/토픽 확인'); return
    print(f'  {len(g.frames)}장 수집 ({g.frames[0].shape[1]}x{g.frames[0].shape[0]})\n')

    from rebar_vision.rebar_seg import RebarSegmenter, colorize
    from rebar_vision import deck_edge as de
    import torch
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    seg = RebarSegmenter(a.weights, dev, a.imgsz)
    de.set_class_map(seg.class_map)          # ★ 인덱스는 이름에서 유도 (필수)
    name2i = {n: i for i, n in seg.class_map.items()}
    idx_rod = name2i.get('level_rod')
    print(f'클래스맵 {seg.class_map}')
    print(f'level_rod 인덱스 = {idx_rod} · 최소픽셀 {de.ROD_SEG_MIN_PIXELS}\n')
    if idx_rod is None:
        print('이 모델엔 level_rod 클래스가 없다 — 색 기반만 동작한다.'); return

    os.makedirs(a.out, exist_ok=True)
    rows, best = [], (-1.0, None)
    for i, img in enumerate(g.frames):
        mask = seg.predict(img)
        m = (mask == idx_rod).astype(np.uint8)
        px = int(m.sum())
        val = float(de.level_rod_seg(mask))
        big = 0
        if px:
            n, _l, st, _c = cv2.connectedComponentsWithStats(m, 8)
            big = int(max(st[1:, cv2.CC_STAT_AREA], default=0)) if n > 1 else 0
        rows.append((i, px, big, val))
        if val > best[0]:
            best = (val, (img, mask, m))

    print(f'{"프레임":>5s} {"level_rod px":>13s} {"최대덩어리":>10s} {"rod_seg":>8s}')
    print('-' * 42)
    for i, px, big, val in rows:
        mark = '  ← 임계 초과' if val >= 0.55 else ''
        print(f'{i:5d} {px:13d} {big:10d} {val:8.3f}{mark}')
    vals = [r[3] for r in rows]
    over = sum(1 for v in vals if v >= 0.55)
    print(f'\n최대 {max(vals):.3f} · 평균 {sum(vals)/len(vals):.3f} · '
          f'임계(0.55) 초과 {over}/{len(vals)}프레임')

    img, mask, m = best[1]
    H, W = img.shape[:2]
    cv2.imwrite(f'{a.out}/raw.png', img)
    cv2.imwrite(f'{a.out}/mask.png', colorize(mask, seg.class_map))

    # ⚠ 반투명 합성은 쓸모없다 — 덩어리가 100~200px이라 화면에서 안 보인다(첫 시도 실패).
    #   덩어리마다 **박스와 좌표**를 그리고, 가장 큰 것은 확대 크롭을 따로 저장한다.
    mm = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST)
    ov = img.copy()
    ov[mm > 0] = (0, 0, 255)
    ov = cv2.addWeighted(img, 0.4, ov, 0.6, 0)
    n, _l, st, cen = cv2.connectedComponentsWithStats(mm, 8)
    order = sorted(range(1, n), key=lambda i: -st[i, cv2.CC_STAT_AREA])
    print(f'\n■ 최대값 프레임의 level_rod 덩어리 (원본 {W}x{H} 기준)')
    for r, i in enumerate(order[:6]):
        x, y, w, h, ar = (st[i, cv2.CC_STAT_LEFT], st[i, cv2.CC_STAT_TOP],
                          st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT],
                          st[i, cv2.CC_STAT_AREA])
        keep = '통과' if ar >= de.ROD_SEG_MIN_PIXELS else '걸러짐'
        print(f'  #{r+1} ({x:4d},{y:4d}) {w:3d}x{h:3d}  {ar:5d}px  '
              f'밑동 y={y+h:4d} ({(y+h)/H:.2f}H)  {keep}')
        c = (0, 255, 255) if ar >= de.ROD_SEG_MIN_PIXELS else (255, 128, 0)
        cv2.rectangle(ov, (x-8, y-8), (x+w+8, y+h+8), c, 2)
        cv2.putText(ov, f'{ar}px', (x-8, max(12, y-14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
        if r == 0:                                  # 가장 큰 것은 확대 크롭
            pad = 90
            x0, y0 = max(0, x-pad), max(0, y-pad)
            x1, y1 = min(W, x+w+pad), min(H, y+h+pad)
            crop = img[y0:y1, x0:x1]
            if crop.size:
                cv2.imwrite(f'{a.out}/biggest_crop.png',
                            cv2.resize(crop, None, fx=3, fy=3,
                                       interpolation=cv2.INTER_NEAREST))
    cv2.imwrite(f'{a.out}/level_rod_overlay.png', ov)
    print(f'\n저장: {a.out}/')
    print('  raw.png · mask.png · level_rod_overlay.png · biggest_crop.png')
    print('  노란 박스 = 150px 필터를 통과한 덩어리(= 정지를 유발) · 주황 = 걸러진 것')


if __name__ == '__main__':
    main()
