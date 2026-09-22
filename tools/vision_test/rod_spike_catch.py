#!/usr/bin/env python3
"""레벨봉 오검출이 **튀는 순간의 프레임**을 건져낸다.

## 왜
오검출이 **주행 중에만** 난다. 정지 상태로 캡처하면 재현이 안 된다
(2026-09-15: 주행 중 seg 0.56~0.80으로 STOP → 멈추면 0.44, 12프레임 중 0번 초과).
그래서 "그 순간 화면이 어땠나"를 끝내 못 봤다.

이 도구는 후방 프레임을 **링버퍼**에 계속 담아두다가, `/deck_edge_status`의
`rod_seg_frac`이 임계를 넘는 순간 **앞뒤 프레임을 통째로 저장**한다.
deck_edge가 실제로 본 그 프레임이 남으므로 사후에 seg를 다시 돌려 덩어리를 볼 수 있다.

⚠ deck_edge와 같은 프레임을 보려면 **같은 토픽·같은 압축본**이어야 한다.
   여기서 따로 디코딩해 저장하되 원본 바이트도 함께 남긴다.

## 사용
    # 터미널 1
    python3 tools/vision_test/rod_spike_catch.py --cam back --trig 0.55
    # 터미널 2
    python3 tools/drive/heading_zero_drive.py --back --dist 2.0 --speed 0.10

    # 주행이 끝나면 저장된 프레임을 분석
    python3 tools/vision_test/level_rod_debug.py --replay /tmp/rod_spike

저장물: spike_<시각>_<frac>.png (튄 순간 앞뒤) + status.csv (전 구간 기록)
"""
import argparse
import csv
import json
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

TOPIC = {
    'back':  '/zedxmini1/zed_node/rgb/color/rect/image/compressed',
    'front': '/zedxmini2/zed_node/rgb/color/rect/image/compressed',
}


class Catcher(Node):
    def __init__(self, a):
        super().__init__('rod_spike_catch')
        self.a = a
        self.buf = deque(maxlen=a.keep)       # (t, bgr)
        self.rows = []
        self.saved = 0
        self.last_save = 0.0
        self.create_subscription(CompressedImage, TOPIC[a.cam], self._img,
                                 qos_profile_sensor_data)
        self.create_subscription(
            String, '/deck_edge_status', self._status,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))

    def _img(self, m):
        img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.buf.append((time.time(), img))

    def _status(self, m):
        try:
            r = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if r.get('cam') != self.a.cam:
            return
        t = time.time()
        seg = float(r.get('rod_seg_frac', 0) or 0)
        col = float(r.get('rod_color_frac', 0) or 0)
        self.rows.append({
            't': round(t, 3), 'verdict': r.get('verdict'),
            'rebar_frac': r.get('rebar_frac'), 'rod_near': r.get('rod_near_frac'),
            'rod_color': col, 'rod_seg': seg, 'rod_n': r.get('rod_n'),
            'heading_deg': r.get('heading_deg'),
        })
        # ★ 튐 감지 — seg가 임계를 넘고, 색 기반과 크게 어긋날 때가 오검출 신호다.
        #   (둘이 일치하면 진짜 봉이 가까워진 것이므로 건질 이유가 없다)
        gap = seg - col
        if seg >= self.a.trig and gap >= self.a.gap and t - self.last_save > 1.0:
            self.last_save = t
            self._dump(t, seg, col)

    def _dump(self, t, seg, col):
        n = 0
        for (ft, img) in list(self.buf):
            if abs(ft - t) > self.a.window:
                continue
            path = os.path.join(
                self.a.out,
                f'spike_{time.strftime("%H%M%S", time.localtime(t))}'
                f'_seg{seg:.2f}_col{col:.2f}_{ft - t:+.2f}s.png')
            cv2.imwrite(path, img)
            n += 1
        self.saved += 1
        self.get_logger().warn(
            f'★ 튐 포착 seg={seg:.2f} 색={col:.2f} (차이 {seg-col:.2f}) → {n}장 저장')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', default='back', choices=list(TOPIC))
    ap.add_argument('--trig', type=float, default=0.55, help='seg 임계')
    ap.add_argument('--gap', type=float, default=0.08,
                    help='색 기반과의 최소 차이(이만큼 벌어져야 오검출로 본다)')
    ap.add_argument('--keep', type=int, default=30, help='링버퍼 프레임 수')
    ap.add_argument('--window', type=float, default=1.5, help='앞뒤 저장 범위(초)')
    ap.add_argument('--min', type=float, default=5.0, help='감시 시간(분)')
    ap.add_argument('--out', default='/tmp/rod_spike')
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    n = Catcher(a)
    print(f'■ 레벨봉 튐 포착 감시 {a.min:.0f}분 · {a.cam}')
    print(f'  조건: seg >= {a.trig} **그리고** seg-색 >= {a.gap}')
    print(f'  저장: {a.out}\n  지금 주행을 시작하세요. Ctrl+C로 종료.\n')
    t_end = time.time() + a.min * 60
    try:
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n(중단)')

    csv_path = os.path.join(a.out, 'status.csv')
    if n.rows:
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(n.rows[0]))
            w.writeheader()
            w.writerows(n.rows)
    segs = [r['rod_seg'] for r in n.rows]
    cols = [r['rod_color'] for r in n.rows]
    print(f'\n판정 {len(n.rows)}건 · 튐 포착 {n.saved}회')
    if segs:
        print(f'  seg  {min(segs):.3f}~{max(segs):.3f}   색 {min(cols):.3f}~{max(cols):.3f}')
        print(f'  seg>={a.trig} 인 프레임: {sum(1 for v in segs if v >= a.trig)}건')
    print(f'  기록: {csv_path}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
