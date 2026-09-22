#!/usr/bin/env python3
"""실시간 뷰 + Enter 저장 — YOLO 학습 데이터 손수 모으기.

`field_collect.py`는 주행 중에 알아서 걸러 담는 도구다. 이건 반대로,
**보면서 원하는 장면만** 담는다. 레벨봉·바닥 철판처럼 일부러 놓고 찍는
대상은 자동 수집으로는 같은 장면만 쌓여서 쓸모가 없다.

## 사용
    python3 tools/collect/live_capture.py                     # right + back
    python3 tools/collect/live_capture.py --cams front,left,back
    python3 tools/collect/live_capture.py --tag levelrod      # 폴더명에 태그
    python3 tools/collect/live_capture.py --burst 5           # 한 번에 5장

## 조작 (뷰 창에 포커스를 두고)
    Enter / Space / s   현재 프레임 저장 (모든 카메라 동시)
    b                   버스트 저장 (--burst 장, 0.3초 간격)
    q / ESC             종료

## 저장물
    data/collect/<세션>/<카메라>/NNNNNN_<시각>.jpg
    data/collect/<세션>/meta.jsonl     저장 시각·카메라·해상도

⚠ 구독만 한다 — 아무 토픽도 발행하지 않아 주행·결속에 영향이 없다.
⚠ 창이 안 뜨면 DISPLAY를 확인할 것(NoMachine 원격은 보통 그냥 된다).
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

# 이름 → (토픽, 회전각, 설명). 회전은 deck_edge_node 파라미터와 맞춘 값.
CAMS = {
    'front': ('/zedxmini2/zed_node/rgb/color/rect/image/compressed',   0, '전방(주행)'),
    'back':  ('/zedxmini1/zed_node/rgb/color/rect/image/compressed',   0, '후방(주행)'),
    'left':  ('/camera_left/color/image_raw/compressed',             180, '좌측(Orbbec 305)'),
    'right': ('/zedxone/zed_node/rgb/color/rect/image/compressed',      0, '우측(ZED X One)'),
    'work':  ('/camera/color/image_raw/compressed',                    0, '작업영역 Orbbec'),
}
OUT_ROOT = '/home/koceti/ros2_ws/data/collect'
ROT = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
       270: cv2.ROTATE_90_COUNTERCLOCKWISE}


class Live(Node):
    def __init__(self, names, out, burst, quality, view_h):
        super().__init__('live_capture')
        self.names, self.out, self.burst = names, out, burst
        self.quality, self.view_h = quality, view_h
        self.img = {n: None for n in names}
        self.seen = {n: 0 for n in names}
        self.saved = {n: 0 for n in names}
        self.flash = 0.0
        self.meta = open(os.path.join(out, 'meta.jsonl'), 'a')
        for n in names:
            topic, _, _ = CAMS[n]
            os.makedirs(os.path.join(out, n), exist_ok=True)
            self.create_subscription(
                CompressedImage, topic,
                (lambda nm: lambda m: self._cb(nm, m))(n),
                qos_profile_sensor_data)

    def _cb(self, name, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        rot = CAMS[name][1]
        if rot in ROT:
            img = cv2.rotate(img, ROT[rot])
        self.img[name] = img
        self.seen[name] += 1

    # ---------- 저장 ----------
    def save_once(self):
        ts = time.strftime('%H%M%S') + f'_{int(time.time()*1000)%1000:03d}'
        got = []
        for n in self.names:
            im = self.img[n]
            if im is None:
                continue
            fn = f'{self.saved[n]:06d}_{ts}.jpg'
            cv2.imwrite(os.path.join(self.out, n, fn), im,
                        [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            self.meta.write(json.dumps({
                't': time.time(), 'cam': n, 'file': f'{n}/{fn}',
                'w': im.shape[1], 'h': im.shape[0]}, ensure_ascii=False) + '\n')
            self.saved[n] += 1
            got.append(n)
        self.meta.flush()
        if got:
            self.flash = time.time()
            print(f'  💾 저장 {ts}  [{", ".join(got)}]  '
                  f'누적 ' + ' '.join(f'{n}:{self.saved[n]}' for n in self.names),
                  flush=True)
        else:
            print('  ⚠ 저장할 영상이 없다 (아직 수신 전)', flush=True)

    # ---------- 화면 ----------
    def compose(self):
        tiles = []
        for n in self.names:
            im = self.img[n]
            if im is None:
                im = np.full((self.view_h, int(self.view_h * 4 / 3), 3), 40, np.uint8)
                cv2.putText(im, f'{n}: 수신 대기', (14, self.view_h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                h, w = im.shape[:2]
                im = cv2.resize(im, (int(w * self.view_h / h), self.view_h))
            bar = np.full((30, im.shape[1], 3), 25, np.uint8)
            cv2.putText(bar, f'{n}  saved {self.saved[n]}  frames {self.seen[n]}',
                        (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
            tiles.append(np.vstack([bar, im]))
        H = max(t.shape[0] for t in tiles)
        tiles = [np.vstack([t, np.full((H - t.shape[0], t.shape[1], 3), 25, np.uint8)])
                 if t.shape[0] < H else t for t in tiles]
        canvas = np.hstack(tiles)
        foot = np.full((34, canvas.shape[1], 3), 25, np.uint8)
        cv2.putText(foot, 'Enter/Space/s = 저장   b = 버스트   q/ESC = 종료',
                    (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 220, 255), 1)
        canvas = np.vstack([canvas, foot])
        if time.time() - self.flash < 0.35:          # 저장 피드백
            cv2.rectangle(canvas, (0, 0), (canvas.shape[1] - 1, canvas.shape[0] - 1),
                          (0, 230, 0), 8)
        return canvas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cams', default='right,back',
                    help=f'쉼표 구분. 가능: {",".join(CAMS)}')
    ap.add_argument('--tag', default='', help='세션 폴더명에 붙일 태그')
    ap.add_argument('--burst', type=int, default=5, help="b 키로 저장할 장수")
    ap.add_argument('--quality', type=int, default=92)
    ap.add_argument('--view-h', type=int, default=420, help='창에 띄울 높이(px)')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()

    if a.list:
        for n, (t, r, d) in CAMS.items():
            print(f'  {n:<7} {d:<20} rot{r:>4}  {t}')
        return

    names = [n.strip() for n in a.cams.split(',') if n.strip()]
    bad = [n for n in names if n not in CAMS]
    if bad:
        print(f'❌ 모르는 카메라: {bad}   가능: {list(CAMS)}')
        return

    stamp = time.strftime('%Y%m%d_%H%M%S') + (f'_{a.tag}' if a.tag else '')
    out = os.path.join(OUT_ROOT, stamp)
    os.makedirs(out, exist_ok=True)
    print(f'▶ 저장 위치: {out}')
    print(f'  카메라: {", ".join(f"{n}({CAMS[n][2]})" for n in names)}')
    print('  Enter/Space/s = 저장,  b = 버스트,  q/ESC = 종료\n')

    rclpy.init()
    node = Live(names, out, a.burst, a.quality, a.view_h)
    win = 'live_capture — Enter로 저장'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            cv2.imshow(win, node.compose())
            k = cv2.waitKey(20) & 0xFF
            if k in (13, 10, 32, ord('s')):          # Enter / Space / s
                node.save_once()
            elif k == ord('b'):
                for _ in range(a.burst):
                    node.save_once()
                    t0 = time.time()
                    while time.time() - t0 < 0.3:
                        rclpy.spin_once(node, timeout_sec=0.02)
                        cv2.imshow(win, node.compose()); cv2.waitKey(1)
            elif k in (ord('q'), 27):
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        tot = sum(node.saved.values())
        print(f'\n■ 종료 — 총 {tot}장')
        for n in names:
            print(f'   {n:<7} {node.saved[n]:>4}장   ({out}/{n})')
        if tot:
            print(f'\n업로드: python3 tools/vision_test/upload_to_roboflow.py --dir {out}')
        node.meta.close()
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


main()
