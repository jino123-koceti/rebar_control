#!/usr/bin/env python3
"""주행 데이터 수집기 — 전/후 카메라 이미지 저장 (모델 학습용).

카메라별 용도:
  front(zedxmini2) / back(zedxmini1)  : 전·후진 주행 (장애물·경로)

[2026-08-06] zedxmini1/2를 좌·우 측면 → 전방/후방으로 물리 이동 (zedxmini2=전방, zedxmini1=후방).
  기존 left/right(측면 철근 카운팅) 항목은 제거 — 해당 위치에 카메라가 없고
  측면 이동 자체가 아직 미구현이다. 측면 수집 재개 시 카메라 배치부터 결정할 것.

카메라별로 폴더를 나눠 독립 저장한다(서로 다른 모델용). 각 폴더 자체 카운터.

사전: 전/후 카메라 노드 실행 중 (full_system.launch.py 또는 수동).

  # GUI (키보드 조작, NoMachine/로컬 디스플레이 필요)
  python3 tools/drive/drive_collect.py
      SPACE/s 전체 저장   a 자동수집 토글   f 강제(흐림무시)   q/ESC 종료

  # 헤드리스 자동수집 (주행 중 리모콘 잡고 SSH로 돌릴 때 — 권장)
  python3 tools/drive/drive_collect.py --headless --interval 0.7
      Ctrl+C 로 종료

  # 특정 카메라만
  python3 tools/drive/drive_collect.py --cams front
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT_BASE = '/home/koceti/ros2_ws/data/drive_dataset'

CAMS = {
    'front': '/zedxmini2/zed_node/rgb/color/rect/image',
    'back':  '/zedxmini1/zed_node/rgb/color/rect/image',
}


class Cam:
    def __init__(self, name, out_base):
        self.name = name
        self.latest = None
        self.last_saved = None
        self.out = os.path.join(out_base, name)
        os.makedirs(self.out, exist_ok=True)
        existing = [f for f in os.listdir(self.out)
                    if f.startswith(name + '_') and f.endswith('.png')]
        self.n = max([int(f[len(name) + 1:-4]) for f in existing
                      if f[len(name) + 1:-4].isdigit()], default=0)
        self.start_n = self.n
        # 표시용 FPS
        self.cnt = 0
        self.t0 = time.time()
        self.fps = 0.0

    def on_frame(self, img):
        self.latest = img
        self.cnt += 1
        now = time.time()
        if now - self.t0 >= 1.0:
            self.fps = self.cnt / (now - self.t0)
            self.cnt = 0
            self.t0 = now

    @staticmethod
    def sharpness(img):
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return cv2.Laplacian(g, cv2.CV_64F).var()

    def changed(self, img, thresh=8.0):
        if self.last_saved is None:
            return True
        a = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (160, 100)).astype(int)
        b = cv2.resize(cv2.cvtColor(self.last_saved, cv2.COLOR_BGR2GRAY),
                       (160, 100)).astype(int)
        return float(np.mean(np.abs(a - b))) > thresh

    def save(self, min_sharp, force=False, only_changed=False):
        """저장 성공 시 파일명, 아니면 사유 문자열 반환."""
        if self.latest is None:
            return None, 'no-signal'
        img = self.latest
        if only_changed and not self.changed(img):
            return None, 'unchanged'
        sh = self.sharpness(img)
        if not force and sh < min_sharp:
            return None, f'blur({sh:.0f})'
        self.n += 1
        fn = f'{self.name}_{self.n:04d}.png'
        cv2.imwrite(os.path.join(self.out, fn), img)
        self.last_saved = img.copy()
        return fn, f'sharp {sh:.0f}'


def build_nodes(node, cams):
    br = CvBridge()
    for c in cams.values():
        def mk(cam):
            return lambda m: cam.on_frame(br.imgmsg_to_cv2(m, 'bgr8'))
        node.create_subscription(Image, CAMS[c.name], mk(c), qos_profile_sensor_data)


def grid(cams, cell_h=360):
    """2x2 그리드 이미지 생성 (front,back / left,right 순)."""
    order = ['front', 'back', 'left', 'right']
    tiles = []
    for name in order:
        c = cams.get(name)
        if c is not None and c.latest is not None:
            im = c.latest
            s = cell_h / im.shape[0]
            t = cv2.resize(im, (int(im.shape[1] * s), cell_h))
            saved = c.n - c.start_n
            cv2.putText(t, f'{name}  {c.fps:.0f}fps  +{saved}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            w = int(cell_h * 16 / 9)
            t = np.zeros((cell_h, w, 3), np.uint8)
            cv2.putText(t, f'{name}: NO SIGNAL', (20, cell_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        tiles.append(t)
    w = max(t.shape[1] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, 0, 0, w - t.shape[1], cv2.BORDER_CONSTANT)
             for t in tiles]
    top = np.hstack([tiles[0], tiles[1]])
    bot = np.hstack([tiles[2], tiles[3]])
    return np.vstack([top, bot])


def save_all(cams, min_sharp, force=False, only_changed=False):
    msgs = []
    for name, c in cams.items():
        fn, info = c.save(min_sharp, force=force, only_changed=only_changed)
        if fn:
            msgs.append(f'{name}✓')
        else:
            msgs.append(f'{name}✗{info}')
    return msgs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=OUT_BASE)
    ap.add_argument('--cams', default='front,back',
                    help='수집할 카메라 (쉼표구분): front,back')
    ap.add_argument('--interval', type=float, default=0.7,
                    help='자동수집 간격(초)')
    ap.add_argument('--min-sharp', type=float, default=15.0,
                    help='이 선명도 미만은 흐림으로 건너뜀 (주행이라 낮게)')
    ap.add_argument('--headless', action='store_true',
                    help='창 없이 자동수집만 (주행 중 SSH용, Ctrl+C 종료)')
    ap.add_argument('--auto', action='store_true', help='시작하자마자 자동수집 ON')
    args = ap.parse_args()

    names = [x.strip() for x in args.cams.split(',') if x.strip() in CAMS]
    if not names:
        print('유효한 카메라 없음. 선택지:', ', '.join(CAMS)); return

    rclpy.init()
    node = Node('drive_collect')
    cams = {n: Cam(n, args.out) for n in names}
    build_nodes(node, cams)

    print('=' * 64)
    print(f' 주행 수집 — 카메라 {names}')
    for n, c in cams.items():
        print(f'   {n}: 기존 {c.start_n}장 → {c.out}')
    print('=' * 64)

    # ---- 헤드리스 자동수집 ----
    if args.headless:
        print(f'헤드리스 자동수집 {args.interval}s 간격. Ctrl+C 종료.\n')
        # 카메라 워밍업
        t = time.time()
        while time.time() - t < 3.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        last = time.time()
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                if time.time() - last >= args.interval:
                    msgs = save_all(cams, args.min_sharp, only_changed=True)
                    total = sum(c.n - c.start_n for c in cams.values())
                    print(f'  [{time.strftime("%H:%M:%S")}] '
                          + ' '.join(msgs) + f'  (누적 {total})')
                    last = time.time()
        except KeyboardInterrupt:
            pass
        finally:
            _summary(cams)
            node.destroy_node(); rclpy.shutdown()
        return

    # ---- GUI ----
    win = 'drive collect  SPACE=save a=auto f=force q=quit'
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1280, 720)
    except Exception as e:
        print(f'⚠️ 디스플레이 없음({e}) → --headless 사용 권장')
        node.destroy_node(); rclpy.shutdown(); return

    auto = args.auto
    last_auto = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            disp = grid(cams)
            total = sum(c.n - c.start_n for c in cams.values())
            cv2.putText(disp, f'AUTO:{"ON" if auto else "off"}  이번세션 저장:{total}',
                        (10, disp.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255) if auto else (200, 200, 200), 2)
            cv2.putText(disp, 'SPACE=save  a=auto  f=force  q=quit',
                        (10, disp.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255), 1)
            cv2.imshow(win, disp)

            if auto and time.time() - last_auto >= args.interval:
                msgs = save_all(cams, args.min_sharp, only_changed=True)
                print('  auto: ' + ' '.join(msgs))
                last_auto = time.time()

            k = cv2.waitKey(20) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k in (ord(' '), ord('s')):
                print('  save: ' + ' '.join(save_all(cams, args.min_sharp)))
            elif k == ord('f'):
                print('  force: ' + ' '.join(save_all(cams, args.min_sharp, force=True)))
            elif k == ord('a'):
                auto = not auto
                print(f'  자동수집 {"ON" if auto else "OFF"}')
    finally:
        cv2.destroyAllWindows()
        _summary(cams)
        node.destroy_node(); rclpy.shutdown()


def _summary(cams):
    print('\n=== 수집 요약 ===')
    for n, c in cams.items():
        print(f'  {n}: +{c.n - c.start_n}장 (총 {c.n}) → {c.out}')


if __name__ == '__main__':
    main()
