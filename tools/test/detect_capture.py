#!/usr/bin/env python3
"""검출된 교차점에 **planner 와 같은 번호**를 붙여 이미지로 저장한다 (읽기만).

번호는 `/rebar/crossings` 의 `detections` 배열 순서다 — `tying_planner` 가
`enumerate(dets)` 로 매기는 것과 같아서, 로그의 "N번 교차점" 과 그대로 대응한다.
"""
import argparse, os, sys, time
import cv2, numpy as np, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from rebar_base_interfaces.msg import RebarGrid


class Mark(Node):
    def __init__(self):
        super().__init__('mark_detect')
        self.br = CvBridge()
        self.img = None
        self.grid = None
        self.create_subscription(Image, '/camera/color/image_raw', self._img, 10)
        self.create_subscription(RebarGrid, '/rebar/crossings', self._grid, 10)
        self.trig = self.create_publisher(Empty, '/rebar/detect', 10)

    def _img(self, m):
        self.img = self.br.imgmsg_to_cv2(m, 'bgr8')

    def _grid(self, m):
        self.grid = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--min-conf', type=float, default=0.6,
                    help='이 값 미만은 번호 옆에 "제외" 로 표시한다')
    a = ap.parse_args()

    rclpy.init()
    n = Mark()

    def pump(sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(n, timeout_sec=0.05)

    # ⚠ 디스커버리가 끝나기 전에 포기하면 안 된다 — 영상이 10Hz 로 멀쩡히
    #   나오는데도 "안 온다" 로 죽었다 (2026-10-15). 올 때까지 기다린다.
    t0 = time.time()
    while n.img is None and time.time() - t0 < 15.0:
        pump(0.2)
    if n.img is None:
        print("⚠ 15초 동안 컬러 영상이 안 온다 — /camera/color/image_raw 를 확인하라")
        return 1
    # 검출 요청 — 검출기는 여러 프레임을 합치므로 기다려야 한다
    while n.trig.get_subscription_count() == 0 and time.time():
        pump(0.3); break
    n.grid = None
    n.trig.publish(Empty())
    t0 = time.time()
    while n.grid is None and time.time() - t0 < 25.0:
        pump(0.1)
    if n.grid is None:
        print("⚠ 25초 안에 /rebar/crossings 가 오지 않았다"); return 1

    img = n.img.copy()
    dets = list(n.grid.detections)
    h, w = img.shape[:2]
    print(f"  영상 {w}x{h} · 검출 {len(dets)}개 (총 검출 {n.grid.total_detected})")

    for i, d in enumerate(dets):
        u, v = int(d.pixel_u), int(d.pixel_v)
        keep = d.confidence >= a.min_conf
        col = (0, 200, 0) if keep else (0, 140, 255)      # 초록=사용 / 주황=임계 미만
        cv2.circle(img, (u, v), 26, col, 3)
        cv2.drawMarker(img, (u, v), col, cv2.MARKER_CROSS, 20, 2)
        tag = f"{i}"
        # 번호는 크게, 바탕을 깔아 철근 위에서도 읽히게
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 3)
        bx, by = u + 32, v - 28
        cv2.rectangle(img, (bx - 4, by - th - 6), (bx + tw + 4, by + 6), (0, 0, 0), -1)
        cv2.putText(img, tag, (bx, by), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3)
        # ⚠ OpenCV Hershey 폰트는 한글을 못 그린다 (???? 로 나온다). 영문만 쓴다.
        sub = f"{d.confidence:.2f}" + ("" if keep else " SKIP") + \
              ("" if d.depth_mm > 0 else " NO-DEPTH")
        cv2.putText(img, sub, (bx, by + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        print(f"   {i:>2}번  화소({u:>4},{v:>4})  conf {d.confidence:.2f}  "
              f"깊이 {d.depth_mm:7.1f}mm  카메라({d.x:7.1f},{d.y:7.1f},{d.z:7.1f})"
              + ("" if keep else "   ← 임계 미만, 계획 제외"))

    used = sum(1 for d in dets if d.confidence >= a.min_conf)
    hdr = (f"{time.strftime('%Y-%m-%d %H:%M:%S')}   det {len(dets)}  "
           f"conf>={a.min_conf:.2f} -> {used}   (number = /rebar/crossings index)")
    cv2.rectangle(img, (0, 0), (w, 34), (0, 0, 0), -1)
    cv2.putText(img, hdr, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    cv2.imwrite(a.out, img)
    print(f"  저장: {a.out}")
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
