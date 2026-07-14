#!/usr/bin/env python3
"""Orbbec 교차점 검출 → 호모그래피 → 로봇 XY 실시간 테스트.
YOLO(orbbec_crossing)로 교차점 검출, 자세별 호모그래피로 로봇 (x,y)mm 산출,
도달가능 자세(r/l) 판정해 오버레이 + 콘솔 출력.

  ros2 launch orbbec_camera gemini2L.launch.py
  python3 orbbec_localize.py

  s  스냅샷(/tmp/orbbec_localize.png)   p  콘솔에 좌표 출력   q 종료
"""
import os, argparse
import numpy as np
import cv2, yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt'
HOMO = '/home/koceti/ros2_ws/data/calibration/homography_orbbec.yaml'
# 자세별 Y 가동범위 (config 실측)
YRANGE = {'r': (0.0, 142.0), 'l': (124.0, 288.0)}
XMAX = 411.0


def apply_H(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return p[0]/p[2], p[1]/p[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conf', type=float, default=0.3)
    args = ap.parse_args()
    hom = yaml.safe_load(open(HOMO))
    H = {'r': np.array(hom['right']['H']) if 'right' in hom else None,
         'l': np.array(hom['left']['H']) if 'left' in hom else None}
    print(f'호모그래피: 우측{"O" if H["r"] is not None else "X"} '
          f'좌측{"O" if H["l"] is not None else "X"}')

    rclpy.init()
    node = Node('orbbec_localize')
    br = CvBridge(); buf = []
    node.create_subscription(Image, '/camera/color/image_raw',
                             lambda m: buf.append(br.imgmsg_to_cv2(m, 'bgr8')),
                             qos_profile_sensor_data)
    model = YOLO(MODEL)
    win = 'orbbec localize (s=snap p=print q=quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 1280, 800)

    def locate(u, v):
        """도달가능 자세 판정 + 로봇XY. 반환 (pose, x, y) 또는 None."""
        cand = []
        for pose in ('r', 'l'):
            if H[pose] is None:
                continue
            x, y = apply_H(H[pose], u, v)
            ymin, ymax = YRANGE[pose]
            if ymin <= y <= ymax and 0 <= x <= XMAX:
                cand.append((pose, x, y))
        if not cand:
            return None
        # 겹침이면 우측 우선
        cand.sort(key=lambda c: 0 if c[0] == 'r' else 1)
        return cand[0]

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if not buf:
                continue
            img = buf[-1]
            if len(buf) > 4:
                del buf[:-2]
            r = model(img, conf=args.conf, verbose=False)[0]
            disp = img.copy()
            found = []
            for b in r.boxes:
                u = int((b.xyxy[0][0]+b.xyxy[0][2])/2)
                v = int((b.xyxy[0][1]+b.xyxy[0][3])/2)
                loc = locate(u, v)
                if loc is None:
                    cv2.circle(disp, (u, v), 12, (120, 120, 120), 2)
                    continue
                pose, x, y = loc
                col = (0, 165, 255) if pose == 'r' else (255, 100, 0)
                cv2.circle(disp, (u, v), 14, col, 3)
                cv2.putText(disp, f'{pose}({x:.0f},{y:.0f})', (u+12, v-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
                found.append((pose, x, y, u, v))
            cv2.putText(disp, f'crossings:{len(found)}  (r=orange l=blue, gray=out-of-range)',
                        (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(win, disp)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('s'):
                cv2.imwrite('/tmp/orbbec_localize.png', disp)
                print('  저장 /tmp/orbbec_localize.png')
            elif k == ord('p'):
                print(f'\n=== 교차점 {len(found)}개 ===')
                for pose, x, y, u, v in sorted(found, key=lambda f: (f[0], -f[2])):
                    print(f'  [{pose}] robot=({x:6.1f},{y:6.1f})mm  px=({u},{v})')
    finally:
        cv2.destroyAllWindows(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
