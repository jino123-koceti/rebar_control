#!/usr/bin/env python3
"""좌측/우측 자세에서 툴캠(zedxone) 뷰 + YOLO 교차점 검출 확인.
자세변경 후 툴캠이 교차점을 보는지, 검출되는지, 결속점이 어느 픽셀인지 파악용.

사용:
  1) 로봇을 원하는 자세(예: 좌측)로 이동시킨 뒤 (CHN_POS=left) 실행
  2) python3 toolcam_pose_check.py --tag left
저장: /tmp/toolcam_view_<tag>.png (원본), /tmp/toolcam_det_<tag>.png (검출 오버레이)
"""
import sys, time, argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/toolcam_crossing.pt'
TOPIC = '/zedxone/zed_node/rgb/rect/image'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', default='left', help='저장 태그 (left/right)')
    ap.add_argument('--conf', type=float, default=0.15)
    args = ap.parse_args()

    rclpy.init()
    n = Node('toolcam_pose_check')
    br = CvBridge(); buf = []
    n.create_subscription(Image, TOPIC,
                          lambda m: buf.append(br.imgmsg_to_cv2(m, 'bgr8')),
                          qos_profile_sensor_data)
    print(f'툴캠 뷰 획득중 ({TOPIC}) ...')
    t = time.time()
    while len(buf) < 5 and time.time() - t < 8:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not buf:
        print('❌ 툴캠 프레임 수신 실패 — zedxone 토픽 확인'); return
    img = buf[-1]
    H, W = img.shape[:2]
    print(f'✅ 프레임 {W}x{H}')
    cv2.imwrite(f'/tmp/toolcam_view_{args.tag}.png', img)

    model = YOLO(MODEL)
    r = model(img, conf=args.conf, verbose=False)[0]
    cs = [(float(b.xyxy[0][0]+b.xyxy[0][2])/2,
           float(b.xyxy[0][1]+b.xyxy[0][3])/2,
           float(b.conf[0])) for b in r.boxes]
    disp = img.copy()
    # 우측자세 목표픽셀(694,468) 참고 표시
    cv2.drawMarker(disp, (694, 468), (255, 0, 255),
                   cv2.MARKER_TILTED_CROSS, 40, 2)
    cv2.putText(disp, 'RIGHT-pose target (694,468)', (700, 468),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
    for x, y, c in cs:
        cv2.circle(disp, (int(x), int(y)), 16, (0, 255, 0), 3)
        cv2.putText(disp, f'({x:.0f},{y:.0f}) {c:.2f}', (int(x)+18, int(y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imwrite(f'/tmp/toolcam_det_{args.tag}.png', disp)
    print(f'\n검출 교차점 {len(cs)}개 (conf>{args.conf}):')
    for x, y, c in sorted(cs, key=lambda p: -p[2]):
        print(f'  ({x:.0f}, {y:.0f})  conf={c:.2f}')
    print(f'\n저장: /tmp/toolcam_view_{args.tag}.png  /tmp/toolcam_det_{args.tag}.png')
    if not cs:
        print('⚠️  검출 0개 — 자세 회전으로 뷰가 달라 재학습 필요할 수 있음')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
