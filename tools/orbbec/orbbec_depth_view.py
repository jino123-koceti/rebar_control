#!/usr/bin/env python3
"""Orbbec Gemini 2L depth 실시간 뷰어 (ROS2).
depth를 컬러맵으로 플롯 + 클릭하면 그 픽셀 거리(mm) + 중앙 ROI depth 커버리지/거리.
교차점 3d 검출 전에 depth 품질/범위 확인용.

  ros2 launch orbbec_camera gemini2L.launch.py   # 먼저 카메라 실행
  python3 orbbec_depth_view.py
  python3 orbbec_depth_view.py --near 200 --far 1500   # 컬러맵 범위(mm)

  마우스이동/클릭  그 픽셀 거리(mm) 표시     c  컬러 창 토글
  s  스냅샷 저장(/tmp/orbbec_depth.png)      q/ESC 종료
"""
import argparse
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthView(Node):
    def __init__(self, args):
        super().__init__('orbbec_depth_view')
        self.br = CvBridge()
        self.args = args
        self.depth = None      # 원본 depth (mm, uint16)
        self.color = None
        self.mouse = (0, 0)
        self.create_subscription(Image, args.depth, self._depth_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, args.color, self._color_cb,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'depth={args.depth}  color={args.color}')

    def _depth_cb(self, m):
        d = self.br.imgmsg_to_cv2(m, desired_encoding='passthrough')
        self.depth = d.astype(np.uint16) if d.dtype != np.uint16 else d

    def _color_cb(self, m):
        self.color = self.br.imgmsg_to_cv2(m, 'bgr8')

    def on_mouse(self, ev, x, y, flags, param):
        self.mouse = (x, y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--depth', default='/camera/depth/image_raw')
    ap.add_argument('--color', default='/camera/color/image_raw')
    ap.add_argument('--near', type=int, default=200, help='컬러맵 근거리(mm)')
    ap.add_argument('--far', type=int, default=1500, help='컬러맵 원거리(mm)')
    args = ap.parse_args()

    rclpy.init()
    node = DepthView(args)
    win = 'Orbbec depth (mouse=dist  c=color  s=save  q=quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 1280, 800)
    cv2.setMouseCallback(win, node.on_mouse)
    show_color = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.depth is None:
                continue
            d = node.depth
            H, W = d.shape[:2]
            valid = d > 0
            # 컬러맵 (near~far mm → JET, 무효=검정)
            dn = np.clip((d.astype(np.float32) - args.near) /
                         max(1, args.far - args.near), 0, 1)
            cm = cv2.applyColorMap((dn * 255).astype(np.uint8), cv2.COLORMAP_JET)
            cm[~valid] = (0, 0, 0)
            # 중앙 ROI 통계 (결속영역 가늠)
            r = 60
            cy, cx = H // 2, W // 2
            roi = d[cy-r:cy+r, cx-r:cx+r]
            rv = roi[roi > 0]
            cov = 100.0 * valid.mean()
            roi_cov = 100.0 * (rv.size / roi.size) if roi.size else 0
            roi_mm = int(np.median(rv)) if rv.size else 0
            cv2.rectangle(cm, (cx-r, cy-r), (cx+r, cy+r), (255, 255, 255), 1)
            cv2.drawMarker(cm, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)
            # 마우스 픽셀 거리
            mx, my = node.mouse
            if 0 <= mx < W and 0 <= my < H:
                mm = int(d[my, mx])
                txt = f'({mx},{my}) {mm}mm' if mm > 0 else f'({mx},{my}) 무효'
                cv2.circle(cm, (mx, my), 5, (255, 255, 255), 1)
                cv2.putText(cm, txt, (mx+8, my-8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)
            cv2.putText(cm, f'{W}x{H}  cov{cov:.0f}%  ROI:{roi_mm}mm/{roi_cov:.0f}%'
                        f'  range{args.near}-{args.far}mm',
                        (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow(win, cm)
            if show_color and node.color is not None:
                cv2.imshow('color', node.color)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('c'):
                show_color = not show_color
                if not show_color:
                    cv2.destroyWindow('color')
            elif k == ord('s'):
                cv2.imwrite('/tmp/orbbec_depth.png', cm)
                if node.color is not None:
                    cv2.imwrite('/tmp/orbbec_color.png', node.color)
                print('  저장 /tmp/orbbec_depth.png (+color)')
    finally:
        cv2.destroyAllWindows(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
