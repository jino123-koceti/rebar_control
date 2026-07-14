#!/usr/bin/env python3
"""
주행 프레임 캡처 (전면 카메라 + odom 정답값)

로봇이 철근 위를 주행하는 동안 전면 카메라 프레임을 일정 주기로 저장하고,
각 프레임에 odom(위치/헤딩)을 함께 기록한다. 이렇게 하면 이후
하이브리드 검출기의 이동거리/누적카운팅 추적을 "정답값(odom)"과 대조 검증할 수 있다.

저장물 (outdir):
  frame_00001.jpg ...           # 프레임
  manifest.csv                  # idx,filename,t_sec,odom_x,odom_y,odom_yaw_deg,dx_from_start

사용:
  # 1) 먼저 전면 카메라 실행 (별도 터미널)
  ros2 launch zed_wrapper zed_camera.launch.py \
       camera_name:=zed_front camera_model:=zedx serial_number:=45320958

  # 2) 캡처 시작 → 로봇 저속 전진 주행 → Ctrl-C 로 종료
  python3 tools/vision_test/capture_drive_frames.py --rate 5

옵션:
  --topic       이미지 토픽 (default: /zed_front/zed_node/left/image_rect_color)
  --odom-topic  odom 토픽   (default: /zed_front/zed_node/odom)
  --rate        저장 주기 Hz (default: 5)
  --outdir      저장 폴더 (default: data/vision_test/drive_<타임스탬프>)
  --max-frames  최대 프레임 수 (default: 0=무제한)
"""

import argparse
import csv
import math
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


class DriveCapture(Node):
    def __init__(self, args):
        super().__init__('drive_capture')
        self.bridge = CvBridge()
        self.args = args
        self.frame = None
        self.odom = None          # (x, y, yaw_deg)
        self.start_odom = None
        self.idx = 0
        self.t0 = time.time()

        os.makedirs(args.outdir, exist_ok=True)
        self.csv_path = os.path.join(args.outdir, 'manifest.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['idx', 'filename', 't_sec', 'odom_x', 'odom_y',
                              'odom_yaw_deg', 'dx_from_start_m'])

        self.create_subscription(Image, args.topic, self._img_cb, 1)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 10)
        self.timer = self.create_timer(1.0 / args.rate, self._save)

        self.get_logger().info(f"이미지 토픽: {args.topic}")
        self.get_logger().info(f"odom 토픽 : {args.odom_topic}")
        self.get_logger().info(f"저장 폴더 : {args.outdir}  @ {args.rate}Hz")
        self.get_logger().info("이미지 대기중... (로봇 전진 주행 시작하세요)")

    def _img_cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        if self.start_odom is None:
            self.start_odom = self.odom

    def _save(self):
        if self.frame is None:
            return
        self.idx += 1
        fn = f"frame_{self.idx:05d}.jpg"
        cv2.imwrite(os.path.join(self.args.outdir, fn), self.frame)

        t = time.time() - self.t0
        if self.odom and self.start_odom:
            ox, oy, oyaw = self.odom
            dx = math.hypot(ox - self.start_odom[0], oy - self.start_odom[1])
        else:
            ox = oy = oyaw = dx = float('nan')
        self.writer.writerow([self.idx, fn, f"{t:.3f}", f"{ox:.4f}",
                              f"{oy:.4f}", f"{oyaw:.2f}", f"{dx:.4f}"])
        self.csv_file.flush()

        if self.idx % 10 == 0:
            self.get_logger().info(
                f"[{self.idx}] t={t:.1f}s  이동={dx:.3f}m  yaw={oyaw:.1f}deg")

        if self.args.max_frames and self.idx >= self.args.max_frames:
            self.get_logger().info("최대 프레임 도달 → 종료")
            raise KeyboardInterrupt

    def close(self):
        self.csv_file.close()
        self.get_logger().info(
            f"저장 완료: {self.idx} 프레임 → {self.args.outdir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic',
                    default='/zed_front/zed_node/left/image_rect_color')
    ap.add_argument('--odom-topic', default='/zed_front/zed_node/odom')
    ap.add_argument('--rate', type=float, default=5.0)
    ap.add_argument('--max-frames', type=int, default=0)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
    ap.add_argument('--outdir',
                    default=os.path.join(root, 'data', 'vision_test',
                                         f'drive_{stamp}'))
    args = ap.parse_args()

    rclpy.init()
    node = DriveCapture(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
