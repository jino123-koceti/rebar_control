#!/usr/bin/env python3
"""
Orbbec 작업영역 이미지 캡처 (교차점 결속여부 YOLO 학습 데이터 수집)

리모콘 주행 중 Orbbec Gemini2L 컬러 이미지를 일정 주기로 저장한다.
이후 Roboflow에서 교차점 bound/unbound(결속/미결속) 라벨링 → YOLO 학습.

특징:
- odom 불필요 (작업영역 정지 이미지 수집 목적)
- 풀프레임 컬러 저장 (교차점 여러 개 한 프레임에 라벨 가능)

사용:
  python3 tools/vision_test/capture_orbbec_frames.py --rate 3

옵션:
  --topic       컬러 토픽 (default: /camera/color/image_raw)
  --rate        저장 주기 Hz (default: 3)
  --outdir      저장 폴더 (default: data/vision_test/orbbec_<타임스탬프>)
  --max-frames  최대 프레임 수 (default: 0=무제한)
  --min-move    직전 저장분과 픽셀차 이 값 미만이면 스킵(정지중복 방지, 0=off)
"""

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class OrbbecCapture(Node):
    def __init__(self, args):
        super().__init__('orbbec_capture')
        self.bridge = CvBridge()
        self.args = args
        self.frame = None
        self.idx = 0
        self.t0 = time.time()
        self.prev_small = None
        os.makedirs(args.outdir, exist_ok=True)

        # QoS: 센서 데이터(BEST_EFFORT)일 수 있어 기본 10 depth
        self.create_subscription(Image, args.topic, self._cb, 10)
        self.timer = self.create_timer(1.0 / args.rate, self._save)
        self.get_logger().info(f"토픽: {args.topic}  @ {args.rate}Hz")
        self.get_logger().info(f"저장: {args.outdir}")
        self.get_logger().info("이미지 대기중... (리모콘 주행 시작하세요, Ctrl-C 종료)")

    def _cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")

    def _changed_enough(self, frame):
        """정지 중 거의 동일 프레임 스킵 (min-move)."""
        if self.args.min_move <= 0:
            return True
        small = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (64, 40))
        if self.prev_small is None:
            self.prev_small = small
            return True
        diff = float(np.mean(np.abs(small.astype(int) - self.prev_small.astype(int))))
        if diff >= self.args.min_move:
            self.prev_small = small
            return True
        return False

    def _save(self):
        if self.frame is None:
            return
        if not self._changed_enough(self.frame):
            return
        self.idx += 1
        fn = f"orbbec_{self.idx:05d}.png"
        cv2.imwrite(os.path.join(self.args.outdir, fn), self.frame)
        if self.idx % 10 == 0:
            t = time.time() - self.t0
            self.get_logger().info(f"[{self.idx}] 저장중... (경과 {t:.0f}s)")
        if self.args.max_frames and self.idx >= self.args.max_frames:
            self.get_logger().info("최대 프레임 도달 → 종료")
            raise KeyboardInterrupt

    def close(self):
        self.get_logger().info(f"저장 완료: {self.idx} 프레임 → {self.args.outdir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/color/image_raw')
    ap.add_argument('--rate', type=float, default=3.0)
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--min-move', type=float, default=0.0,
                    help='직전 저장분 대비 평균 픽셀차 임계(정지중복 방지). 예 3.0')
    stamp = time.strftime('%Y%m%d_%H%M%S')
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
    ap.add_argument('--outdir',
                    default=os.path.join(root, 'data', 'vision_test',
                                         f'orbbec_{stamp}'))
    args = ap.parse_args()

    rclpy.init()
    node = OrbbecCapture(args)
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
