#!/usr/bin/env python3
"""USB(UVC) 웹캠 → 측면 판정용 압축 영상 발행.

## 왜 (2026-09-22)
GMSL 카메라 4대 동시 가동은 가동 6~26분 만에 무너졌다(좌측 모노 NOT INITIALIZED →
스테레오 CORRUPTED/REBOOTING). 3대는 8시간 무사. 측면 카메라는 **횡이동 직전 몇 장**만
찍어 "옆으로 갈 수 있나"를 보는 용도라 고성능이 필요 없다 → 좌측(추후 우측)을 USB 웹캠으로.

## 발행
  /<name>/image/compressed  (sensor_msgs/CompressedImage, jpeg, header.stamp = 촬영 직후)
  deck_edge `side_left_topic` 을 여기로 돌리면 된다. 판정은 header.stamp 로 신선도를 본다.

## 동작
  · 구독자가 없으면 **인코딩·발행을 쉬고** 프레임만 흘려보낸다(버퍼에 옛 프레임이 쌓이지 않게).
  · 장치가 빠지거나 읽기 실패가 이어지면 닫고 2초마다 다시 연다(USB 끊김 대비).
  · 초점/노출은 v4l2-ctl 로 시작 때 적용(지원 안 하는 항목은 조용히 넘어감).

## 장치 지정
  device:='' (기본) → /dev/v4l/by-id/ 중 Orbbec 이 아닌 첫 `*-video-index0`
  device:=/dev/v4l/by-id/usb-XXXX-video-index0  (권장: by-id 는 꽂는 순서와 무관)

    ros2 run rebar_vision usb_side_cam --ros-args -p name:=side_left
"""
import glob
import os
import subprocess
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

SKIP = ('orbbec', 'gemini')


def auto_device():
    for p in sorted(glob.glob('/dev/v4l/by-id/*-video-index0')):
        if not any(s in p.lower() for s in SKIP):
            return p
    return None


class UsbSideCam(Node):
    def __init__(self):
        super().__init__('usb_side_cam')
        self.declare_parameter('name', 'side_left')
        self.declare_parameter('device', '')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 15)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('frame_id', 'side_left_camera')
        # v4l2-ctl -c 로 넘길 설정. 고정초점 제품은 focus 항목이 없어 무시된다.
        self.declare_parameter('v4l2_controls',
                               'focus_automatic_continuous=0,power_line_frequency=1')
        g = lambda k: self.get_parameter(k).value
        self.name = g('name')
        self.dev_param = g('device')
        self.w, self.h, self.fps = int(g('width')), int(g('height')), int(g('fps'))
        self.q = int(g('jpeg_quality'))
        self.frame_id = g('frame_id')
        self.ctrls = g('v4l2_controls')
        self.pub = self.create_publisher(CompressedImage, f'/{self.name}/image/compressed',
                                         qos_profile_sensor_data)
        self.cap = None
        self.dev = None
        self.fail = 0
        self.last_open_try = 0.0
        self.sent = 0
        self.create_timer(1.0 / max(1, self.fps), self._tick)
        self.create_timer(30.0, self._report)

    def _open(self):
        self.last_open_try = time.monotonic()
        dev = self.dev_param or auto_device()
        if not dev or not os.path.exists(dev):
            self.get_logger().warn(f'USB 카메라 없음 (device={dev or "자동탐색 실패"})',
                                   throttle_duration_sec=30.0)
            return
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f'{dev} 열기 실패', throttle_duration_sec=30.0)
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for c in [c for c in self.ctrls.split(',') if c.strip()]:
            subprocess.run(['v4l2-ctl', '-d', dev, '-c', c.strip()],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        aw, ah = cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.cap, self.dev, self.fail = cap, dev, 0
        self.get_logger().info(f'✅ {dev} 열림 {int(aw)}x{int(ah)} @{self.fps} → '
                               f'/{self.name}/image/compressed')

    def _close(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        self.cap = None

    def _tick(self):
        if self.cap is None:
            if time.monotonic() - self.last_open_try > 2.0:
                self._open()
            return
        ok = self.cap.grab()          # 구독자 없어도 grab 해서 버퍼를 비운다
        if not ok:
            self.fail += 1
            if self.fail >= 10:
                self.get_logger().error(f'{self.dev} 읽기 실패 연속 → 다시 연다')
                self._close()
            return
        self.fail = 0
        if self.pub.get_subscription_count() == 0:
            return
        ok, frame = self.cap.retrieve()
        if not ok or frame is None:
            return
        stamp = self.get_clock().now().to_msg()
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.q])
        if not ok:
            return
        m = CompressedImage()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.format = 'jpeg'
        m.data = buf.tobytes()
        self.pub.publish(m)
        self.sent += 1

    def _report(self):
        if self.sent:
            self.get_logger().info(f'30초간 {self.sent}장 발행 ({self.sent / 30:.1f}Hz)')
        self.sent = 0


def main(args=None):
    rclpy.init(args=args)
    n = UsbSideCam()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n._close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
