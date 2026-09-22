#!/usr/bin/env python3
"""전·후방 ZED 카메라 동시 mp4 레코딩 (리모콘 주행 데이터 수집용).

데크끝/floor 감지 코드 개발용 주행 영상 수집. 프리즈 이력(ZED GMSL 부하)을 고려해
**compressed 토픽 구독 + 출력 fps로만 디코드**(저부하). 카메라별 mp4 개별 저장.

[2026-08-06] zedxmini1/2를 좌·우 측면 → 전방/후방 물리 이동 (zedxmini2=전방, zedxmini1=후방). 측면 이동이 아직
  미구현이라 left/right 항목은 제거했다 (해당 위치에 카메라 없음).

  python3 tools/drive/record_drive_cams.py                 # 전·후방, 15fps, 640폭
  python3 tools/drive/record_drive_cams.py --cams front
  python3 tools/drive/record_drive_cams.py --fps 10 --width 0   # 원해상도
  python3 tools/drive/record_drive_cams.py --duration 60        # 60초 후 자동정지
  # 정지: Ctrl+C (mp4 정상 마감)

저장: data/drive_rec/<타임스탬프>/{front,back}.mp4 + meta.json
"""
import os
import json
import time
import argparse
from datetime import datetime

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

ROOT = '/home/koceti/ros2_ws'

# 전/후방 → ZED rgb rect_color 토픽 (base, compressed는 '/compressed' 추가)
CAM_TOPICS = {
    'front': '/zedxmini2/zed_node/rgb/color/rect/image',
    'back':  '/zedxmini1/zed_node/rgb/color/rect/image',
}


class DriveCamRecorder(Node):
    def __init__(self, cams, fps, width, outdir, use_raw, duration):
        super().__init__('drive_cam_recorder')
        self.fps = fps
        self.width = width
        self.outdir = outdir
        self.use_raw = use_raw
        self.duration = duration
        self.br = CvBridge()
        self.latest = {c: None for c in cams}      # 최신 메시지(디코드 전)
        self.writers = {c: None for c in cams}
        self.sizes = {c: None for c in cams}
        self.nwrite = {c: 0 for c in cams}
        self.nrecv = {c: 0 for c in cams}
        os.makedirs(outdir, exist_ok=True)

        for c in cams:
            topic = CAM_TOPICS[c]
            if use_raw:
                self.create_subscription(
                    Image, topic,
                    lambda m, cc=c: self._store(cc, m), qos_profile_sensor_data)
            else:
                self.create_subscription(
                    CompressedImage, topic + '/compressed',
                    lambda m, cc=c: self._store(cc, m), qos_profile_sensor_data)
            self.get_logger().info(
                f'  구독: {c} ← {topic}{"" if use_raw else "/compressed"}')

        self.t0 = time.time()
        self.t_stat = time.time()
        self.timer = self.create_timer(1.0 / fps, self._tick)   # 출력 fps로 기록
        self.get_logger().info(
            f'레코딩 시작: {len(cams)}대 @ {fps}fps '
            f'{"원해상도" if width == 0 else f"폭{width}"} → {outdir}')

    def _store(self, cam, msg):
        self.latest[cam] = msg
        self.nrecv[cam] += 1

    def _decode(self, cam, msg):
        if self.use_raw:
            img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        else:
            arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return None
        if self.width and img.shape[1] > self.width:
            h = int(img.shape[0] * self.width / img.shape[1])
            img = cv2.resize(img, (self.width, h))
        return img

    def _tick(self):
        now = time.time()
        for cam, msg in self.latest.items():
            if msg is None:
                continue
            img = self._decode(cam, msg)
            if img is None:
                continue
            if self.writers[cam] is None:
                h, w = img.shape[:2]
                path = os.path.join(self.outdir, f'{cam}.mp4')
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writers[cam] = cv2.VideoWriter(path, fourcc, self.fps, (w, h))
                self.sizes[cam] = (w, h)
                if not self.writers[cam].isOpened():
                    self.get_logger().error(f'  ⚠ {cam} VideoWriter 열기 실패: {path}')
            self.writers[cam].write(img)
            self.nwrite[cam] += 1
        # 3초마다 상태
        if now - self.t_stat >= 3.0:
            el = now - self.t0
            stat = '  '.join(
                f'{c}:{self.nwrite[c]}f({self.nrecv[c]}rx)' for c in self.latest)
            self.get_logger().info(f'[{el:5.1f}s] {stat}')
            self.t_stat = now
        if self.duration and (now - self.t0) >= self.duration:
            self.get_logger().info('지정 시간 도달 → 정지')
            raise KeyboardInterrupt

    def finalize(self):
        meta = {'time': datetime.now().isoformat(), 'fps': self.fps,
                'width': self.width, 'use_raw': self.use_raw,
                'cams': {}, 'duration_s': round(time.time() - self.t0, 1)}
        for cam, w in self.writers.items():
            if w is not None:
                w.release()
            meta['cams'][cam] = {'topic': CAM_TOPICS[cam],
                                 'frames': self.nwrite[cam],
                                 'recv': self.nrecv[cam],
                                 'size': self.sizes[cam]}
        with open(os.path.join(self.outdir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f'저장 완료: {self.outdir}')
        for cam in self.writers:
            self.get_logger().info(
                f'  {cam}.mp4: {self.nwrite[cam]}프레임 (수신 {self.nrecv[cam]})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cams', default='front,back',
                    help='기록할 카메라 (쉼표): front,back')
    ap.add_argument('--fps', type=int, default=15, help='출력 mp4 fps')
    ap.add_argument('--width', type=int, default=640, help='다운스케일 폭(0=원본)')
    ap.add_argument('--raw', action='store_true', help='compressed 대신 raw 구독')
    ap.add_argument('--duration', type=float, default=0.0, help='자동정지 초(0=무한)')
    ap.add_argument('--outdir', default=None)
    args = ap.parse_args()

    cams = [c.strip() for c in args.cams.split(',') if c.strip() in CAM_TOPICS]
    if not cams:
        print('⚠ 유효 카메라 없음 (front,back,left,right)'); return
    outdir = args.outdir or os.path.join(
        ROOT, 'data/drive_rec', datetime.now().strftime('%Y%m%d_%H%M%S'))

    rclpy.init()
    node = DriveCamRecorder(cams, args.fps, args.width, outdir,
                            args.raw, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('중단(Ctrl+C) → mp4 마감 중...')
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
