#!/usr/bin/env python3
"""프레임 스트리밍 서버 (Jetson/로봇 측)  ─ 학생 Windows 클라이언트로 WLAN 전송용.

우리 시스템의 카메라 ROS2 토픽(컬러 + 뎁스)을 구독해서, 학생 프로그램이
TCP로 요청할 때마다 "최신 한 프레임"을 소켓으로 내려보낸다.
학생 쪽(frame_client.hpp)은 이걸 받아 cv::Mat image_bgr / depth_f32(mm)로 복원.

■ 요청/응답 방식: 클라이언트가 1바이트 'G'를 보내면 서버가 최신 프레임 1장 전송.
  (연속 push가 아니라 요청 기반이라 WLAN 버퍼 적체/지연이 없다.)

■ 프로토콜(고정 44바이트 헤더, little-endian. Jetson=ARM64 LE, Windows=x64 LE 둘 다 LE):
    magic      4s   b'RBF1'
    frame_id   u32
    fx,fy,cx,cy 4xf32   (컬러 카메라 내참)
    width,height 2xu32  (컬러 해상도)
    depth_scale f32     (뎁스 raw값 → mm 배율. Orbbec 16UC1=mm 이므로 1.0)
    color_len  u32
    depth_len  u32
  이후: color_len 바이트(JPEG) + depth_len 바이트(16bit PNG, 무손실).

실행:
    ros2 run 없이 단독:  python3 frame_stream_server.py
    (Orbbec 카메라가 이미 떠 있어야 함: robot-control 서비스가 자동 기동)

인자:
    --port 5001
    --color /camera/color/image_raw
    --depth /camera/depth/image_raw
    --info  /camera/color/camera_info
    --jpeg-quality 80

⚠ 뎁스-컬러 정합: 이 파이프라인은 뎁스에서 만든 점을 "컬러 내참"으로 이미지에
   재투영해 컬러를 마스킹한다. 따라서 뎁스가 컬러 프레임에 정합(D2C)돼 있어야
   픽셀이 맞는다. gemini2L.yaml 에서 depth_registration: true 로 켤 것.
   (정합을 켜면 뎁스가 컬러 내참/프레임을 공유 → 여기서 컬러 내참만 보내면 됨.)
"""
import argparse
import socket
import struct
import threading

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

MAGIC = b'RBF1'
HEADER_FMT = '<4sI4f2IfII'          # 44 bytes
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 44, HEADER_SIZE


class FrameHub(Node):
    """카메라 토픽 구독 → 최신 (컬러 JPEG, 뎁스 PNG16, 내참) 보관."""

    def __init__(self, args):
        super().__init__('frame_stream_server')
        self.br = CvBridge()
        self.jpeg_q = args.jpeg_quality
        self.depth_scale = args.depth_scale
        self._lock = threading.Lock()
        self._msgs = None              # (color_msg, depth_msg, frame_id)  ← 원본만 보관
        self._fid = 0
        self._K = None                 # (fx,fy,cx,cy) from camera_info

        self.create_subscription(CameraInfo, args.info, self._info_cb,
                                 qos_profile_sensor_data)
        color_sub = Subscriber(self, Image, args.color,
                               qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, args.depth,
                               qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self._frame_cb)
        self.get_logger().info(
            f'구독: color={args.color} depth={args.depth} info={args.info}')

    def _info_cb(self, m):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self._K = (float(m.k[0]), float(m.k[4]), float(m.k[2]), float(m.k[5]))

    def _frame_cb(self, color_msg, depth_msg):
        # ★유휴 부담 0: 여기선 인코딩하지 않고 "원본 메시지 참조"만 보관.
        #   JPEG/PNG 인코딩은 클라이언트 요청이 실제로 올 때만(encode_latest) 수행.
        with self._lock:
            self._fid += 1
            self._msgs = (color_msg, depth_msg, self._fid)

    def encode_latest(self):
        """요청 시점에만 호출 → 최신 원본을 JPEG+PNG16으로 인코딩해 패킷 페이로드 반환.
        (frame_id, fx,fy,cx,cy, w,h, color_bytes, depth_bytes) 또는 None."""
        with self._lock:
            msgs = self._msgs
            K = self._K
        if msgs is None or K is None:
            return None
        color_msg, depth_msg, fid = msgs
        color = self.br.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.br.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth.dtype != np.uint16:
            # 16UC1(mm) 가정. float 등으로 오면 mm 정수로 환산.
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            depth = np.clip(depth / self.depth_scale, 0, 65535).astype(np.uint16)
        ok_c, cbuf = cv2.imencode(
            '.jpg', color, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
        ok_d, dbuf = cv2.imencode('.png', depth)   # 16bit 무손실
        if not (ok_c and ok_d):
            return None
        h, w = color.shape[:2]
        fx, fy, cx, cy = K
        return (fid, fx, fy, cx, cy, w, h, cbuf.tobytes(), dbuf.tobytes())


def serve_client(conn, addr, hub, scale, log):
    log.info(f'클라이언트 접속: {addr}')
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        while True:
            req = conn.recv(1)          # 1바이트 요청 대기
            if not req:
                break
            fr = hub.encode_latest()    # ★요청이 왔을 때만 인코딩 (유휴 시 CPU 0)
            if fr is None:
                # 아직 프레임 없음 → 빈 헤더(len 0) 회신
                conn.sendall(struct.pack(
                    HEADER_FMT, MAGIC, 0, 0, 0, 0, 0, 0, 0, scale, 0, 0))
                continue
            fid, fx, fy, cx, cy, w, h, cbytes, dbytes = fr
            hdr = struct.pack(HEADER_FMT, MAGIC, fid, fx, fy, cx, cy,
                              w, h, scale, len(cbytes), len(dbytes))
            conn.sendall(hdr + cbytes + dbytes)
    except (ConnectionError, OSError) as e:
        log.info(f'클라이언트 종료 {addr}: {e}')
    finally:
        conn.close()


def accept_loop(hub, port, scale, log):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', port))
    srv.listen(4)
    log.info(f'TCP 대기 중: 0.0.0.0:{port}')
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=serve_client,
                         args=(conn, addr, hub, scale, log),
                         daemon=True).start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=5001)
    ap.add_argument('--color', default='/camera/color/image_raw')
    ap.add_argument('--depth', default='/camera/depth/image_raw')
    ap.add_argument('--info', default='/camera/color/camera_info')
    ap.add_argument('--jpeg-quality', type=int, default=80)
    ap.add_argument('--depth-scale', type=float, default=1.0,
                    dest='depth_scale', help='뎁스 raw 1단위 = ? mm (Orbbec=1.0)')
    args = ap.parse_args()

    rclpy.init()
    hub = FrameHub(args)
    threading.Thread(target=accept_loop,
                     args=(hub, args.port, args.depth_scale, hub.get_logger()),
                     daemon=True).start()
    try:
        rclpy.spin(hub)
    except KeyboardInterrupt:
        pass
    finally:
        hub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
