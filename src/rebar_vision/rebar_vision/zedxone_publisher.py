#!/usr/bin/env python3
"""
ZED X One (mono) 캡처 노드 — 결속 미세보정용 top-view 카메라.

⚠️ ZED X One open/close 가 동시 구동중인 ZED X 스테레오의 GMSL/데몬 상태를 교란하여
   스테레오가 크래시하는 문제가 있음. 이를 피하기 위한 전략:
     1) 시작 시 1회만 open (스테레오 안정 후 retry) — 늦은 반복 open/close 회피
     2) 절대 close 안 함 (열어둔 채 유지) — open/close 사이클 제거
     3) 연속 grab 안 함 — 요청(/zedxone/capture) 시에만 grab (대역폭 부하 최소화)
     4) 저해상도(SVGA) — GMSL 부하 감소 (미세보정엔 충분)

ZED 래퍼 모노 composable 로드 실패로 pyzed SDK 직접 사용.

서비스: /zedxone/capture  (std_srvs/Trigger) → 1프레임 grab·발행·PNG저장
발행:   /zedxone/image_rect_color (Image, bgr8, BEST_EFFORT), /zedxone/camera_info
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import pyzed.sl as sl


RES_MAP = {
    'HD4K': sl.RESOLUTION.HD4K, 'QHDPLUS': sl.RESOLUTION.QHDPLUS,
    'HD1200': sl.RESOLUTION.HD1200, 'HD1080': sl.RESOLUTION.HD1080,
    'SVGA': sl.RESOLUTION.SVGA, 'AUTO': sl.RESOLUTION.AUTO,
}


class ZedXOnePublisher(Node):
    def __init__(self):
        super().__init__('zedxone_publisher')
        self.declare_parameter('serial_number', 313997679)
        self.declare_parameter('resolution', 'SVGA')   # 저부하
        self.declare_parameter('fps', 15)
        self.declare_parameter('frame_id', 'zedxone_left_camera_frame')
        self.declare_parameter('open_delay_sec', 25.0)  # 스테레오 안정 대기 후 open

        self._sn = int(self.get_parameter('serial_number').value)
        res_name = self.get_parameter('resolution').value
        self._fps = int(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value
        self._open_delay = float(self.get_parameter('open_delay_sec').value)
        self._res = RES_MAP.get(res_name, sl.RESOLUTION.SVGA)

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(
            Image, '/zedxone/image_rect_color', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, '/zedxone/camera_info', qos_profile_sensor_data)

        self.cam = sl.CameraOne()
        self._init = sl.InitParametersOne()
        self._init.set_from_serial_number(self._sn)
        self._init.camera_resolution = self._res
        self._init.camera_fps = self._fps
        self.cam_info_msg = None
        self.mat = sl.Mat()
        self._opened = False
        self._attempts = 0

        self.srv = self.create_service(Trigger, '/zedxone/capture', self._on_capture)
        # open_delay 후 첫 시도, 이후 실패 시 5초마다 재시도 (스테레오 안정 후 1회 open)
        import time
        self._start_mono = time.monotonic()
        self.open_timer = self.create_timer(5.0, self._try_open)
        self.get_logger().info(
            f'ZED X One 노드 시작 — {self._open_delay:.0f}초 후 1회 open 시도 '
            f'({res_name}@{self._fps}, 이후 안 닫음, grab은 on-demand)')

    def _try_open(self):
        import time
        if self._opened:
            return
        if time.monotonic() - self._start_mono < self._open_delay:
            return  # 스테레오 안정 대기
        self._attempts += 1
        status = self.cam.open(self._init)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn(f'open 시도 {self._attempts} 실패: {status} → 재시도')
            return
        self._opened = True
        self.open_timer.cancel()
        ci = self.cam.get_camera_information()
        self.w = ci.camera_configuration.resolution.width
        self.h = ci.camera_configuration.resolution.height
        self.cam_info_msg = self._build_camera_info(ci)
        self.get_logger().info(
            f'✅ ZED X One open 성공 ({self.w}x{self.h}, {self._attempts}회) — 열린 채 유지, grab 대기')

    def _on_capture(self, request, response):
        if not self._opened:
            response.success = False
            response.message = 'ZED X One 아직 open 안 됨 (대기 중)'
            return response
        ok = False
        for _ in range(3):
            if self.cam.grab() == sl.ERROR_CODE.SUCCESS:
                ok = True
                break
        if not ok:
            response.success = False
            response.message = 'grab 실패'
            return response
        self.cam.retrieve_image(self.mat, sl.VIEW.LEFT)
        bgr = np.ascontiguousarray(self.mat.get_data()[:, :, :3])
        msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.img_pub.publish(msg)
        self.cam_info_msg.header = msg.header
        self.info_pub.publish(self.cam_info_msg)
        save_path = ''
        try:
            import os, cv2
            d = '/home/koceti/ros2_ws/data/images'
            os.makedirs(d, exist_ok=True)
            save_path = os.path.join(d, 'zedxone_capture.png')
            cv2.imwrite(save_path, bgr)
        except Exception as e:
            self.get_logger().warn(f'PNG 저장 실패: {e}')
        response.success = True
        response.message = f'{self.w}x{self.h} 캡처·발행 (저장: {save_path})'
        return response

    def _build_camera_info(self, cam_info):
        ci = CameraInfo()
        ci.width = int(self.w)
        ci.height = int(self.h)
        try:
            calib = cam_info.camera_configuration.calibration_parameters
            cp = getattr(calib, 'cam', None) or getattr(calib, 'left_cam', None)
            if cp is not None:
                fx, fy, cx, cy = cp.fx, cp.fy, cp.cx, cp.cy
                ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
                ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
                ci.distortion_model = 'plumb_bob'
        except Exception:
            pass
        return ci

    def destroy_node(self):
        # 의도적으로 close 안 함 (close가 스테레오 교란) — 프로세스 종료 시 OS가 정리
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZedXOnePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
