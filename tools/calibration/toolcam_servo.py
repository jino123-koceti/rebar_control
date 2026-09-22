#!/usr/bin/env python3
"""[1단계] 툴캠 서보-정렬 테스트 (Z/트리거 없이 정렬만).
툴캠 YOLO로 교차점 검출 → 목표픽셀(477,886)과의 오차 → Jinv·오차 = ΔXY →
스테이지 이동 → 재검출 반복 → 수렴 확인.

⚠️ 실제 스테이지 XY가 움직임. control_mode='auto' + 호밍완료 + 교차점이 툴캠 시야 필요.
⚠️ 안전: 매 이동 전 제안 ΔXY 표시 후 [Enter] 확인해야 실행. ΔXY는 max_step로 제한.

사용: python3 toolcam_servo.py
"""
import time, yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rebar_base_interfaces.msg import JointControl, MotorFeedback
from ultralytics import YOLO

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/toolcam_crossing.pt'
GAIN_YAML = '/home/koceti/ros2_ws/data/calibration/toolcam_gain.yaml'
TARGET = np.array([694.0, 468.0])  # 카메라 반대편 이동 후 재설정 (new_pose_onecam 주석 기준)
DEG_PER_MM_X = 4.497
DEG_PER_MM_Y = 4.462
TOL_PX = 15.0          # 수렴 픽셀오차
MAX_STEP_MM = 35.0     # 1회 이동 제한
GAIN_FACTOR = 0.8      # 안정성 (1.0=풀게인)
VEL = 200.0            # 이동 속도 dps


class Servo(Node):
    def __init__(self):
        super().__init__('toolcam_servo')
        self.br = CvBridge(); self.buf = []
        self.pos = {}     # 0x44:Xdeg, 0x45:Ydeg
        self.create_subscription(Image, '/zedxone/zed_node/rgb/rect/image',
                                 lambda m: self.buf.append(self.br.imgmsg_to_cv2(m, 'bgr8')),
                                 qos_profile_sensor_data)
        self.create_subscription(MotorFeedback, '/motor_feedback', self._fb, 10)
        self.jpub = self.create_publisher(JointControl, '/joint_control', 10)
        self.model = YOLO(MODEL)
        g = yaml.safe_load(open(GAIN_YAML))['toolcam_gain']
        self.Jinv = np.array(g['Jinv_mm_per_px'])
        self.get_logger().info(f'Jinv 로드:\n{self.Jinv}')

    def _fb(self, m):
        if m.motor_id in (0x44, 0x45) and m.status == 0x92:
            self.pos[m.motor_id] = m.current_position

    def spin(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def detect(self, n=8):
        pts = []
        for _ in range(n):
            self.buf.clear(); t = time.time()
            while not self.buf and time.time()-t < 3:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not self.buf:
                continue
            r = self.model(self.buf[-1], conf=0.15, verbose=False)[0]
            cs = [(float(b.xyxy[0][0]+b.xyxy[0][2])/2,
                   float(b.xyxy[0][1]+b.xyxy[0][3])/2) for b in r.boxes]
            if cs:
                cs.sort(key=lambda c: np.hypot(c[0]-TARGET[0], c[1]-TARGET[1]))
                pts.append(cs[0])
        if len(pts) < 2:
            return None
        a = np.array(pts); med = np.median(a, 0)
        keep = a[np.linalg.norm(a-med, axis=1) < 40]
        return keep.mean(0) if len(keep) else med

    def cur_xy_deg(self):
        self.spin(0.4)
        if 0x44 not in self.pos or 0x45 not in self.pos:
            return None
        return self.pos[0x44], self.pos[0x45]

    def move_xy_deg(self, x_deg, y_deg):
        for jid, deg in [(0x144, x_deg), (0x145, y_deg)]:
            msg = JointControl()
            msg.joint_id = jid; msg.position = float(deg)
            msg.velocity = VEL; msg.control_mode = JointControl.MODE_ABSOLUTE
            self.jpub.publish(msg); time.sleep(0.05)

    def wait_arrival(self, x_deg, y_deg, tol=1.0, timeout=6.0):
        t = time.time()
        while time.time()-t < timeout:
            self.spin(0.1)
            if (abs(self.pos.get(0x44, 1e9)-x_deg) < tol and
                    abs(self.pos.get(0x45, 1e9)-y_deg) < tol):
                return True
        return False


def main():
    rclpy.init(); node = Servo()
    print('='*60)
    print(' [1단계] 툴캠 서보-정렬 (Z/트리거 없음, 정렬만)')
    print(' ⚠️ control_mode=auto + 호밍완료 + 교차점이 툴캠 시야에 필요')
    print('='*60)
    node.spin(1.5)
    try:
        for it in range(8):
            cross = node.detect()
            if cross is None:
                print('교차점 검출 실패 — 교차점이 시야에 있나? 종료'); break
            err = TARGET - cross   # 픽셀오차
            epx = np.hypot(*err)
            print(f"\n[{it+1}] 교차점=({cross[0]:.0f},{cross[1]:.0f}) "
                  f"목표=({TARGET[0]:.0f},{TARGET[1]:.0f}) 오차={epx:.0f}px")
            if epx < TOL_PX:
                print(f"  ✅ 수렴! (오차 {epx:.0f}px < {TOL_PX}px) — 정렬 완료")
                break
            dxy = GAIN_FACTOR * (node.Jinv @ err)   # mm
            mag = np.hypot(*dxy)
            if mag > MAX_STEP_MM:
                dxy = dxy / mag * MAX_STEP_MM
            print(f"  제안 이동 ΔXY = ({dxy[0]:+.1f}, {dxy[1]:+.1f})mm")
            cur = node.cur_xy_deg()
            if cur is None:
                print('  ⚠️ 모터위치 수신 안됨(/motor_feedback) — 종료'); break
            xd, yd = cur
            nx = xd - dxy[0]*DEG_PER_MM_X    # x_deg = home - x_mm·dpm
            ny = yd + dxy[1]*DEG_PER_MM_Y
            ans = input(f"  실행? [Enter=이동 / s=건너뛰기 / q=종료]: ").strip()
            if ans == 'q':
                break
            if ans == 's':
                continue
            node.move_xy_deg(nx, ny)
            ok = node.wait_arrival(nx, ny)
            print(f"  이동 {'완료' if ok else '타임아웃(계속)'}")
            node.spin(0.5)
        else:
            print("\n최대 반복 도달")
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
