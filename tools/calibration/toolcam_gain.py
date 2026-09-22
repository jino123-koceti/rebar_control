#!/usr/bin/env python3
"""툴캠 픽셀→XY 보정 게인 캘리브 (자코비안).
로봇 스테이지를 알려진 mm만큼 X/Y 이동 → 교차점 픽셀 이동량 측정 →
J = d(pixel)/d(XY), 역행렬 = 서보게인 d(XY)/d(pixel) 도출·저장.

절차:
  1) 'ref'  현재 교차점 검출(기준)
  2) UI에서 X만 +Nmm 이동 후 → 'x N' 입력 (예: x 20)
  3) UI에서 Y만 +Nmm 이동 후 → 'y N' 입력
     (X/Y 각각 독립 이동이면 됨. 순서·복귀 무관, 직전점 기준 상대측정)
  4) 'calc'  자코비안 + 역행렬 저장
저장: data/calibration/toolcam_gain.yaml
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/toolcam_crossing.pt'
# 아래 두 값은 main()에서 --pose / --target 인자로 재설정됨 (pose별 게인 캘리브)
TARGET = (694, 468)  # 우측자세 기본. 좌측은 --target 로 지정
OUT = '/home/koceti/ros2_ws/data/calibration/toolcam_gain_right.yaml'


class Gain(Node):
    def __init__(self):
        super().__init__('toolcam_gain')
        self.br = CvBridge(); self.buf = []
        self.create_subscription(Image, '/zedxone/zed_node/rgb/rect/image',
                                 lambda m: self.buf.append(
                                     self.br.imgmsg_to_cv2(m, 'bgr8')),
                                 qos_profile_sensor_data)
        self.model = YOLO(MODEL)
        self.prev = None        # 직전 검출 픽셀
        self.dPdX = None; self.dPdY = None
        self.dPdX_samples = []  # 다회 측정 누적 (평균 → 노이즈↓)
        self.dPdY_samples = []

    def detect(self, near=None, n=8):
        """n프레임 검출 중앙값. near 주어지면 그 점에 가장 가까운 교차점."""
        pts = []
        for _ in range(n):
            self.buf.clear(); t = time.time()
            while not self.buf and time.time() - t < 3:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not self.buf:
                continue
            r = self.model(self.buf[-1], conf=0.15, verbose=False)[0]
            cs = [((float(b.xyxy[0][0]+b.xyxy[0][2])/2),
                   float(b.xyxy[0][1]+b.xyxy[0][3])/2,
                   float(b.conf[0])) for b in r.boxes]
            if not cs:
                continue
            ref = near if near is not None else TARGET
            cs.sort(key=lambda c: np.hypot(c[0]-ref[0], c[1]-ref[1]))
            pts.append(cs[0][:2])
        if len(pts) < 2:
            return None
        arr = np.array(pts)
        # 중앙값에서 멀리 튄 프레임 제거 후 평균
        med = np.median(arr, axis=0)
        keep = arr[np.linalg.norm(arr - med, axis=1) < 40]
        return keep.mean(axis=0) if len(keep) else med

    def detect_once(self):
        """단일프레임 빠른 검출 (오버레이용). 반환 (목표근처점, 전체점들)."""
        if not self.buf:
            return None, []
        r = self.model(self.buf[-1], conf=0.15, verbose=False)[0]
        cs = [(float(b.xyxy[0][0]+b.xyxy[0][2])/2,
               float(b.xyxy[0][1]+b.xyxy[0][3])/2) for b in r.boxes]
        if not cs:
            return None, []
        ref = self.prev if self.prev is not None else TARGET
        near = min(cs, key=lambda c: np.hypot(c[0]-ref[0], c[1]-ref[1]))
        return near, cs

    def ref(self):
        p = self.detect()
        if p is None:
            print('  검출 실패'); return
        self.prev = p
        print(f'  기준 교차점 = ({p[0]:.0f},{p[1]:.0f})')

    def move(self, axis, mm):
        if self.prev is None:
            print('  먼저 ref'); return
        p = self.detect(near=self.prev)
        if p is None:
            print('  검출 실패'); return
        dP = (p - self.prev) / mm     # 픽셀이동/ mm
        if axis == 'X':
            self.dPdX_samples.append(dP)
            self.dPdX = np.mean(self.dPdX_samples, axis=0)
            n = len(self.dPdX_samples)
        else:
            self.dPdY_samples.append(dP)
            self.dPdY = np.mean(self.dPdY_samples, axis=0)
            n = len(self.dPdY_samples)
        cur = self.dPdX if axis == 'X' else self.dPdY
        print(f'  {axis}{mm:+.0f}mm: ({self.prev[0]:.0f},{self.prev[1]:.0f})→'
              f'({p[0]:.0f},{p[1]:.0f})  이번=({dP[0]:.2f},{dP[1]:.2f})  '
              f'평균({n}회)=({cur[0]:.2f},{cur[1]:.2f})px/mm')
        self.prev = p

    def calc(self):
        if self.dPdX is None or self.dPdY is None:
            print('  X,Y 둘 다 측정 필요'); return
        J = np.array([[self.dPdX[0], self.dPdY[0]],
                      [self.dPdX[1], self.dPdY[1]]])   # d(px,py)/d(X,Y)
        det = np.linalg.det(J)
        if abs(det) < 1e-6:
            print('  자코비안 특이(이동이 평행?) — 재측정'); return
        Jinv = np.linalg.inv(J)        # d(X,Y)/d(px,py) = 서보게인
        print(f'\n  자코비안 J (px/mm):\n   {J[0]}\n   {J[1]}')
        print(f'  서보게인 Jinv (mm/px):\n   {Jinv[0]}\n   {Jinv[1]}')
        print(f'  스케일: {np.hypot(J[0,0],J[1,0]):.2f}px/mm(X), '
              f'{np.hypot(J[0,1],J[1,1]):.2f}px/mm(Y)')
        import yaml
        yaml.safe_dump({'toolcam_gain': {
            'J_px_per_mm': J.tolist(),
            'Jinv_mm_per_px': Jinv.tolist(),
            'target_pixel': list(TARGET),
            'note': 'dXY(mm) = Jinv @ (target_px - crossing_px)',
        }}, open(OUT, 'w'))
        print(f'  → 저장: {OUT}')


def main():
    import argparse, cv2
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', type=float, default=20.0, help='이동 단위(mm)')
    ap.add_argument('--pose', choices=['right', 'left'], default='right',
                    help='자세 (right/left) → toolcam_gain_{pose}.yaml 로 저장')
    ap.add_argument('--target', type=int, nargs=2, metavar=('U', 'V'),
                    default=None, help='결속 목표픽셀 (미지정시 pose 기본값)')
    args = ap.parse_args()
    global TARGET, OUT
    OUT = f'/home/koceti/ros2_ws/data/calibration/toolcam_gain_{args.pose}.yaml'
    if args.target is not None:
        TARGET = tuple(args.target)
    print(f'  자세={args.pose}, 목표픽셀={TARGET}, 저장→{OUT}')
    rclpy.init(); node = Gain()
    step = args.step
    print('=' * 60)
    print(' 툴캠 게인 캘리브 — 창에서 보며 조정')
    print("  [r]기준  [x]X+step이동후  [y]Y+step이동후  [c]계산")
    print("  [+/-]step조정  [q]종료.  교차점을 중앙(시안)에 가깝게 두고 시작")
    print('=' * 60)
    win = 'toolcam gain (r=ref x=X y=Y c=calc q=quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 960, 600)
    sc = 0.5; last_infer = 0.0; near = None; allc = []
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.buf:
                img = node.buf[-1]
                if time.time() - last_infer > 0.3:
                    near, allc = node.detect_once(); last_infer = time.time()
                disp = cv2.resize(img, None, fx=sc, fy=sc)
                H, W = disp.shape[:2]
                cv2.drawMarker(disp, (W//2, H//2), (255, 255, 0),
                               cv2.MARKER_CROSS, 26, 1)      # 중앙(시안)
                cv2.drawMarker(disp, (int(TARGET[0]*sc), int(TARGET[1]*sc)),
                               (255, 0, 255), cv2.MARKER_TILTED_CROSS, 26, 2)
                for c in allc:
                    cv2.circle(disp, (int(c[0]*sc), int(c[1]*sc)), 10, (0, 200, 0), 2)
                if near:
                    cv2.circle(disp, (int(near[0]*sc), int(near[1]*sc)), 14,
                               (0, 255, 0), 3)
                    cv2.putText(disp, f'({near[0]:.0f},{near[1]:.0f})',
                                (int(near[0]*sc)+16, int(near[1]*sc)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                st = (f"step:{step:.0f}mm  ref:{'O' if node.prev is not None else 'X'}"
                      f"  dPdX:{len(node.dPdX_samples)}회"
                      f"  dPdY:{len(node.dPdY_samples)}회")
                cv2.putText(disp, st, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)
                cv2.putText(disp, "r=ref  x=X+  y=Y+  c=calc  +/-=step  q",
                            (10, H-12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.imshow(win, disp)
            k = cv2.waitKey(20) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('r'):
                node.ref()
            elif k == ord('x'):
                node.move('X', step)
            elif k == ord('y'):
                node.move('Y', step)
            elif k == ord('c'):
                node.calc()
            elif k == ord('+') or k == ord('='):
                step += 5; print(f'  step={step:.0f}mm')
            elif k == ord('-'):
                step = max(5, step-5); print(f'  step={step:.0f}mm')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
