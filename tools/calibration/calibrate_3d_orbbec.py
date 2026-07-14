#!/usr/bin/env python3
"""Orbbec 교차점 3D 캘리브 (depth 3D 복원 + 강체변환).
YOLO로 교차점(u,v) 검출 → 컬러정렬 depth robust 샘플링 → 컬러 intrinsic 역투영
→ 카메라 3D. 공구 tip으로 그 교차점 touch 후 실측 (x,y,z) 수동입력 → 쌍 저장.
calc: P_robot = R·P_cam + t (Umeyama/rigid) 피팅 + LOO 잔차.

사전: ros2 launch orbbec_camera gemini2L.launch.py depth_registration:=true
      (컬러정렬 depth 필수)

  python3 calibrate_3d_orbbec.py
  [Enter]=검출+실측입력  calc=적합  show=목록  del N  q=종료
저장: data/calibration/calib3d_data_orbbec.json / calib3d_result_orbbec.yaml
"""
import os, sys, json
from collections import deque
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rigid_transform_3d import RigidTransform3D

DATA_DIR = '/home/koceti/ros2_ws/data/calibration'
MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt'


class Calib(Node):
    def __init__(self, win, depth_frames):
        super().__init__('calibrate_3d_orbbec')
        self.win = win
        self.br = CvBridge()
        self.K = None
        self.color = None
        self.depth_buf = deque(maxlen=depth_frames)
        self.model = YOLO(MODEL)
        # 컬러 intrinsic (검출이 컬러 프레임 → 컬러정렬 depth 가정)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._color_cb, qos_profile_sensor_data)
        self.data_file = os.path.join(DATA_DIR, 'calib3d_data_orbbec.json')
        self.result_file = os.path.join(DATA_DIR, 'calib3d_result_orbbec.yaml')
        self.pairs = self._load()
        self.get_logger().info(f'기존 데이터: {len(self.pairs)}쌍')

    def _info_cb(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = dict(fx=m.k[0], fy=m.k[4], cx=m.k[2], cy=m.k[5])

    def _depth_cb(self, m):
        d = self.br.imgmsg_to_cv2(m, 'passthrough')
        if d.dtype == np.uint16:        # Orbbec: mm → m
            d = d.astype(np.float32) / 1000.0
        else:
            d = d.astype(np.float32)
        self.depth_buf.append(d)

    def _color_cb(self, m):
        self.color = self.br.imgmsg_to_cv2(m, 'bgr8')

    def _load(self):
        if os.path.exists(self.data_file):
            return json.load(open(self.data_file))
        return []

    def _save_data(self):
        json.dump(self.pairs, open(self.data_file, 'w'), indent=1)

    def spin_some(self, sec=0.6):
        import time
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _pool(self, u, v, r_out):
        """버퍼 전체 (u,v) 주변 r_out 유효 depth(mm) 수집."""
        vals = []
        for d in list(self.depth_buf):
            h, w = d.shape[:2]
            y0, y1 = max(0, v-r_out), min(h, v+r_out+1)
            x0, x1 = max(0, u-r_out), min(w, u+r_out+1)
            sub = d[y0:y1, x0:x1]
            m = np.isfinite(sub) & (sub > 0.05) & (sub < 3.0)
            vals.extend((sub[m] * 1000.0).tolist())
        return np.array(vals)

    def rebar_depth_mm(self, u, v):
        """교차점=최근접 표면(철근면). 윈도우 유효 depth 중 최근접 밴드 평균."""
        near = self._pool(u, v, self.win // 2)
        if near.size < 10:
            return None, 0.0
        lo = np.percentile(near, 5)
        band = near[near < lo + 25.0]
        if band.size < 3:
            band = near
        return float(band.mean()), float(band.std())

    def reconstruct(self, u, v):
        Z, noise = self.rebar_depth_mm(u, v)
        if Z is None or self.K is None:
            return None, noise
        X = (u - self.K['cx']) * Z / self.K['fx']
        Y = (v - self.K['cy']) * Z / self.K['fy']
        return [X, Y, Z], noise

    def detect(self, conf=0.3):
        if self.color is None:
            return []
        r = self.model(self.color, conf=conf, verbose=False)[0]
        cs = [((float(b.xyxy[0][0]+b.xyxy[0][2])/2),
               (float(b.xyxy[0][1]+b.xyxy[0][3])/2)) for b in r.boxes]
        cs.sort(key=lambda c: (round(c[1]/50), c[0]))   # 행→열 순
        return [(int(u), int(v)) for u, v in cs]

    def save_viz(self, pts, pcams):
        img = self.color.copy()
        for i, ((u, v), pc) in enumerate(zip(pts, pcams)):
            ok = pc is not None
            col = (0, 200, 0) if ok else (0, 0, 255)
            cv2.circle(img, (u, v), 16, col, 3)
            cv2.putText(img, f'P{i+1}', (u+16, v), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, col, 2)
            if ok:
                cv2.putText(img, f'{pc[2]:.0f}mm', (u+16, v+22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
        p = os.path.join(DATA_DIR, 'calib3d_detect_orbbec.png')
        cv2.imwrite(p, img)
        print(f'  검출 이미지 → {p}')

    def _fit_pose(self, sub):
        """한 자세 부분집합 적합 → (R,t,rms,loo,Pt). 4쌍 미만이면 None."""
        if len(sub) < 4:
            return None
        Pc = np.array([p['pcam'] for p in sub], float)
        Pt = np.array([p['ptool'] for p in sub], float)
        R, t, rms = RigidTransform3D.fit(Pc, Pt)
        loo = []
        for i in range(len(Pc)):
            m = np.ones(len(Pc), bool); m[i] = False
            Ri, ti, _ = RigidTransform3D.fit(Pc[m], Pt[m])
            p = Ri @ Pc[i] + ti
            loo.append(np.linalg.norm(p[:2] - Pt[i][:2]))
        return R, t, rms, np.array(loo), Pt

    def calc(self):
        """자세(r/l)별로 별도 변환 피팅. 물리위치는 공통이지만 touch 스테이지값이
        자세마다 달라서 (yaw 공구오프셋) 자세별 R,t 필요."""
        import yaml
        out = {}
        for pose, key in (('r', 'right'), ('l', 'left')):
            sub = [p for p in self.pairs if p.get('pose') == pose]
            fit = self._fit_pose(sub)
            if fit is None:
                print(f'  [{key}] {len(sub)}쌍 (최소 4 필요) → skip'); continue
            R, t, rms, loo, Pt = fit
            print(f'\n  === [{key}] {len(sub)}쌍 ===')
            print(f'  RMS(3D) {rms:.1f}mm  LOO(실사용) XY 평균 {loo.mean():.1f} '
                  f'최대 {loo.max():.1f}mm')
            for i in range(len(Pt)):
                print(f'    ({Pt[i,0]:5.0f},{Pt[i,1]:5.0f}) LOO {loo[i]:4.1f}mm')
            out[key] = {'R': R.tolist(), 't': t.tolist(),
                        'rms_mm': float(rms), 'loo_xy_mm': float(loo.mean()),
                        'num_pairs': len(sub)}
        if not out:
            print('  적합된 자세 없음 (자세별 4쌍 이상 필요)'); return
        out['note'] = 'P_robot = R @ P_cam + t (mm). Orbbec color-aligned depth. 자세별.'
        yaml.safe_dump(out, open(self.result_file, 'w'))
        print(f'  → 저장 {self.result_file} (자세: {[k for k in out if k!="note"]})')

    def delete(self, n):
        if 1 <= n <= len(self.pairs):
            self.pairs.pop(n-1); self._save_data(); print(f'  P{n} 삭제')


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--win', type=int, default=9, help='depth 샘플 윈도우(px)')
    ap.add_argument('--depth-frames', type=int, default=30)
    ap.add_argument('--conf', type=float, default=0.3)
    args = ap.parse_args()
    rclpy.init()
    node = Calib(args.win, args.depth_frames)
    print('='*60)
    print(' Orbbec 3D 캘리브 — Enter=검출+실측  calc=적합  show  del N  q')
    print(' (depth_registration:=true 로 실행했는지 확인)')
    print('='*60)
    print('워밍업...'); node.spin_some(2.0)
    if node.K is None:
        print('⚠️ color camera_info 못받음 — 토픽 확인')
    try:
        while True:
            cmd = input('\n[Enter=검출 / calc / show / del N / q]: ').strip()
            if cmd == 'q':
                break
            if cmd == 'calc':
                node.calc(); continue
            if cmd == 'show':
                for i, p in enumerate(node.pairs):
                    print(f'  P{i+1} [{p.get("pose","?")}] px{p["pixel"]} '
                          f'cam{[round(x) for x in p["pcam"]]} robot{p["ptool"]}')
                continue
            if cmd.startswith('del '):
                try: node.delete(int(cmd.split()[1]))
                except (ValueError, IndexError): print('형식: del 번호')
                continue
            # 검출
            node.depth_buf.clear(); node.spin_some(1.5)
            pts = node.detect(args.conf)
            if not pts:
                print('  검출 0개'); continue
            pcams = [node.reconstruct(u, v)[0] for (u, v) in pts]
            node.save_viz(pts, pcams)
            print(f'  {len(pts)}개 검출. 이미지에서 P# 확인 → 그 교차점에 공구 대고 실측 입력')
            for i, ((u, v), pc) in enumerate(zip(pts, pcams)):
                if pc is None:
                    print(f'  P{i+1} px=({u},{v}): depth 소실 → skip'); continue
                print(f'  P{i+1} px=({u},{v}) P_cam=({pc[0]:.0f},{pc[1]:.0f},{pc[2]:.0f})mm')
                raw = input(f'     공구 touch 실측 "X Y Z r/l"(mm+자세) [Enter=skip]: ').strip()
                if not raw:
                    continue
                try:
                    parts = raw.replace(',', ' ').split()
                    ptool = [float(parts[0]), float(parts[1]), float(parts[2])]
                    pose = parts[3].lower()[0] if len(parts) > 3 else ''
                except (ValueError, IndexError):
                    print('     형식오류: X Y Z r/l'); continue
                if pose not in ('r', 'l'):
                    print('     자세(r/l) 필요 — 4번째에 r 또는 l'); continue
                node.pairs.append({'pixel': [u, v], 'pcam': pc,
                                   'ptool': ptool, 'pose': pose})
                node._save_data()
                nr = sum(p.get('pose') == 'r' for p in node.pairs)
                nl = sum(p.get('pose') == 'l' for p in node.pairs)
                print(f'     ✅ 저장 (총 {len(node.pairs)}쌍: 우{nr}/좌{nl})')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
