#!/usr/bin/env python3
"""Femto Bolt 3D 캘리브 + depth 품질 테스트.

calibrate_3d_orbbec.py(Gemini2L용) 기반. Femto Bolt는 진짜 ToF라 depth가
액티브스테레오(Gemini2L)보다 나을 수 있어 depth 품질을 정량 진단하는 기능 추가.

YOLO로 교차점(u,v) 검출 → 컬러정렬 depth robust 샘플링 → 컬러 intrinsic 역투영
→ 카메라 3D. 공구 tip으로 그 교차점 touch 후 실측 (x,y,z) 입력 → 쌍 저장.
calc: P_robot = R·P_cam + t (Umeyama/rigid) 피팅 + LOO 잔차.

사전: 서비스가 femto_bolt.launch.py depth_registration:=true 로 카메라 실행중
      (또는 수동: ros2 launch orbbec_camera femto_bolt.launch.py depth_registration:=true)

명령:
  Enter  검출 + 각 교차점 depth 진단 표시 + 실측입력(선택)
  depth  터치 없이 depth 품질만 진단 (반복측정 통계) ← Femto depth 테스트용
  calc   자세별 rigid 변환 적합 + LOO
  show   수집목록 / del N 삭제 / q 종료

데이터: data/calibration/calib3d_data_femto.json / calib3d_result_femto.yaml
        (Gemini2L용 calib3d_data_orbbec.json 과 분리)
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
        super().__init__('calibrate_3d_femto')
        self.win = win
        self.br = CvBridge()
        self.K = None
        self.color = None
        self.color_wh = None
        self.depth_buf = deque(maxlen=depth_frames)
        self.model = YOLO(MODEL)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._color_cb, qos_profile_sensor_data)
        self.data_file = os.path.join(DATA_DIR, 'calib3d_data_femto.json')
        self.result_file = os.path.join(DATA_DIR, 'calib3d_result_femto.yaml')
        self.pairs = self._load()
        self.get_logger().info(f'기존 데이터: {len(self.pairs)}쌍 (Femto)')

    def _info_cb(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = dict(fx=m.k[0], fy=m.k[4], cx=m.k[2], cy=m.k[5])

    def _depth_cb(self, m):
        d = self.br.imgmsg_to_cv2(m, 'passthrough')
        if d.dtype == np.uint16:        # Orbbec: mm 단위 → m
            d = d.astype(np.float32) / 1000.0
        else:
            d = d.astype(np.float32)
        self.depth_buf.append(d)

    def _color_cb(self, m):
        self.color = self.br.imgmsg_to_cv2(m, 'bgr8')
        self.color_wh = (self.color.shape[1], self.color.shape[0])

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

    def _to_depth_px(self, u, v, dshape):
        """color(u,v) → depth 배열 인덱스. depth 해상도가 color와 다르면 스케일."""
        dh, dw = dshape[:2]
        cw, ch = self.color_wh if self.color_wh else (dw, dh)
        if (dw, dh) == (cw, ch):
            return u, v
        return int(u * dw / cw), int(v * dh / ch)

    def _pool(self, u, v, r_out):
        """버퍼 전체에서 (u,v) 주변 r_out 유효 depth(mm) 수집."""
        vals = []
        for d in list(self.depth_buf):
            du, dv = self._to_depth_px(u, v, d.shape)
            h, w = d.shape[:2]
            y0, y1 = max(0, dv-r_out), min(h, dv+r_out+1)
            x0, x1 = max(0, du-r_out), min(w, du+r_out+1)
            sub = d[y0:y1, x0:x1]
            m = np.isfinite(sub) & (sub > 0.05) & (sub < 5.0)
            vals.extend((sub[m] * 1000.0).tolist())
        return np.array(vals)

    def depth_quality(self, u, v):
        """(u,v) depth 품질 진단. 반환 dict(mean,std,valid_ratio,n,band_std)."""
        r = self.win // 2
        allpix = 0
        for d in list(self.depth_buf):
            du, dv = self._to_depth_px(u, v, d.shape)
            h, w = d.shape[:2]
            allpix += (min(h, dv+r+1)-max(0, dv-r)) * (min(w, du+r+1)-max(0, du-r))
        near = self._pool(u, v, r)
        if near.size < 5:
            return dict(mean=None, std=0, valid=0.0, n=int(near.size), band_std=0)
        lo = np.percentile(near, 5)
        band = near[near < lo + 25.0]
        if band.size < 3:
            band = near
        return dict(mean=float(band.mean()), std=float(near.std()),
                    valid=float(near.size / max(1, allpix)),
                    n=int(near.size), band_std=float(band.std()))

    def rebar_depth_mm(self, u, v):
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
        p = os.path.join(DATA_DIR, 'calib3d_detect_femto.png')
        cv2.imwrite(p, img)
        print(f'  검출 이미지 → {p}')

    def _fit_pose(self, sub):
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
        out['note'] = 'P_robot = R @ P_cam + t (mm). Femto Bolt ToF, color-aligned depth. 자세별.'
        yaml.safe_dump(out, open(self.result_file, 'w'))
        print(f'  → 저장 {self.result_file} (자세: {[k for k in out if k!="note"]})')

    def depth_test(self, conf=0.3):
        """터치 없이 depth 품질만 진단 — Femto ToF depth 테스트용.
        검출된 각 교차점에서 depth 평균/노이즈/유효율을 반복 측정해 표로 출력."""
        self.depth_buf.clear(); self.spin_some(2.0)   # 여러 프레임 축적
        pts = self.detect(conf)
        if not pts:
            print('  검출 0개'); return
        print(f'\n  === Femto depth 품질 진단 ({len(pts)}개 교차점, '
              f'{len(self.depth_buf)}프레임 누적) ===')
        print(f'  {"P#":>3} {"픽셀(u,v)":>12} {"Z(mm)":>8} {"노이즈std":>9} '
              f'{"밴드std":>8} {"유효율":>7} {"표본":>5}')
        zs, stds, valids = [], [], []
        for i, (u, v) in enumerate(pts):
            q = self.depth_quality(u, v)
            if q['mean'] is None:
                print(f'  P{i+1:<2d} ({u:4d},{v:4d})   depth 소실 (표본 {q["n"]})')
                continue
            print(f'  P{i+1:<2d} ({u:4d},{v:4d}) {q["mean"]:8.1f} {q["std"]:9.1f} '
                  f'{q["band_std"]:8.1f} {q["valid"]*100:6.0f}% {q["n"]:5d}')
            zs.append(q['mean']); stds.append(q['std']); valids.append(q['valid'])
        if zs:
            print(f'\n  요약: 검출 {len(pts)}개 중 depth유효 {len(zs)}개')
            print(f'    Z 범위 {min(zs):.0f}~{max(zs):.0f}mm '
                  f'(틸트로 v에 따라 변하는게 정상)')
            print(f'    노이즈(std) 중앙값 {np.median(stds):.1f}mm '
                  f'← Gemini2L 액티브스테레오 대비 이게 작으면 Femto ToF가 유리')
            print(f'    유효율 중앙값 {np.median(valids)*100:.0f}% '
                  f'← 금속반사로 구멍 나면 낮아짐')
        # 시각화 저장
        img = self.color.copy()
        for i, (u, v) in enumerate(pts):
            q = self.depth_quality(u, v)
            ok = q['mean'] is not None
            col = (0, 200, 0) if ok else (0, 0, 255)
            cv2.circle(img, (u, v), 14, col, 2)
            txt = f'{q["mean"]:.0f}' if ok else 'X'
            cv2.putText(img, txt, (u+14, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        p = os.path.join(DATA_DIR, 'femto_depth_test.png')
        cv2.imwrite(p, img)
        print(f'  → depth 시각화 저장: {p}')

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
    print('='*62)
    print(' Femto Bolt 3D 캘리브 + depth 테스트')
    print(' Enter=검출+실측  depth=depth품질진단  calc=적합  show  del N  q')
    print('='*62)
    print('워밍업...'); node.spin_some(2.0)
    if node.K is None:
        print('⚠️ color camera_info 못받음 — 토픽/서비스 확인')
    if not node.depth_buf:
        print('⚠️ depth 프레임 못받음 — depth_registration:=true 로 실행됐는지 확인')
    try:
        while True:
            cmd = input('\n[Enter=검출 / depth / calc / show / del N / q]: ').strip()
            if cmd == 'q':
                break
            if cmd == 'depth':
                node.depth_test(args.conf); continue
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
            # 검출 + 실측입력
            node.depth_buf.clear(); node.spin_some(1.5)
            pts = node.detect(args.conf)
            if not pts:
                print('  검출 0개'); continue
            pcams = [node.reconstruct(u, v)[0] for (u, v) in pts]
            node.save_viz(pts, pcams)
            print(f'  {len(pts)}개 검출. 이미지에서 P# 확인 → 그 교차점에 공구 대고 실측 입력')
            for i, ((u, v), pc) in enumerate(zip(pts, pcams)):
                q = node.depth_quality(u, v)
                dtxt = (f'Z={pc[2]:.0f}mm std={q["std"]:.1f} 유효{q["valid"]*100:.0f}%'
                        if pc is not None else 'depth 소실')
                print(f'  P{i+1} px=({u},{v}) {dtxt}')
                if pc is None:
                    print(f'     → depth 없어도 픽셀+실측만 있으면 호모그래피엔 사용 가능')
                raw = input(f'     공구 touch 실측 "X Y Z r/l" [Enter=skip]: ').strip()
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
                # depth 없으면 pcam은 null 저장(호모그래피는 pixel만 씀)
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
