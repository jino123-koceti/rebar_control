#!/usr/bin/env python3
"""Orbbec 교차점 depth-3D 로컬라이제이션 (XYZ 전부 depth) — 층분리 버전.

호모그래피(단일평면 가정) 대신 점별 실제 depth로 3D 복원 → 강체변환으로 로봇 XYZ.
높이편차(요철/다층)에 강함. 호모그래피 XY와 나란히 출력해 차이를 본다.

파이프라인:
  [color] YOLO 교차점(u,v)
  [depth] ─ 각 교차점 rough depth → 교차점들로 "철근층 평면" RANSAC 적합
                                     (프레임 오염점은 아웃라이어로 자동 배제)
          ─ 각 교차점: 윈도우 depth 중 "철근층 평면 ±tol" 점만 채택 (바닥·프레임 배제)
                        · 채택점 있음 → top(실제 교차점 높이) 사용
                        · 홀 → 카메라레이 ∩ 층평면 으로 Z 복원
          ─ P_cam = 역투영 → P_robot = R·P_cam + t (자세별)

사전: ros2 launch ... depth_registration:=true (컬러정렬 depth)
      calib3d_result_orbbec.yaml R,t 는 정합 켠 상태 재캘리브본이어야 절대정확.
      (재캘리브: python3 tools/calibration/calibrate_3d_orbbec.py)

  python3 tools/orbbec/depth_localize_3d.py            # GUI (SPACE=출력 s=스냅 q=종료)
  python3 tools/orbbec/depth_localize_3d.py --no-gui   # 헤드리스 1회
     --win 9  --depth-frames 30  --conf 0.3  --layer-tol 30
"""
import os
import argparse
from collections import deque
import numpy as np
import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt'
CALIB3D = '/home/koceti/ros2_ws/data/calibration/calib3d_result_orbbec.yaml'
HOMO = '/home/koceti/ros2_ws/data/calibration/homography_orbbec.yaml'
YRANGE = {'r': (0.0, 142.0), 'l': (124.0, 288.0)}
XMAX = 411.0


def ransac_plane(pts, iters=400, thr=15.0, seed=0):
    """pts Nx3(mm) → (normal, D, inlier_mask). 평면 n·p + D = 0. 아웃라이어 배제."""
    rng = np.random.default_rng(seed)
    n = len(pts)
    best_inl, best_cnt = None, -1
    for _ in range(iters):
        i = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[i]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-6:
            continue
        nrm = nrm / ln
        D = -nrm.dot(p0)
        inl = np.abs(pts @ nrm + D) < thr
        c = int(inl.sum())
        if c > best_cnt:
            best_cnt, best_inl = c, inl
    ip = pts[best_inl]
    c = ip.mean(0)
    _, _, vt = np.linalg.svd(ip - c)
    nrm = vt[2]
    if nrm[2] < 0:
        nrm = -nrm
    D = -nrm.dot(c)
    return nrm, D, best_inl


class DepthLocalizer(Node):
    def __init__(self, win, depth_frames, conf, layer_tol):
        super().__init__('depth_localize_3d')
        self.win = win
        self.conf = conf
        self.layer_tol = layer_tol
        self.br = CvBridge()
        self.K = None
        self.color = None
        self.depth_buf = deque(maxlen=depth_frames)
        self.model = YOLO(MODEL)
        self.layer = None            # (normal, D) 철근층 평면 (camera frame)

        self.RT = {}
        c3 = yaml.safe_load(open(CALIB3D)) if os.path.exists(CALIB3D) else {}
        for pose, key in (('r', 'right'), ('l', 'left')):
            if key in c3 and 'R' in c3[key]:
                self.RT[pose] = (np.array(c3[key]['R'], float), np.array(c3[key]['t'], float))
        self.H = {}
        hom = yaml.safe_load(open(HOMO)) if os.path.exists(HOMO) else {}
        for pose, key in (('r', 'right'), ('l', 'left')):
            if key in hom and 'H' in hom[key]:
                self.H[pose] = np.array(hom[key]['H'], float)

        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._color_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'depth-3D 자세 {list(self.RT.keys())} | 호모그래피 {list(self.H.keys())}')
        if not self.RT:
            self.get_logger().warn('  calib3d_result_orbbec.yaml R,t 없음 → 재캘리브 필요')

    def _info_cb(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = dict(fx=m.k[0], fy=m.k[4], cx=m.k[2], cy=m.k[5])

    def _depth_cb(self, m):
        d = self.br.imgmsg_to_cv2(m, 'passthrough')
        self.depth_buf.append(d.astype(np.float32))     # mm

    def _color_cb(self, m):
        self.color = self.br.imgmsg_to_cv2(m, 'bgr8')

    def _bp(self, u, v, z):
        return np.array([(u - self.K['cx']) * z / self.K['fx'],
                         (v - self.K['cy']) * z / self.K['fy'], z])

    # ── (A) rough: 근접밴드 top (층평면 적합 부트스트랩용) ──
    def _rough_pcam(self, u, v):
        vals = []
        r = self.win // 2
        for d in list(self.depth_buf):
            h, w = d.shape[:2]
            sub = d[max(0, v-r):min(h, v+r+1), max(0, u-r):min(w, u+r+1)]
            m = np.isfinite(sub) & (sub > 50.0) & (sub < 3000.0)
            vals.extend(sub[m].tolist())
        if len(vals) < 10:
            return None
        near = np.array(vals)
        lo = np.percentile(near, 5)
        band = near[near < lo + 25.0]
        Z = float(band.mean() if band.size >= 3 else near.mean())
        return self._bp(u, v, Z)

    def fit_layer(self, crossings):
        """교차점 rough 3D 로 철근층 평면 RANSAC 적합. 프레임 오염점은 아웃라이어 배제."""
        self.layer = None
        rough = [self._rough_pcam(u, v) for (u, v) in crossings]
        pts = np.array([p for p in rough if p is not None])
        if len(pts) >= 4:
            nrm, D, inl = ransac_plane(pts, thr=self.layer_tol)
            self.layer = (nrm, D)
            return int(inl.sum()), len(pts)
        return 0, len(pts)

    # ── (B) 층분리 샘플: 층평면 ±tol 점만 채택(바닥·프레임 배제) → top / 홀은 평면복원 ──
    def layer_pcam(self, u, v):
        """반환 (P_cam, src, height_mm). src='top'|'plane'|'rough'|None."""
        if self.layer is None:
            p = self._rough_pcam(u, v)
            return (p, 'rough', 0.0) if p is not None else (None, None, 0.0)
        n, D = self.layer
        r = self.win // 2
        zs = []
        for d in list(self.depth_buf):
            h, w = d.shape[:2]
            y0, y1 = max(0, v-r), min(h, v+r+1)
            x0, x1 = max(0, u-r), min(w, u+r+1)
            sub = d[y0:y1, x0:x1]
            ys, xs = np.where(np.isfinite(sub) & (sub > 50.0) & (sub < 3000.0))
            for yy, xx in zip(ys, xs):
                z = float(sub[yy, xx])
                P = self._bp(x0 + xx, y0 + yy, z)
                if abs(P @ n + D) < self.layer_tol:       # 철근층 ±tol 만
                    zs.append(z)
        if len(zs) >= 3:
            zs = np.array(zs)
            lo = np.percentile(zs, 5)                      # top 우선(가장 가까운 밴드)
            band = zs[zs < lo + 25.0]
            Z = float(band.mean() if band.size >= 3 else zs.mean())
            P = self._bp(u, v, Z)
            return P, 'top', float(-(P @ n + D))           # 평면 위로 솟은 양
        # 홀 → 카메라레이 ∩ 층평면
        dirv = np.array([(u - self.K['cx']) / self.K['fx'],
                         (v - self.K['cy']) / self.K['fy'], 1.0])
        denom = n @ dirv
        if abs(denom) < 1e-6:
            return None, None, 0.0
        s = -D / denom
        return s * dirv, 'plane', 0.0

    def detect(self):
        r = self.model(self.color, conf=self.conf, verbose=False)[0]
        cs = [((float(b.xyxy[0][0]+b.xyxy[0][2])/2),
               (float(b.xyxy[0][1]+b.xyxy[0][3])/2)) for b in r.boxes]
        cs.sort(key=lambda c: (round(c[1]/50), c[0]))
        return [(int(u), int(v)) for u, v in cs]

    def homography_xy(self, pose, u, v):
        if pose not in self.H:
            return None
        p = self.H[pose] @ np.array([u, v, 1.0])
        return p[0]/p[2], p[1]/p[2]

    def localize(self):
        cros = self.detect()
        inl, npair = self.fit_layer(cros)          # 철근층 평면 적합 (프레임 배제)
        out = []
        for (u, v) in cros:
            pcam, src, hgt = self.layer_pcam(u, v)
            rec = dict(u=u, v=v, pcam=pcam, src=src, hgt=hgt,
                       pose=None, xyz=None, homo=None, dxy=None)
            if pcam is not None:
                for pose in ('r', 'l'):
                    if pose not in self.RT:
                        continue
                    R, t = self.RT[pose]
                    P = R @ pcam + t
                    ymin, ymax = YRANGE[pose]
                    if ymin <= P[1] <= ymax and 0 <= P[0] <= XMAX:
                        rec['pose'] = pose; rec['xyz'] = P
                        hxy = self.homography_xy(pose, u, v)
                        rec['homo'] = hxy
                        if hxy is not None:
                            rec['dxy'] = float(np.hypot(P[0]-hxy[0], P[1]-hxy[1]))
                        break
            out.append(rec)
        return out, (inl, npair)


def print_report(recs, layerinfo):
    inl, npair = layerinfo
    print('\n' + '=' * 76)
    print('  depth-3D 로컬라이제이션 (XYZ 전부 depth, 층분리) vs 호모그래피')
    print('=' * 76)
    print(f'  철근층 평면: 교차점 {npair}개 중 인라이어 {inl}개로 적합 '
          f'(프레임/바닥 오염점 {npair-inl}개 배제)')
    print(f'  {"px(u,v)":>11} | {"src":>4} | {"top높이":>6} | {"depth-3D robot XYZ":>22} | '
          f'{"homo XY":>12} | Δxy')
    print('  ' + '-' * 72)
    dxys = []
    for r in recs:
        u, v = r['u'], r['v']
        if r['xyz'] is not None:
            X, Y, Z = r['xyz']
            xyz = f'({X:6.1f},{Y:6.1f},{Z:6.1f})'
            homo = f'({r["homo"][0]:.0f},{r["homo"][1]:.0f})' if r['homo'] else '   --   '
            dxy = f'{r["dxy"]:4.1f}mm' if r['dxy'] is not None else '  -- '
            hgt = f'{r["hgt"]:+4.1f}' if r['src'] == 'top' else '  - '
            if r['dxy'] is not None:
                dxys.append(r['dxy'])
            print(f'  ({u:4d},{v:4d}) | {r["src"]:>4} | {hgt:>6} | {xyz:>22} | {homo:>12} | {dxy}')
        else:
            print(f'  ({u:4d},{v:4d}) | {str(r["src"]):>4} |    -  | '
                  f'{"range밖/holE":>22} |         -- |   --')
    print('  ' + '-' * 72)
    if dxys:
        print(f'  depth-3D vs 호모그래피 XY 차이: 평균 {np.mean(dxys):.1f}mm  최대 {np.max(dxys):.1f}mm')
        print('  (차이 큰 지점 = 높이편차로 호모그래피가 틀어진 곳 → depth-3D가 정밀)')
    print('  src: top=실측 철근면  plane=홀→층평면복원  rough=층미적합 폴백')
    print('=' * 76)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--win', type=int, default=9)
    ap.add_argument('--depth-frames', type=int, default=30)
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--layer-tol', type=float, default=30.0, help='철근층 평면 ±tol(mm)')
    ap.add_argument('--no-gui', action='store_true')
    ap.add_argument('--save-dir', default='/home/koceti/ros2_ws/data/orbbec_depth_test')
    args = ap.parse_args()

    rclpy.init()
    node = DepthLocalizer(args.win, args.depth_frames, args.conf, args.layer_tol)
    gui = not args.no_gui
    winname = 'depth-3D localize (SPACE=print s=snap q=quit)'
    if gui:
        cv2.namedWindow(winname, cv2.WINDOW_NORMAL); cv2.resizeWindow(winname, 1280, 800)

    def draw(recs):
        img = node.color.copy()
        for r in recs:
            u, v = r['u'], r['v']
            if r['xyz'] is not None:
                col = (0, 165, 255) if r['pose'] == 'r' else (255, 100, 0)
                X, Y, Z = r['xyz']
                cv2.circle(img, (u, v), 14, col, 3)
                cv2.putText(img, f'{r["pose"]}({X:.0f},{Y:.0f},{Z:.0f})', (u+12, v-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                tag = f'{r["src"]}'
                if r['dxy'] is not None:
                    tag += f' d{r["dxy"]:.0f}'
                cv2.putText(img, tag, (u+12, v+14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            else:
                cv2.circle(img, (u, v), 12, (120, 120, 120), 2)
        cv2.putText(img, 'orange=r blue=l  src=top/plane  d=depth-vs-homo(mm)',
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return img

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.color is None or not node.depth_buf or node.K is None:
                continue
            if gui:
                recs, li = node.localize()
                cv2.imshow(winname, draw(recs))
                k = cv2.waitKey(1) & 0xFF
                if k in (ord('q'), 27):
                    break
                elif k in (ord(' '), ord('p')):
                    print_report(recs, li)
                elif k == ord('s'):
                    os.makedirs(args.save_dir, exist_ok=True)
                    p = os.path.join(args.save_dir, 'depth_localize_3d.png')
                    cv2.imwrite(p, draw(recs)); print(f'  저장 {p}')
            else:
                if len(node.depth_buf) >= args.depth_frames:
                    recs, li = node.localize()
                    print_report(recs, li)
                    os.makedirs(args.save_dir, exist_ok=True)
                    cv2.imwrite(os.path.join(args.save_dir, 'depth_localize_3d.png'), draw(recs))
                    break
    finally:
        if gui:
            cv2.destroyAllWindows()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
