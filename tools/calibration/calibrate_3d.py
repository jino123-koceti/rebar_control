#!/usr/bin/env python3
"""
[대안 1] depth 3D 복원 + 강체변환(Hand-Eye) 캘리브레이션 수집/평가.

기존 2D 픽셀 회귀(calibrate_wo_depth_user.py) 대체:
  교차점 픽셀(u,v) + ZED depth(시간+공간 중앙값) + intrinsic
    → 카메라 3D 점 P_cam=[Xc,Yc,Zc]
  공구 TCP 실측 3D P_tool=[Xt,Yt,Zt]
    → RigidTransform3D.fit 로 R,t 도출 (P_tool = R·P_cam + t)

워크플로우 (camera별 별도):
  Enter   : 교차점 검출 → 각 점 P_cam 복원 표시 → 공구를 그 점에 대고 실측 X Y Z 입력
  calc    : 강체변환 적합 + LOO 정확도 (이상치 표시)
  show    : 수집 데이터 목록
  del N   : N번 삭제
  q       : 저장 후 종료

사용법: python3 calibrate_3d.py --camera right   (right=zedxmini2 / left=zedxmini1)
저장:   calib3d_data_{cam}.json (쌍),  calib3d_result_{cam}.yaml (R,t)
"""
import argparse
import json
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rebar_base_interfaces.srv import DetectCrossings

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rigid_transform_3d import RigidTransform3D

DATA_DIR = '/home/koceti/ros2_ws/data/calibration'
CAM = {
    'right': {'sel': 2, 'ns': 'zedxmini2'},
    'left':  {'sel': 1, 'ns': 'zedxmini1'},
}


def loo_inplane_subset(Pc, Pt, keep):
    """keep 인덱스 부분집합으로 affine in-plane LOO 평균(mm)."""
    import cv2
    Pc, Pt = Pc[keep], Pt[keep]
    n = len(Pc)
    errs = []
    for i in range(n):
        m = np.ones(n, bool); m[i] = False
        _, M, _ = cv2.estimateAffine3D(
            Pc[m].astype(np.float32), Pt[m].astype(np.float32))
        pred = M[:, :3] @ Pc[i] + M[:, 3]
        errs.append(np.linalg.norm(pred[:2] - Pt[i][:2]))
    return float(np.mean(errs))


class Calib3D(Node):
    def __init__(self, camera, depth_frames, win, detect_runs=3, min_above=40.0):
        super().__init__('calibrate_3d')
        self.camera = camera
        self.cfg = CAM[camera]
        ns = self.cfg['ns']
        self.win = win
        self.detect_runs = detect_runs
        self.min_above = min_above   # 철근이 주변 철판보다 최소 이만큼(mm) 위
        self.reject_mm = 12.0        # 예상오차 이 이상이면 결속 제외
        self.affine_margin = 3.0     # affine은 강체보다 이만큼(mm)+ 나을 때만 선택
        self.br = CvBridge()
        self.K = None
        self.depth_buf = deque(maxlen=depth_frames)

        self.create_subscription(
            CameraInfo, f'/{ns}/zed_node/depth/camera_info',
            self._info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, f'/{ns}/zed_node/depth/depth_registered',
            self._depth_cb, qos_profile_sensor_data)
        self.rgb = None
        self.create_subscription(
            Image, f'/{ns}/zed_node/left/image_rect_color',
            self._rgb_cb, qos_profile_sensor_data)
        self.cli = self.create_client(DetectCrossings, '/rebar/detect_crossings')

        os.makedirs(DATA_DIR, exist_ok=True)
        self.data_file = os.path.join(DATA_DIR, f'calib3d_data_{camera}.json')
        self.result_file = os.path.join(DATA_DIR, f'calib3d_result_{camera}.yaml')
        self.pairs = self._load()
        self.get_logger().info(
            f'[{camera}] 기존 데이터: {len(self.pairs)}쌍')

    # ---- subs ----
    def _info_cb(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = dict(fx=m.k[0], fy=m.k[4], cx=m.k[2], cy=m.k[5])

    def _depth_cb(self, m):
        self.depth_buf.append(
            self.br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32))

    def _rgb_cb(self, m):
        try:
            self.rgb = self.br.imgmsg_to_cv2(m, 'bgr8')
        except Exception:
            self.rgb = self.br.imgmsg_to_cv2(m, 'passthrough')[:, :, :3]

    def save_viz(self, pts, pcams, flags=None):
        """검출 교차점을 이미지에 라벨링(P1,P2..)해서 저장 → 사용자가 어느 점인지 식별.
        flags가 주어지면 직선이탈(편향의심) 점은 빨강 BIAS로 표시."""
        import cv2
        if self.rgb is None:
            print('  ⚠️ rgb 영상 없음 → 이미지 저장 생략')
            return None
        img = self.rgb.copy()
        for i, ((u, v), pc) in enumerate(zip(pts, pcams)):
            ok = pc is not None
            bad = flags is not None and not flags[i]
            color = (0, 0, 255) if (not ok or bad) else (0, 255, 0)
            cv2.circle(img, (u, v), 16, color, 2)
            cv2.circle(img, (u, v), 2, color, -1)
            tag = f"P{i+1}" + (" BIAS" if bad else "")
            cv2.putText(img, tag, (u + 18, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            if ok:
                cv2.putText(img, f"z={pc[2]:.0f}mm", (u + 18, v + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        path = os.path.join(DATA_DIR, f'calib3d_detect_{self.camera}.png')
        cv2.imwrite(path, img)
        print(f"  📷 검출 이미지: {path}  (P# 라벨 확인 후 실측)")
        return path

    # ---- io ----
    def _load(self):
        if os.path.exists(self.data_file):
            return json.load(open(self.data_file))
        return []

    def _save_data(self):
        json.dump(self.pairs, open(self.data_file, 'w'), indent=2)

    def spin_some(self, sec=0.5):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---- 3D 복원 ----
    def depth_robust_mm(self, u, v):
        """공간(win)+시간(버퍼) 전체 유효샘플 풀 → MAD outlier 제거 후 평균.
        반환: (depth_mm, noise_mm). 샘플 부족 시 (None, 0)."""
        r = self.win // 2
        pool = []
        for d in list(self.depth_buf):
            h, w = d.shape[:2]
            if not (0 <= u < w and 0 <= v < h):
                continue
            patch = d[max(0, v-r):v+r+1, max(0, u-r):u+r+1].ravel()
            valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 3.0)]
            pool.extend(valid.tolist())
        if len(pool) < 10:
            return None, 0.0
        pool = np.array(pool)
        med = np.median(pool)
        mad = np.median(np.abs(pool - med)) + 1e-9
        inl = pool[np.abs(pool - med) < 3.0 * 1.4826 * mad]  # 로버스트 outlier 제거
        if inl.size < 5:
            inl = pool
        return float(inl.mean()) * 1000.0, float(inl.std()) * 1000.0

    def _pool(self, u, v, r_in, r_out):
        """버퍼 전체에서 chebyshev거리 (r_in<dist<=r_out) 환형 유효 depth(mm) 수집."""
        vals = []
        for d in list(self.depth_buf):
            h, w = d.shape[:2]
            y0, y1 = max(0, v-r_out), min(h, v+r_out+1)
            x0, x1 = max(0, u-r_out), min(w, u+r_out+1)
            sub = d[y0:y1, x0:x1]
            yy, xx = np.ogrid[y0:y1, x0:x1]
            dist = np.maximum(np.abs(yy - v), np.abs(xx - u))
            mask = (dist > r_in) & (dist <= r_out) & np.isfinite(sub) \
                & (sub > 0.05) & (sub < 3.0)
            vals.extend((sub[mask] * 1000.0).tolist())
        return np.array(vals)

    def rebar_depth_mm(self, u, v):
        """철근 교차점 = 그 영역의 '최근접 표면'(겹친 철근의 가장 높은 점).
        윈도우에 철판이 섞여도 근접밴드만 평균해 철근면을 고정.
        윈도우 전체가 철판이면 z도 높아짐 → 직선성 가드가 별도로 잡음.
        반환: (z_mm, noise_mm, status: ok/no_depth)."""
        near = self._pool(u, v, -1, self.win // 2)
        if near.size < 10:
            return None, 0.0, 'no_depth'
        lo = np.percentile(near, 5)                  # 노이즈 제외한 최근접
        band = near[near < lo + 25.0]                # 최근접 표면(철근) 밴드
        if band.size < 3:
            band = near
        return float(band.mean()), float(band.std()), 'ok'

    def reconstruct(self, u, v):
        Z, noise, status = self.rebar_depth_mm(u, v)
        if Z is None or self.K is None:
            return None, noise, status
        X = (u - self.K['cx']) * Z / self.K['fx']
        Y = (v - self.K['cy']) * Z / self.K['fy']
        return [X, Y, Z], noise, status

    def collinearity_flags(self, pcams, pts, tol=15.0, row_gap_px=60):
        """[그리드 대응] 검출 교차점을 '행(가로철근)'별로 그룹지어, 같은 행 안에서만
        3D 직선성 검사. 이전엔 그리드(여러 철근) 전체에 직선 1개를 적합해
        멀쩡한 점을 오탐했음 → 원근 z변화는 3D 직선이 흡수하므로 단일선 자체는
        문제 없으나, 서로 다른 철근을 한 선에 욱여넣은 게 버그였음.
        행 클러스터링은 픽셀 v 기준(뎁스가 틀려도 픽셀 행은 정확).
        같은 행 안에서 선 이탈 tol(mm) 초과 = depth 편향 의심.
        반환: (flags[bool], devs[mm])."""
        n = len(pcams)
        flags = [True] * n
        devs = [0.0] * n
        valid_idx = [i for i in range(n) if pcams[i] is not None]
        if len(valid_idx) < 3:
            return flags, devs  # 3점 미만 → 검증 불가, 통과
        # 픽셀 v(행)로 가로철근 클러스터링
        order = sorted(valid_idx, key=lambda i: pts[i][1])
        groups = [[order[0]]]
        for i in order[1:]:
            if pts[i][1] - pts[groups[-1][-1]][1] > row_gap_px:
                groups.append([i])
            else:
                groups[-1].append(i)
        # 각 행 그룹 안에서만 직선성 검사 (그룹당 점 3개 이상일 때만 검증 가능)
        for g in groups:
            if len(g) < 3:
                continue  # 2점 이하는 항상 직선 → 통과
            P = np.array([pcams[i] for i in g], float)

            def resid(keep):
                Q = P[keep]; c = Q.mean(0)
                _, _, Vt = np.linalg.svd(Q - c, full_matrices=False)
                d = Vt[0]
                ap = P - c
                return np.linalg.norm(ap - np.outer(ap @ d, d), axis=1)

            keep = list(range(len(P)))
            while len(keep) >= 3:                   # 최악점 반복 제거
                r = resid(keep)
                kw = max(keep, key=lambda i: r[i])
                if r[kw] > tol:
                    keep.remove(kw)
                else:
                    break
            rfinal = resid(keep)
            for j in range(len(P)):
                if j not in keep:
                    flags[g[j]] = False
                    devs[g[j]] = float(rfinal[j])
        return flags, devs

    # ---- 검출 ----
    def detect(self):
        if not self.cli.wait_for_service(timeout_sec=3.0):
            print('검출 서비스 없음'); return []
        req = DetectCrossings.Request()
        req.camera_selection = self.cfg['sel']
        req.confidence_threshold = 0.4
        req.expected_count = 6
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        if r is None or not r.success:
            print(f"검출 실패: {r.message if r else '응답없음'}")
            return []
        return [(int(d.pixel_u), int(d.pixel_v)) for d in r.grid.detections]

    def detect_stable(self, runs=3, tol=15):
        """검출 여러 번 → 첫 검출 기준 근접매칭 → 다수에 나타난 점만 픽셀 중앙값."""
        all_runs = []
        for _ in range(runs):
            pts = self.detect()
            if pts:
                all_runs.append(pts)
            self.spin_some(0.2)
        if not all_runs:
            return []
        base = all_runs[0]
        out = []
        for (u0, v0) in base:
            matched = [(u0, v0)]
            for r in all_runs[1:]:
                best, bd = None, tol
                for (u, v) in r:
                    dd = ((u - u0) ** 2 + (v - v0) ** 2) ** 0.5
                    if dd < bd:
                        bd, best = dd, (u, v)
                if best:
                    matched.append(best)
            if len(matched) >= max(2, len(all_runs) // 2 + 1):  # 다수결
                a = np.array(matched)
                out.append((int(np.median(a[:, 0])), int(np.median(a[:, 1]))))
        print(f"검출 {len(out)}개 (안정화 {len(all_runs)}회 매칭)")
        return out

    # ---- 적합/평가 ----
    def calc(self):
        if len(self.pairs) < 4:
            print(f"데이터 부족: {len(self.pairs)}쌍 (최소 4)")
            return
        import cv2
        Pc = np.array([p['pcam'] for p in self.pairs], float)
        Pt = np.array([p['ptool'] for p in self.pairs], float)
        n = len(self.pairs)

        # in-plane(X,Y) LOO — 운용은 같은 평면이라 X,Y만 평가
        def loo_inplane(affine):
            errs = []
            for i in range(n):
                m = np.ones(n, bool); m[i] = False
                if affine:
                    _, M, _ = cv2.estimateAffine3D(
                        Pc[m].astype(np.float32), Pt[m].astype(np.float32))
                    pred = M[:, :3] @ Pc[i] + M[:, 3]
                else:
                    R, t, _ = RigidTransform3D.fit(Pc[m], Pt[m])
                    pred = R @ Pc[i] + t
                errs.append(np.linalg.norm(pred[:2] - Pt[i][:2]))
            return np.array(errs)

        loo_r = loo_inplane(False)
        loo_a = loo_inplane(True)
        # affine(RANSAC 내장) 전체 적합
        _, M, inl = cv2.estimateAffine3D(
            Pc.astype(np.float32), Pt.astype(np.float32))
        R_a, t_a = M[:, :3], M[:, 3]
        n_inl = int(inl.sum()) if inl is not None else n

        # 강체 기본 선호 — affine은 명확히(margin mm+) 나을 때만 (과적합/외삽 회피)
        use_rigid = loo_r.mean() <= loo_a.mean() + self.affine_margin
        if use_rigid:
            R_s, t_s = RigidTransform3D.fit(Pc, Pt)[:2]
            label = 'rigid'
        else:
            R_s, t_s = R_a, t_a
            label = 'affine'
        # 저장모델의 점별 예측 + LOO 예측(정직한 일반화)
        pred_s = (R_s @ Pc.T).T + t_s
        res = np.linalg.norm(pred_s[:, :2] - Pt[:, :2], axis=1)
        loo_pred = self._loo_predict(Pc, Pt, use_rigid)

        print(f"\n=== 적합 ({n}쌍, in-plane X·Y 평가) ===")
        print(f"  강체   LOO 평균/최대 : {loo_r.mean():5.1f} / {loo_r.max():5.1f}mm"
              f"{'   ← 저장' if use_rigid else ''}")
        print(f"  affine LOO 평균/최대 : {loo_a.mean():5.1f} / {loo_a.max():5.1f}mm"
              f"{'   ← 저장' if not use_rigid else ''} (RANSAC inlier {n_inl}/{n})")
        if Pt[:, 2].std() < 2.0:
            print(f"  ⚠️ ptool Z 거의 일정 → 평면 (in-plane 평가가 정답)")

        # 점별 예측 vs 실측 (저장모델 + LOO예측)
        from itertools import combinations
        viol = np.zeros(n)
        for i, j in combinations(range(n), 2):
            dd = abs(np.linalg.norm(Pc[i]-Pc[j]) - np.linalg.norm(Pt[i]-Pt[j]))
            viol[i] = max(viol[i], dd); viol[j] = max(viol[j], dd)
        print(f"\n  점별 예측 vs 실측 ({label} 모델):")
        print(f"  {'#':>3} {'실측(X,Y)':>14} {'예측(X,Y)':>14} {'오차':>6} "
              f"{'LOO오차':>7} {'거리위반':>7}")
        for i in np.argsort(res)[::-1]:
            le = np.linalg.norm(loo_pred[i, :2] - Pt[i, :2])
            flag = ' ←outlier' if (le > 12 and viol[i] > 12) else ''
            print(f"  {i:>3} ({Pt[i,0]:5.0f},{Pt[i,1]:5.0f}) "
                  f"({pred_s[i,0]:5.0f},{pred_s[i,1]:5.0f}) "
                  f"{res[i]:5.1f} {le:6.1f} {viol[i]:6.1f}{flag}")

        RigidTransform3D(R_s, t_s).save_yaml(self.result_file)
        print(f"  → 저장({label}): {self.result_file}")

    def _build_model(self):
        """현재 데이터로 모델 + 점별 LOO오차. 반환 (R,t,label,cal_Pc,cal_loo)."""
        import cv2
        Pc = np.array([p['pcam'] for p in self.pairs], float)
        Pt = np.array([p['ptool'] for p in self.pairs], float)
        n = len(Pc)

        def loo(aff):
            e = []
            for i in range(n):
                m = np.ones(n, bool); m[i] = False
                if aff:
                    _, M, _ = cv2.estimateAffine3D(
                        Pc[m].astype(np.float32), Pt[m].astype(np.float32))
                    p = M[:, :3] @ Pc[i] + M[:, 3]
                else:
                    R, t, _ = RigidTransform3D.fit(Pc[m], Pt[m]); p = R @ Pc[i] + t
                e.append(np.linalg.norm(p[:2] - Pt[i][:2]))
            return np.array(e)

        lr, la = loo(False), loo(True)
        if lr.mean() <= la.mean() + self.affine_margin:   # 강체 기본 선호
            R, t, _ = RigidTransform3D.fit(Pc, Pt)
            return R, t, 'rigid', Pc, lr
        _, M, _ = cv2.estimateAffine3D(Pc.astype(np.float32), Pt.astype(np.float32))
        return M[:, :3], M[:, 3], 'affine', Pc, la

    def predict_confidence(self, pc, R, t, cal_Pc, cal_loo, k=4):
        """새 P_cam → 공구예측 + 예상오차.
        예상오차 = 근처 캘리브점 LOO 가중평균 + 외삽패널티(가까운 점이 멀수록 ↑)."""
        pc = np.asarray(pc, float)
        pred = R @ pc + t
        d = np.linalg.norm(cal_Pc - pc, axis=1)
        order = np.argsort(d)[:min(k, len(d))]
        w = 1.0 / (d[order] + 1.0)
        exp = float(np.average(cal_loo[order], weights=w))
        nearest = float(d[order[0]])
        if nearest > 60.0:                       # 외삽 패널티
            exp += (nearest - 60.0) * 0.3
        return pred, exp, nearest

    def predict_round(self):
        """검출 → 각 점 공구위치 예측 + 예상오차 → 결속 가능/제외 판정."""
        if len(self.pairs) < 6:
            print("데이터 부족 (최소 6쌍 필요)"); return
        R, t, label, cal_Pc, cal_loo = self._build_model()
        self.spin_some(1.0)
        pts = self.detect_stable(self.detect_runs)
        if not pts:
            return
        recon, stats = [], []
        for (u, v) in pts:
            self.spin_some(0.5)
            pc, nz, st = self.reconstruct(u, v)
            recon.append(pc); stats.append(st)
        cflags, devs = self.collinearity_flags(recon, pts, 15.0)
        print(f"\n  {label} 모델 예측 + 신뢰도 (제외 임계 {self.reject_mm:.0f}mm):")
        print(f"  {'P':>3} {'예측(X,Y)':>14} {'예상오차':>9} {'결속':>7}")
        for i, (u, v) in enumerate(pts):
            if recon[i] is None or stats[i] != 'ok':
                print(f"  P{i+1:<2} depth 소실 → ⛔제외"); continue
            if not cflags[i]:
                print(f"  P{i+1:<2} 직선이탈 {devs[i]:.0f}mm → ⛔제외"); continue
            pred, exp, near = self.predict_confidence(
                recon[i], R, t, cal_Pc, cal_loo)
            ok = exp <= self.reject_mm
            extra = ' (외삽)' if near > 60 else ''
            print(f"  P{i+1:<2} ({pred[0]:5.0f},{pred[1]:5.0f}) "
                  f"~{exp:5.1f}mm  {'✅결속' if ok else '⛔제외'}{extra}")

    def _loo_predict(self, Pc, Pt, use_rigid):
        """각 점을 빼고 적합한 모델로 그 점 예측 (정직한 일반화 예측값)."""
        import cv2
        n = len(Pc); out = np.zeros((n, 3))
        for i in range(n):
            m = np.ones(n, bool); m[i] = False
            if use_rigid:
                R, t, _ = RigidTransform3D.fit(Pc[m], Pt[m])
                out[i] = R @ Pc[i] + t
            else:
                _, M, _ = cv2.estimateAffine3D(
                    Pc[m].astype(np.float32), Pt[m].astype(np.float32))
                out[i] = M[:, :3] @ Pc[i] + M[:, 3]
        return out

    def show(self):
        print(f"\n수집 {len(self.pairs)}쌍:")
        for i, p in enumerate(self.pairs):
            print(f"  #{i} pix={p['pixel']} cam={np.round(p['pcam'],1).tolist()} "
                  f"tool={p['ptool']}")

    def delete(self, idx):
        if 0 <= idx < len(self.pairs):
            self.pairs.pop(idx); self._save_data()
            print(f"  삭제 #{idx} → {len(self.pairs)}쌍")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='right', choices=['right', 'left'])
    ap.add_argument('--depth-frames', type=int, default=30)
    ap.add_argument('--win', type=int, default=9)
    ap.add_argument('--detect-runs', type=int, default=3,
                    help='검출 반복 횟수(픽셀 중앙값 안정화)')
    ap.add_argument('--colinear-tol', type=float, default=15.0,
                    help='직선이탈 허용(mm). 초과 점은 편향의심 차단')
    ap.add_argument('--min-above', type=float, default=40.0,
                    help='철근이 주변 철판보다 최소 이만큼(mm) 위여야 유효')
    ap.add_argument('--reject-mm', type=float, default=12.0,
                    help='예상오차 이 이상이면 결속 제외')
    ap.add_argument('--affine-margin', type=float, default=3.0,
                    help='affine은 강체보다 이만큼(mm)+ 나을 때만 선택(강체 기본선호)')
    args = ap.parse_args()

    rclpy.init()
    node = Calib3D(args.camera, args.depth_frames, args.win, args.detect_runs,
                   args.min_above)
    node.reject_mm = args.reject_mm
    node.affine_margin = args.affine_margin
    print('=' * 60)
    print(f' 대안1 3D 캘리브레이션 [{args.camera} / {node.cfg["ns"]}]')
    print(' Enter=검출+실측입력  calc=적합  predict=예측/신뢰도  '
          'show=목록  del N=삭제  q=종료')
    print('=' * 60)
    # intrinsic/buffer 워밍업
    print('카메라 워밍업...')
    node.spin_some(2.0)
    if node.K is None:
        print('⚠️ camera_info 못 받음 — depth 토픽/카메라 확인')

    try:
        while True:
            cmd = input('\n[Enter=검출 / calc / show / del N / q]: ').strip()
            if cmd == 'q':
                node.calc()
                break
            if cmd == 'calc':
                node.calc(); continue
            if cmd == 'predict':
                node.predict_round(); continue
            if cmd == 'show':
                node.show(); continue
            if cmd.startswith('del '):
                try: node.delete(int(cmd.split()[1]))
                except (ValueError, IndexError): print('형식: del 번호')
                continue
            # 검출 라운드 (depth 버퍼 충분히 채우기)
            node.spin_some(max(1.5, args.depth_frames / 15.0))
            pts = node.detect_stable(node.detect_runs)
            if not pts:
                continue
            # 모든 점 P_cam 복원 + 검출 이미지 저장 (사용자 식별용)
            recon, noises, stats = [], [], []
            for (u, v) in pts:
                node.spin_some(0.6)
                pc, nz, st = node.reconstruct(u, v)
                recon.append(pc); noises.append(nz); stats.append(st)
            # 최근접표면 depth + 직선성 가드
            cflags, devs = node.collinearity_flags(recon, pts, args.colinear_tol)
            flags = [cflags[i] and stats[i] == 'ok' for i in range(len(pts))]
            node.save_viz(pts, recon, flags)
            nbad = sum(1 for f in flags if not f)
            print(f"  → 검사: {len(pts)-nbad}개 ✅ / {nbad}개 ❌ "
                  f"(직선이탈 or depth소실)")
            print("  → 이미지에서 각 P# 확인하고, 그 점에 공구 대고 실측 입력")
            # 점별 실측 입력
            for i, (u, v) in enumerate(pts):
                pcam = recon[i]
                if pcam is None:
                    print(f"  P{i+1} pixel=({u},{v}): depth 소실 → skip")
                    continue
                warn = '  ⚠️노이즈큼' if noises[i] > 8 else ''
                if not flags[i]:
                    reason = ('직선이탈 %.0fmm' % devs[i]) if not cflags[i] else \
                             'depth 소실'
                    print(f"  P{i+1} ❌ {reason} — 편향 의심 (다른 카메라가 볼 점)")
                    raw = input(f"     기본 차단. 그래도 넣으려면 'X Y Z', "
                                f"건너뛰기 Enter: ").strip()
                    if not raw or raw == 's':
                        continue
                    if raw == 'q':
                        break
                else:
                    print(f"  P{i+1} ✅ pixel=({u},{v}) → P_cam="
                          f"({pcam[0]:.1f}, {pcam[1]:.1f}, {pcam[2]:.1f})mm"
                          f"  [depth노이즈 {noises[i]:.1f}mm]{warn}")
                    raw = input(f"     공구를 이 교차점에 대고 실측(mm) 'X Y Z' "
                                f"(s=skip, q=종료): ").strip()
                    if raw == 'q':
                        break
                    if raw == 's' or not raw:
                        continue
                try:
                    parts = raw.replace(',', ' ').split()
                    ptool = [float(parts[0]), float(parts[1]), float(parts[2])]
                except (ValueError, IndexError):
                    print('     형식오류: X Y Z (3개)')
                    continue
                node.pairs.append({'pixel': [u, v], 'pcam': pcam, 'ptool': ptool})
                node._save_data()
                print(f"     ✅ 저장 [{len(node.pairs)}쌍]")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
