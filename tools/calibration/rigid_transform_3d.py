#!/usr/bin/env python3
"""
3D 강체변환(Rigid Transform, Hand-Eye) 공통 모듈 — 대안1/대안2 공용.

카메라 기준 3D 점 P_cam = [Xc,Yc,Zc] 들과
공구(TCP) 기준 실측 3D 점 P_tool = [Xt,Yt,Zt] 들의 대응쌍으로부터
        P_tool = R @ P_cam + t
의 회전 R(3x3), 평행이동 t(3,)를 SVD(Arun/Kabsch) 또는 cv2.estimateAffine3D로 도출.

2D 다항식 회귀(기존 calibrate_wo_depth_user.py)와의 차이:
  - 40° 투시왜곡/수직압축/Z민감도를 "측정된 3D"로 직접 처리 → 회귀 외삽 오차 없음
  - 최소 4쌍(이론), 실무 8~15쌍 권장 (노이즈 평균화)

자체검증: python3 rigid_transform_3d.py  (합성데이터로 R,t 복원 + 노이즈 잔차 확인)
"""
import numpy as np


class RigidTransform3D:
    def __init__(self, R=None, t=None):
        self.R = R              # (3,3)
        self.t = t              # (3,)

    # ---- 피팅 (Kabsch/Arun SVD) ----
    @staticmethod
    def fit(P_cam, P_tool):
        """P_cam, P_tool: (N,3). 반환: (R, t, rms_mm)."""
        P_cam = np.asarray(P_cam, float)
        P_tool = np.asarray(P_tool, float)
        assert P_cam.shape == P_tool.shape and P_cam.shape[0] >= 3
        cen_c = P_cam.mean(axis=0)
        cen_t = P_tool.mean(axis=0)
        Pc = P_cam - cen_c
        Pt = P_tool - cen_t
        H = Pc.T @ Pt                      # (3,3) 공분산
        U, S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        D = np.diag([1.0, 1.0, d])         # 반사(거울) 방지
        R = Vt.T @ D @ U.T
        t = cen_t - R @ cen_c
        # 잔차
        pred = (R @ P_cam.T).T + t
        err = np.linalg.norm(pred - P_tool, axis=1)
        rms = float(np.sqrt(np.mean(err ** 2)))
        return R, t, rms

    @staticmethod
    def fit_cv(P_cam, P_tool):
        """cv2.estimateAffine3D 기반 (스케일 허용 affine — 비교용)."""
        import cv2
        P_cam = np.asarray(P_cam, np.float32)
        P_tool = np.asarray(P_tool, np.float32)
        retval, M, inliers = cv2.estimateAffine3D(P_cam, P_tool)
        R = M[:, :3]
        t = M[:, 3]
        pred = (R @ P_cam.T).T + t
        err = np.linalg.norm(pred - P_tool, axis=1)
        return R, t, float(np.sqrt(np.mean(err ** 2)))

    def apply(self, P_cam):
        P_cam = np.asarray(P_cam, float).reshape(-1, 3)
        return (self.R @ P_cam.T).T + self.t

    # ---- 정확도 평가: Leave-One-Out CV ----
    @staticmethod
    def loo_error(P_cam, P_tool, use_cv=False):
        """각 점을 빼고 피팅 → 그 점 예측오차. (정직한 일반화 정확도)"""
        P_cam = np.asarray(P_cam, float)
        P_tool = np.asarray(P_tool, float)
        n = len(P_cam)
        errs = []
        for i in range(n):
            m = np.ones(n, bool); m[i] = False
            if use_cv:
                R, t, _ = RigidTransform3D.fit_cv(P_cam[m], P_tool[m])
            else:
                R, t, _ = RigidTransform3D.fit(P_cam[m], P_tool[m])
            pred = R @ P_cam[i] + t
            errs.append(np.linalg.norm(pred - P_tool[i]))
        errs = np.array(errs)
        return errs  # mm

    def save_yaml(self, path):
        import yaml
        data = {'rigid_transform': {
            'R': self.R.tolist(),
            't': self.t.tolist(),
            'note': 'P_tool = R @ P_cam + t  (mm)',
        }}
        with open(path, 'w') as f:
            yaml.dump(data, f, sort_keys=False)

    @classmethod
    def load_yaml(cls, path):
        import yaml
        d = yaml.safe_load(open(path))['rigid_transform']
        return cls(np.array(d['R']), np.array(d['t']))


def _rand_rotation(seed):
    rng = np.random.default_rng(seed)
    A = rng.standard_normal((3, 3))
    Q, _ = np.linalg.qr(A)
    if np.linalg.det(Q) < 0:
        Q[:, 0] = -Q[:, 0]
    return Q


def self_test():
    print("=== RigidTransform3D 자체검증 (합성데이터) ===")
    rng = np.random.default_rng(42)
    R_true = _rand_rotation(1)
    t_true = np.array([120.0, -40.0, 300.0])
    # 작업영역 모사: X 0~400, Y 0~144, Z 약간 변동 (철근 처짐/플레이트 굴곡)
    N = 12
    P_tool = np.column_stack([
        rng.uniform(0, 400, N),
        rng.uniform(0, 144, N),
        rng.uniform(-5, 5, N),   # Z 미세 변동
    ])
    # 카메라 3D = 역변환 (P_cam = R^T (P_tool - t))
    P_cam_clean = (R_true.T @ (P_tool - t_true).T).T

    for noise_mm in [0.0, 1.0, 3.0]:
        P_cam = P_cam_clean + rng.normal(0, noise_mm, P_cam_clean.shape)
        R, t, rms = RigidTransform3D.fit(P_cam, P_tool)
        rot_err = np.degrees(np.arccos(np.clip((np.trace(R_true.T @ R) - 1) / 2, -1, 1)))
        t_err = np.linalg.norm(t - t_true)
        loo = RigidTransform3D.loo_error(P_cam, P_tool)
        print(f"\n[카메라3D 노이즈 σ={noise_mm}mm]  (N={N})")
        print(f"  R 복원오차: {rot_err:.3f}°   t 복원오차: {t_err:.2f}mm")
        print(f"  in-sample RMS: {rms:.2f}mm   |  LOO 평균/최대: "
              f"{loo.mean():.2f}/{loo.max():.2f}mm")
    print("\n→ 노이즈 σ만큼의 입력오차가 평균화되어 비슷한 수준 잔차로 나오면 알고리즘 정상.")
    print("  (3D 측정만 정확하면 변환은 mm급으로 정확함이 확인됨)")


if __name__ == '__main__':
    self_test()
