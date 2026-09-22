#!/usr/bin/env python3
"""depth 맵 → 3D 역투영 → 지배평면(철판) 기준 회전 → top-view 정사영.
높이(평면 위 mm)로 색칠 → 철근층(평판 +12~13cm)이 분리되어 보임.
교차점도 같은 좌표계로 변환해 오버레이.

사용법: python3 depth_topview.py --camera right
저장:   data/calibration/topview_{cam}.png
"""
import argparse, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
try:
    from rebar_base_interfaces.srv import DetectCrossings
    HAVE = True
except Exception:
    HAVE = False

CAM = {'right': {'sel': 2, 'ns': 'zedxmini2'}, 'left': {'sel': 1, 'ns': 'zedxmini1'}}
OUT = '/home/koceti/ros2_ws/data/calibration'


def fit_plane_robust(P, iters=5, band=15.0):
    """반복 SVD 평면적합 (지배평면 수렴). 반환 (point0, normal)."""
    idx = np.ones(len(P), bool)
    for _ in range(iters):
        Q = P[idx]
        c = Q.mean(0)
        _, _, Vt = np.linalg.svd(Q - c, full_matrices=False)
        n = Vt[2]
        d = (P - c) @ n
        idx = np.abs(d) < band
        if idx.sum() < 50:
            break
    return c, n


def rot_to_z(n):
    """법선 n을 +Z로 보내는 회전행렬."""
    n = n / np.linalg.norm(n)
    z = np.array([0, 0, 1.0])
    v = np.cross(n, z); s = np.linalg.norm(v); cth = n @ z
    if s < 1e-8:
        return np.eye(3) if cth > 0 else np.diag([1, -1, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - cth) / (s * s))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='right', choices=['right', 'left'])
    ap.add_argument('--frames', type=int, default=15)
    ap.add_argument('--zmin', type=float, default=280.0, help='작업영역 depth 하한(mm)')
    ap.add_argument('--zmax', type=float, default=600.0, help='작업영역 depth 상한(mm)')
    args = ap.parse_args()
    cfg = CAM[args.camera]; ns = cfg['ns']
    rclpy.init(); node = Node('topview'); br = CvBridge()

    K = {}
    node.create_subscription(CameraInfo, f'/{ns}/zed_node/depth/camera_info',
        lambda m: K.update(fx=m.k[0], fy=m.k[4], cx=m.k[2], cy=m.k[5]) if not K else None,
        qos_profile_sensor_data)
    frames = []
    node.create_subscription(Image, f'/{ns}/zed_node/depth/depth_registered',
        lambda m: frames.append(br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)),
        qos_profile_sensor_data)
    t0 = time.time()
    while (len(frames) < args.frames or not K) and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not frames or not K:
        print('depth/intrinsic 못 받음'); rclpy.shutdown(); return

    depth = np.nanmedian(np.stack(frames[-args.frames:]), axis=0) * 1000.0  # mm
    h, w = depth.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    valid = np.isfinite(depth) & (depth > args.zmin) & (depth < args.zmax)
    Z = depth[valid]
    X = (uu[valid] - K['cx']) * Z / K['fx']
    Y = (vv[valid] - K['cy']) * Z / K['fy']
    P = np.stack([X, Y, Z], 1)

    # 지배평면 적합 + 정렬 회전
    c, n = fit_plane_robust(P)
    tilt = np.degrees(np.arccos(abs(n @ np.array([0, 0, 1.0]))))
    R = rot_to_z(n)
    Q = (R @ (P - c).T).T          # 평면 프레임: z=평면위 높이
    # 높이가 위로(+) 가도록 부호 정렬 (상위 표면=철근이 양수)
    flip = 1.0
    if np.median(Q[:, 2]) > 0:
        flip = -1.0; Q[:, 2] *= -1.0
    height = Q[:, 2]

    # 교차점 변환
    pts3 = []
    if HAVE:
        cli = node.create_client(DetectCrossings, '/rebar/detect_crossings')
        if cli.wait_for_service(timeout_sec=3.0):
            req = DetectCrossings.Request()
            req.camera_selection = cfg['sel']; req.confidence_threshold = 0.4
            req.expected_count = 6
            fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            r = fut.result()
            if r and r.success:
                for d in r.grid.detections:
                    u, v = int(d.pixel_u), int(d.pixel_v)
                    if valid[v, u]:
                        z = depth[v, u]
                        p = np.array([(u-K['cx'])*z/K['fx'], (v-K['cy'])*z/K['fy'], z])
                        q = R @ (p - c)
                        q[2] *= flip                 # 동일 부호정렬
                        pts3.append(q)
    pts3 = np.array(pts3) if pts3 else np.empty((0, 3))

    print(f"점 {len(P)}개, 평면틸트 {tilt:.1f}° (nominal 40°), "
          f"높이범위 {height.min():.0f}~{height.max():.0f}mm")

    # top-view 산점도 (subsample), 색=높이
    s = max(1, len(Q) // 120000)
    Qs = Q[::s]
    fig, ax = plt.subplots(figsize=(11, 9))
    sc = ax.scatter(Qs[:, 0], Qs[:, 1], c=Qs[:, 2], s=2,
                    cmap='turbo', vmin=np.percentile(Qs[:, 2], 2),
                    vmax=np.percentile(Qs[:, 2], 98))
    cb = fig.colorbar(sc, ax=ax, fraction=0.04); cb.set_label('height above plane (mm)')
    for i, q in enumerate(pts3):
        ax.plot(q[0], q[1], 'k+', ms=16, mew=3)
        ax.text(q[0]+5, q[1], f"P{i+1}({q[2]:.0f})", fontsize=9, color='k')
    ax.set_aspect('equal'); ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)')
    ax.set_title(f'{ns} TOP-VIEW (plane-aligned, tilt {tilt:.0f}deg)')
    ax.grid(True, alpha=0.3)
    path = f'{OUT}/topview_{args.camera}.png'
    fig.savefig(path, dpi=120, bbox_inches='tight')
    print(f"저장: {path}")
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
