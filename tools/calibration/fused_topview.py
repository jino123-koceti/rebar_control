#!/usr/bin/env python3
"""좌/우 카메라 top-view 비교 + 융합 오버레이.
각 카메라: depth→3D→지배평면 정렬 top-view, 검출 교차점 + 양끝직선 표시.
좌측은 교차점이 일직선, 우측은 P2가 선에서 튀는 걸 시각화.
오버레이: 공유 교차점으로 우측을 좌측 프레임에 2D 정렬(Procrustes)해 겹쳐 그림.

사용법: python3 fused_topview.py
저장:   data/calibration/fused_topview.png
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rebar_base_interfaces.srv import DetectCrossings
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = '/home/koceti/ros2_ws/data/calibration'
CAMS = [('zedxmini1', 1, 'LEFT'), ('zedxmini2', 2, 'RIGHT')]


def fit_plane_robust(P, iters=5, band=15.0):
    idx = np.ones(len(P), bool)
    c = P.mean(0); n = np.array([0, 0, 1.0])
    for _ in range(iters):
        Q = P[idx]; c = Q.mean(0)
        _, _, Vt = np.linalg.svd(Q - c, full_matrices=False)
        n = Vt[2]; d = (P - c) @ n; idx = np.abs(d) < band
        if idx.sum() < 50:
            break
    return c, n


def rot_to_z(n):
    n = n / np.linalg.norm(n); z = np.array([0, 0, 1.0])
    v = np.cross(n, z); s = np.linalg.norm(v); cth = n @ z
    if s < 1e-8:
        return np.eye(3) if cth > 0 else np.diag([1, -1, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - cth) / (s * s))


def capture(ns, sel, zmin=280, zmax=600, frames_n=15):
    node = Node('cap_' + ns); br = CvBridge(); K = {}; frames = []
    node.create_subscription(CameraInfo, f'/{ns}/zed_node/depth/camera_info',
        lambda m: K.update(fx=m.k[0], fy=m.k[4], cx=m.k[2], cy=m.k[5]) if not K else None,
        qos_profile_sensor_data)
    node.create_subscription(Image, f'/{ns}/zed_node/depth/depth_registered',
        lambda m: frames.append(br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)),
        qos_profile_sensor_data)
    cli = node.create_client(DetectCrossings, '/rebar/detect_crossings')
    cli.wait_for_service(timeout_sec=5)
    # 3점 이상 잡힐 때까지 최대 6회 재시도 (가장 많이 검출된 run 채택)
    pts = []
    for _ in range(6):
        req = DetectCrossings.Request(); req.camera_selection = sel
        req.confidence_threshold = 0.4; req.expected_count = 6
        fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
        r = fut.result()
        cur = [(int(d.pixel_u), int(d.pixel_v)) for d in r.grid.detections] if (r and r.success) else []
        if len(cur) > len(pts):
            pts = cur
        if len(pts) >= 3:
            break
        time.sleep(0.3)
    t0 = time.time()
    while (len(frames) < frames_n or not K) and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    depth = np.nanmedian(np.stack(frames[-frames_n:]), axis=0) * 1000.0
    h, w = depth.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    valid = np.isfinite(depth) & (depth > zmin) & (depth < zmax)
    Z = depth[valid]
    P = np.stack([(uu[valid]-K['cx'])*Z/K['fx'], (vv[valid]-K['cy'])*Z/K['fy'], Z], 1)
    c, n = fit_plane_robust(P); R = rot_to_z(n)
    Q = (R @ (P - c).T).T
    flip = -1.0 if np.median(Q[:, 2]) > 0 else 1.0
    Q[:, 2] *= flip
    cross = []
    for (u, v) in sorted(pts, key=lambda p: p[0]):
        if valid[v, u]:
            z = depth[v, u]
            q = R @ (np.array([(u-K['cx'])*z/K['fx'], (v-K['cy'])*z/K['fy'], z]) - c)
            q[2] *= flip; cross.append(q)
    node.destroy_node()
    return Q, np.array(cross), np.degrees(np.arccos(abs(n @ np.array([0, 0, 1.0]))))


def dev(cross):
    if len(cross) < 3:
        return None
    a, b = cross[0], cross[-1]; ab = b - a
    return max(np.linalg.norm((cross[i]-a) - np.dot(cross[i]-a, ab)/np.dot(ab, ab)*ab)
               for i in range(1, len(cross)-1))


def main():
    rclpy.init()
    data = {}
    for ns, sel, tag in CAMS:
        print(f"캡처 {tag} ({ns})...")
        Q, cross, tilt = capture(ns, sel)
        data[tag] = (Q, cross, tilt)
        d = dev(cross)
        print(f"  점 {len(Q)}, 교차점 {len(cross)}, tilt {tilt:.0f}°, "
              f"직선이탈 {('%.1fmm' % d) if d else 'N/A'}")
    rclpy.shutdown()

    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    for ax, tag in zip(axes, ['LEFT', 'RIGHT']):
        Q, cross, tilt = data[tag]
        s = max(1, len(Q)//80000)
        Qs = Q[::s]
        ax.scatter(Qs[:, 0], Qs[:, 1], c=Qs[:, 2], s=2, cmap='turbo',
                   vmin=np.percentile(Qs[:, 2], 2), vmax=np.percentile(Qs[:, 2], 98))
        if len(cross) >= 2:
            ax.plot([cross[0, 0], cross[-1, 0]], [cross[0, 1], cross[-1, 1]],
                    'k--', lw=1.5, label='line thru ends')
        for i, q in enumerate(cross):
            ax.plot(q[0], q[1], 'k+', ms=18, mew=3)
            ax.text(q[0]+6, q[1]+10, f"P{i+1}", fontsize=11, fontweight='bold')
        d = dev(cross)
        if len(cross) >= 2:  # 교차점 영역으로 확대
            cx, cy = cross[:, 0], cross[:, 1]
            ax.set_xlim(cx.min()-250, cx.max()+250)
            ax.set_ylim(cy.min()-250, cy.max()+250)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_title(f"{tag}  (tilt {tilt:.0f}deg, P2 off-line "
                     f"{('%.1fmm' % d) if d else 'N/A'})", fontsize=13)
        ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)')
        if len(cross) >= 2:
            ax.legend(loc='upper right')
    fig.suptitle('LEFT vs RIGHT top-view — crossings should be ONE straight line',
                 fontsize=14)
    path = f'{OUT}/fused_topview.png'
    fig.savefig(path, dpi=110, bbox_inches='tight')
    print(f"저장: {path}")


if __name__ == '__main__':
    main()
