#!/usr/bin/env python3
"""[프로토타입] 철근 평면 RANSAC 적합 → 교차점 depth를 평면에 스냅.
스테레오 오매칭(period-skip)으로 틀린 교차점 depth를 평면으로 교정하는지 검증.
현재 near-surface depth vs 평면스냅 depth 비교 + 평면 적합 품질 표시.

사용: python3 plane_filter_test.py --camera right
"""
import argparse, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rebar_base_interfaces.srv import DetectCrossings

CAM = {'right': {'sel': 2, 'ns': 'zedxmini2'}, 'left': {'sel': 1, 'ns': 'zedxmini1'}}


def near_surface_z(dbuf, u, v, win=21):
    """현재 방식: 윈도우 최근접표면(5%ile+25mm 밴드) 평균."""
    r = win // 2
    vals = []
    for d in dbuf:
        h, w = d.shape
        if not (0 <= u < w and 0 <= v < h):
            continue
        sub = d[max(0, v-r):v+r+1, max(0, u-r):u+r+1] * 1000.0
        sub = sub[np.isfinite(sub) & (sub > 50) & (sub < 3000)]
        vals.extend(sub.tolist())
    if len(vals) < 10:
        return None
    a = np.array(vals)
    lo = np.percentile(a, 5)
    band = a[a < lo + 25.0]
    return float(band.mean()) if band.size >= 3 else float(a.mean())


def fit_plane_ransac(P, thresh=8.0, iters=300):
    """P:(N,3). 평면 n·X=p 적합 (perpendicular dist). 반환 (n, p, inlier_mask)."""
    n_pts = len(P)
    best_inl = None
    best_cnt = 0
    rng_idx = 0
    for it in range(iters):
        # 결정적 샘플링 (Math.random 불가 환경 대비 인덱스 회전)
        i, j, k = (it * 3) % n_pts, (it * 3 + 1) % n_pts, (it * 3 + 2) % n_pts
        if len({i, j, k}) < 3:
            continue
        v1 = P[j] - P[i]; v2 = P[k] - P[i]
        nrm = np.cross(v1, v2)
        nn = np.linalg.norm(nrm)
        if nn < 1e-6:
            continue
        nrm = nrm / nn
        p = nrm @ P[i]
        dist = np.abs(P @ nrm - p)
        inl = dist < thresh
        if inl.sum() > best_cnt:
            best_cnt = inl.sum(); best_inl = inl
    if best_inl is None or best_inl.sum() < 3:
        # fallback: 전체 최소제곱
        c = P.mean(0); _, _, Vt = np.linalg.svd(P - c)
        nrm = Vt[2]; p = nrm @ c
        return nrm, p, np.ones(n_pts, bool)
    # inlier로 재적합 (SVD)
    Q = P[best_inl]; c = Q.mean(0); _, _, Vt = np.linalg.svd(Q - c)
    nrm = Vt[2]; p = nrm @ c
    return nrm, p, best_inl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='right', choices=['right', 'left'])
    args = ap.parse_args()
    cfg = CAM[args.camera]; ns = cfg['ns']
    rclpy.init(); node = Node('plane_test'); br = CvBridge()
    dbuf = []; K = {}
    node.create_subscription(Image, f'/{ns}/zed_node/depth/depth_registered',
        lambda m: dbuf.append(br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)),
        qos_profile_sensor_data)
    node.create_subscription(CameraInfo, f'/{ns}/zed_node/left/camera_info',
        lambda m: K.update({'fx': m.k[0], 'fy': m.k[4], 'cx': m.k[2], 'cy': m.k[5]}),
        qos_profile_sensor_data)
    t = time.time()
    while (not K or len(dbuf) < 15) and time.time() - t < 10:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not K:
        print('camera_info 못받음'); rclpy.shutdown(); return

    cli = node.create_client(DetectCrossings, '/rebar/detect_crossings')
    cli.wait_for_service(timeout_sec=3.0)
    req = DetectCrossings.Request()
    req.camera_selection = cfg['sel']; req.confidence_threshold = 0.4; req.expected_count = 6
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    r = fut.result()
    pts = [(int(d.pixel_u), int(d.pixel_v)) for d in r.grid.detections] if (r and r.success) else []
    print(f'검출 {len(pts)}점\n')

    # 1) 현재 near-surface depth로 3D 복원
    dwin = dbuf[-15:]
    P, valid_pts = [], []
    for (u, v) in pts:
        z = near_surface_z(dwin, u, v)
        if z is None:
            P.append(None); continue
        X = (u - K['cx']) * z / K['fx']; Y = (v - K['cy']) * z / K['fy']
        P.append([X, Y, z]); valid_pts.append((u, v))
    Pv = np.array([p for p in P if p is not None])
    if len(Pv) < 3:
        print('유효점 부족'); rclpy.shutdown(); return

    # 2) 철근 평면 적합 — 교차점은 일직선될 수 있어 밀집 depth로 robust 적합
    # (2a) 교차점 SVD 초기평면 (전체점, collinear-3보다 안정)
    c0 = Pv.mean(0); _, _, Vt0 = np.linalg.svd(Pv - c0)
    n0 = Vt0[2]; p0 = n0 @ c0
    # (2b) 밀집 depth점 수집 (격자 샘플) → 초기평면 ±25mm 밴드(철근표면) → 재적합
    dref = dwin[-1]
    h, w = dref.shape
    dense = []
    for vv in range(0, h, 4):
        for uu in range(0, w, 4):
            z = dref[vv, uu] * 1000.0
            if not (np.isfinite(z) and 50 < z < 3000):
                continue
            X = (uu - K['cx']) * z / K['fx']; Y = (vv - K['cy']) * z / K['fy']
            dense.append([X, Y, z])
    dense = np.array(dense)
    band = np.abs(dense @ n0 - p0) < 25.0
    Pdense = dense[band]
    if len(Pdense) >= 50:
        cd = Pdense.mean(0); _, _, Vtd = np.linalg.svd(Pdense - cd)
        nrm = Vtd[2]; p_off = nrm @ cd
    else:
        nrm, p_off = n0, p0
    if nrm[2] < 0:
        nrm, p_off = -nrm, -p_off
    resid = np.abs(Pv @ nrm - p_off)
    print(f'평면 적합(밀집 {len(Pdense)}점 기반): 교차점 평면잔차 평균 {resid.mean():.1f} max {resid.max():.1f}mm')
    print(f'  법선 {np.round(nrm,3)}  (z성분 {abs(nrm[2]):.2f})\n')

    # 3) 각 교차점 depth를 평면에 스냅 (ray-plane 교차) + 비교
    print(f"{'px(u,v)':>14} {'현재z':>7} {'평면z':>7} {'차이':>6} {'평면잔차':>7}")
    print('-' * 50)
    vi = 0
    for i, (u, v) in enumerate(pts):
        if P[i] is None:
            print(f"({u:4d},{v:4d}) depth소실"); continue
        z_now = P[i][2]
        ray = np.array([(u - K['cx']) / K['fx'], (v - K['cy']) / K['fy'], 1.0])
        denom = nrm @ ray
        z_plane = p_off / denom if abs(denom) > 1e-6 else np.nan
        d = resid[vi]; vi += 1
        mark = ' ←오매칭의심' if abs(z_plane - z_now) > 12 else ''
        print(f"({u:4d},{v:4d}) {z_now:7.0f} {z_plane:7.0f} {z_plane-z_now:+6.0f} {d:7.1f}{mark}")
    print('\n현재z=near-surface, 평면z=평면스냅. 차이 크면 그 점 depth가 평면에서 벗어남(오매칭).')
    print('평면스냅이 교차점들을 일관된 한 평면에 정렬 → 캘리브 잔차 개선 기대')
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
