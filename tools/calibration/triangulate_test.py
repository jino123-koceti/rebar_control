#!/usr/bin/env python3
"""[와이드베이스라인 삼각측량 테스트] 좌우 카메라 교차점 → 삼각측량 3D.
- 좌우에서 교차점 검출(YOLO 서비스) → 재투영오차로 대응 매칭
- 삼각측량(내부 depth 안 씀 → period-skip 면역) → 우측카메라 프레임 3D
- 우측 calib3d(R,t)로 로봇좌표 변환
- 우측 단일카메라 depth 추정과 비교 → period-skip 자동검출

사용: python3 triangulate_test.py
"""
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rebar_base_interfaces.srv import DetectCrossings
import yaml

EXT = '/home/koceti/ros2_ws/data/calibration/stereo_extrinsics.yaml'
CAL3D = '/home/koceti/ros2_ws/data/calibration/calib3d_result_right.yaml'
REPROJ_TOL = 8.0   # px, 대응 매칭 임계 (마주보는 뷰라 교차점정의 차이 여유)


def detect_crossings(node, cli, sel):
    req = DetectCrossings.Request()
    req.camera_selection = sel
    req.confidence_threshold = 0.25
    req.expected_count = 12
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    r = fut.result()
    if r is None or not r.success:
        return []
    return [(float(d.pixel_u), float(d.pixel_v)) for d in r.grid.detections]


def near_surface_z(dbuf, u, v, fx, fy, cx, cy, win=21):
    r = win // 2; vals = []
    u, v = int(u), int(v)
    for d in dbuf:
        h, w = d.shape
        if not (0 <= u < w and 0 <= v < h):
            continue
        sub = d[max(0, v-r):v+r+1, max(0, u-r):u+r+1] * 1000.0
        sub = sub[np.isfinite(sub) & (sub > 50) & (sub < 3000)]
        vals.extend(sub.tolist())
    if len(vals) < 10:
        return None
    a = np.array(vals); lo = np.percentile(a, 5)
    band = a[a < lo + 25.0]
    z = float(band.mean()) if band.size >= 3 else float(a.mean())
    return [(u - cx) * z / fx, (v - cy) * z / fy, z]


def main():
    ext = yaml.safe_load(open(EXT))
    K_L = np.array(ext['K_L']); K_R = np.array(ext['K_R'])
    R = np.array(ext['R_RtoL'])
    T = np.array(ext['T_RtoL_m']).reshape(3, 1) * 1000.0  # m→mm (calib3d가 mm 기준)
    # 우측 카메라 프레임 기준: P_R=K_R[I|0], P_L=K_L[R|T]
    P_R = K_R @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P_L = K_L @ np.hstack([R, T])
    cal = yaml.safe_load(open(CAL3D))['rigid_transform']
    Rt = np.array(cal['R']); tt = np.array(cal['t'])

    rclpy.init(); node = Node('triang'); br = CvBridge()
    dbuf = []
    node.create_subscription(Image, '/zedxmini2/zed_node/depth/depth_registered',
        lambda m: dbuf.append(br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)),
        qos_profile_sensor_data)
    cli = node.create_client(DetectCrossings, '/rebar/detect_crossings')
    cli.wait_for_service(timeout_sec=3.0)
    t = time.time()
    while len(dbuf) < 12 and time.time() - t < 8:
        rclpy.spin_once(node, timeout_sec=0.05)

    ptsL = detect_crossings(node, cli, 1)
    ptsR = detect_crossings(node, cli, 2)
    print(f'검출: 좌 {len(ptsL)}점, 우 {len(ptsR)}점\n')
    if not ptsL or not ptsR:
        print('한쪽 검출 0 — 종료'); rclpy.shutdown(); return

    # 대응 매칭: 모든 (L,R)쌍 삼각측량 → 재투영오차 최소 매칭
    print(f"{'좌px':>12} {'우px':>12} {'삼각측량3D(우프레임)':>22} {'재투영px':>8}")
    print('-' * 70)
    used_R = set()
    results = []
    for (uL, vL) in ptsL:
        best = None
        for j, (uR, vR) in enumerate(ptsR):
            if j in used_R:
                continue
            X = cv2.triangulatePoints(P_R, P_L,
                np.array([[uR], [vR]]), np.array([[uL], [vL]]))
            X = (X[:3] / X[3]).flatten()
            # 재투영
            pr = P_R @ np.append(X, 1); pr = pr[:2] / pr[2]
            pl = P_L @ np.append(X, 1); pl = pl[:2] / pl[2]
            err = (np.hypot(pr[0]-uR, pr[1]-vR) + np.hypot(pl[0]-uL, pl[1]-vL)) / 2
            if best is None or err < best[0]:
                best = (err, j, X, (uR, vR))
        if best and best[0] < REPROJ_TOL:
            err, j, X, (uR, vR) = best
            used_R.add(j)
            robot = Rt @ X + tt
            results.append((uL, vL, uR, vR, X, robot, err))
            print(f"({uL:5.0f},{vL:5.0f}) ({uR:5.0f},{vR:5.0f}) "
                  f"({X[0]:6.1f},{X[1]:6.1f},{X[2]:6.1f}) {err:8.1f}")
    if not results:
        print('대응 매칭 실패 (재투영오차 큼 — 캘리브/검출 확인)')
        rclpy.shutdown(); return

    # 단일카메라 depth와 비교 (period-skip 검출)
    fx, fy, cx, cy = K_R[0, 0], K_R[1, 1], K_R[0, 2], K_R[1, 2]
    print(f"\n{'우px':>12} {'삼각측량 robotXY':>18} {'단일cam robotXY':>18} {'차이mm':>7}")
    print('-' * 65)
    for (uL, vL, uR, vR, X, robot, err) in results:
        Xc = near_surface_z(dbuf[-12:], uR, vR, fx, fy, cx, cy)
        if Xc is None:
            print(f"({uR:5.0f},{vR:5.0f}) depth소실"); continue
        robot_single = Rt @ np.array(Xc) + tt
        diff = np.hypot(robot[0]-robot_single[0], robot[1]-robot_single[1])
        mark = ' ←불일치(period-skip?)' if diff > 10 else ''
        print(f"({uR:5.0f},{vR:5.0f}) ({robot[0]:6.1f},{robot[1]:6.1f}) "
              f"({robot_single[0]:6.1f},{robot_single[1]:6.1f}) {diff:7.1f}{mark}")
    print('\n삼각측량=내부depth 안씀(period-skip 면역). 단일cam=현재방식.')
    print('차이 크면 단일cam이 period-skip된 것 → 삼각측량이 옳음')
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
