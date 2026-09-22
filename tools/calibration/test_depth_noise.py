#!/usr/bin/env python3
"""
[대안 1 viability 테스트] 교차점에서 ZED depth 반복정밀도(노이즈) 측정.

RigidTransform3D 검증 결과: 최종정확도 ≈ 카메라3D(P_cam) 노이즈 수준.
→ 대안1(depth 기반)이 쓸만한지는 "교차점 depth가 얼마나 안정적인가"가 전부.

측정:
  1) /rebar/detect_crossings 로 교차점 픽셀 획득
  2) depth 토픽 N프레임 수집
  3) 각 교차점에서: raw depth std + 5x5 윈도우중앙값 depth std (mm)
  4) intrinsic으로 3D 역투영 → P_cam의 X/Y/Z 흔들림(std, mm)

판정: P_cam std < ~2~3mm → 대안1 충분 / ~cm → 대안2 필요

사용법: python3 test_depth_noise.py --camera right   (right=zedxmini2 / left=zedxmini1)
"""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rebar_base_interfaces.srv import DetectCrossings

CAM = {
    'right': {'sel': 2, 'ns': 'zedxmini2'},
    'left':  {'sel': 1, 'ns': 'zedxmini1'},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='right', choices=['right', 'left'])
    ap.add_argument('--frames', type=int, default=30)
    ap.add_argument('--win', type=int, default=5, help='공간 중앙값 윈도우 (홀수)')
    args = ap.parse_args()
    cfg = CAM[args.camera]
    ns = cfg['ns']
    depth_topic = f'/{ns}/zed_node/depth/depth_registered'
    info_topic = f'/{ns}/zed_node/depth/camera_info'

    rclpy.init()
    node = Node('depth_noise_test')
    br = CvBridge()

    # --- intrinsic ---
    K = {}
    def info_cb(m):
        if not K:
            K['fx'], K['fy'] = m.k[0], m.k[4]
            K['cx'], K['cy'] = m.k[2], m.k[5]
    node.create_subscription(CameraInfo, info_topic, info_cb, qos_profile_sensor_data)

    # --- depth 프레임 수집 ---
    frames = []
    def depth_cb(m):
        if len(frames) < args.frames:
            d = br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)
            frames.append(d)
    node.create_subscription(Image, depth_topic, depth_cb, qos_profile_sensor_data)

    # --- 교차점 검출 ---
    cli = node.create_client(DetectCrossings, '/rebar/detect_crossings')
    print("검출 서비스 대기...")
    cli.wait_for_service(timeout_sec=5.0)
    req = DetectCrossings.Request()
    req.camera_selection = cfg['sel']
    req.confidence_threshold = 0.4
    req.expected_count = 6
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    resp = fut.result()
    if resp is None or not resp.success:
        print(f"검출 실패: {resp.message if resp else '응답없음'}")
        rclpy.shutdown(); return
    dets = list(resp.grid.detections)
    print(f"검출 {len(dets)}개  ({resp.message})")
    pts = [(int(d.pixel_u), int(d.pixel_v)) for d in dets]

    # --- depth 프레임 N개 모으기 + intrinsic ---
    print(f"depth {args.frames}프레임 수집 중...")
    import time
    t0 = time.time()
    while (len(frames) < args.frames or not K) and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(frames) < 3 or not K:
        print(f"수집 부족 (frames={len(frames)}, K={bool(K)})")
        rclpy.shutdown(); return
    print(f"수집 완료: {len(frames)}프레임, fx={K['fx']:.1f} fy={K['fy']:.1f} "
          f"cx={K['cx']:.1f} cy={K['cy']:.1f}")

    def depth_at(d, u, v, win):
        h, w = d.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return np.nan
        r = win // 2
        patch = d[max(0, v-r):v+r+1, max(0, u-r):u+r+1].ravel()
        valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 3.0)]
        return float(np.median(valid)) if valid.size else np.nan

    print("\n=== 교차점별 depth 노이즈 (N프레임) ===")
    print(f"{'#':>2} {'pixel(u,v)':>12} {'raw평균(mm)':>11} {'raw std':>8} "
          f"{'win평균':>9} {'win std':>8} {'3D std(X,Y,Z)mm':>18}")
    summary = []
    for i, (u, v) in enumerate(pts):
        raws, wins = [], []
        for d in frames:
            h, w = d.shape[:2]
            rv = d[v, u] if (0 <= u < w and 0 <= v < h) else np.nan
            if np.isfinite(rv) and 0.05 < rv < 3.0:
                raws.append(rv * 1000.0)   # m→mm
            wv = depth_at(d, u, v, args.win)
            if np.isfinite(wv):
                wins.append(wv * 1000.0)
        if len(wins) < 3:
            print(f"{i+1:>2} {str((u,v)):>12}   (유효 depth 부족: {len(wins)})")
            continue
        raws = np.array(raws); wins = np.array(wins)
        # 3D 역투영 (win 중앙값 depth 사용), 프레임별 점 → std
        P = []
        for zmm in wins:
            Z = zmm
            X = (u - K['cx']) * Z / K['fx']
            Y = (v - K['cy']) * Z / K['fy']
            P.append([X, Y, Z])
        P = np.array(P)
        p_std = P.std(axis=0)
        rstd = raws.std() if len(raws) > 2 else float('nan')
        print(f"{i+1:>2} {str((u,v)):>12} {raws.mean():>11.1f} {rstd:>8.1f} "
              f"{wins.mean():>9.1f} {wins.std():>8.1f} "
              f"({p_std[0]:.1f},{p_std[1]:.1f},{p_std[2]:.1f})")
        summary.append((wins.std(), np.linalg.norm(p_std)))

    if summary:
        wstd = np.array([s[0] for s in summary])
        pstd = np.array([s[1] for s in summary])
        print(f"\n=== 요약 ===")
        print(f"win depth std: 평균 {wstd.mean():.1f}mm, 최대 {wstd.max():.1f}mm")
        print(f"3D점 std(노름): 평균 {pstd.mean():.1f}mm, 최대 {pstd.max():.1f}mm")
        print(f"\n판정: 3D std < ~3mm → 대안1 충분 / ~cm → 대안2(시차) 필요")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
