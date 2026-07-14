#!/usr/bin/env python3
"""검출 교차점 각각에서 윈도우 depth '분포'를 본다.
철근만 보면 단봉(unimodal), 틈으로 철판까지 섞이면 쌍봉(near=철근/far=철판).
→ '최근접 표면 depth'가 효과 있을지 직접 판정."""
import argparse, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rebar_base_interfaces.srv import DetectCrossings

CAM = {'right': {'sel': 2, 'ns': 'zedxmini2'}, 'left': {'sel': 1, 'ns': 'zedxmini1'}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='right', choices=['right', 'left'])
    ap.add_argument('--frames', type=int, default=20)
    args = ap.parse_args()
    cfg = CAM[args.camera]; ns = cfg['ns']
    rclpy.init(); node = Node('probe'); br = CvBridge()
    frames = []
    node.create_subscription(
        Image, f'/{ns}/zed_node/depth/depth_registered',
        lambda m: frames.append(br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)),
        qos_profile_sensor_data)
    cli = node.create_client(DetectCrossings, '/rebar/detect_crossings')
    cli.wait_for_service(timeout_sec=5.0)
    req = DetectCrossings.Request()
    req.camera_selection = cfg['sel']; req.confidence_threshold = 0.4; req.expected_count = 6
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    r = fut.result()
    pts = [(int(d.pixel_u), int(d.pixel_v)) for d in r.grid.detections] if r and r.success else []
    print(f"검출 {len(pts)}개")
    t0 = time.time()
    while len(frames) < args.frames and time.time()-t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    stack = np.stack(frames[-args.frames:]) * 1000.0  # mm

    def win(u, v, w):
        r = w//2; vals = []
        for d in stack:
            p = d[max(0,v-r):v+r+1, max(0,u-r):u+r+1].ravel()
            vals.extend(p[np.isfinite(p) & (p>50) & (p<3000)].tolist())
        return np.array(vals)

    for i, (u, v) in enumerate(pts):
        print(f"\n=== P{i+1} pixel=({u},{v}) ===")
        for w in [3, 9, 21]:
            vals = win(u, v, w)
            if vals.size < 5:
                print(f"  win{w:2d}x{w:<2d}: 샘플부족"); continue
            pcts = np.percentile(vals, [0, 10, 50, 90, 100])
            # 쌍봉 탐지: 히스토그램 갭
            hist, edges = np.histogram(vals, bins=20)
            near = vals[vals < np.median(vals)]
            far = vals[vals >= np.median(vals)]
            spread = pcts[4]-pcts[0]
            print(f"  win{w:2d}x{w:<2d}: n={vals.size:5d}  "
                  f"min={pcts[0]:5.0f} p10={pcts[1]:5.0f} med={pcts[2]:5.0f} "
                  f"p90={pcts[3]:5.0f} max={pcts[4]:5.0f}  폭={spread:4.0f}mm  "
                  f"std={vals.std():4.0f}")
        # 최근접 클러스터(철근면) 추정: 최소값 부근 band
        vals = win(u, v, 9)
        if vals.size >= 5:
            near_band = vals[vals < vals.min() + 25]  # 가장 가까운 25mm
            print(f"  → 최근접표면(철근) 추정: {np.median(near_band):.0f}mm "
                  f"(n={near_band.size}) vs 전체median {np.median(vals):.0f}mm  "
                  f"차이 {np.median(vals)-np.median(near_band):.0f}mm")
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
