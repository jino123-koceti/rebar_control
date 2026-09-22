#!/usr/bin/env python3
"""우측/좌측 카메라 depth 맵 플롯 (mm 컬러바 + 검출 교차점 오버레이).

사용법: python3 plot_depth.py --camera right
저장:   data/calibration/depth_map_{cam}.png
"""
import argparse, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from rebar_base_interfaces.srv import DetectCrossings
    HAVE_SRV = True
except Exception:
    HAVE_SRV = False

CAM = {'right': {'sel': 2, 'ns': 'zedxmini2'},
       'left':  {'sel': 1, 'ns': 'zedxmini1'}}
OUT = '/home/koceti/ros2_ws/data/calibration'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='right', choices=['right', 'left'])
    ap.add_argument('--frames', type=int, default=15)
    ap.add_argument('--tag', default='', help='파일명 접미사 (예: sheet → depth_map_right_sheet.png)')
    ap.add_argument('--flip', action='store_true', help='상하 반전(가까운쪽이 위로)')
    args = ap.parse_args()
    cfg = CAM[args.camera]; ns = cfg['ns']

    rclpy.init()
    node = Node('plot_depth')
    br = CvBridge()
    frames = []
    node.create_subscription(
        Image, f'/{ns}/zed_node/depth/depth_registered',
        lambda m: frames.append(br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)),
        qos_profile_sensor_data)

    print(f'depth {args.frames}프레임 수집...')
    t0 = time.time()
    while len(frames) < args.frames and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not frames:
        print('depth 못 받음'); rclpy.shutdown(); return

    # 시간 중앙값 (mm)
    stack = np.stack(frames[-args.frames:])
    stack[~np.isfinite(stack)] = np.nan
    depth_m = np.nanmedian(stack, axis=0)
    depth_mm = depth_m * 1000.0
    valid = np.isfinite(depth_mm) & (depth_mm > 50) & (depth_mm < 3000)

    # 검출 교차점 (선택)
    pts = []
    if HAVE_SRV:
        cli = node.create_client(DetectCrossings, '/rebar/detect_crossings')
        if cli.wait_for_service(timeout_sec=3.0):
            req = DetectCrossings.Request()
            req.camera_selection = cfg['sel']; req.confidence_threshold = 0.4
            req.expected_count = 6
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            r = fut.result()
            if r and r.success:
                pts = [(int(d.pixel_u), int(d.pixel_v)) for d in r.grid.detections]

    # 통계
    vd = depth_mm[valid]
    print(f"유효픽셀 {valid.mean()*100:.0f}%  depth(mm) min {vd.min():.0f} "
          f"max {vd.max():.0f} 중앙 {np.median(vd):.0f}")

    # 플롯: 작업영역 범위로 컬러 클립
    lo, hi = np.percentile(vd, [2, 98])
    disp = np.where(valid, depth_mm, np.nan)
    H = disp.shape[0]
    if args.flip:
        disp = np.flipud(disp)
        depth_mm = np.flipud(depth_mm)
        valid = np.flipud(valid)
        pts = [(u, H - 1 - v) for (u, v) in pts]
    fig, ax = plt.subplots(figsize=(12, 7))
    im = ax.imshow(disp, cmap='turbo', vmin=lo, vmax=hi)
    cb = fig.colorbar(im, ax=ax, fraction=0.035); cb.set_label('depth (mm)')
    for i, (u, v) in enumerate(pts):
        z = depth_mm[v, u] if valid[v, u] else np.nan
        ax.plot(u, v, 'wo', ms=8, mfc='none', mew=2)
        ax.text(u + 8, v, f"P{i+1}\n{z:.0f}", color='w', fontsize=8,
                bbox=dict(fc='black', alpha=0.5, pad=1))
    ttl = f'{ns} depth map ({len(frames)} frame median, {len(pts)} crossings)'
    if args.tag:
        ttl += f' [{args.tag}]'
    ax.set_title(ttl)
    suffix = f'_{args.tag}' if args.tag else ''
    path = f'{OUT}/depth_map_{args.camera}{suffix}.png'
    fig.savefig(path, dpi=120, bbox_inches='tight')
    print(f"저장: {path}")

    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
