#!/usr/bin/env python3
"""고정 교차점 뎁스 반복성(노이즈) 측정.
검출 서비스로 교차점 픽셀을 한 번 잡고, 그 픽셀들의 depth를 N프레임 모아
시간축 std(센서 노이즈)와 3D 위치 std를 보고. (장면 정지 전제)

사용: python3 depth_repeatability.py --camera right --frames 60
"""
import argparse, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rebar_base_interfaces.srv import DetectCrossings

CAM = {'right': {'sel': 2, 'ns': 'zedxmini2'},
       'left':  {'sel': 1, 'ns': 'zedxmini1'}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='right', choices=['right', 'left'])
    ap.add_argument('--frames', type=int, default=60)
    ap.add_argument('--kernel', type=int, default=5, help='픽셀 주변 중앙값 커널')
    args = ap.parse_args()
    cfg = CAM[args.camera]; ns = cfg['ns']

    rclpy.init(); node = Node('depth_repeat'); br = CvBridge()
    depth_frames = []; K = {}
    node.create_subscription(
        Image, f'/{ns}/zed_node/depth/depth_registered',
        lambda m: depth_frames.append(br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)),
        qos_profile_sensor_data)
    node.create_subscription(
        CameraInfo, f'/{ns}/zed_node/left/camera_info',
        lambda m: K.update({'fx': m.k[0], 'fy': m.k[4], 'cx': m.k[2], 'cy': m.k[5]}),
        qos_profile_sensor_data)

    # intrinsic + 첫 프레임 대기
    print('카메라 정보/프레임 대기...')
    t = time.time()
    while (not K or not depth_frames) and time.time() - t < 8:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not K:
        print('camera_info 못받음'); rclpy.shutdown(); return
    print(f'intrinsic fx={K["fx"]:.1f} fy={K["fy"]:.1f} cx={K["cx"]:.1f} cy={K["cy"]:.1f}')

    # 교차점 검출 (픽셀 고정용)
    cli = node.create_client(DetectCrossings, '/rebar/detect_crossings')
    pts = []
    if cli.wait_for_service(timeout_sec=3.0):
        req = DetectCrossings.Request()
        req.camera_selection = cfg['sel']; req.confidence_threshold = 0.4
        req.expected_count = 6
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        r = fut.result()
        if r and r.success:
            pts = [(int(d.pixel_u), int(d.pixel_v)) for d in r.grid.detections]
    if not pts:
        print('교차점 검출 실패 — 화면 중앙 픽셀로 대체')
        h, w = depth_frames[-1].shape; pts = [(w // 2, h // 2)]
    print(f'고정 교차점 {len(pts)}개: {pts}')

    # N프레임 수집
    depth_frames.clear()
    print(f'{args.frames}프레임 depth 수집...')
    t = time.time()
    while len(depth_frames) < args.frames and time.time() - t < 30:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'수집 {len(depth_frames)}프레임\n')

    half = args.kernel // 2
    print(f"{'교차점(u,v)':>16} {'유효%':>6} {'평균mm':>8} {'std_mm':>7} "
          f"{'min':>6} {'max':>6} {'3D_std(x,y,z)mm':>18}")
    print('-' * 80)
    for (u, v) in pts:
        depths = []
        cam_pts = []
        for f in depth_frames:
            patch = f[max(0, v - half):v + half + 1, max(0, u - half):u + half + 1] * 1000.0
            patch = patch[np.isfinite(patch) & (patch > 50) & (patch < 3000)]
            if patch.size == 0:
                continue
            z = float(np.median(patch))
            depths.append(z)
            # 3D 역투영 (카메라 프레임)
            X = (u - K['cx']) * z / K['fx']
            Y = (v - K['cy']) * z / K['fy']
            cam_pts.append([X, Y, z])
        if len(depths) < 2:
            print(f"({u:4d},{v:4d})  유효프레임 부족({len(depths)})")
            continue
        d = np.array(depths); cp = np.array(cam_pts)
        valid_pct = len(depths) / len(depth_frames) * 100
        s3 = cp.std(axis=0)
        print(f"({u:4d},{v:4d}) {valid_pct:6.0f} {d.mean():8.1f} {d.std():7.2f} "
              f"{d.min():6.0f} {d.max():6.0f}   "
              f"({s3[0]:.2f},{s3[1]:.2f},{s3[2]:.2f})")
    print('\nstd_mm = 그 교차점 거리(depth)의 시간축 표준편차 = 센서 반복성 노이즈')
    print('3D_std = 역투영한 카메라좌표 X,Y,Z의 std (위치 반복성)')

    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
