#!/usr/bin/env python3
"""RC 배근에서 Orbbec depth 사용 가능성 자체검증 (호모그래피 XY + depth Z 하이브리드 타당성).

예전에 depth를 접은 지표들(커버리지·반복성·평면 깨끗함)을 Orbbec+RC 조합에서 재측정하고,
결정적으로 **교차점이 철근평면 위로 얼마나 솟아있는지(=depth-Z 가치)** 를 잰다.

측정 항목:
  1) depth 커버리지 (전체 / 교차점 ROI)          ← 예전 33%→82% 이슈 재확인
  2) 교차점별 depth 반복성 (N프레임 std)          ← 예전 1~4mm, 정확도≠반복성
  3) 철근평면 RANSAC 적합 (법선/틸트/RMS 잔차)     ← 평면 깨끗한가
  4) 교차점의 평면 위 높이 (signed dist)           ← 예전 "교차점=평면보다 5~11mm 고점"
  5) (참고) 호모그래피 로봇 XY                     ← 현재 파이프라인이 쓰는 값

전제: depth_registration:true (컬러=뎁스 정합, 같은 내참). robot-control 자동 기동 카메라 사용.

  python3 tools/orbbec/depth_quality_test.py
     [SPACE] 최근 N프레임으로 전체 분석 + 리포트/이미지 저장   [q/ESC] 종료
     --frames 30  --conf 0.3  --plane-thr 5.0  --save-dir data/orbbec_depth_test
"""
import os
import argparse
import numpy as np
import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt'
HOMO = '/home/koceti/ros2_ws/data/calibration/homography_orbbec.yaml'
COLOR_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/depth/image_raw'
INFO_TOPIC = '/camera/color/camera_info'
YRANGE = {'r': (0.0, 142.0), 'l': (124.0, 288.0)}
XMAX = 411.0


def apply_H(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return p[0] / p[2], p[1] / p[2]


def backproject(u, v, z, fx, fy, cx, cy):
    """픽셀(u,v)+깊이z(mm) → 카메라프레임 3D(mm)."""
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], float)


def patch_depth(depth, u, v, r=2):
    """(u,v) 주변 (2r+1)^2 유효 depth 중앙값(mm). 없으면 0."""
    h, w = depth.shape
    y0, y1 = max(0, v - r), min(h, v + r + 1)
    x0, x1 = max(0, u - r), min(w, u + r + 1)
    win = depth[y0:y1, x0:x1]
    win = win[win > 0]
    return float(np.median(win)) if win.size else 0.0


def ransac_plane(pts, iters=600, thr=5.0, seed=0):
    """pts Nx3(mm) → (normal, d, inlier_mask, rms_mm). 평면: n·p + d = 0."""
    rng = np.random.default_rng(seed)
    n = len(pts)
    best_inl, best_cnt = None, -1
    for _ in range(iters):
        i = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[i]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-6:
            continue
        nrm = nrm / ln
        d = -nrm.dot(p0)
        dist = np.abs(pts @ nrm + d)
        inl = dist < thr
        c = int(inl.sum())
        if c > best_cnt:
            best_cnt, best_inl = c, inl
    # 인라이어로 SVD 재적합
    ip = pts[best_inl]
    c = ip.mean(0)
    _, _, vt = np.linalg.svd(ip - c)
    nrm = vt[2]
    if nrm[2] < 0:
        nrm = -nrm
    d = -nrm.dot(c)
    dist = pts @ nrm + d
    rms = float(np.sqrt((dist[best_inl] ** 2).mean()))
    return nrm, d, best_inl, rms


def detect_crossings(model, img, conf):
    """YOLO 교차점 검출 → 근접(<25px) 병합된 픽셀 중심 리스트."""
    r = model(img, conf=conf, verbose=False)[0]
    cen = [((float(b.xyxy[0][0] + b.xyxy[0][2]) / 2),
            (float(b.xyxy[0][1] + b.xyxy[0][3]) / 2)) for b in r.boxes]
    if not cen:
        return []
    arr = np.array(cen)
    used = np.zeros(len(arr), bool)
    out = []
    for i in range(len(arr)):
        if used[i]:
            continue
        near = (np.linalg.norm(arr - arr[i], axis=1) < 25) & ~used
        out.append(arr[near].mean(0))
        used |= near
    return [(int(u), int(v)) for u, v in out]


def analyze(color_buf, depth_buf, K, H, model, conf, plane_thr, save_dir):
    """최근 프레임 버퍼로 전체 분석 → 콘솔 리포트 + 이미지 저장."""
    fx, fy, cx, cy = K
    color = color_buf[-1]
    depth = depth_buf[-1].astype(np.float32)
    h, w = depth.shape

    # (1) 커버리지 (전체)
    cov_full = 100.0 * (depth > 0).mean()

    # 교차점 검출
    cros = detect_crossings(model, color, conf)
    if not cros:
        print('  [!] 교차점 미검출 — 카메라/조명/모델 확인');
    us = [u for u, _ in cros]; vs = [v for _, v in cros]

    # 교차점 ROI (검출점 바운딩 + 여유) → 커버리지
    cov_roi = None
    roi = None
    if cros:
        m = 60
        x0, x1 = max(0, min(us) - m), min(w, max(us) + m)
        y0, y1 = max(0, min(vs) - m), min(h, max(vs) + m)
        roi = (x0, y0, x1, y1)
        d_roi = depth[y0:y1, x0:x1]
        cov_roi = 100.0 * (d_roi > 0).mean()

    # (2) 교차점별 반복성 (N프레임 patch 중앙값 std)
    rep = []  # (u,v, z_med_mm, std_mm)
    for (u, v) in cros:
        zs = [patch_depth(dp.astype(np.float32), u, v) for dp in depth_buf]
        zs = [z for z in zs if z > 0]
        if len(zs) >= 2:
            rep.append((u, v, float(np.median(zs)), float(np.std(zs))))
        else:
            rep.append((u, v, zs[0] if zs else 0.0, float('nan')))

    # (3a) ROI 지배평면 RANSAC — depth 정밀도/노이즈 지표 (배경 포함일 수 있음)
    roi_plane = None
    if roi:
        x0, y0, x1, y1 = roi
        ys, xs = np.where(depth[y0:y1, x0:x1] > 0)
        if len(xs) >= 100:
            xs_f = xs + x0; ys_f = ys + y0
            zs_f = depth[ys_f, xs_f]
            pts = np.stack([(xs_f - cx) * zs_f / fx,
                            (ys_f - cy) * zs_f / fy, zs_f], 1)
            if len(pts) > 8000:                    # 속도 위해 서브샘플
                sel = np.random.default_rng(0).choice(len(pts), 8000, False)
                pts = pts[sel]
            nrm, d, inl, rms = ransac_plane(pts, thr=plane_thr)
            tilt = np.degrees(np.arccos(abs(nrm[2])))
            roi_plane = (rms, 100.0 * inl.mean(), tilt)

    # 불량 교차점 필터: depth 있고 반복성 양호(<10mm)한 것만 "유효"
    STD_OK = 10.0
    valid_idx = [i for i, (u, v, z, sd) in enumerate(rep)
                 if z > 0 and (sd == sd) and sd < STD_OK]
    cross3d = np.array([backproject(rep[i][0], rep[i][1], rep[i][2], fx, fy, cx, cy)
                        for i in valid_idx]) if valid_idx else np.empty((0, 3))

    # (3b) 결속층 평면 = "교차점들 자체"로 적합 → 결정지표
    #   교차점이 한 평면에 놓이면(작은 RMS) 단층 → 호모그래피 XY 유효, depth-Z 이득 작음.
    #   높이가 흩어지면(큰 RMS) 다층/요철 → 호모그래피 열화 + 고정 z_down 취약 → depth-Z 필요.
    layer = None
    dev = None      # 각 유효 교차점의 결속층 평면 이탈량(mm, 부호)
    if len(cross3d) >= 4:
        c = cross3d.mean(0)
        _, _, vt = np.linalg.svd(cross3d - c)
        nrm = vt[2]
        if nrm[2] < 0:
            nrm = -nrm
        d = -nrm.dot(c)
        dev = cross3d @ nrm + d
        rms = float(np.sqrt((dev ** 2).mean()))
        tilt = np.degrees(np.arccos(abs(nrm[2])))
        layer = (nrm, rms, float(np.abs(dev).max()), tilt)

    # (5) 호모그래피 로봇 XY (참고)
    homo_xy = []
    for (u, v, z, _) in rep:
        best = None
        for pose in ('r', 'l'):
            if H.get(pose) is None:
                continue
            x, y = apply_H(H[pose], u, v)
            ymin, ymax = YRANGE[pose]
            if ymin <= y <= ymax and 0 <= x <= XMAX:
                best = (pose, x, y); break
        homo_xy.append(best)

    dev_of = {vi: float(dev[k]) for k, vi in enumerate(valid_idx)} if dev is not None else {}

    # ── 콘솔 리포트 ──
    print('\n' + '=' * 70)
    print('  Orbbec depth 타당성 리포트 (RC)')
    print('=' * 70)
    print(f'  프레임 {len(depth_buf)}장 | 해상도 {w}x{h} | 교차점 {len(cros)}개 '
          f'(유효 {len(valid_idx)})')
    print(f'  [1] 커버리지  전체 {cov_full:5.1f}%'
          + (f'  |  교차점ROI {cov_roi:5.1f}%' if cov_roi is not None else ''))
    if roi_plane:
        rms, inlpct, tilt = roi_plane
        print(f'  [3a] ROI 지배평면(배경포함 가능) RMS {rms:4.2f}mm '
              f'인라이어 {inlpct:4.1f}% 틸트 {tilt:4.1f}°  ← depth 노이즈 지표')
    print(f'  {"px(u,v)":>12} | {"depthZ":>8} | {"반복std":>8} | '
          f'{"층이탈":>8} | 호모그래피robotXY')
    print('  ' + '-' * 66)
    for i, (u, v, z, sd) in enumerate(rep):
        sd_s = f'{sd:6.2f}mm' if sd == sd else '   n/a  '
        dv_s = f'{dev_of[i]:+6.1f}mm' if i in dev_of else ('  제외 ' if z <= 0 or (sd == sd and sd >= STD_OK) else '   -   ')
        hx = homo_xy[i]
        hx_s = f'{hx[0]}({hx[1]:.0f},{hx[2]:.0f})' if hx else 'out-of-range'
        print(f'  ({u:4d},{v:4d}) | {z:6.0f}mm | {sd_s} | {dv_s} | {hx_s}')
    print('  ' + '-' * 66)

    valid_sd = [rep[i][3] for i in valid_idx]
    if valid_sd:
        print(f'  [2] 반복성 중앙값 {np.median(valid_sd):.2f}mm '
              f'(예전 ZED 1~4mm; depth 정밀도)')
    if layer:
        nrm, rms, maxdev, tilt = layer
        print(f'  [4] 결속층 평면(교차점 적합)  coplanarity RMS {rms:4.2f}mm  '
              f'최대이탈 {maxdev:4.1f}mm  틸트 {tilt:4.1f}°')
    else:
        print('  [4] 결속층 평면  유효 교차점 부족(<4)으로 미산출')
    print('=' * 70)
    print('  판정 가이드:')
    print('   · 커버리지>80% & 반복성<5mm & ROI평면RMS<8mm  → depth 품질 충분')
    print('   · 결속층 coplanarity RMS 작음(≈수mm) → 단층: 호모그래피 XY로 충분')
    print('   · 결속층 RMS/최대이탈 큼(>1~2cm) → 다층/요철: depth-Z(점별 하강) 필요')

    # ── 이미지 저장 ──
    os.makedirs(save_dir, exist_ok=True)
    dvis = depth.copy()
    dn = np.clip((dvis - 100) / (1000 - 100), 0, 1)
    dcol = cv2.applyColorMap((dn * 255).astype(np.uint8), cv2.COLORMAP_JET)
    dcol[depth <= 0] = 0
    over = color.copy()
    if roi:
        cv2.rectangle(over, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
    for i, (u, v, z, sd) in enumerate(rep):
        hh = dev_of.get(i)
        cv2.circle(over, (u, v), 12, (0, 165, 255), 2)
        lab = f'{z:.0f}'
        if hh is not None:
            lab += f' d{hh:+.0f}'
        cv2.putText(over, lab, (u + 10, v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        cv2.circle(dcol, (u, v), 12, (255, 255, 255), 2)
    cv2.imwrite(os.path.join(save_dir, 'depth_test_color.png'), over)
    cv2.imwrite(os.path.join(save_dir, 'depth_test_depth.png'), dcol)
    print(f'  저장: {save_dir}/depth_test_color.png , depth_test_depth.png\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=30, help='반복성/분석 프레임 수')
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--plane-thr', type=float, default=5.0, help='RANSAC 인라이어 임계(mm)')
    ap.add_argument('--save-dir', default='/home/koceti/ros2_ws/data/orbbec_depth_test')
    ap.add_argument('--no-gui', action='store_true', help='창 없이 뜨면 바로 분석 후 종료')
    args = ap.parse_args()

    hom = yaml.safe_load(open(HOMO)) if os.path.exists(HOMO) else {}
    H = {'r': np.array(hom['right']['H']) if 'right' in hom else None,
         'l': np.array(hom['left']['H']) if 'left' in hom else None}
    print(f'호모그래피: 우측{"O" if H["r"] is not None else "X"} '
          f'좌측{"O" if H["l"] is not None else "X"}  |  모델 로딩...')

    rclpy.init()
    node = Node('depth_quality_test')
    br = CvBridge()
    color_buf, depth_buf = [], []
    K = [0, 0, 0, 0]

    def info_cb(m):
        K[0], K[1], K[2], K[3] = m.k[0], m.k[4], m.k[2], m.k[5]
    node.create_subscription(CameraInfo, INFO_TOPIC, info_cb, qos_profile_sensor_data)

    cs = Subscriber(node, Image, COLOR_TOPIC, qos_profile=qos_profile_sensor_data)
    ds = Subscriber(node, Image, DEPTH_TOPIC, qos_profile=qos_profile_sensor_data)

    def frame_cb(cm, dm):
        color_buf.append(br.imgmsg_to_cv2(cm, 'bgr8'))
        d = br.imgmsg_to_cv2(dm, desired_encoding='passthrough')
        depth_buf.append(d.astype(np.uint16) if d.dtype != np.uint16 else d)
        if len(color_buf) > args.frames:
            del color_buf[0]; del depth_buf[0]
    sync = ApproximateTimeSynchronizer([cs, ds], queue_size=5, slop=0.05)
    sync.registerCallback(frame_cb)

    model = YOLO(MODEL)
    print('준비 완료. depth_registration:true 확인했는지 체크. '
          '[SPACE]=분석  [q]=종료')

    gui = not args.no_gui
    win = 'depth quality test (SPACE=analyze q=quit)'
    if gui:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 1280, 800)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if not color_buf or K[0] == 0:
                continue
            if gui:
                disp = color_buf[-1].copy()
                cv2.putText(disp, f'buffer {len(depth_buf)}/{args.frames}  '
                            f'cov {100.0*(depth_buf[-1]>0).mean():.0f}%  SPACE=analyze',
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow(win, disp)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord('q'), 27):
                    break
                elif k == ord(' ') and len(depth_buf) >= 2:
                    analyze(color_buf, depth_buf, K, H, model,
                            args.conf, args.plane_thr, args.save_dir)
            else:
                if len(depth_buf) >= args.frames:
                    analyze(color_buf, depth_buf, K, H, model,
                            args.conf, args.plane_thr, args.save_dir)
                    break
    finally:
        if gui:
            cv2.destroyAllWindows()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
