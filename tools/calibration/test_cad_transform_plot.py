#!/usr/bin/env python3
"""CAD 변환행렬 테스트 — orbbec 교차점 → 결속건(EEF) 좌표 변환 → 플롯.

orbbec_localize.py 기반. 호모그래피 대신 **CAD로 계산한 camera→EEF 강체변환** 사용.
파이프라인:
  YOLO 교차점(u,v) → depth 근접밴드 샘플 → 컬러 intrinsic 역투영 → P_cam(카메라 광학3D)
  → P_eef = R·P_cam + t (CAD) → 이미지 오버레이 + matplotlib 플롯(결속건 좌표계)

CAD 좌표(XYZ 스테이지 홈 글로벌, Y-up)로부터 R,t를 스크립트에서 계산 → 값 바뀌면 상단만 수정.
추가로 **자세별 touch 오프셋(POSE_OFFSET)** 적용 → 최종 **스테이지 명령 좌표** 출력.

사전: gemini2L (depth_registration:=true) 실행 중
  python3 tools/calibration/test_cad_transform_plot.py --pose r [--deck-z 420] [--depth-z] [--frames 15]
    --pose r|l  : 결속 자세(오프셋 선택)   --deck-z: 단층 덱 하강 Z   --depth-z: 다층(depth-Z 사용)
저장: data/calibration/cad_transform_overlay.png (stage 좌표 오버레이),  cad_transform_plot.png,
      cad_offset_points.json (label/pixel/pcam/peef/stage)
"""
import os
import argparse
from collections import deque
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO

ROOT = '/home/koceti/ros2_ws'
MODEL = os.path.join(ROOT, 'src/rebar_vision/model/orbbec_crossing.pt')
SAVE = os.path.join(ROOT, 'data/calibration')

# ── CAD 좌표계 (XYZ 스테이지 홈 글로벌 기준, Y-up) ──  값 바뀌면 여기만 수정
CAM_AXANG = ([0.66, -0.38, 0.65], 138.03); CAM_POS = [-221.43, -18.07, -25.38]  # LCS002 카메라(광학)
EEF_AXANG = ([0.58, -0.58, 0.58], 120.0);  EEF_POS = [267.16, -266.50, -4.98]   # LCS001 결속건 EEF(신)
# 결속건 프레임 축을 스테이지 관례(X=근/원 리치, Y=좌/우)에 맞추는 하강축 중심 보정.
# +90° → near-far가 +X (P7=335가 X). Y부호가 다르면 -90/180 으로 조정.
EEF_AXIS_FIX_DEG = 90.0

# ── 자세별 touch 오프셋 (2026-08-05 4점 실측: P1/P6=r, P3/P4=l) ──
# stage_XY = P_eef_XY + POSE_OFFSET[pose].  tip이 yaw축에서 ~30mm 벗어나 있어
# 우↔좌(약 36° 회전) 시 tip XY가 shift → 자세별로 오프셋이 다름(X,Y 각 ~13mm 차이).
# 잔차(점간 σ): X~2mm, Y~1mm. 스케일≈1.0이라 회전/스케일 보정 불필요, 평행이동만.
POSE_OFFSET = {'r': (-50.8, 146.8), 'l': (-63.8, 164.4)}   # (ΔX, ΔY) mm
# l의 Y: 159.4 → 164.4 (+5mm, 2026-08-06 실주행 보정). orbbec_cad_transform.py와 동일 유지.
# 단층 덱: depth-Z는 노이즈(σ 8~15mm)라 안 쓰고 스테이지 절대 Z 상수로 하강.
DECK_Z_MM = 420.0            # 덱 하강 스테이지 절대 Z (--deck-z 로 조정)
DEPTH_Z_OFFSET_MM = 317.5    # --depth-z(다층) 시: stage_Z = P_eef_Z + 이 값 (자세 무관)


def axang(ax, deg):
    a = np.array(ax, float); a /= np.linalg.norm(a); th = np.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def _Rz(deg):
    t = np.radians(deg); c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def cad_transform():
    """CAD 배치값 → camera→EEF 강체변환 (P_eef = R·P_cam + t).
    EEF 프레임에 하강축 중심 축보정(EEF_AXIS_FIX_DEG) 적용해 스테이지 관례에 정렬."""
    Rc = axang(*CAM_AXANG); pc = np.array(CAM_POS)
    Re = axang(*EEF_AXANG) @ _Rz(EEF_AXIS_FIX_DEG); pe = np.array(EEF_POS)
    R = Re.T @ Rc
    t = Re.T @ (pc - pe)
    return R, t


def to_stage(peef, pose, deck_z, use_depth_z=False):
    """P_eef(EEF/결속건 프레임) → 스테이지 명령 좌표.
    자세별 XY 오프셋 적용 + Z는 단층 덱 상수(기본) 또는 depth-Z+오프셋(다층)."""
    dx, dy = POSE_OFFSET[pose]
    z = float(peef[2]) + DEPTH_Z_OFFSET_MM if use_depth_z else float(deck_z)
    return np.array([float(peef[0]) + dx, float(peef[1]) + dy, z])


def rebar_depth_mm(depth_bufs, u, v, r=4):
    """(u,v) 주변 근접밴드 depth(mm) — 철근 top (calibrate_3d_orbbec 방식)."""
    vals = []
    for d in depth_bufs:
        h, w = d.shape[:2]
        sub = d[max(0, v-r):min(h, v+r+1), max(0, u-r):min(w, u+r+1)]
        m = (sub > 50) & (sub < 3000)
        vals.extend(sub[m].tolist())
    if len(vals) < 10:
        return None
    near = np.array(vals); lo = np.percentile(near, 5)
    band = near[near < lo + 25.0]
    return float(band.mean() if band.size >= 3 else near.mean())


def detect_crossings(model, img, conf):
    r = model(img, conf=conf, verbose=False)[0]
    cen = [((float(b.xyxy[0][0]+b.xyxy[0][2])/2),
            (float(b.xyxy[0][1]+b.xyxy[0][3])/2)) for b in r.boxes]
    if not cen:
        return []
    arr = np.array(cen); used = np.zeros(len(arr), bool); out = []
    for i in range(len(arr)):
        if used[i]:
            continue
        near = (np.linalg.norm(arr - arr[i], axis=1) < 25) & ~used
        out.append(arr[near].mean(0)); used |= near
    return [(int(u), int(v)) for u, v in out]


def plot_eef(pts_stage, labels, path, pose='r'):
    """보정된 스테이지 좌표 플롯: top-view(X-Y) + 3D."""
    P = np.array(pts_stage)
    fig = plt.figure(figsize=(12, 5))
    # top-view (X-Y)
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.scatter(P[:, 0], P[:, 1], c='tab:orange', s=80, edgecolors='k')
    for (x, y, z), lb in zip(P, labels):
        ax1.annotate(lb, (x, y), textcoords="offset points", xytext=(6, 4), fontsize=9)
    ax1.set_xlabel('Stage X (mm)'); ax1.set_ylabel('Stage Y (mm)')
    ax1.set_title(f'Stage frame (corrected, pose={pose})  top-view (X-Y)')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal'); ax1.axhline(0, color='gray', lw=0.5); ax1.axvline(0, color='gray', lw=0.5)
    # 3D
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax2.scatter(P[:, 0], P[:, 1], P[:, 2], c='tab:blue', s=60)
    for (x, y, z), lb in zip(P, labels):
        ax2.text(x, y, z, lb, fontsize=8)
    ax2.set_xlabel('Stage X'); ax2.set_ylabel('Stage Y'); ax2.set_zlabel('Stage Z (down)')
    ax2.set_title(f'Stage frame (corrected, pose={pose})  3D')
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=15)
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--pose', choices=['r', 'l'], default='r',
                    help='결속 자세(오프셋 선택): r=우, l=좌')
    ap.add_argument('--deck-z', type=float, default=DECK_Z_MM,
                    help='단층 덱 하강 스테이지 절대 Z (기본 %(default)s)')
    ap.add_argument('--depth-z', action='store_true',
                    help='덱 상수 대신 depth-Z+오프셋 사용(다층)')
    ap.add_argument('--show', action='store_true', help='창 표시(디스플레이 필요)')
    args = ap.parse_args()

    R, t = cad_transform()
    print("=== CAD camera→EEF 변환 ===")
    print("R=\n", np.array2string(R, precision=4, suppress_small=True))
    print("t=", np.round(t, 1), "mm")
    dx, dy = POSE_OFFSET[args.pose]
    zdesc = f"depth-Z+{DEPTH_Z_OFFSET_MM}" if args.depth_z else f"덱상수 {args.deck_z}"
    print(f"자세={args.pose}  오프셋 XY=({dx:+.1f},{dy:+.1f})  Z={zdesc}")

    rclpy.init(); node = Node('cad_transform_test'); br = CvBridge()
    color = [None]; depth_buf = deque(maxlen=args.frames); K = [0, 0, 0, 0]

    def info_cb(m):
        if K[0] == 0 and m.k[0] > 0:
            K[0], K[1], K[2], K[3] = m.k[0], m.k[4], m.k[2], m.k[5]
    node.create_subscription(CameraInfo, '/camera/color/camera_info', info_cb, qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/color/image_raw',
                             lambda m: color.__setitem__(0, br.imgmsg_to_cv2(m, 'bgr8')),
                             qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/depth/image_raw',
                             lambda m: depth_buf.append(br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)),
                             qos_profile_sensor_data)

    model = YOLO(MODEL)
    print('워밍업(프레임 수집)...')
    import time
    t0 = time.time()
    while rclpy.ok() and (color[0] is None or len(depth_buf) < args.frames or K[0] == 0) and time.time()-t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.05)
    if color[0] is None or K[0] == 0:
        print('⚠ 카메라/내참 수신 실패'); node.destroy_node(); rclpy.shutdown(); return
    fx, fy, cx, cy = K

    img = color[0]
    cros = detect_crossings(model, img, args.conf)
    disp = img.copy()
    pts_stage, labels, records = [], [], []
    print(f"\n=== 교차점 {len(cros)}개 → P_cam → P_eef → stage(자세={args.pose}) ===")
    for i, (u, v) in enumerate(cros):
        Z = rebar_depth_mm(list(depth_buf), u, v)
        if Z is None:
            cv2.circle(disp, (u, v), 12, (120, 120, 120), 2)
            print(f"  P{i+1} px({u},{v}): depth 소실 → skip"); continue
        pcam = np.array([(u-cx)*Z/fx, (v-cy)*Z/fy, Z])
        peef = R @ pcam + t
        stage = to_stage(peef, args.pose, args.deck_z, args.depth_z)
        lb = f'P{i+1}'
        pts_stage.append(stage.tolist()); labels.append(lb)
        records.append({'label': lb, 'pixel': [u, v], 'pose': args.pose,
                        'pcam': [round(float(x), 2) for x in pcam],
                        'peef': [round(float(x), 2) for x in peef],
                        'stage': [round(float(x), 2) for x in stage]})
        col = (0, 165, 255)
        cv2.circle(disp, (u, v), 14, col, 3)
        cv2.putText(disp, f'{lb}({stage[0]:.0f},{stage[1]:.0f},{stage[2]:.0f})', (u+12, v-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        print(f"  {lb} px({u},{v}) Z{Z:.0f} | P_eef({peef[0]:.1f},{peef[1]:.1f},{peef[2]:.1f}) "
              f"→ stage({stage[0]:.1f},{stage[1]:.1f},{stage[2]:.1f})mm")

    os.makedirs(SAVE, exist_ok=True)
    # 점 정보(라벨/픽셀/P_cam/P_eef) JSON 저장 → 보정 fit에서 라벨-P_cam 연결 고정
    import json
    pts_json = os.path.join(SAVE, 'cad_offset_points.json')
    json.dump(records, open(pts_json, 'w'), indent=1, ensure_ascii=False)
    ov = os.path.join(SAVE, 'cad_transform_overlay.png')
    cv2.imwrite(ov, disp); print(f"\n이미지 오버레이 저장: {ov}")
    print(f"점 정보 저장: {pts_json}")
    if pts_stage:
        pp = os.path.join(SAVE, 'cad_transform_plot.png')
        plot_eef(pts_stage, labels, pp, args.pose); print(f"좌표 플롯 저장: {pp}")
        print(f"\n→ 위 stage 좌표로 이동 + 자세 '{args.pose}' 명령해서 tip이 교차점에 닿는지 확인")
    else:
        print("⚠ 변환된 교차점 없음")

    if args.show:
        cv2.imshow('CAD transform (q=quit)', disp)
        while cv2.waitKey(100) & 0xFF not in (ord('q'), 27):
            pass
        cv2.destroyAllWindows()
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
