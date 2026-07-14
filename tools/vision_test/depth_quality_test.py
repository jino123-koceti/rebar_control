#!/usr/bin/env python3
"""
ZED depth 품질(구멍률) 정량 비교 테스트 — 데크플레이트 반사면 대응

목적:
  depth_mode를 NEURAL_LIGHT vs NEURAL_PLUS 로 바꿔가며,
  데크플레이트/철근 작업영역에서 "유효 depth 픽셀 비율"이 얼마나 개선되는지 수치로 비교.

워크플로우:
  1) 현재 모드(NEURAL_LIGHT)로 실행:
       python3 depth_quality_test.py --camera right --label neural_light
  2) src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml 의
       depth_mode: 'NEURAL_LIGHT'  →  'NEURAL_PLUS'  로 수정 후
       sudo systemctl restart robot-control
  3) 새 모드로 다시 실행:
       python3 depth_quality_test.py --camera right --label neural_plus
  4) 두 번째 실행 시 자동으로 비교표 출력 (results json 누적)

측정 항목:
  - 전체 프레임 유효 depth 비율(%)  (유효 = NaN/inf 아님 & min~max 범위 내)
  - 세로 5밴드별 유효율 (상단=먼 쪽, foreshortening 심한 영역)
  - 유효 depth 통계(mean/std/min/max, m)
  - 시각화 PNG: depth 컬러맵 + 구멍 마스크(빨강)

사용법:
  python3 depth_quality_test.py --camera right --label neural_light
  python3 depth_quality_test.py --camera both  --label neural_plus --frames 50
"""
import argparse
import json
import os
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


WS_DIR = '/home/koceti/ros2_ws'
OUT_DIR = os.path.join(WS_DIR, 'data', 'images', 'depth_quality')
RESULTS_JSON = os.path.join(OUT_DIR, 'depth_quality_results.json')

# 검출 노드와 동일한 유효 depth 범위 (m)
MIN_DEPTH_M = 0.05
MAX_DEPTH_M = 2.0
N_BANDS = 5

CAMERA_CONFIG = {
    'right': {
        'name': 'zedxmini2 (right)',
        'depth_topic': '/zedxmini2/zed_node/depth/depth_registered',
        'rgb_topic': '/zedxmini2/zed_node/left/image_rect_color',
    },
    'left': {
        'name': 'zedxmini1 (left)',
        'depth_topic': '/zedxmini1/zed_node/depth/depth_registered',
        'rgb_topic': '/zedxmini1/zed_node/left/image_rect_color',
    },
}


class DepthQualityTester(Node):
    def __init__(self, camera, n_frames):
        super().__init__('depth_quality_tester')
        self.bridge = CvBridge()
        self.n_frames = n_frames
        self.cfg = CAMERA_CONFIG[camera]
        self.camera = camera

        self.depth_frames = []     # 수집된 depth np 배열
        self.latest_rgb = None
        self.done = False

        # ZED 이미지/depth는 BEST_EFFORT(sensor_data) QoS로 발행됨
        self.create_subscription(Image, self.cfg['depth_topic'],
                                 self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.cfg['rgb_topic'],
                                 self._rgb_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"[{self.cfg['name']}] depth 토픽 대기: {self.cfg['depth_topic']}")
        self.get_logger().info(f"수집 목표: {n_frames} 프레임")

    def _rgb_cb(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _depth_cb(self, msg):
        if self.done:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().warn(f'depth 변환 실패: {e}')
            return
        self.depth_frames.append(np.asarray(depth, dtype=np.float32))
        n = len(self.depth_frames)
        if n % 10 == 0:
            self.get_logger().info(f'  수집 {n}/{self.n_frames}')
        if n >= self.n_frames:
            self.done = True

    @staticmethod
    def _valid_mask(depth):
        return np.isfinite(depth) & (depth > MIN_DEPTH_M) & (depth < MAX_DEPTH_M)

    def compute_stats(self):
        # 프레임별 유효율 평균 + 마지막 프레임 밴드 분석
        ratios = []
        for d in self.depth_frames:
            m = self._valid_mask(d)
            ratios.append(float(m.mean()) * 100.0)
        avg_valid = float(np.mean(ratios))
        std_valid = float(np.std(ratios))

        # 누적 유효 depth 값 통계
        all_valid_vals = []
        for d in self.depth_frames:
            m = self._valid_mask(d)
            all_valid_vals.append(d[m])
        all_valid_vals = (np.concatenate(all_valid_vals)
                          if all_valid_vals else np.array([]))

        # 세로 밴드별 유효율 (전 프레임 평균)
        h = self.depth_frames[0].shape[0]
        band_h = h // N_BANDS
        band_valid = []
        for b in range(N_BANDS):
            y0 = b * band_h
            y1 = (b + 1) * band_h if b < N_BANDS - 1 else h
            br = []
            for d in self.depth_frames:
                m = self._valid_mask(d[y0:y1, :])
                br.append(float(m.mean()) * 100.0)
            band_valid.append(round(float(np.mean(br)), 1))

        stats = {
            'camera': self.camera,
            'frames': len(self.depth_frames),
            'valid_pct': round(avg_valid, 1),
            'valid_pct_std': round(std_valid, 1),
            'band_valid_pct_top_to_bottom': band_valid,
            'depth_mean_m': (round(float(all_valid_vals.mean()), 3)
                             if all_valid_vals.size else None),
            'depth_std_m': (round(float(all_valid_vals.std()), 3)
                            if all_valid_vals.size else None),
            'depth_min_m': (round(float(all_valid_vals.min()), 3)
                            if all_valid_vals.size else None),
            'depth_max_m': (round(float(all_valid_vals.max()), 3)
                            if all_valid_vals.size else None),
        }
        return stats

    def save_visualization(self, label):
        os.makedirs(OUT_DIR, exist_ok=True)
        depth = self.depth_frames[-1]
        valid = self._valid_mask(depth)

        # depth 컬러맵 (유효 범위 정규화)
        vis = np.zeros((*depth.shape, 3), dtype=np.uint8)
        if valid.any():
            dn = np.clip((depth - MIN_DEPTH_M) / (MAX_DEPTH_M - MIN_DEPTH_M),
                         0, 1)
            dn = np.nan_to_num(dn, nan=0.0, posinf=0.0, neginf=0.0)
            dn8 = (dn * 255).astype(np.uint8)
            cm = cv2.applyColorMap(dn8, cv2.COLORMAP_JET)
            vis[valid] = cm[valid]
        # 구멍(무효 depth)은 빨강
        vis[~valid] = (0, 0, 255)

        out = os.path.join(OUT_DIR, f'depth_{self.camera}_{label}.png')
        cv2.imwrite(out, vis)

        # rgb 위에 구멍 오버레이도 저장 (가능하면)
        if self.latest_rgb is not None and self.latest_rgb.shape[:2] == depth.shape:
            ov = self.latest_rgb.copy()
            ov[~valid] = (0, 0, 255)
            blended = cv2.addWeighted(self.latest_rgb, 0.5, ov, 0.5, 0)
            out2 = os.path.join(OUT_DIR, f'overlay_{self.camera}_{label}.png')
            cv2.imwrite(out2, blended)
        return out


def print_report(stats, label):
    print('\n' + '=' * 60)
    print(f"[{stats['camera']}] depth 품질 — label='{label}'  "
          f"({stats['frames']} frames)")
    print('=' * 60)
    print(f"  전체 유효 depth 비율 : {stats['valid_pct']}%  "
          f"(±{stats['valid_pct_std']})")
    print(f"  세로밴드 유효율 (상단=먼쪽 → 하단=가까움):")
    bands = stats['band_valid_pct_top_to_bottom']
    for i, bv in enumerate(bands):
        bar = '#' * int(bv / 2)
        print(f"    밴드{i+1}: {bv:5.1f}%  {bar}")
    print(f"  유효 depth: mean={stats['depth_mean_m']}m "
          f"std={stats['depth_std_m']}m "
          f"range=[{stats['depth_min_m']}, {stats['depth_max_m']}]m")


def update_and_compare(stats, label):
    os.makedirs(OUT_DIR, exist_ok=True)
    data = {}
    if os.path.exists(RESULTS_JSON):
        try:
            with open(RESULTS_JSON) as f:
                data = json.load(f)
        except Exception:
            data = {}
    cam = stats['camera']
    data.setdefault(cam, {})[label] = stats
    with open(RESULTS_JSON, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    labels = list(data[cam].keys())
    if len(labels) < 2:
        return
    print('\n' + '=' * 60)
    print(f"[{cam}] 모드 비교")
    print('=' * 60)
    header = '  항목'.ljust(28) + ''.join(l.ljust(16) for l in labels)
    print(header)
    print(f"  전체 유효율(%)".ljust(28)
          + ''.join(f"{data[cam][l]['valid_pct']}".ljust(16) for l in labels))
    for i in range(N_BANDS):
        row = f"  밴드{i+1} 유효율(%)".ljust(28)
        for l in labels:
            row += f"{data[cam][l]['band_valid_pct_top_to_bottom'][i]}".ljust(16)
        print(row)
    # 개선폭 (마지막 두 label 비교)
    a, b = labels[-2], labels[-1]
    diff = data[cam][b]['valid_pct'] - data[cam][a]['valid_pct']
    print(f"\n  → {b} - {a} : 전체 유효율 {diff:+.1f}%p")


def run_one(camera, label, n_frames, wait_timeout=20.0):
    node = DepthQualityTester(camera, n_frames)
    try:
        t0 = time.time()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.time() - t0 > wait_timeout:
                node.get_logger().warn(
                    f'{wait_timeout}s 내 {n_frames} 프레임 미달 '
                    f'(수집 {len(node.depth_frames)}개) → 모인 만큼으로 분석')
                break
        if not node.depth_frames:
            node.get_logger().error(
                'depth 프레임을 못 받음 — 토픽/카메라/QoS 확인')
            return
        stats = node.compute_stats()
        png = node.save_visualization(label)
        print_report(stats, label)
        print(f"  시각화 저장: {png}")
        update_and_compare(stats, label)
    finally:
        node.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default='right',
                    choices=['right', 'left', 'both'])
    ap.add_argument('--label', required=True,
                    help="모드 라벨 (예: neural_light, neural_plus)")
    ap.add_argument('--frames', type=int, default=30)
    args = ap.parse_args()

    rclpy.init()
    cams = ['right', 'left'] if args.camera == 'both' else [args.camera]
    for cam in cams:
        run_one(cam, args.label, args.frames)
    rclpy.shutdown()
    print(f"\n결과 누적: {RESULTS_JSON}")


if __name__ == '__main__':
    main()
