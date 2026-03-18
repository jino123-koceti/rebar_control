#!/usr/bin/env python3
"""
VSLAM 특징점 시각화 스크립트
ZED X 주행 카메라(zed_front, zed_back) 이미지에서 ORB 특징점을 추출하고
프레임 간 매칭을 시각화합니다.

장비 구성:
  - zed_front (ZED X, SN: 45320958): 물리적 전방 장착, 후진시 VSLAM 사용
  - zed_back  (ZED X, SN: 46674448): 물리적 후방 장착, 전진시 VSLAM 사용
  - pose_mux: cmd_vel 방향에 따라 활성 카메라 선택 → /robot_pose 발행

사용법:
  # 양쪽 카메라에서 동시 캡처 (기본)
  python3 visualize_vslam_features.py

  # 특정 카메라만
  python3 visualize_vslam_features.py --camera front
  python3 visualize_vslam_features.py --camera back
"""

import argparse
import sys
import os
import threading
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from datetime import datetime


# ── 카메라 토픽 정의 ──
CAMERA_TOPICS = {
    'front': '/zed_front/zed_node/left/image_rect_color',
    'back':  '/zed_back/zed_node/left/image_rect_color',
}
CAMERA_LABELS = {
    'front': 'ZED X Front (SN:45320958) - Backward VSLAM',
    'back':  'ZED X Back (SN:46674448) - Forward VSLAM',
}


def extract_features_orb(image, max_features=1500):
    """ORB 특징점 추출 (VSLAM 내부와 유사한 방식)"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    orb = cv2.ORB_create(nfeatures=max_features, scaleFactor=1.2, nlevels=8)
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    return keypoints, descriptors


def match_features(desc1, desc2, ratio_thresh=0.75):
    """BFMatcher + Lowe's ratio test"""
    if desc1 is None or desc2 is None:
        return []
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    try:
        matches = bf.knnMatch(desc1, desc2, k=2)
    except cv2.error:
        return []
    good = []
    for pair in matches:
        if len(pair) == 2 and pair[0].distance < ratio_thresh * pair[1].distance:
            good.append(pair[0])
    return good


def create_feature_density_heatmap(keypoints, img_shape):
    """특징점 밀도 히트맵"""
    h, w = img_shape[:2]
    heatmap = np.zeros((h, w), dtype=np.float32)
    for kp in keypoints:
        x, y = int(kp.pt[0]), int(kp.pt[1])
        if 0 <= x < w and 0 <= y < h:
            cv2.circle(heatmap, (x, y), 20, 1.0, -1)
    heatmap = cv2.GaussianBlur(heatmap, (51, 51), 0)
    if heatmap.max() > 0:
        heatmap /= heatmap.max()
    return heatmap


def create_single_camera_figure(frames, camera_name, output_path):
    """단일 카메라 특징점 분석 (5-panel)"""
    if len(frames) < 2:
        print(f"  [{camera_name}] 프레임 부족, 건너뜀")
        return

    img1, img2 = frames[0], frames[1]
    kp1, desc1 = extract_features_orb(img1)
    kp2, desc2 = extract_features_orb(img2)
    good_matches = match_features(desc1, desc2)

    fig = plt.figure(figsize=(20, 16), facecolor='white')
    label = CAMERA_LABELS.get(camera_name, camera_name)
    fig.suptitle(f'VSLAM Feature Point Analysis\n{label}',
                 fontsize=18, fontweight='bold', y=0.98)

    gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.2,
                  top=0.92, bottom=0.06, left=0.05, right=0.95)

    # (a) Frame 1 특징점
    ax1 = fig.add_subplot(gs[0, 0])
    img1_kp = cv2.drawKeypoints(img1, kp1, None, color=(0, 255, 0),
                                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    ax1.imshow(cv2.cvtColor(img1_kp, cv2.COLOR_BGR2RGB))
    ax1.set_title(f'(a) Feature Detection - Frame 1\nDetected: {len(kp1)} keypoints',
                  fontsize=12, fontweight='bold')
    ax1.axis('off')

    # (b) Frame 2 특징점
    ax2 = fig.add_subplot(gs[0, 1])
    img2_kp = cv2.drawKeypoints(img2, kp2, None, color=(0, 255, 0),
                                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    ax2.imshow(cv2.cvtColor(img2_kp, cv2.COLOR_BGR2RGB))
    ax2.set_title(f'(b) Feature Detection - Frame 2\nDetected: {len(kp2)} keypoints',
                  fontsize=12, fontweight='bold')
    ax2.axis('off')

    # (c) 프레임 간 매칭
    ax3 = fig.add_subplot(gs[1, :])
    match_img = cv2.drawMatches(img1, kp1, img2, kp2, good_matches[:50], None,
                                matchColor=(0, 255, 0), singlePointColor=(255, 0, 0),
                                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    ax3.imshow(cv2.cvtColor(match_img, cv2.COLOR_BGR2RGB))
    min_kp = max(min(len(kp1), len(kp2)), 1)
    ax3.set_title(f'(c) Feature Matching Between Consecutive Frames\n'
                  f'Good matches: {len(good_matches)} / {min_kp} '
                  f'(Match rate: {len(good_matches)/min_kp*100:.1f}%)',
                  fontsize=12, fontweight='bold')
    ax3.axis('off')

    # (d) 밀도 히트맵
    ax4 = fig.add_subplot(gs[2, 0])
    heatmap = create_feature_density_heatmap(kp1, img1.shape)
    im = ax4.imshow(heatmap, cmap='jet', interpolation='bilinear')
    ax4.set_title('(d) Feature Density Heatmap\n(Brighter = More features)',
                  fontsize=12, fontweight='bold')
    ax4.axis('off')
    plt.colorbar(im, ax=ax4, fraction=0.046, pad=0.04, label='Density')

    # (e) 스케일 분포
    ax5 = fig.add_subplot(gs[2, 1])
    if len(kp1) > 0:
        sizes = [kp.size for kp in kp1]
        octaves = [kp.octave & 0xFF for kp in kp1]
        ax5_twin = ax5.twinx()
        ax5.hist(sizes, bins=30, alpha=0.7, color='steelblue', label='Keypoint Size')
        ax5.set_xlabel('Keypoint Size (pixels)', fontsize=11)
        ax5.set_ylabel('Count (Size)', color='steelblue', fontsize=11)
        ax5.tick_params(axis='y', labelcolor='steelblue')
        unique_oct, counts = np.unique(octaves, return_counts=True)
        ax5_twin.bar([f'L{o}' for o in unique_oct], counts,
                     alpha=0.5, color='coral', width=0.4, label='Scale Level')
        ax5_twin.set_ylabel('Count (Scale Level)', color='coral', fontsize=11)
        ax5_twin.tick_params(axis='y', labelcolor='coral')
        ax5.set_title('(e) Feature Scale Distribution\n(Multi-scale pyramid)',
                      fontsize=12, fontweight='bold')
        h1, l1 = ax5.get_legend_handles_labels()
        h2, l2 = ax5_twin.get_legend_handles_labels()
        ax5.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=9)

    # 정보 박스
    info = (f"Camera: {label} | Resolution: {img1.shape[1]}x{img1.shape[0]}\n"
            f"Feature Detector: ORB (Oriented FAST + Rotated BRIEF)\n"
            f"Frame 1: {len(kp1)} | Frame 2: {len(kp2)} | "
            f"Good Matches: {len(good_matches)} ({len(good_matches)/min_kp*100:.1f}%)\n"
            f"VSLAM Mode: GEN_2 (ZED SDK) + IMU Fusion | Depth: NEURAL_LIGHT")
    fig.text(0.5, 0.005, info, ha='center', va='bottom', fontsize=10,
             fontfamily='monospace',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.8))

    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"  저장: {output_path}")
    plt.close()


def create_tracking_statistics(frames, camera_name, output_path):
    """다중 프레임 추적 통계"""
    feat_counts, match_counts, match_rates = [], [], []
    prev_kp, prev_desc = None, None

    for frame in frames:
        kp, desc = extract_features_orb(frame)
        feat_counts.append(len(kp))
        if prev_desc is not None and desc is not None:
            good = match_features(prev_desc, desc)
            match_counts.append(len(good))
            match_rates.append(len(good) / max(min(len(kp), len(prev_kp)), 1) * 100)
        else:
            match_counts.append(0)
            match_rates.append(0)
        prev_kp, prev_desc = kp, desc

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), facecolor='white')
    label = CAMERA_LABELS.get(camera_name, camera_name)
    fig.suptitle(f'VSLAM Feature Tracking Statistics\n{label}',
                 fontsize=14, fontweight='bold')

    idx = range(len(frames))

    # 특징점 수
    axes[0].fill_between(idx, feat_counts, alpha=0.3, color='steelblue')
    axes[0].plot(idx, feat_counts, 'o-', color='steelblue', markersize=6, linewidth=2)
    axes[0].set_ylabel('Feature Count')
    axes[0].set_title('Detected Features per Frame')
    axes[0].grid(True, alpha=0.3)
    avg = np.mean(feat_counts)
    axes[0].axhline(y=avg, color='red', linestyle='--', alpha=0.7, label=f'Avg: {avg:.0f}')
    axes[0].legend()

    # 매칭 수
    axes[1].fill_between(idx, match_counts, alpha=0.3, color='forestgreen')
    axes[1].plot(idx, match_counts, 's-', color='forestgreen', markersize=6, linewidth=2)
    axes[1].set_ylabel('Match Count')
    axes[1].set_title('Good Matches Between Consecutive Frames')
    axes[1].grid(True, alpha=0.3)
    avg_m = np.mean(match_counts[1:]) if len(match_counts) > 1 else 0
    axes[1].axhline(y=avg_m, color='red', linestyle='--', alpha=0.7, label=f'Avg: {avg_m:.0f}')
    axes[1].legend()

    # 매칭률
    axes[2].fill_between(idx, match_rates, alpha=0.3, color='coral')
    axes[2].plot(idx, match_rates, 'D-', color='coral', markersize=6, linewidth=2)
    axes[2].set_ylabel('Match Rate (%)')
    axes[2].set_xlabel('Frame Index')
    axes[2].set_title('Feature Match Rate (Higher = Better Tracking)')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim(0, 100)
    avg_r = np.mean(match_rates[1:]) if len(match_rates) > 1 else 0
    axes[2].axhline(y=avg_r, color='red', linestyle='--', alpha=0.7, label=f'Avg: {avg_r:.1f}%')
    axes[2].axhspan(60, 100, alpha=0.1, color='green', label='Good')
    axes[2].axhspan(30, 60, alpha=0.1, color='yellow', label='Marginal')
    axes[2].axhspan(0, 30, alpha=0.1, color='red', label='Poor')
    axes[2].legend(fontsize=9, loc='lower right')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"  저장: {output_path}")
    plt.close()


def create_dual_camera_comparison(front_frames, back_frames, output_path):
    """듀얼 카메라 특징점 비교 (개발문서용 핵심 그림)"""
    fig = plt.figure(figsize=(22, 14), facecolor='white')
    fig.suptitle('Dual ZED X Camera VSLAM Feature Analysis\n'
                 'Robot Self-Localization via Visual Feature Tracking',
                 fontsize=18, fontweight='bold', y=0.98)

    gs = GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.25,
                  top=0.91, bottom=0.08, left=0.04, right=0.96)

    cam_data = [
        ('back', back_frames, 'Forward Motion (zed_back)', 0),
        ('front', front_frames, 'Backward Motion (zed_front)', 1),
    ]

    stats_text_parts = []

    for cam_name, frames, title, row in cam_data:
        if len(frames) < 2:
            continue

        img1, img2 = frames[0], frames[1]
        kp1, desc1 = extract_features_orb(img1)
        kp2, desc2 = extract_features_orb(img2)
        good = match_features(desc1, desc2)
        min_kp = max(min(len(kp1), len(kp2)), 1)
        rate = len(good) / min_kp * 100

        # (col 0) 특징점 오버레이
        ax_feat = fig.add_subplot(gs[row, 0])
        img_kp = cv2.drawKeypoints(img1, kp1, None, color=(0, 255, 0),
                                   flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        ax_feat.imshow(cv2.cvtColor(img_kp, cv2.COLOR_BGR2RGB))
        ax_feat.set_title(f'{title}\nFeatures: {len(kp1)}', fontsize=11, fontweight='bold')
        ax_feat.axis('off')

        # (col 1) 매칭
        ax_match = fig.add_subplot(gs[row, 1])
        match_img = cv2.drawMatches(img1, kp1, img2, kp2, good[:40], None,
                                    matchColor=(0, 255, 0),
                                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        ax_match.imshow(cv2.cvtColor(match_img, cv2.COLOR_BGR2RGB))
        ax_match.set_title(f'Frame-to-Frame Matching\n'
                           f'Matches: {len(good)} ({rate:.1f}%)',
                           fontsize=11, fontweight='bold')
        ax_match.axis('off')

        # (col 2) 히트맵
        ax_heat = fig.add_subplot(gs[row, 2])
        hm = create_feature_density_heatmap(kp1, img1.shape)
        im = ax_heat.imshow(hm, cmap='jet', interpolation='bilinear')
        ax_heat.set_title('Feature Density', fontsize=11, fontweight='bold')
        ax_heat.axis('off')
        plt.colorbar(im, ax=ax_heat, fraction=0.046, pad=0.04)

        stats_text_parts.append(
            f"{title}: {len(kp1)} features, {len(good)} matches ({rate:.1f}%)"
        )

    info = ("Dual Camera VSLAM: pose_mux selects active camera based on cmd_vel direction\n"
            + " | ".join(stats_text_parts) + "\n"
            "ZED SDK: GEN_2 pos_tracking + IMU fusion | Depth: NEURAL_LIGHT | 30 FPS")
    fig.text(0.5, 0.01, info, ha='center', va='bottom', fontsize=10,
             fontfamily='monospace',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.8))

    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"  저장: {output_path}")
    plt.close()


def capture_frames_from_topic(topic, num_frames=10, frame_skip=5, timeout_sec=30):
    """ROS2 토픽에서 프레임 캡처"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    frames = []
    frame_count = [0]
    bridge = CvBridge()
    done = threading.Event()

    class Capturer(Node):
        def __init__(self):
            super().__init__(f'vslam_cap_{topic.replace("/", "_")[:30]}')
            self.sub = self.create_subscription(Image, topic, self.cb, 10)
            self.timer = self.create_timer(timeout_sec, self.timeout_cb)
            self.get_logger().info(f'구독: {topic} ({num_frames}프레임, skip={frame_skip})')

        def cb(self, msg):
            frame_count[0] += 1
            if frame_count[0] % frame_skip != 0:
                return
            try:
                img = bridge.imgmsg_to_cv2(msg, 'bgr8')
                frames.append(img.copy())
                self.get_logger().info(f'  {len(frames)}/{num_frames} 캡처 ({img.shape[1]}x{img.shape[0]})')
                if len(frames) >= num_frames:
                    done.set()
            except Exception as e:
                self.get_logger().error(f'변환 실패: {e}')

        def timeout_cb(self):
            self.get_logger().warn(f'타임아웃 ({timeout_sec}s) - {len(frames)}프레임 수집됨')
            done.set()

    node = Capturer()

    while not done.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    return frames


def run_dual_capture(cameras, output_dir):
    """듀얼 카메라에서 순차적으로 캡처 후 시각화 생성"""
    try:
        import rclpy
    except ImportError:
        print("ERROR: rclpy를 찾을 수 없습니다.")
        print("  source /opt/ros/humble/setup.bash && source install/setup.bash")
        sys.exit(1)

    rclpy.init()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    all_frames = {}

    try:
        for cam in cameras:
            topic = CAMERA_TOPICS[cam]
            print(f"\n[{cam.upper()}] {topic} 에서 프레임 캡처 중...")
            frames = capture_frames_from_topic(topic, num_frames=10, frame_skip=5, timeout_sec=30)
            all_frames[cam] = frames
            print(f"  → {len(frames)}프레임 수집 완료")
    finally:
        rclpy.shutdown()

    # 시각화 생성
    print(f"\n시각화 생성 중...")
    outputs = []

    for cam, frames in all_frames.items():
        if len(frames) >= 2:
            # 개별 카메라 상세 분석
            out1 = os.path.join(output_dir, f'vslam_features_{cam}_{timestamp}.png')
            create_single_camera_figure(frames, cam, out1)
            outputs.append(out1)

            # 추적 통계
            out2 = os.path.join(output_dir, f'vslam_tracking_{cam}_{timestamp}.png')
            create_tracking_statistics(frames, cam, out2)
            outputs.append(out2)

    # 듀얼 카메라 비교 (양쪽 모두 있을 때)
    if 'front' in all_frames and 'back' in all_frames:
        if len(all_frames['front']) >= 2 and len(all_frames['back']) >= 2:
            out3 = os.path.join(output_dir, f'vslam_dual_comparison_{timestamp}.png')
            create_dual_camera_comparison(all_frames['front'], all_frames['back'], out3)
            outputs.append(out3)

    print(f"\n{'='*60}")
    print(f"생성된 파일 ({len(outputs)}개):")
    for f in outputs:
        print(f"  {f}")
    print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(description='VSLAM 특징점 시각화 (ZED X 주행 카메라)')
    parser.add_argument('--camera', choices=['front', 'back', 'both'], default='both',
                        help='캡처 대상 카메라 (기본: both)')
    parser.add_argument('--output', type=str, default='/home/koceti/ros2_ws',
                        help='출력 디렉토리')
    args = parser.parse_args()

    cameras = ['front', 'back'] if args.camera == 'both' else [args.camera]

    print("="*60)
    print("VSLAM Feature Visualization - ZED X Navigation Cameras")
    print(f"대상: {', '.join(cameras)}")
    for c in cameras:
        print(f"  {c}: {CAMERA_TOPICS[c]}")
    print("="*60)

    run_dual_capture(cameras, args.output)


if __name__ == '__main__':
    main()
