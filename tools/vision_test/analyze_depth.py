#!/usr/bin/env python3
"""
Phase 2: Depth 맵 분석 — 철근 분리 가능성 확인

캡처된 RGB+Depth 데이터에서:
1. Depth 히스토그램 — 어떤 depth 범위에 픽셀이 집중되는지
2. Depth 범위별 마스킹 시각화 — 특정 범위만 추출했을 때 뭐가 보이는지
3. 캘리브레이션 데이터의 YOLO 검출점 depth 분포
4. Depth 마스킹 + Canny 엣지 시각화

사용법:
    python3 analyze_depth.py <capture_dir>
    python3 analyze_depth.py data/vision_test/20260320_090211
"""

import numpy as np
import cv2
import os
import sys
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_data(capture_dir, cam):
    """캡처 데이터 로드"""
    rgb_path = os.path.join(capture_dir, f'{cam}_rgb.png')
    depth_path = os.path.join(capture_dir, f'{cam}_depth.npy')

    if not os.path.exists(rgb_path) or not os.path.exists(depth_path):
        return None, None

    rgb = cv2.imread(rgb_path)
    depth = np.load(depth_path)  # float32, meters
    return rgb, depth


def analyze_depth_histogram(depth, cam, output_dir):
    """Depth 히스토그램 분석"""
    valid = depth[np.isfinite(depth) & (depth > 0.01) & (depth < 5.0)]

    if len(valid) == 0:
        print(f'[{cam}] 유효한 depth 데이터 없음')
        return

    print(f'\n[{cam}] Depth 통계:')
    print(f'  유효 픽셀: {len(valid):,} / {depth.size:,} ({100*len(valid)/depth.size:.1f}%)')
    print(f'  범위: {valid.min()*1000:.0f} ~ {valid.max()*1000:.0f} mm')
    print(f'  평균: {valid.mean()*1000:.0f} mm')
    print(f'  중간값: {np.median(valid)*1000:.0f} mm')

    # 히스토그램 (mm 단위)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # 전체 범위
    axes[0].hist(valid * 1000, bins=200, color='steelblue', alpha=0.8)
    axes[0].set_xlabel('Depth (mm)')
    axes[0].set_ylabel('Pixel count')
    axes[0].set_title(f'{cam} camera - Depth histogram (full range)')
    axes[0].axvline(x=np.median(valid)*1000, color='red', linestyle='--',
                     label=f'median={np.median(valid)*1000:.0f}mm')
    axes[0].legend()

    # 작업 영역 확대 (100~800mm)
    close = valid[(valid > 0.1) & (valid < 0.8)]
    if len(close) > 0:
        axes[1].hist(close * 1000, bins=200, color='steelblue', alpha=0.8)
        axes[1].set_xlabel('Depth (mm)')
        axes[1].set_ylabel('Pixel count')
        axes[1].set_title(f'{cam} camera - Depth histogram (100~800mm)')

        # 피크 찾기: 데크플레이트와 철근의 depth가 다르면 이봉 분포
        from scipy import signal
        counts, bin_edges = np.histogram(close * 1000, bins=200)
        bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
        # 스무딩 후 피크 탐지
        smoothed = np.convolve(counts, np.ones(5)/5, mode='same')
        peaks, props = signal.find_peaks(smoothed, height=len(close)*0.005,
                                          distance=10, prominence=len(close)*0.002)
        for p in peaks:
            axes[1].axvline(x=bin_centers[p], color='red', linestyle='--', alpha=0.7)
            axes[1].text(bin_centers[p], smoothed[p],
                         f' {bin_centers[p]:.0f}mm', color='red', fontsize=9)
        print(f'  피크 depth: {[f"{bin_centers[p]:.0f}mm" for p in peaks]}')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{cam}_depth_histogram.png'), dpi=120)
    plt.close()


def analyze_depth_slices(rgb, depth, cam, output_dir):
    """Depth 범위별 슬라이스 시각화"""
    # 작업 영역에 맞는 범위 설정 (mm 단위)
    slices = [
        (150, 250, 'very_close'),
        (250, 350, 'close'),
        (350, 450, 'mid'),
        (450, 550, 'far'),
        (550, 700, 'very_far'),
    ]

    depth_mm = depth * 1000.0

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()

    # 원본 RGB
    axes[0].imshow(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
    axes[0].set_title(f'{cam} - Original RGB')
    axes[0].axis('off')

    for i, (d_min, d_max, name) in enumerate(slices):
        mask = (depth_mm > d_min) & (depth_mm < d_max) & np.isfinite(depth_mm)
        masked_rgb = rgb.copy()
        masked_rgb[~mask] = 0

        # 유효 픽셀 비율
        pct = 100 * mask.sum() / mask.size

        axes[i+1].imshow(cv2.cvtColor(masked_rgb, cv2.COLOR_BGR2RGB))
        axes[i+1].set_title(f'{name}: {d_min}~{d_max}mm ({pct:.1f}%)')
        axes[i+1].axis('off')

        # 개별 저장도
        cv2.imwrite(os.path.join(output_dir, f'{cam}_slice_{d_min}_{d_max}mm.png'), masked_rgb)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{cam}_depth_slices.png'), dpi=120)
    plt.close()


def analyze_depth_edge(rgb, depth, cam, output_dir):
    """Depth 마스킹 + Canny 엣지 비교"""
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)

    depth_mm = depth * 1000.0

    # (1) 마스킹 없이 Canny
    edges_raw = cv2.Canny(enhanced, 50, 150)

    # (2) depth 범위별 마스킹 후 Canny
    # 다양한 범위 시도
    ranges = [
        (200, 400),
        (250, 450),
        (300, 500),
        (200, 500),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()

    axes[0].imshow(enhanced, cmap='gray')
    axes[0].set_title(f'{cam} - Enhanced grayscale')
    axes[0].axis('off')

    axes[1].imshow(edges_raw, cmap='gray')
    axes[1].set_title(f'{cam} - Canny (no depth mask)\nedge pixels: {edges_raw.sum()//255}')
    axes[1].axis('off')

    for i, (d_min, d_max) in enumerate(ranges):
        mask = (depth_mm > d_min) & (depth_mm < d_max) & np.isfinite(depth_mm)
        # 마스크 모폴로지 정리 (구멍 메우기)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_u8 = mask.astype(np.uint8) * 255
        mask_clean = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel, iterations=2)

        masked_gray = cv2.bitwise_and(enhanced, enhanced, mask=mask_clean)
        edges_masked = cv2.Canny(masked_gray, 50, 150)

        # 엣지 위에 depth mask 윤곽 표시
        vis = cv2.cvtColor(edges_masked, cv2.COLOR_GRAY2BGR)
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, contours, -1, (0, 0, 255), 1)

        axes[i+2].imshow(vis)
        axes[i+2].set_title(
            f'{cam} - Canny + depth {d_min}~{d_max}mm\n'
            f'edge pixels: {edges_masked.sum()//255}, mask: {100*mask.sum()/mask.size:.1f}%')
        axes[i+2].axis('off')

        cv2.imwrite(os.path.join(output_dir,
                                  f'{cam}_edges_depth_{d_min}_{d_max}mm.png'), vis)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{cam}_depth_edge_comparison.png'), dpi=120)
    plt.close()


def analyze_calibration_depth(cam, output_dir):
    """캘리브레이션 데이터에서 YOLO 검출점의 depth 분포 분석"""
    calib_path = f'/home/koceti/ros2_ws/data/calibration/calibration_data_{cam}.json'
    if not os.path.exists(calib_path):
        print(f'[{cam}] 캘리브레이션 데이터 없음: {calib_path}')
        return

    with open(calib_path) as f:
        data = json.load(f)

    points = data.get('points', data) if isinstance(data, dict) else data
    if isinstance(points, dict):
        points = points.get('points', [])

    depths = [p['depth_mm'] for p in points if 'depth_mm' in p]
    if not depths:
        print(f'[{cam}] depth_mm 필드 없음')
        return

    depths = np.array(depths)
    print(f'\n[{cam}] YOLO 검출점 depth 분포 ({len(depths)}개):')
    print(f'  범위: {depths.min():.0f} ~ {depths.max():.0f} mm')
    print(f'  평균: {depths.mean():.0f} mm, 표준편차: {depths.std():.0f} mm')
    print(f'  10%ile: {np.percentile(depths, 10):.0f} mm')
    print(f'  90%ile: {np.percentile(depths, 90):.0f} mm')
    print(f'  → 제안 depth 마스크: {np.percentile(depths, 5):.0f} ~ {np.percentile(depths, 95):.0f} mm')

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(depths, bins=30, color='coral', alpha=0.8, edgecolor='black')
    ax.set_xlabel('Depth (mm)')
    ax.set_ylabel('Count')
    ax.set_title(f'{cam} - YOLO detection point depth distribution (N={len(depths)})')
    ax.axvline(x=depths.mean(), color='red', linestyle='--',
               label=f'mean={depths.mean():.0f}mm')
    ax.axvline(x=np.percentile(depths, 5), color='green', linestyle=':',
               label=f'5%ile={np.percentile(depths, 5):.0f}mm')
    ax.axvline(x=np.percentile(depths, 95), color='green', linestyle=':',
               label=f'95%ile={np.percentile(depths, 95):.0f}mm')
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{cam}_calib_depth_dist.png'), dpi=120)
    plt.close()


def main():
    if len(sys.argv) < 2:
        # 가장 최근 캡처 폴더 자동 선택
        base = '/home/koceti/ros2_ws/data/vision_test'
        if os.path.exists(base):
            dirs = sorted([d for d in os.listdir(base)
                          if os.path.isdir(os.path.join(base, d))])
            if dirs:
                capture_dir = os.path.join(base, dirs[-1])
            else:
                print('캡처 데이터 없음. capture_rgbd.py를 먼저 실행하세요.')
                return
        else:
            print(f'usage: python3 analyze_depth.py <capture_dir>')
            return
    else:
        capture_dir = sys.argv[1]
        if not capture_dir.startswith('/'):
            capture_dir = os.path.join('/home/koceti/ros2_ws', capture_dir)

    output_dir = os.path.join(capture_dir, 'analysis')
    os.makedirs(output_dir, exist_ok=True)

    print(f'분석 대상: {capture_dir}')
    print(f'결과 저장: {output_dir}')

    for cam in ['left', 'right']:
        rgb, depth = load_data(capture_dir, cam)
        if rgb is None:
            print(f'[{cam}] 데이터 없음, 건너뜀')
            continue

        print(f'\n{"="*60}')
        print(f'[{cam}] 분석 시작 (RGB: {rgb.shape}, Depth: {depth.shape})')
        print(f'{"="*60}')

        # 1. Depth 히스토그램
        analyze_depth_histogram(depth, cam, output_dir)

        # 2. Depth 범위별 슬라이스
        analyze_depth_slices(rgb, depth, cam, output_dir)

        # 3. Depth 마스킹 + Canny 엣지 비교
        analyze_depth_edge(rgb, depth, cam, output_dir)

        # 4. 캘리브레이션 YOLO 검출점 depth 분포
        analyze_calibration_depth(cam, output_dir)

    print(f'\n분석 완료! 결과: {output_dir}/')
    print('주요 파일:')
    for f in sorted(os.listdir(output_dir)):
        print(f'  {f}')


if __name__ == '__main__':
    main()
