#!/usr/bin/env python3
"""
Phase 2b: RANSAC 평면 피팅으로 철근 분리 가능성 테스트

데크플레이트(바닥)를 RANSAC으로 평면 피팅한 후:
1. 평면 잔차(residual) 맵 — 평면에서 벗어난 픽셀 시각화
2. 평면보다 가까운 픽셀 마스크 — 철근 후보
3. NaN 패턴 — depth가 안 잡히는 얇은 구조물(철근)
4. 결합 마스크 → Canny → HoughLinesP 테스트

사용법:
    python3 test_ransac_plane.py [capture_dir]
"""

import numpy as np
import cv2
import os
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_data(capture_dir, cam):
    rgb = cv2.imread(os.path.join(capture_dir, f'{cam}_rgb.png'))
    depth = np.load(os.path.join(capture_dir, f'{cam}_depth.npy'))
    return rgb, depth


def ransac_plane_fit(depth, num_iterations=2000, distance_threshold=0.005):
    """RANSAC으로 depth 맵에서 주요 평면(데크플레이트) 피팅

    Args:
        depth: (H, W) float32, meters
        distance_threshold: inlier 판정 거리 (meters), 5mm

    Returns:
        plane_coeffs: (a, b, c, d) where ax + by + cz + d = 0
        inlier_mask: (H, W) bool
    """
    h, w = depth.shape

    # 유효 depth 픽셀의 3D 좌표 (u, v, depth)
    # 카메라 intrinsics 없이도 (u, v, depth) 공간에서 평면 피팅 가능
    # 실제로는 depth가 카메라 Z이므로 (u, v)에 따라 depth가 선형 변화하면 평면
    valid_mask = np.isfinite(depth) & (depth > 0.05) & (depth < 2.0)
    ys, xs = np.where(valid_mask)
    zs = depth[valid_mask]

    n_valid = len(zs)
    print(f'  유효 3D 포인트: {n_valid:,}개')

    if n_valid < 100:
        print('  유효 포인트 부족!')
        return None, None

    # 좌표 배열 (pixel_u, pixel_v, depth_m)
    points = np.column_stack([xs.astype(float), ys.astype(float), zs])

    # 서브샘플링: 최대 10000개로 줄여서 RANSAC 속도 확보
    max_sample = 10000
    if n_valid > max_sample:
        sub_idx = np.random.choice(n_valid, max_sample, replace=False)
        sample_pts = points[sub_idx]
        print(f'  서브샘플링: {n_valid:,} → {max_sample:,}개')
    else:
        sample_pts = points

    n_sample = len(sample_pts)
    best_inliers = 0
    best_plane = None

    # 벡터화 RANSAC: 배치로 3점씩 뽑아서 한번에 처리
    batch_size = 500
    for batch_start in range(0, num_iterations, batch_size):
        batch_n = min(batch_size, num_iterations - batch_start)

        # 배치 랜덤 인덱스 (batch_n, 3)
        rand_idx = np.array([np.random.choice(n_sample, 3, replace=False)
                             for _ in range(batch_n)])

        p1s = sample_pts[rand_idx[:, 0]]  # (batch_n, 3)
        p2s = sample_pts[rand_idx[:, 1]]
        p3s = sample_pts[rand_idx[:, 2]]

        v1s = p2s - p1s
        v2s = p3s - p1s
        normals = np.cross(v1s, v2s)  # (batch_n, 3)
        norms = np.linalg.norm(normals, axis=1, keepdims=True)

        # 퇴화 삼각형 제거
        valid = norms.flatten() > 1e-10
        if not np.any(valid):
            continue

        normals[valid] = normals[valid] / norms[valid]
        ds = -np.sum(normals * p1s, axis=1)  # (batch_n,)

        # 서브샘플에 대해 거리 계산: (batch_n, n_sample)
        distances = np.abs(sample_pts @ normals.T + ds[np.newaxis, :])
        inlier_counts = np.sum(distances < distance_threshold, axis=0)  # (batch_n,)

        # 유효하지 않은 것 제외
        inlier_counts[~valid] = 0

        batch_best = np.argmax(inlier_counts)
        if inlier_counts[batch_best] > best_inliers:
            best_inliers = inlier_counts[batch_best]
            n = normals[batch_best]
            best_plane = (n[0], n[1], n[2], ds[batch_best])

    if best_plane is None:
        return None, None

    a, b, c, d = best_plane
    inlier_ratio = best_inliers / n_valid

    print(f'  RANSAC 평면: {a:.6f}*u + {b:.6f}*v + {c:.6f}*z + {d:.4f} = 0')
    print(f'  인라이어: {best_inliers:,} / {n_valid:,} ({100*inlier_ratio:.1f}%)')

    # 전체 depth 맵에 대해 inlier 마스크 생성
    # 각 픽셀 (u, v)에서 depth와 평면 depth의 차이
    uu, vv = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    # 평면 위의 expected depth: a*u + b*v + c*z + d = 0 → z = -(a*u + b*v + d) / c
    if abs(c) < 1e-10:
        print('  평면이 depth축에 수직 — 유효하지 않음')
        return None, None

    plane_depth = -(a * uu + b * vv + d) / c  # (H, W) 평면 예상 depth

    # 실제 depth와 평면 depth의 차이 (잔차)
    residual = np.full_like(depth, np.nan)
    residual[valid_mask] = depth[valid_mask] - plane_depth[valid_mask]

    # inlier 마스크
    inlier_mask = valid_mask & (np.abs(depth - plane_depth) < distance_threshold)

    return {
        'plane': best_plane,
        'plane_depth': plane_depth,
        'residual': residual,
        'inlier_mask': inlier_mask,
        'inlier_ratio': inlier_ratio,
        'valid_mask': valid_mask,
    }, None


def visualize_results(rgb, depth, result, cam, output_dir):
    """분석 결과 시각화"""
    h, w = depth.shape
    plane_depth = result['plane_depth']
    residual = result['residual']
    inlier_mask = result['inlier_mask']
    valid_mask = result['valid_mask']

    # ========================================
    # 1. 평면 잔차 맵 (residual)
    # ========================================
    fig, axes = plt.subplots(2, 3, figsize=(20, 13))

    # 원본 RGB
    axes[0, 0].imshow(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
    axes[0, 0].set_title(f'{cam} - Original RGB')
    axes[0, 0].axis('off')

    # 잔차 맵 (양수=평면보다 멀리, 음수=평면보다 가까이=철근 후보)
    res_vis = residual.copy()
    res_vis[~valid_mask] = 0
    vmax = 0.03  # ±30mm 범위로 시각화
    im = axes[0, 1].imshow(res_vis, cmap='RdBu_r', vmin=-vmax, vmax=vmax)
    axes[0, 1].set_title(f'{cam} - Residual (blue=closer=rebar?)\n±{vmax*1000:.0f}mm range')
    axes[0, 1].axis('off')
    plt.colorbar(im, ax=axes[0, 1], label='meters')

    # 인라이어(데크플레이트) 마스크
    inlier_vis = np.zeros((h, w, 3), dtype=np.uint8)
    inlier_vis[inlier_mask] = [0, 255, 0]  # 초록 = 데크플레이트
    axes[0, 2].imshow(inlier_vis)
    axes[0, 2].set_title(
        f'{cam} - RANSAC inliers (deck plate)\n'
        f'{inlier_mask.sum():,} pixels ({100*inlier_mask.sum()/valid_mask.sum():.1f}% of valid)')
    axes[0, 2].axis('off')

    # ========================================
    # 2. 철근 후보 마스크들
    # ========================================

    # (A) 평면보다 가까운 픽셀 (negative residual)
    closer_mask = valid_mask & (residual < -0.005)  # 5mm 이상 가까운
    closer_vis = rgb.copy()
    closer_vis[~closer_mask] = 0
    axes[1, 0].imshow(cv2.cvtColor(closer_vis, cv2.COLOR_BGR2RGB))
    pct_closer = 100 * closer_mask.sum() / (h * w)
    axes[1, 0].set_title(f'{cam} - Closer than plane (>5mm)\n{closer_mask.sum():,} px ({pct_closer:.1f}%)')
    axes[1, 0].axis('off')

    # (B) NaN 패턴 (depth 안 잡히는 영역)
    nan_mask = ~np.isfinite(depth) | (depth <= 0.05)
    # 데크플레이트 영역 내의 NaN만 관심 (이미지 밖 NaN 제외)
    # 평면 depth가 유효한 범위 내의 NaN
    plane_valid = (plane_depth > 0.1) & (plane_depth < 1.5)
    nan_in_plane = nan_mask & plane_valid
    nan_vis = np.zeros((h, w, 3), dtype=np.uint8)
    nan_vis[nan_in_plane] = [255, 255, 0]  # 노란색 = NaN in plane area
    axes[1, 1].imshow(nan_vis)
    pct_nan = 100 * nan_in_plane.sum() / (h * w)
    axes[1, 1].set_title(f'{cam} - NaN in plane region (thin objects?)\n{nan_in_plane.sum():,} px ({pct_nan:.1f}%)')
    axes[1, 1].axis('off')

    # (C) 결합: closer OR NaN-in-plane → 철근 후보
    rebar_candidate = closer_mask | nan_in_plane
    combined_vis = rgb.copy()
    # 후보 영역만 밝게, 나머지 어둡게
    dark = (rgb * 0.3).astype(np.uint8)
    combined_vis[~rebar_candidate] = dark[~rebar_candidate]
    axes[1, 2].imshow(cv2.cvtColor(combined_vis, cv2.COLOR_BGR2RGB))
    pct_comb = 100 * rebar_candidate.sum() / (h * w)
    axes[1, 2].set_title(f'{cam} - Rebar candidates (closer + NaN)\n{rebar_candidate.sum():,} px ({pct_comb:.1f}%)')
    axes[1, 2].axis('off')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{cam}_ransac_analysis.png'), dpi=120)
    plt.close()

    # 개별 마스크 저장
    cv2.imwrite(os.path.join(output_dir, f'{cam}_closer_mask.png'),
                closer_vis)
    cv2.imwrite(os.path.join(output_dir, f'{cam}_rebar_candidate.png'),
                combined_vis)

    return rebar_candidate, closer_mask, nan_in_plane


def test_edge_detection(rgb, rebar_mask, cam, output_dir):
    """철근 후보 마스크 위에서 Canny + HoughLinesP 테스트"""
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)

    # 마스크 모폴로지 정리
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask_u8 = rebar_mask.astype(np.uint8) * 255
    # 약간의 dilation으로 철근 주변 포함
    mask_dilated = cv2.dilate(mask_u8, kernel, iterations=2)

    fig, axes = plt.subplots(2, 3, figsize=(20, 13))

    # (1) 마스크 없이 Canny + Hough
    edges_raw = cv2.Canny(enhanced, 50, 150)
    lines_raw = cv2.HoughLinesP(edges_raw, 1, np.pi/180,
                                 threshold=80, minLineLength=80, maxLineGap=20)
    vis_raw = cv2.cvtColor(edges_raw, cv2.COLOR_GRAY2BGR)
    n_raw = 0
    if lines_raw is not None:
        n_raw = len(lines_raw)
        for line in lines_raw:
            x1, y1, x2, y2 = line[0]
            cv2.line(vis_raw, (x1, y1), (x2, y2), (0, 0, 255), 1)
    axes[0, 0].imshow(vis_raw)
    axes[0, 0].set_title(f'{cam} - No mask: {n_raw} lines')
    axes[0, 0].axis('off')

    # (2) 철근 마스크 적용 후 Canny + Hough
    masked_gray = cv2.bitwise_and(enhanced, enhanced, mask=mask_dilated)
    edges_masked = cv2.Canny(masked_gray, 50, 150)
    lines_masked = cv2.HoughLinesP(edges_masked, 1, np.pi/180,
                                    threshold=50, minLineLength=60, maxLineGap=15)
    vis_masked = cv2.cvtColor(edges_masked, cv2.COLOR_GRAY2BGR)
    n_masked = 0
    if lines_masked is not None:
        n_masked = len(lines_masked)
        for line in lines_masked:
            x1, y1, x2, y2 = line[0]
            cv2.line(vis_masked, (x1, y1), (x2, y2), (0, 0, 255), 1)
    axes[0, 1].imshow(vis_masked)
    axes[0, 1].set_title(f'{cam} - RANSAC mask: {n_masked} lines')
    axes[0, 1].axis('off')

    # (3) 마스크된 RGB 위에 직선 오버레이
    overlay = rgb.copy()
    overlay[~rebar_mask] = (overlay[~rebar_mask] * 0.3).astype(np.uint8)
    if lines_masked is not None:
        for line in lines_masked:
            x1, y1, x2, y2 = line[0]
            cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
    axes[0, 2].imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
    axes[0, 2].set_title(f'{cam} - Lines on RGB (RANSAC masked)')
    axes[0, 2].axis('off')

    # (4) 직선 각도 분포 히스토그램
    if lines_masked is not None and len(lines_masked) > 0:
        angles = []
        for line in lines_masked:
            x1, y1, x2, y2 = line[0]
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
            angles.append(angle)
        axes[1, 0].hist(angles, bins=36, range=(0, 180),
                         color='steelblue', alpha=0.8, edgecolor='black')
        axes[1, 0].set_xlabel('Angle (degrees)')
        axes[1, 0].set_ylabel('Count')
        axes[1, 0].set_title(f'{cam} - Line angle distribution (masked)')
    else:
        axes[1, 0].text(0.5, 0.5, 'No lines detected', ha='center', va='center')
        axes[1, 0].set_title(f'{cam} - Line angle distribution')

    # (5) 직선 각도 기반 클러스터링 후 교차점 계산 테스트
    if lines_masked is not None and len(lines_masked) > 5:
        # 각도로 2그룹 분리 (간단 기준: 45~135° vs 나머지)
        group_a, group_b = [], []
        for line in lines_masked:
            x1, y1, x2, y2 = line[0]
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
            if 30 < angle < 150:
                group_a.append(line[0])
            else:
                group_b.append(line[0])

        # 교차점 계산
        intersections = []
        for la in group_a:
            for lb in group_b:
                pt = line_intersection(la, lb)
                if pt is not None:
                    ix, iy = pt
                    if 0 <= ix < rgb.shape[1] and 0 <= iy < rgb.shape[0]:
                        intersections.append((int(ix), int(iy)))

        inter_vis = rgb.copy()
        inter_vis[~rebar_mask] = (inter_vis[~rebar_mask] * 0.3).astype(np.uint8)
        # 직선 그리기
        for line in group_a:
            cv2.line(inter_vis, (line[0], line[1]), (line[2], line[3]), (255, 0, 0), 1)
        for line in group_b:
            cv2.line(inter_vis, (line[0], line[1]), (line[2], line[3]), (0, 255, 0), 1)
        # 교차점 표시
        for ix, iy in intersections:
            cv2.circle(inter_vis, (ix, iy), 5, (0, 0, 255), -1)

        axes[1, 1].imshow(cv2.cvtColor(inter_vis, cv2.COLOR_BGR2RGB))
        axes[1, 1].set_title(
            f'{cam} - Intersections: {len(intersections)}\n'
            f'GroupA(blue): {len(group_a)}, GroupB(green): {len(group_b)}')
        axes[1, 1].axis('off')

        # 교차점 클러스터링 (근접 교차점 병합)
        if intersections:
            clustered = cluster_points(intersections, distance=30)
            clust_vis = rgb.copy()
            for cx, cy in clustered:
                cv2.circle(clust_vis, (cx, cy), 10, (0, 0, 255), 3)
                cv2.circle(clust_vis, (cx, cy), 2, (0, 0, 255), -1)
            axes[1, 2].imshow(cv2.cvtColor(clust_vis, cv2.COLOR_BGR2RGB))
            axes[1, 2].set_title(
                f'{cam} - Clustered intersections: {len(clustered)}\n'
                f'(from {len(intersections)} raw)')
            axes[1, 2].axis('off')
        else:
            axes[1, 2].text(0.5, 0.5, 'No intersections', ha='center', va='center')
            axes[1, 2].axis('off')
    else:
        for ax in [axes[1, 1], axes[1, 2]]:
            ax.text(0.5, 0.5, 'Not enough lines', ha='center', va='center')
            ax.axis('off')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{cam}_ransac_lines.png'), dpi=120)
    plt.close()

    print(f'  직선 검출: 마스크 없음={n_raw}개, RANSAC 마스크={n_masked}개')


def line_intersection(line_a, line_b):
    """두 선분의 교차점 (확장선 기준)"""
    x1, y1, x2, y2 = line_a
    x3, y3, x4, y4 = line_b

    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3

    det = dx1 * dy2 - dx2 * dy1
    if abs(det) < 1e-6:
        return None

    t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / det
    ix = x1 + t * dx1
    iy = y1 + t * dy1
    return (ix, iy)


def cluster_points(points, distance=30):
    """근접 포인트 클러스터링 (단순 greedy)"""
    if not points:
        return []
    points = list(points)
    clusters = []

    while points:
        seed = points.pop(0)
        cluster = [seed]
        i = 0
        while i < len(points):
            px, py = points[i]
            # 클러스터 내 아무 점과 가까우면 병합
            for cx, cy in cluster:
                if abs(px - cx) + abs(py - cy) < distance * 1.5:
                    cluster.append(points.pop(i))
                    break
            else:
                i += 1

        # 클러스터 중심
        cx = int(np.mean([p[0] for p in cluster]))
        cy = int(np.mean([p[1] for p in cluster]))
        clusters.append((cx, cy))

    return clusters


def main():
    if len(sys.argv) < 2:
        base = '/home/koceti/ros2_ws/data/vision_test'
        dirs = sorted([d for d in os.listdir(base)
                      if os.path.isdir(os.path.join(base, d))])
        if dirs:
            capture_dir = os.path.join(base, dirs[-1])
        else:
            print('캡처 데이터 없음')
            return
    else:
        capture_dir = sys.argv[1]
        if not capture_dir.startswith('/'):
            capture_dir = os.path.join('/home/koceti/ros2_ws', capture_dir)

    output_dir = os.path.join(capture_dir, 'analysis')
    os.makedirs(output_dir, exist_ok=True)

    print(f'RANSAC 평면 피팅 테스트')
    print(f'데이터: {capture_dir}')

    for cam in ['left', 'right']:
        rgb_path = os.path.join(capture_dir, f'{cam}_rgb.png')
        depth_path = os.path.join(capture_dir, f'{cam}_depth.npy')
        if not os.path.exists(rgb_path):
            continue

        rgb = cv2.imread(rgb_path)
        depth = np.load(depth_path)

        print(f'\n{"="*60}')
        print(f'[{cam}] RANSAC 평면 피팅')
        print(f'{"="*60}')

        result, _ = ransac_plane_fit(depth,
                                      num_iterations=3000,
                                      distance_threshold=0.008)  # 8mm
        if result is None:
            print(f'[{cam}] 평면 피팅 실패')
            continue

        # 시각화
        rebar_mask, closer_mask, nan_mask = visualize_results(
            rgb, depth, result, cam, output_dir)

        # 엣지 검출 테스트
        test_edge_detection(rgb, rebar_mask, cam, output_dir)

    print(f'\n결과: {output_dir}/')
    for f in sorted(os.listdir(output_dir)):
        if 'ransac' in f:
            print(f'  {f}')


if __name__ == '__main__':
    main()
