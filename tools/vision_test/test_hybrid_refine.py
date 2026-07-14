#!/usr/bin/env python3
"""
Phase 3: 하이브리드 테스트 — YOLO ROI + OpenCV 서브픽셀 정밀화

1. YOLO로 교차점 바운딩 박스 검출
2. 바운딩 박스를 15% 확대
3. ROI 내에서 OpenCV 직선 검출 → 교차점 계산
4. YOLO 중심점 vs OpenCV 교차점 비교

사용법:
    python3 test_hybrid_refine.py [capture_dir]
"""

import numpy as np
import cv2
import os
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_yolo_model():
    from ultralytics import YOLO
    model_path = '/home/koceti/ros2_ws/src/rebar_vision/model/best_260313.pt'
    model = YOLO(model_path)
    try:
        model.to('cuda')
        print('YOLO 모델 로드 (GPU)')
    except Exception:
        print('YOLO 모델 로드 (CPU)')
    return model


def expand_bbox(x1, y1, x2, y2, ratio, img_w, img_h):
    """바운딩 박스를 ratio만큼 확대"""
    w = x2 - x1
    h = y2 - y1
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    new_w = w * (1 + ratio)
    new_h = h * (1 + ratio)
    # 최소 크기 보장 (너무 작은 ROI 방지)
    new_w = max(new_w, 60)
    new_h = max(new_h, 60)
    nx1 = max(0, int(cx - new_w / 2))
    ny1 = max(0, int(cy - new_h / 2))
    nx2 = min(img_w, int(cx + new_w / 2))
    ny2 = min(img_h, int(cy + new_h / 2))
    return nx1, ny1, nx2, ny2


def refine_intersection_opencv(rgb_roi, method='hough'):
    """ROI 내에서 OpenCV로 교차점 서브픽셀 좌표 추출

    Args:
        rgb_roi: 작은 ROI 이미지 (BGR)
        method: 'hough' or 'corner'

    Returns:
        (refined_x, refined_y) ROI 내 좌표, 실패 시 None
    """
    h, w = rgb_roi.shape[:2]
    gray = cv2.cvtColor(rgb_roi, cv2.COLOR_BGR2GRAY)

    # CLAHE 적용
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
    enhanced = clahe.apply(gray)

    # 가우시안 블러로 노이즈 제거
    blurred = cv2.GaussianBlur(enhanced, (3, 3), 0)

    # === Method 1: HoughLinesP 기반 교차점 ===
    edges = cv2.Canny(blurred, 50, 150)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                             threshold=15, minLineLength=15, maxLineGap=8)

    if lines is None or len(lines) < 2:
        return None, None, edges, lines

    # 각도 기반 2그룹 분류
    group_a, group_b = [], []
    angles = []
    for l in lines:
        x1, y1, x2, y2 = l[0]
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
        angles.append(angle)

    if not angles:
        return None, None, edges, lines

    # 각도 중간값 기준으로 분리 (2개 방향이 있다고 가정)
    angles_arr = np.array(angles)
    # K-means 2클러스터 (간단히 구현)
    median_angle = np.median(angles_arr)
    for i, l in enumerate(lines):
        if abs(angles[i] - median_angle) < 30:
            group_a.append(l[0])
        else:
            group_b.append(l[0])

    # 둘 다 있어야 교차점 계산 가능
    if len(group_a) == 0 or len(group_b) == 0:
        # 각도 차이가 클 때: 정렬된 각도에서 가장 큰 gap으로 분리
        sorted_angles = np.sort(angles_arr)
        gaps = np.diff(sorted_angles)
        if len(gaps) > 0:
            split_idx = np.argmax(gaps)
            split_angle = (sorted_angles[split_idx] + sorted_angles[split_idx + 1]) / 2
            group_a, group_b = [], []
            for i, l in enumerate(lines):
                if angles[i] < split_angle:
                    group_a.append(l[0])
                else:
                    group_b.append(l[0])

    if len(group_a) == 0 or len(group_b) == 0:
        return None, None, edges, lines

    # 각 그룹을 하나의 대표선으로 피팅 (cv2.fitLine)
    pts_a = []
    for l in group_a:
        pts_a.extend([(l[0], l[1]), (l[2], l[3])])
    pts_a = np.array(pts_a, dtype=np.float32)
    vx_a, vy_a, cx_a, cy_a = cv2.fitLine(pts_a, cv2.DIST_L2, 0, 0.01, 0.01)

    pts_b = []
    for l in group_b:
        pts_b.extend([(l[0], l[1]), (l[2], l[3])])
    pts_b = np.array(pts_b, dtype=np.float32)
    vx_b, vy_b, cx_b, cy_b = cv2.fitLine(pts_b, cv2.DIST_L2, 0, 0.01, 0.01)

    # 두 대표선의 교차점 계산
    vx_a, vy_a = float(vx_a), float(vy_a)
    vx_b, vy_b = float(vx_b), float(vy_b)
    cx_a, cy_a = float(cx_a), float(cy_a)
    cx_b, cy_b = float(cx_b), float(cy_b)

    det = vx_a * vy_b - vx_b * vy_a
    if abs(det) < 1e-6:
        return None, None, edges, lines

    t = ((cx_b - cx_a) * vy_b - (cy_b - cy_a) * vx_b) / det
    ix = cx_a + t * vx_a
    iy = cy_a + t * vy_a

    # ROI 범위 체크
    if ix < -5 or ix > w + 5 or iy < -5 or iy > h + 5:
        return None, None, edges, lines

    return float(ix), float(iy), edges, lines


def main():
    if len(sys.argv) < 2:
        base = '/home/koceti/ros2_ws/data/vision_test'
        dirs = sorted([d for d in os.listdir(base)
                      if os.path.isdir(os.path.join(base, d))])
        capture_dir = os.path.join(base, dirs[-1]) if dirs else None
    else:
        capture_dir = sys.argv[1]
        if not capture_dir.startswith('/'):
            capture_dir = os.path.join('/home/koceti/ros2_ws', capture_dir)

    output_dir = os.path.join(capture_dir, 'analysis')
    os.makedirs(output_dir, exist_ok=True)

    model = load_yolo_model()

    expand_ratio = 0.15  # 15% 확대

    for cam in ['left', 'right']:
        rgb_path = os.path.join(capture_dir, f'{cam}_rgb.png')
        if not os.path.exists(rgb_path):
            continue

        rgb = cv2.imread(rgb_path)
        h, w = rgb.shape[:2]

        print(f'\n{"="*60}')
        print(f'[{cam}] YOLO 검출 + OpenCV 정밀화')
        print(f'{"="*60}')

        # YOLO 검출
        results = model(rgb, conf=0.4, verbose=False)
        if not results or len(results[0].boxes) == 0:
            print(f'  YOLO 검출 없음')
            continue

        boxes = results[0].boxes
        n_det = len(boxes)
        print(f'  YOLO 검출: {n_det}개')

        # 전체 결과 시각화
        n_cols = min(4, n_det)
        n_rows = 2 + (n_det + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(5*n_cols, 5*n_rows))
        if n_rows == 1:
            axes = axes[np.newaxis, :]

        # Row 0: 전체 이미지 + YOLO 결과 + 정밀화 결과
        overview_yolo = rgb.copy()
        overview_refined = rgb.copy()

        yolo_centers = []
        refined_centers = []
        deltas = []

        for det_idx, box in enumerate(boxes):
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            conf = float(box.conf[0])

            # YOLO 중심점
            yolo_cx = int((x1 + x2) / 2)
            yolo_cy = int((y1 + y2) / 2)
            yolo_centers.append((yolo_cx, yolo_cy))

            # 확대된 ROI
            ex1, ey1, ex2, ey2 = expand_bbox(x1, y1, x2, y2, expand_ratio, w, h)
            roi = rgb[ey1:ey2, ex1:ex2].copy()

            # OpenCV 정밀화
            ref_x, ref_y, edges, lines = refine_intersection_opencv(roi)

            # 결과 매핑 (ROI → 전체 이미지 좌표)
            if ref_x is not None:
                refined_cx = ex1 + ref_x
                refined_cy = ey1 + ref_y
                refined_centers.append((refined_cx, refined_cy))

                dx = refined_cx - yolo_cx
                dy = refined_cy - yolo_cy
                dist = np.sqrt(dx**2 + dy**2)
                deltas.append(dist)

                print(f'  #{det_idx} YOLO=({yolo_cx},{yolo_cy}) → '
                      f'Refined=({refined_cx:.1f},{refined_cy:.1f}) '
                      f'Δ={dist:.1f}px conf={conf:.2f}')
            else:
                refined_centers.append(None)
                print(f'  #{det_idx} YOLO=({yolo_cx},{yolo_cy}) → 정밀화 실패 conf={conf:.2f}')

            # YOLO overview
            cv2.rectangle(overview_yolo, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(overview_yolo, (yolo_cx, yolo_cy), 6, (0, 0, 255), -1)
            cv2.putText(overview_yolo, f'#{det_idx}',
                        (yolo_cx + 8, yolo_cy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # Refined overview
            cv2.rectangle(overview_refined, (ex1, ey1), (ex2, ey2), (255, 255, 0), 1)
            cv2.circle(overview_refined, (yolo_cx, yolo_cy), 5, (0, 0, 255), -1)  # YOLO: red
            if ref_x is not None:
                rcx, rcy = int(refined_cx), int(refined_cy)
                cv2.circle(overview_refined, (rcx, rcy), 5, (0, 255, 0), -1)  # Refined: green
                cv2.line(overview_refined, (yolo_cx, yolo_cy), (rcx, rcy), (255, 0, 255), 2)

            # 개별 ROI 시각화 (하단 행들)
            row_idx = 2 + det_idx // n_cols
            col_idx = det_idx % n_cols
            if row_idx < n_rows and col_idx < n_cols:
                roi_vis = roi.copy()
                # ROI 내 YOLO 중심
                roi_ycx = yolo_cx - ex1
                roi_ycy = yolo_cy - ey1
                cv2.circle(roi_vis, (roi_ycx, roi_ycy), 4, (0, 0, 255), -1)

                if lines is not None:
                    for l in lines:
                        cv2.line(roi_vis, (l[0][0], l[0][1]), (l[0][2], l[0][3]),
                                 (255, 255, 0), 1)
                if ref_x is not None:
                    cv2.circle(roi_vis, (int(ref_x), int(ref_y)), 4, (0, 255, 0), -1)
                    cv2.line(roi_vis, (roi_ycx, roi_ycy),
                             (int(ref_x), int(ref_y)), (255, 0, 255), 1)

                # ROI가 작으니 확대
                roi_vis = cv2.resize(roi_vis, None, fx=3, fy=3,
                                      interpolation=cv2.INTER_NEAREST)
                axes[row_idx, col_idx].imshow(cv2.cvtColor(roi_vis, cv2.COLOR_BGR2RGB))
                title = f'#{det_idx} conf={conf:.2f}'
                if ref_x is not None:
                    title += f'\nΔ={deltas[-1]:.1f}px'
                else:
                    title += '\nFAILED'
                axes[row_idx, col_idx].set_title(title, fontsize=9)
                axes[row_idx, col_idx].axis('off')

        # Overview 행
        axes[0, 0].imshow(cv2.cvtColor(overview_yolo, cv2.COLOR_BGR2RGB))
        axes[0, 0].set_title(f'{cam} - YOLO centers (red)')
        axes[0, 0].axis('off')

        axes[0, 1].imshow(cv2.cvtColor(overview_refined, cv2.COLOR_BGR2RGB))
        axes[0, 1].set_title(f'{cam} - Red=YOLO, Green=Refined, Pink=delta')
        axes[0, 1].axis('off')

        # 통계
        if deltas:
            stats_text = (f'정밀화 성공: {len(deltas)}/{n_det}\n'
                          f'평균 Δ: {np.mean(deltas):.1f}px\n'
                          f'최대 Δ: {np.max(deltas):.1f}px\n'
                          f'최소 Δ: {np.min(deltas):.1f}px')
        else:
            stats_text = '정밀화 실패'
        if n_cols > 2:
            axes[0, 2].text(0.1, 0.5, stats_text, fontsize=14,
                            family='monospace', va='center', transform=axes[0, 2].transAxes)
            axes[0, 2].set_title(f'{cam} - Statistics')
            axes[0, 2].axis('off')

        # 빈 축 숨기기
        for ax in axes.flatten():
            if not ax.has_data() and not ax.texts:
                ax.set_visible(False)

        # Delta 히스토그램
        if deltas and n_cols > 3:
            axes[1, 0].hist(deltas, bins=max(5, len(deltas)//2),
                            color='coral', edgecolor='black')
            axes[1, 0].set_xlabel('Delta (pixels)')
            axes[1, 0].set_ylabel('Count')
            axes[1, 0].set_title(f'{cam} - Refinement delta distribution')

        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f'{cam}_hybrid_refine.png'), dpi=120)
        plt.close()

        if deltas:
            print(f'\n  [요약] 성공 {len(deltas)}/{n_det}, '
                  f'평균Δ={np.mean(deltas):.1f}px, '
                  f'최대Δ={np.max(deltas):.1f}px')

    print(f'\n결과: {output_dir}/')


if __name__ == '__main__':
    main()
