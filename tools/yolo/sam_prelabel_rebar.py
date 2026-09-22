#!/usr/bin/env python3
"""
SAM 기반 철근 세그멘테이션 Pre-labeling 스크립트

SAM(Segment Anything Model)으로 이미지를 자동 세그멘테이션한 뒤,
철근처럼 가늘고 긴(elongated) 마스크만 필터링하여 YOLO 세그멘테이션 포맷으로 저장.

사용법:
  python3 sam_prelabel_rebar.py                          # 전체 데이터셋
  python3 sam_prelabel_rebar.py --input_dir path/to/imgs  # 특정 디렉토리
  python3 sam_prelabel_rebar.py --visualize               # 시각화 포함
  python3 sam_prelabel_rebar.py --sample 10               # 10장만 테스트

출력:
  {output_dir}/images/   ← 원본 이미지 복사 (YOLO 학습용)
  {output_dir}/labels/   ← YOLO seg 포맷 (.txt)
  {output_dir}/viz/      ← 시각화 이미지 (--visualize 시)

YOLO seg 라벨 포맷 (한 줄 = 한 인스턴스):
  class_id x1 y1 x2 y2 ... xN yN  (normalized 0~1)

클래스:
  0: rebar (철근)
"""

import argparse
import os
import sys
import glob
import shutil
import time
import cv2
import numpy as np
from pathlib import Path


def find_images(input_dir):
    """이미지 파일 검색"""
    exts = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    images = []
    for ext in exts:
        images.extend(glob.glob(os.path.join(input_dir, '**', ext), recursive=True))
    images.sort()
    return images


def mask_to_polygon(mask, epsilon_ratio=0.005):
    """바이너리 마스크 → 폴리곤 좌표 (normalized)"""
    h, w = mask.shape
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 가장 큰 contour 선택
    contour = max(contours, key=cv2.contourArea)

    # 단순화
    epsilon = epsilon_ratio * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    if len(approx) < 3:
        return None

    # normalize (0~1)
    points = approx.reshape(-1, 2).astype(np.float64)
    points[:, 0] /= w
    points[:, 1] /= h

    return points


def is_rebar_like(mask, min_area=500, min_aspect=3.0, max_solidity=0.85):
    """마스크가 철근처럼 가늘고 긴 형상인지 판별

    Args:
        mask: binary mask
        min_area: 최소 면적 (pixels)
        min_aspect: 최소 종횡비 (길이/폭)
        max_solidity: 최대 solidity (가느다란 형상은 solidity 낮음)
    """
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, {}

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)

    if area < min_area:
        return False, {'reason': 'area_small', 'area': area}

    # 최소 외접 사각형 → 종횡비
    rect = cv2.minAreaRect(contour)
    (_, _), (w, h), _ = rect
    if w == 0 or h == 0:
        return False, {'reason': 'zero_dim'}

    aspect = max(w, h) / min(w, h)

    # solidity (면적 / convex hull 면적)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area > 0 else 0

    # 철근 판별: 길쭉하고 (aspect >= 3) 또는 면적 크고 가느다란 것
    is_rebar = aspect >= min_aspect

    info = {
        'area': area,
        'aspect': aspect,
        'solidity': solidity,
        'width': min(w, h),
        'length': max(w, h),
    }

    return is_rebar, info


def process_image_sam(sam_model, img_path, args):
    """SAM으로 한 이미지 처리 → 철근 마스크 필터링 → 폴리곤 반환"""
    img = cv2.imread(img_path)
    if img is None:
        return [], img

    h, w = img.shape[:2]

    # SAM auto segmentation
    results = sam_model(img_path, verbose=False)

    if not results or results[0].masks is None:
        return [], img

    masks = results[0].masks.data.cpu().numpy()
    rebar_polygons = []
    rebar_masks = []

    for i in range(len(masks)):
        mask = masks[i]

        # 마스크를 원본 크기로 리사이즈 (SAM 출력이 다를 수 있음)
        if mask.shape != (h, w):
            mask = cv2.resize(mask.astype(np.float32), (w, h)) > 0.5

        is_rebar, info = is_rebar_like(
            mask,
            min_area=args.min_area,
            min_aspect=args.min_aspect,
        )

        if is_rebar:
            polygon = mask_to_polygon(mask)
            if polygon is not None:
                rebar_polygons.append(polygon)
                rebar_masks.append(mask)

    return list(zip(rebar_polygons, rebar_masks)), img


def process_image_edge(img_path, args):
    """SAM 없이 엣지 기반으로 철근 후보 추출 (fallback)"""
    img = cv2.imread(img_path)
    if img is None:
        return [], img

    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # CLAHE로 대비 강화
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)

    # 엣지 검출
    edges = cv2.Canny(enhanced, 50, 150)

    # 팽창으로 엣지 연결
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dilated = cv2.dilate(edges, kernel, iterations=2)

    # 컨투어 찾기
    contours, _ = cv2.findContours(
        dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rebar_polygons = []
    rebar_masks = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < args.min_area:
            continue

        rect = cv2.minAreaRect(contour)
        (_, _), (rw, rh), _ = rect
        if rw == 0 or rh == 0:
            continue

        aspect = max(rw, rh) / min(rw, rh)
        if aspect < args.min_aspect:
            continue

        # 마스크 생성
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 1, -1)

        polygon = mask_to_polygon(mask)
        if polygon is not None:
            rebar_polygons.append(polygon)
            rebar_masks.append(mask)

    return list(zip(rebar_polygons, rebar_masks)), img


def save_yolo_label(label_path, polygons, class_id=0):
    """YOLO 세그멘테이션 포맷으로 라벨 저장"""
    with open(label_path, 'w') as f:
        for polygon in polygons:
            coords = ' '.join(f'{x:.6f} {y:.6f}' for x, y in polygon)
            f.write(f'{class_id} {coords}\n')


def save_visualization(img, masks_and_polygons, save_path):
    """세그멘테이션 결과 시각화"""
    vis = img.copy()
    h, w = img.shape[:2]
    colors = [
        (0, 255, 0), (255, 0, 0), (0, 0, 255),
        (255, 255, 0), (0, 255, 255), (255, 0, 255),
        (128, 255, 0), (255, 128, 0), (0, 128, 255),
    ]

    for i, (polygon, mask) in enumerate(masks_and_polygons):
        color = colors[i % len(colors)]

        # 반투명 마스크 오버레이
        overlay = vis.copy()
        overlay[mask > 0] = color
        vis = cv2.addWeighted(vis, 0.7, overlay, 0.3, 0)

        # 폴리곤 윤곽
        pts = (polygon * np.array([w, h])).astype(np.int32)
        cv2.polylines(vis, [pts], True, color, 2)

        # 라벨
        cx, cy = pts.mean(axis=0).astype(int)
        cv2.putText(vis, f'rebar_{i}', (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # 카운트 표시
    cv2.putText(vis, f'{len(masks_and_polygons)} rebars detected',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imwrite(save_path, vis)


def main():
    parser = argparse.ArgumentParser(description='SAM 기반 철근 세그멘테이션 Pre-labeling')
    parser.add_argument('--input_dir', type=str,
                        default='/home/koceti/ros2_ws/data/images/rebar_dataset',
                        help='이미지 디렉토리')
    parser.add_argument('--output_dir', type=str,
                        default='/home/koceti/ros2_ws/data/images/rebar_yolo_seg',
                        help='YOLO seg 출력 디렉토리')
    parser.add_argument('--model', type=str, default='sam_b.pt',
                        help='SAM 모델 (sam_b.pt, sam_l.pt, mobile_sam.pt)')
    parser.add_argument('--visualize', action='store_true',
                        help='시각화 이미지 저장')
    parser.add_argument('--sample', type=int, default=0,
                        help='샘플 수 (0=전체)')
    parser.add_argument('--min_area', type=int, default=500,
                        help='최소 마스크 면적 (pixels)')
    parser.add_argument('--min_aspect', type=float, default=3.0,
                        help='최소 종횡비 (길이/폭)')
    parser.add_argument('--no_sam', action='store_true',
                        help='SAM 없이 엣지 기반 처리 (fallback)')
    args = parser.parse_args()

    # 이미지 검색
    images = find_images(args.input_dir)
    if not images:
        print(f'이미지 없음: {args.input_dir}')
        return

    if args.sample > 0:
        images = images[:args.sample]

    print(f'이미지: {len(images)}장')
    print(f'출력: {args.output_dir}')

    # 출력 디렉토리
    img_dir = os.path.join(args.output_dir, 'images')
    lbl_dir = os.path.join(args.output_dir, 'labels')
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)
    if args.visualize:
        viz_dir = os.path.join(args.output_dir, 'viz')
        os.makedirs(viz_dir, exist_ok=True)

    # SAM 모델 로드
    sam_model = None
    if not args.no_sam:
        try:
            from ultralytics import SAM
            print(f'SAM 모델 로드: {args.model}')
            sam_model = SAM(args.model)
            print('SAM 로드 완료')
        except Exception as e:
            print(f'SAM 로드 실패: {e}')
            print('엣지 기반 fallback 사용')
            args.no_sam = True

    # 처리
    total_rebars = 0
    t0 = time.time()

    for idx, img_path in enumerate(images):
        basename = Path(img_path).stem
        # 세션 디렉토리 이름 포함하여 고유 파일명
        parent = Path(img_path).parent.name
        unique_name = f'{parent}_{basename}'

        if args.no_sam:
            results, img = process_image_edge(img_path, args)
        else:
            results, img = process_image_sam(sam_model, img_path, args)

        # 이미지 복사
        dst_img = os.path.join(img_dir, f'{unique_name}.jpg')
        shutil.copy2(img_path, dst_img)

        # 라벨 저장
        dst_lbl = os.path.join(lbl_dir, f'{unique_name}.txt')
        if results:
            polygons = [r[0] for r in results]
            save_yolo_label(dst_lbl, polygons, class_id=0)
            total_rebars += len(results)
        else:
            # 빈 라벨 파일 (negative sample)
            open(dst_lbl, 'w').close()

        # 시각화
        if args.visualize and img is not None:
            viz_path = os.path.join(viz_dir, f'{unique_name}.jpg')
            save_visualization(img, results, viz_path)

        if (idx + 1) % 10 == 0 or idx == len(images) - 1:
            elapsed = time.time() - t0
            fps = (idx + 1) / elapsed if elapsed > 0 else 0
            print(f'  [{idx+1}/{len(images)}] {total_rebars} rebars  '
                  f'({fps:.1f} img/s)')

    elapsed = time.time() - t0
    print(f'\n완료: {len(images)}장, {total_rebars}개 철근 검출')
    print(f'소요: {elapsed:.1f}초 ({len(images)/elapsed:.1f} img/s)')
    print(f'\n출력:')
    print(f'  images: {img_dir}')
    print(f'  labels: {lbl_dir}')
    if args.visualize:
        print(f'  viz:    {viz_dir}')

    # YOLO 학습용 data.yaml 생성
    yaml_path = os.path.join(args.output_dir, 'data.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f'path: {args.output_dir}\n')
        f.write('train: images\n')
        f.write('val: images\n')
        f.write('\n')
        f.write('names:\n')
        f.write('  0: rebar\n')
    print(f'  config: {yaml_path}')
    print('\n라벨 보정 후 YOLO seg 학습:')
    print('  yolo segment train data=data.yaml model=yolo11n-seg.pt epochs=100')


if __name__ == '__main__':
    main()
