#!/usr/bin/env python3
"""
검출 결과 시각화 스크립트

카메라 이미지 위에 검출된 교차점을 오버레이하여 PNG로 저장

사용법:
  python3 visualize_detection.py                # 우측 카메라 (기본, 단일 검출)
  python3 visualize_detection.py --camera left
  python3 visualize_detection.py --camera both
  python3 visualize_detection.py --multi         # 다중 검출 + 추론 모드
  python3 visualize_detection.py --multi --camera both
  python3 visualize_detection.py --multi --tries 5  # 최대 5회 검출
"""

import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rebar_base_interfaces.srv import DetectCrossings
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import sys
import time

CALIBRATION_FILES = {
    'right': '/home/koceti/ros2_ws/calibration_data_right.json',
    'left': '/home/koceti/ros2_ws/calibration_data_left.json',
}

CAMERA_CONFIG = {
    'right': {
        'camera_selection': 2,
        'image_topic': '/zedxmini2/zed_node/left/image_rect_color',
        'label': 'Right Camera (zedxmini2)',
        'output': '/home/koceti/ros2_ws/detection_result.png',
    },
    'left': {
        'camera_selection': 1,
        'image_topic': '/zedxmini1/zed_node/left/image_rect_color',
        'label': 'Left Camera (zedxmini1)',
        'output': '/home/koceti/ros2_ws/detection_result_left.png',
    },
}

EXPECTED_POINTS_PER_CAMERA = 3
CLUSTER_DIST_MM = 40.0


def load_calibration(camera_side):
    """캘리브레이션 데이터에서 회귀 계수 계산"""
    calib_file = CALIBRATION_FILES.get(camera_side)
    if not calib_file:
        return None, None
    try:
        with open(calib_file) as f:
            data = json.load(f)
        if len(data) < 6:
            return None, None

        pixel_u = np.array([d['pixel_u'] for d in data], dtype=np.float64)
        pixel_v = np.array([d['pixel_v'] for d in data], dtype=np.float64)
        depth = np.array([d['depth_mm'] for d in data], dtype=np.float64)
        actual_x = np.array([d['actual_x_mm'] for d in data], dtype=np.float64)
        actual_y = np.array([d['actual_y_mm'] for d in data], dtype=np.float64)

        A = np.column_stack([pixel_u, pixel_v, depth,
                             pixel_u * depth, pixel_v * depth,
                             np.ones(len(data))])
        x_coeffs, _, _, _ = np.linalg.lstsq(A, actual_x, rcond=None)
        y_coeffs, _, _, _ = np.linalg.lstsq(A, actual_y, rcond=None)
        return x_coeffs, y_coeffs
    except Exception:
        return None, None


def apply_mapping(x_coeffs, y_coeffs, pixel_u, pixel_v, depth_mm):
    """캘리브레이션 매핑 적용"""
    v = np.array([pixel_u, pixel_v, depth_mm,
                  pixel_u * depth_mm, pixel_v * depth_mm, 1.0])
    return float(x_coeffs @ v), float(y_coeffs @ v)


def dedup_detections(detections, x_coeffs, y_coeffs, dist_mm=20.0):
    """인접한 검출 포인트를 평균으로 병합"""
    if x_coeffs is None or not detections:
        return detections, []

    mapped = []
    for det in detections:
        mx, my = apply_mapping(x_coeffs, y_coeffs,
                               det.pixel_u, det.pixel_v, det.depth_mm)
        mapped.append((mx, my))

    used = set()
    groups = []
    for i in range(len(detections)):
        if i in used:
            continue
        group = [i]
        used.add(i)
        for j in range(i + 1, len(detections)):
            if j in used:
                continue
            dx = mapped[i][0] - mapped[j][0]
            dy = mapped[i][1] - mapped[j][1]
            if (dx * dx + dy * dy) ** 0.5 < dist_mm:
                group.append(j)
                used.add(j)
        groups.append(group)

    result_dets = []
    result_mapped = []
    for group in groups:
        avg_mx = sum(mapped[i][0] for i in group) / len(group)
        avg_my = sum(mapped[i][1] for i in group) / len(group)
        best_idx = max(group, key=lambda i: detections[i].confidence)
        result_dets.append(detections[best_idx])
        result_mapped.append((avg_mx, avg_my, len(group)))

    return result_dets, result_mapped


# ============================================
# 다중 검출 클러스터링 + 보간
# ============================================

def cluster_points(all_points, dist_mm=CLUSTER_DIST_MM):
    """누적된 (x, y, pixel_u, pixel_v, depth_mm, conf, camera_id) 포인트를 클러스터링.

    Returns:
        list of dict: 각 클러스터의 평균 좌표 및 메타정보
    """
    used = set()
    clusters = []
    for i in range(len(all_points)):
        if i in used:
            continue
        group = [i]
        used.add(i)
        for j in range(i + 1, len(all_points)):
            if j in used:
                continue
            dx = all_points[i]['x'] - all_points[j]['x']
            dy = all_points[i]['y'] - all_points[j]['y']
            if (dx * dx + dy * dy) ** 0.5 < dist_mm:
                group.append(j)
                used.add(j)
        clusters.append(group)

    results = []
    for group in clusters:
        pts = [all_points[i] for i in group]
        results.append({
            'x': np.mean([p['x'] for p in pts]),
            'y': np.mean([p['y'] for p in pts]),
            'pixel_u': int(np.mean([p['pixel_u'] for p in pts])),
            'pixel_v': int(np.mean([p['pixel_v'] for p in pts])),
            'depth_mm': np.mean([p['depth_mm'] for p in pts]),
            'confidence': max(p['confidence'] for p in pts),
            'camera_id': pts[0]['camera_id'],
            'hit_count': len(group),
            'source': 'detected',
        })
    return results


def interpolate_missing(clusters, expected_count=EXPECTED_POINTS_PER_CAMERA):
    """클러스터 포인트가 기대 수보다 적으면 등간격 보간으로 누락 포인트 추론.

    X 기준 등간격 보간. Y는 기존 포인트의 평균 사용.
    """
    if len(clusters) >= expected_count or len(clusters) < 2:
        return clusters

    # X 내림차순 정렬
    clusters.sort(key=lambda c: c['x'], reverse=True)

    x_vals = [c['x'] for c in clusters]
    y_avg = np.mean([c['y'] for c in clusters])
    depth_avg = np.mean([c['depth_mm'] for c in clusters])
    cam_id = clusters[0]['camera_id']

    # 기존 포인트 간 X 간격 추정
    if len(clusters) == 2:
        # 2개로 3개 추론: 간격 = (x_max - x_min) / (expected - 1)
        x_max, x_min = max(x_vals), min(x_vals)
        spacing = (x_max - x_min) / (expected_count - 1)

        # 누락 위치 찾기: expected 위치와 기존 포인트 매칭
        expected_xs = [x_max - i * spacing for i in range(expected_count)]
        existing_xs = sorted(x_vals, reverse=True)

        for ex in expected_xs:
            # 기존 포인트와 40mm 이내면 이미 있음
            is_existing = any(abs(ex - cx) < CLUSTER_DIST_MM for cx in existing_xs)
            if not is_existing:
                # 보간 포인트의 pixel 좌표는 기존 포인트들로부터 선형 보간
                # X좌표 기준으로 pixel_u 보간
                c_high = clusters[0]  # X가 큰 쪽
                c_low = clusters[-1]  # X가 작은 쪽
                x_range = c_high['x'] - c_low['x']
                if x_range > 0:
                    ratio = (ex - c_low['x']) / x_range
                else:
                    ratio = 0.5
                pixel_u = int(c_low['pixel_u'] + ratio * (c_high['pixel_u'] - c_low['pixel_u']))
                pixel_v = int(c_low['pixel_v'] + ratio * (c_high['pixel_v'] - c_low['pixel_v']))

                clusters.append({
                    'x': ex,
                    'y': y_avg,
                    'pixel_u': pixel_u,
                    'pixel_v': pixel_v,
                    'depth_mm': depth_avg,
                    'confidence': 0.0,
                    'camera_id': cam_id,
                    'hit_count': 0,
                    'source': 'interpolated',
                })

    # 다시 X 내림차순 정렬
    clusters.sort(key=lambda c: c['x'], reverse=True)
    return clusters


def draw_multi_detections(img, clusters, camera_label, total_raw, num_tries, total_time_ms):
    """다중 검출 결과 오버레이 (검출/보간 구분 표시)"""
    colors = {
        'detected': (0, 255, 0),       # 초록 - 검출됨
        'interpolated': (0, 165, 255),  # 주황 - 보간됨
        'bbox': (0, 255, 255),
        'text_bg': (0, 0, 0),
        'text': (255, 255, 255),
    }
    bbox_half = 30

    for i, cl in enumerate(clusters):
        u, v = cl['pixel_u'], cl['pixel_v']
        is_interp = cl['source'] == 'interpolated'
        center_color = colors['interpolated'] if is_interp else colors['detected']

        # 바운딩 박스
        pt1 = (u - bbox_half, v - bbox_half)
        pt2 = (u + bbox_half, v + bbox_half)
        style = cv2.LINE_AA if not is_interp else cv2.LINE_4
        cv2.rectangle(img, pt1, pt2, colors['bbox'], 2, style)

        # 중심점
        if is_interp:
            # 보간: 빈 원 + X 표시
            cv2.circle(img, (u, v), 10, center_color, 2)
            cv2.line(img, (u - 6, v - 6), (u + 6, v + 6), center_color, 2)
            cv2.line(img, (u - 6, v + 6), (u + 6, v - 6), center_color, 2)
        else:
            cv2.circle(img, (u, v), 8, center_color, -1)
            cv2.circle(img, (u, v), 10, center_color, 2)

        label = f'P{i+1}'
        coord = f'({cl["x"]:.1f}, {cl["y"]:.1f})'

        lines = [
            (label, center_color),
            (coord, colors['text']),
        ]

        if is_interp:
            lines.append(('INTERPOLATED', colors['interpolated']))
        else:
            lines.append((f'hits:{cl["hit_count"]} conf:{cl["confidence"]:.2f}', colors['text']))
            lines.append((f'd:{cl["depth_mm"]:.0f}mm', colors['text']))

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 2

        tx = u + bbox_half + 5
        ty = v - 15

        for line_idx, (text, color) in enumerate(lines):
            y_offset = ty + line_idx * 22
            (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
            cv2.rectangle(img,
                          (tx - 2, y_offset - th - 4),
                          (tx + tw + 2, y_offset + 4),
                          colors['text_bg'], -1)
            cv2.putText(img, text, (tx, y_offset),
                        font, font_scale, color, thickness)

    detected_count = sum(1 for c in clusters if c['source'] == 'detected')
    interp_count = sum(1 for c in clusters if c['source'] == 'interpolated')
    summary = (f'{camera_label} | {detected_count} detected + {interp_count} interpolated '
               f'| {num_tries} tries, raw:{total_raw} | {total_time_ms:.0f}ms')
    cv2.putText(img, summary, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return img


def fit_trendline(clusters):
    """검출된 포인트들로 직선 추세선 피팅 (로봇 좌표 X,Y 기반).

    직선: Y = slope * X + intercept (X가 주축, 가로 방향)
    또는 X 변화가 적으면: X = slope * Y + intercept

    Returns:
        dict: {
            'corrected': [(x, y), ...] 각 클러스터의 보정 좌표,
            'line_eq': (slope, intercept, axis),
            'errors': [dist_mm, ...] 각 포인트의 직선과의 거리
        }
        or None if < 2 points
    """
    pts = [(c['x'], c['y']) for c in clusters]
    if len(pts) < 2:
        return None

    xs = np.array([p[0] for p in pts])
    ys = np.array([p[1] for p in pts])

    x_range = xs.max() - xs.min()
    y_range = ys.max() - ys.min()

    # 주축 결정: 변화가 큰 축을 독립변수로
    if x_range >= y_range:
        # Y = slope * X + intercept
        coeffs = np.polyfit(xs, ys, 1)
        slope, intercept = coeffs
        axis = 'x'

        # 각 포인트를 직선 위로 투영 (수직 최단거리 투영)
        # 직선: slope*X - Y + intercept = 0
        # a=slope, b=-1, c=intercept
        a, b, c = slope, -1.0, intercept
        corrected = []
        errors = []
        for x0, y0 in pts:
            # 투영점: (x', y') = 직선 위 가장 가까운 점
            x_proj = (b * (b * x0 - a * y0) - a * c) / (a * a + b * b)
            y_proj = (a * (-b * x0 + a * y0) - b * c) / (a * a + b * b)
            corrected.append((x_proj, y_proj))
            dist = abs(a * x0 + b * y0 + c) / np.sqrt(a * a + b * b)
            errors.append(dist)
    else:
        # X = slope * Y + intercept
        coeffs = np.polyfit(ys, xs, 1)
        slope, intercept = coeffs
        axis = 'y'

        a, b, c = -1.0, slope, intercept
        corrected = []
        errors = []
        for x0, y0 in pts:
            x_proj = (b * (b * x0 - a * y0) - a * c) / (a * a + b * b)
            y_proj = (a * (-b * x0 + a * y0) - b * c) / (a * a + b * b)
            corrected.append((x_proj, y_proj))
            dist = abs(a * x0 + b * y0 + c) / np.sqrt(a * a + b * b)
            errors.append(dist)

    return {
        'corrected': corrected,
        'line_eq': (slope, intercept, axis),
        'errors': errors,
    }


def draw_trendline_correction(img, clusters, trendline, calibration=None):
    """추세선 + 보정 포인트를 이미지에 오버레이.

    - 추세선: 하늘색 점선
    - 보정 포인트: 작은 붉은 원 (5px)
    - 검출→보정 연결선: 노란 점선
    - 보정 좌표 텍스트
    """
    if trendline is None or len(clusters) < 2:
        return img

    corrected = trendline['corrected']
    errors = trendline['errors']
    color_line = (255, 200, 100)    # 하늘색 - 추세선
    color_corr = (0, 0, 255)        # 붉은색 - 보정 포인트
    color_link = (0, 200, 255)      # 노란색 - 연결선

    # 추세선을 이미지에 그리기 위해 로봇좌표 → 픽셀 변환 필요
    # 기존 클러스터의 (robot_x, pixel_u) 관계로 간이 역변환
    robot_xs = np.array([c['x'] for c in clusters])
    robot_ys = np.array([c['y'] for c in clusters])
    pixel_us = np.array([c['pixel_u'] for c in clusters], dtype=float)
    pixel_vs = np.array([c['pixel_v'] for c in clusters], dtype=float)

    # 로봇 좌표 → 픽셀 간이 선형회귀 (역변환)
    if len(clusters) >= 2:
        A_rev = np.column_stack([robot_xs, robot_ys, np.ones(len(clusters))])
        try:
            u_coeffs, _, _, _ = np.linalg.lstsq(A_rev, pixel_us, rcond=None)
            v_coeffs, _, _, _ = np.linalg.lstsq(A_rev, pixel_vs, rcond=None)
        except Exception:
            return img

        def robot_to_pixel(rx, ry):
            feat = np.array([rx, ry, 1.0])
            return int(u_coeffs @ feat), int(v_coeffs @ feat)
    else:
        return img

    # 추세선 그리기 (로봇 좌표 범위에서 양쪽으로 확장)
    slope, intercept, axis = trendline['line_eq']
    x_min, x_max = robot_xs.min() - 30, robot_xs.max() + 30

    if axis == 'x':
        line_pts = []
        for x_val in np.linspace(x_min, x_max, 50):
            y_val = slope * x_val + intercept
            pu, pv = robot_to_pixel(x_val, y_val)
            line_pts.append((pu, pv))
    else:
        y_min, y_max = robot_ys.min() - 30, robot_ys.max() + 30
        line_pts = []
        for y_val in np.linspace(y_min, y_max, 50):
            x_val = slope * y_val + intercept
            pu, pv = robot_to_pixel(x_val, y_val)
            line_pts.append((pu, pv))

    # 추세선 점선 그리기
    for k in range(len(line_pts) - 1):
        if k % 3 < 2:  # 점선 패턴
            cv2.line(img, line_pts[k], line_pts[k + 1], color_line, 2)

    # 보정 포인트 그리기
    for i, (cx, cy) in enumerate(corrected):
        corr_u, corr_v = robot_to_pixel(cx, cy)
        orig_u, orig_v = clusters[i]['pixel_u'], clusters[i]['pixel_v']

        # 연결선 (검출 → 보정)
        cv2.line(img, (orig_u, orig_v), (corr_u, corr_v), color_link, 1, cv2.LINE_AA)

        # 보정 포인트 (작은 붉은 원)
        cv2.circle(img, (corr_u, corr_v), 5, color_corr, -1)
        cv2.circle(img, (corr_u, corr_v), 7, color_corr, 1)

        # 보정 좌표 + 오차 텍스트
        err = errors[i]
        text = f'corr:({cx:.1f},{cy:.1f}) err:{err:.1f}mm'
        font = cv2.FONT_HERSHEY_SIMPLEX

        tx = corr_u - 50
        ty = corr_v + 25
        (tw, th), _ = cv2.getTextSize(text, font, 0.45, 1)
        cv2.rectangle(img, (tx - 2, ty - th - 2), (tx + tw + 2, ty + 2),
                      (0, 0, 0), -1)
        cv2.putText(img, text, (tx, ty), font, 0.45, color_corr, 1)

    # 추세선 정보 (이미지 하단)
    if axis == 'x':
        eq_text = f'Trendline: Y = {slope:.4f}*X + {intercept:.1f}'
    else:
        eq_text = f'Trendline: X = {slope:.4f}*Y + {intercept:.1f}'
    avg_err = np.mean(errors)
    max_err = np.max(errors)
    info_text = f'{eq_text} | avg_err:{avg_err:.1f}mm max_err:{max_err:.1f}mm'

    h = img.shape[0]
    cv2.putText(img, info_text, (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_line, 1)

    return img


# ============================================
# 단일 검출용 그리기 (기존)
# ============================================

def draw_detections(img, detections, mapped_list, camera_label, raw_count, detection_time_ms):
    """이미지 위에 검출 결과 오버레이"""
    colors = {
        'bbox': (0, 255, 255),
        'center': (0, 0, 255),
        'text_bg': (0, 0, 0),
        'text': (255, 255, 255),
    }
    bbox_half = 30

    for i, det in enumerate(detections):
        u, v = int(det.pixel_u), int(det.pixel_v)
        label = f'P{i+1}'
        coord = f'({det.x:.1f}, {det.y:.1f})'

        pt1 = (u - bbox_half, v - bbox_half)
        pt2 = (u + bbox_half, v + bbox_half)
        cv2.rectangle(img, pt1, pt2, colors['bbox'], 2)

        cv2.circle(img, (u, v), 8, colors['center'], -1)
        cv2.circle(img, (u, v), 10, colors['center'], 2)

        lines = [
            (label, colors['center']),
            (coord, colors['text']),
            (f'conf:{det.confidence:.2f} d:{det.depth_mm:.0f}mm', colors['text']),
        ]

        if mapped_list:
            mx, my, cnt = mapped_list[i]
            tag = f'mapped:({mx:.1f}, {my:.1f})'
            if cnt > 1:
                tag += f' avg:{cnt}'
            lines.append((tag, (0, 255, 0)))

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 2

        tx = u + bbox_half + 5
        ty = v - 15

        for line_idx, (text, color) in enumerate(lines):
            y_offset = ty + line_idx * 22
            (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
            cv2.rectangle(img,
                          (tx - 2, y_offset - th - 4),
                          (tx + tw + 2, y_offset + 4),
                          colors['text_bg'], -1)
            cv2.putText(img, text, (tx, y_offset),
                        font, font_scale, color, thickness)

    dedup_info = f' (raw:{raw_count})' if raw_count != len(detections) else ''
    summary = f'{camera_label} | {len(detections)} pts{dedup_info} | {detection_time_ms:.1f}ms'
    cv2.putText(img, summary, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return img


class DetectionVisualizer(Node):
    def __init__(self, camera_mode='right', multi_mode=False, max_tries=5):
        super().__init__('detection_visualizer')
        self.bridge = CvBridge()
        self.camera_mode = camera_mode
        self.multi_mode = multi_mode
        self.max_tries = max_tries

        # 카메라별 이미지 캐시
        self.images = {}
        self.got_images = set()

        # 카메라별 캘리브레이션 로드
        self.calibrations = {}

        # 구독할 카메라 결정
        if camera_mode == 'both':
            cameras = ['left', 'right']
        else:
            cameras = [camera_mode]

        for cam in cameras:
            cfg = CAMERA_CONFIG[cam]
            self.images[cam] = None

            x_c, y_c = load_calibration(cam)
            if x_c is not None:
                self.calibrations[cam] = (x_c, y_c)
                self.get_logger().info(f'Calibration loaded [{cam}]')
            else:
                self.get_logger().warn(f'No calibration data [{cam}]')

            self.create_subscription(
                Image, cfg['image_topic'],
                lambda msg, c=cam: self._image_callback(msg, c), 1
            )

        self.detect_client = self.create_client(
            DetectCrossings, '/rebar/detect_crossings'
        )

        mode_str = 'multi' if multi_mode else 'single'
        self.get_logger().info(
            f'Mode: {camera_mode} ({mode_str}, max_tries={max_tries})')

    def _image_callback(self, msg, camera):
        if camera not in self.got_images:
            self.images[camera] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.got_images.add(camera)
            self.get_logger().info(
                f'{camera} camera image captured: {self.images[camera].shape}'
            )

    def _wait_for_images(self):
        if self.camera_mode == 'both':
            needed = {'left', 'right'}
        else:
            needed = {self.camera_mode}

        self.get_logger().info(f'Waiting for camera images: {needed}...')
        timeout = time.monotonic() + 10.0
        while not needed.issubset(self.got_images) and time.monotonic() < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

        missing = needed - self.got_images
        if missing:
            self.get_logger().error(f'No image received from: {missing}')
            return False
        return True

    def _detect(self, camera_selection):
        request = DetectCrossings.Request()
        request.camera_selection = camera_selection
        request.confidence_threshold = 0.3
        request.expected_count = 6

        future = self.detect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is None:
            return None
        return future.result()

    # ============================================
    # 단일 검출 모드
    # ============================================
    def _process_camera(self, camera):
        cfg = CAMERA_CONFIG[camera]
        response = self._detect(cfg['camera_selection'])
        if response is None:
            return

        self.get_logger().info(f'[{camera}] {response.message}')

        img = self.images[camera].copy()
        detections = list(response.grid.detections)

        if not detections:
            self.get_logger().warn(f'[{camera}] No detections')
            cv2.imwrite(cfg['output'], img)
            return

        raw_count = len(detections)

        x_c, y_c = self.calibrations.get(camera, (None, None))
        detections, mapped_list = dedup_detections(
            detections, x_c, y_c, dist_mm=20.0)

        if mapped_list:
            pairs = list(zip(detections, mapped_list))
            pairs.sort(key=lambda p: p[1][0], reverse=True)
            detections, mapped_list = zip(*pairs)
            detections, mapped_list = list(detections), list(mapped_list)
        else:
            detections.sort(key=lambda d: d.x, reverse=True)

        img = draw_detections(img, detections, mapped_list,
                              cfg['label'], raw_count, response.detection_time_ms)

        cv2.imwrite(cfg['output'], img)
        self.get_logger().info(f'Saved: {cfg["output"]}')

        print(f'\n=== {cfg["label"]} ({len(detections)} pts, raw:{raw_count}) ===')
        for i, det in enumerate(detections):
            line = (f'  P{i+1}: conf={det.confidence:.2f}  depth={det.depth_mm:.0f}mm  '
                    f'pixel=({det.pixel_u},{det.pixel_v})')
            if mapped_list:
                mx, my, cnt = mapped_list[i]
                line += f'  → mapped:({mx:.1f}, {my:.1f})'
                if cnt > 1:
                    line += f' (avg of {cnt})'
            print(line)
        print()

    # ============================================
    # 다중 검출 모드
    # ============================================
    def _process_camera_multi(self, camera):
        """다중 검출 + 클러스터링 + 보간"""
        cfg = CAMERA_CONFIG[camera]
        all_points = []
        total_raw = 0
        total_time_ms = 0.0

        self.get_logger().info(
            f'[{camera}] 다중 검출 시작 (최대 {self.max_tries}회)...')

        for trial in range(self.max_tries):
            response = self._detect(cfg['camera_selection'])
            if response is None:
                self.get_logger().warn(f'  [{trial+1}] 서비스 호출 실패')
                continue

            dets = list(response.grid.detections)
            total_time_ms += response.detection_time_ms
            total_raw += len(dets)

            self.get_logger().info(
                f'  [{trial+1}/{self.max_tries}] {len(dets)}개 검출 '
                f'({response.detection_time_ms:.0f}ms)')

            for det in dets:
                # X < -40mm 범위 밖 포인트 제외
                if det.x < -40.0:
                    self.get_logger().debug(
                        f'  범위 외 제외: X={det.x:.1f}mm')
                    continue
                all_points.append({
                    'x': det.x,
                    'y': det.y,
                    'pixel_u': det.pixel_u,
                    'pixel_v': det.pixel_v,
                    'depth_mm': det.depth_mm,
                    'confidence': det.confidence,
                    'camera_id': det.camera_id,
                })

            # 이미 충분한 유효 클러스터가 모이면 조기 종료
            if trial >= 1:
                temp_clusters = cluster_points(all_points, CLUSTER_DIST_MM)
                valid_clusters = [c for c in temp_clusters if c['x'] >= -40.0]
                if len(valid_clusters) >= EXPECTED_POINTS_PER_CAMERA:
                    self.get_logger().info(
                        f'  {len(valid_clusters)}개 유효 클러스터 확보 → 조기 종료')
                    break

            # 다음 검출 전 짧은 대기 (프레임 갱신)
            if trial < self.max_tries - 1:
                time.sleep(0.3)

        if not all_points:
            self.get_logger().warn(f'[{camera}] 검출 결과 없음')
            cv2.imwrite(cfg['output'], self.images[camera])
            return

        # 클러스터링
        clusters = cluster_points(all_points, CLUSTER_DIST_MM)
        self.get_logger().info(
            f'[{camera}] 클러스터링: {len(all_points)}개 누적 → '
            f'{len(clusters)}개 클러스터')

        # 보간
        before_interp = len(clusters)
        clusters = interpolate_missing(clusters, EXPECTED_POINTS_PER_CAMERA)
        interp_count = len(clusters) - before_interp
        if interp_count > 0:
            self.get_logger().info(
                f'[{camera}] 보간: {interp_count}개 포인트 추론 추가')

        # X 내림차순 정렬
        clusters.sort(key=lambda c: c['x'], reverse=True)

        # 추세선 피팅
        trendline = fit_trendline(clusters)

        # 시각화 (검출 + 추세선 통합)
        img = self.images[camera].copy()
        img = draw_multi_detections(
            img, clusters, cfg['label'], total_raw, self.max_tries, total_time_ms)
        if trendline is not None:
            img = draw_trendline_correction(img, clusters, trendline)

        # 출력 파일명에 _multi 추가
        output = cfg['output'].replace('.png', '_multi.png')
        cv2.imwrite(output, img)
        self.get_logger().info(f'Saved: {output}')

        # 콘솔 출력
        print(f'\n=== {cfg["label"]} Multi-Detection ===')
        print(f'  Tries: {self.max_tries}, Total raw: {total_raw}, '
              f'Clusters: {before_interp}, Interpolated: {interp_count}')
        print(f'  Total time: {total_time_ms:.0f}ms')
        for i, cl in enumerate(clusters):
            src = 'INTERP' if cl['source'] == 'interpolated' else f'hits:{cl["hit_count"]}'
            line = (f'  P{i+1}: X={cl["x"]:.1f} Y={cl["y"]:.1f}  '
                    f'pixel=({cl["pixel_u"]},{cl["pixel_v"]})  '
                    f'd={cl["depth_mm"]:.0f}mm  conf={cl["confidence"]:.2f}  [{src}]')
            if trendline:
                cx, cy = trendline['corrected'][i]
                err = trendline['errors'][i]
                line += f'  → corr:({cx:.1f},{cy:.1f}) err:{err:.1f}mm'
            print(line)

        if trendline:
            slope, intercept, axis = trendline['line_eq']
            avg_err = np.mean(trendline['errors'])
            if axis == 'x':
                print(f'  Trendline: Y = {slope:.4f}*X + {intercept:.1f}')
            else:
                print(f'  Trendline: X = {slope:.4f}*Y + {intercept:.1f}')
            print(f'  Avg error: {avg_err:.1f}mm')
        print()

        return clusters

    # ============================================
    # 실행
    # ============================================
    def run(self):
        if not self._wait_for_images():
            return

        if not self.detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Detection service not available')
            return

        if self.multi_mode:
            self._run_multi()
        else:
            self._run_single()

    def _run_single(self):
        if self.camera_mode == 'both':
            for cam in ['left', 'right']:
                self._process_camera(cam)
            self._save_combined()
        else:
            self._process_camera(self.camera_mode)

    def _run_multi(self):
        if self.camera_mode == 'both':
            for cam in ['left', 'right']:
                self._process_camera_multi(cam)
            self._save_combined(multi=True)
        else:
            self._process_camera_multi(self.camera_mode)

    def _save_combined(self, multi=False):
        suffix = '_multi' if multi else ''
        left_path = CAMERA_CONFIG['left']['output'].replace('.png', f'{suffix}.png')
        right_path = CAMERA_CONFIG['right']['output'].replace('.png', f'{suffix}.png')

        left_img = cv2.imread(left_path)
        right_img = cv2.imread(right_path)
        if left_img is not None and right_img is not None:
            h = max(left_img.shape[0], right_img.shape[0])
            if left_img.shape[0] != h:
                left_img = cv2.resize(
                    left_img, (int(left_img.shape[1] * h / left_img.shape[0]), h))
            if right_img.shape[0] != h:
                right_img = cv2.resize(
                    right_img, (int(right_img.shape[1] * h / right_img.shape[0]), h))
            combined = np.hstack([left_img, right_img])
            combined_path = f'/home/koceti/ros2_ws/detection_result_both{suffix}.png'
            cv2.imwrite(combined_path, combined)
            self.get_logger().info(f'Saved combined: {combined_path}')


def main():
    parser = argparse.ArgumentParser(description='검출 결과 시각화')
    parser.add_argument('--camera', choices=['left', 'right', 'both'],
                        default='right', help='카메라 선택 (기본: right)')
    parser.add_argument('--multi', action='store_true',
                        help='다중 검출 + 클러스터링 + 보간 모드')
    parser.add_argument('--tries', type=int, default=5,
                        help='다중 검출 최대 시도 횟수 (기본: 5)')
    args = parser.parse_args()

    rclpy.init()
    node = DetectionVisualizer(
        camera_mode=args.camera,
        multi_mode=args.multi,
        max_tries=args.tries
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
