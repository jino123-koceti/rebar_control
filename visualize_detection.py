#!/usr/bin/env python3
"""
검출 결과 시각화 스크립트

우측 카메라 이미지 위에 검출된 교차점을 오버레이하여 PNG로 저장
- 바운딩 박스 (노란색)
- 중심점 (빨간 원)
- P1~P3 라벨 + (X, Y) 로봇 좌표 (mm)

사용법:
  python3 visualize_detection.py
  # 결과: detection_result.png
"""

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

CALIBRATION_DATA = '/home/koceti/ros2_ws/calibration_data.json'


def load_calibration():
    """캘리브레이션 데이터에서 회귀 계수 계산"""
    try:
        with open(CALIBRATION_DATA) as f:
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

    # 각 검출의 mapped 좌표 계산
    mapped = []
    for det in detections:
        mx, my = apply_mapping(x_coeffs, y_coeffs,
                               det.pixel_u, det.pixel_v, det.depth_mm)
        mapped.append((mx, my))

    # 그룹핑: 이미 그룹에 속한 인덱스 추적
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

    # 각 그룹에서 평균 mapped 좌표 계산, 대표 detection은 최고 confidence
    result_dets = []
    result_mapped = []
    for group in groups:
        avg_mx = sum(mapped[i][0] for i in group) / len(group)
        avg_my = sum(mapped[i][1] for i in group) / len(group)
        best_idx = max(group, key=lambda i: detections[i].confidence)
        result_dets.append(detections[best_idx])
        result_mapped.append((avg_mx, avg_my, len(group)))

    return result_dets, result_mapped


class DetectionVisualizer(Node):
    def __init__(self):
        super().__init__('detection_visualizer')
        self.bridge = CvBridge()
        self.right_image = None
        self.got_image = False
        self.x_coeffs, self.y_coeffs = load_calibration()
        if self.x_coeffs is not None:
            self.get_logger().info('Calibration loaded')
        else:
            self.get_logger().warn('No calibration data')

        # 우측 카메라 RGB 구독
        self.image_sub = self.create_subscription(
            Image,
            '/zedxmini2/zed_node/left/image_rect_color',
            self.image_callback,
            1
        )

        # 검출 서비스 클라이언트
        self.detect_client = self.create_client(
            DetectCrossings,
            '/rebar/detect_crossings'
        )

        self.get_logger().info('Waiting for detection service...')

    def image_callback(self, msg):
        if not self.got_image:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.got_image = True
            self.get_logger().info(
                f'Right camera image captured: {self.right_image.shape}'
            )

    def run(self):
        # 1. 이미지 수신 대기
        self.get_logger().info('Waiting for right camera image...')
        timeout = time.monotonic() + 10.0
        while not self.got_image and time.monotonic() < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

        if not self.got_image:
            self.get_logger().error('No image received from right camera')
            return

        # 2. 검출 서비스 호출
        if not self.detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Detection service not available')
            return

        request = DetectCrossings.Request()
        request.camera_selection = 2  # 우측만
        request.confidence_threshold = 0.5
        request.expected_count = 6

        self.get_logger().info('Calling detection service...')
        future = self.detect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is None:
            self.get_logger().error('Detection service call failed')
            return

        response = future.result()
        self.get_logger().info(f'Detection: {response.message}')

        # 3. 시각화
        img = self.right_image.copy()
        detections = list(response.grid.detections)

        if not detections:
            self.get_logger().warn('No detections to visualize')
            cv2.imwrite('detection_result.png', img)
            return

        # 중복 제거 + 매핑
        raw_count = len(detections)
        detections, mapped_list = dedup_detections(
            detections, self.x_coeffs, self.y_coeffs, dist_mm=20.0)

        # mapped 좌표 기준 X 내림차순 정렬
        if mapped_list:
            pairs = list(zip(detections, mapped_list))
            pairs.sort(key=lambda p: p[1][0], reverse=True)
            detections, mapped_list = zip(*pairs)
            detections, mapped_list = list(detections), list(mapped_list)
        else:
            detections.sort(key=lambda d: d.x, reverse=True)

        colors = {
            'bbox': (0, 255, 255),      # 노란색 (BGR)
            'center': (0, 0, 255),       # 빨간색
            'text_bg': (0, 0, 0),        # 검정 배경
            'text': (255, 255, 255),     # 흰색 텍스트
        }
        bbox_half = 30  # 바운딩 박스 반폭 (픽셀)

        for i, det in enumerate(detections):
            u, v = int(det.pixel_u), int(det.pixel_v)
            label = f'P{i+1}'
            coord = f'({det.x:.1f}, {det.y:.1f})'

            # 바운딩 박스
            pt1 = (u - bbox_half, v - bbox_half)
            pt2 = (u + bbox_half, v + bbox_half)
            cv2.rectangle(img, pt1, pt2, colors['bbox'], 2)

            # 중심점 (빨간 원)
            cv2.circle(img, (u, v), 8, colors['center'], -1)
            cv2.circle(img, (u, v), 10, colors['center'], 2)

            # 텍스트 라인 구성
            lines = [
                (label, colors['center']),
                (coord, colors['text']),
                (f'conf:{det.confidence:.2f} d:{det.depth_mm:.0f}mm', colors['text']),
            ]

            # 캘리브레이션 매핑 적용
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

        # 이미지 상단에 요약 정보
        dedup_info = f' (raw:{raw_count})' if raw_count != len(detections) else ''
        summary = f'Right Camera | {len(detections)} pts{dedup_info} | {response.detection_time_ms:.1f}ms'
        cv2.putText(img, summary, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 4. 저장
        output_path = '/home/koceti/ros2_ws/detection_result.png'
        cv2.imwrite(output_path, img)
        self.get_logger().info(f'Saved: {output_path}')

        # 콘솔에도 좌표 출력
        print(f'\n=== Detection Results ({len(detections)} pts, raw:{raw_count}) ===')
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


def main():
    rclpy.init()
    node = DetectionVisualizer()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
