#!/usr/bin/env python3
"""
검출 좌표 캘리브레이션 스크립트

사용법:
  python3 calibrate_detection.py              # 우측 카메라 (기본)
  python3 calibrate_detection.py --camera right
  python3 calibrate_detection.py --camera left

동작:
  1. 검출 서비스 호출 → 검출된 교차점 좌표 표시
  2. 각 포인트에 대해 실측값(cm) 입력 받음
  3. calibration_data_{camera}.json에 누적 저장
  4. 6쌍 이상 모이면 다중선형회귀 매핑 자동 계산
  5. 매핑 결과를 calibration_result_{camera}.yaml로 저장

명령어:
  - 실측값 입력: "1.6 3" (X Y, cm 단위)
  - 's' : 해당 포인트 스킵
  - 'q' : 종료
  - 'show' : 현재까지 수집된 데이터 표시
  - 'calc' : 현재 데이터로 매핑 행렬 강제 계산
  - 'test' : 매핑 적용 테스트 (검출값 → 보정값)
  - 'reset' : 수집 데이터 초기화
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
import os
import sys
import time
import yaml

CAMERA_CONFIG = {
    'right': {
        'camera_selection': 2,
        'image_topic': '/zedxmini2/zed_node/left/image_rect_color',
        'camera_name': 'Right Camera (zedxmini2)',
        'data_file': '/home/koceti/ros2_ws/calibration_data_right.json',
        'result_file': '/home/koceti/ros2_ws/calibration_result_right.yaml',
        'viz_file': '/home/koceti/ros2_ws/calibration_detect_right.png',
    },
    'left': {
        'camera_selection': 1,
        'image_topic': '/zedxmini1/zed_node/left/image_rect_color',
        'camera_name': 'Left Camera (zedxmini1)',
        'data_file': '/home/koceti/ros2_ws/calibration_data_left.json',
        'result_file': '/home/koceti/ros2_ws/calibration_result_left.yaml',
        'viz_file': '/home/koceti/ros2_ws/calibration_detect_left.png',
    },
}

# 기존 파일 호환 (right 카메라)
LEGACY_DATA_FILE = '/home/koceti/ros2_ws/calibration_data.json'
LEGACY_RESULT_FILE = '/home/koceti/ros2_ws/calibration_result.yaml'


class CalibrationCollector(Node):
    def __init__(self, camera_side='right'):
        super().__init__('calibration_collector')
        self.bridge = CvBridge()
        self.camera_image = None
        self.got_image = False

        # 카메라 설정
        self.camera_side = camera_side
        cfg = CAMERA_CONFIG[camera_side]
        self.camera_selection = cfg['camera_selection']
        self.camera_name = cfg['camera_name']
        self.data_file = cfg['data_file']
        self.result_file = cfg['result_file']
        self.viz_file = cfg['viz_file']

        # 기존 right 파일 마이그레이션 (calibration_data.json → calibration_data_right.json)
        if camera_side == 'right' and not os.path.exists(self.data_file) and os.path.exists(LEGACY_DATA_FILE):
            import shutil
            shutil.copy2(LEGACY_DATA_FILE, self.data_file)
            self.get_logger().info(f'기존 데이터 마이그레이션: {LEGACY_DATA_FILE} → {self.data_file}')
        if camera_side == 'right' and not os.path.exists(self.result_file) and os.path.exists(LEGACY_RESULT_FILE):
            import shutil
            shutil.copy2(LEGACY_RESULT_FILE, self.result_file)

        self.image_sub = self.create_subscription(
            Image,
            cfg['image_topic'],
            self.image_callback,
            1
        )

        self.detect_client = self.create_client(
            DetectCrossings,
            '/rebar/detect_crossings'
        )

        # 캘리브레이션 데이터 로드
        self.cal_data = self._load_data()
        # 다중 선형회귀 계수
        self.x_coeffs = None
        self.y_coeffs = None

        # 기존 데이터가 충분하면 매핑 계수 자동 로드
        if len(self.cal_data) >= 6:
            self._compute_mapping_silent()

        self.get_logger().info(f'[{self.camera_name}] 기존 데이터: {len(self.cal_data)} 쌍')

    def image_callback(self, msg):
        if not self.got_image:
            self.camera_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.got_image = True

    def _load_data(self):
        if os.path.exists(self.data_file):
            with open(self.data_file, 'r') as f:
                return json.load(f)
        return []

    def _save_data(self):
        with open(self.data_file, 'w') as f:
            json.dump(self.cal_data, f, indent=2)

    def detect(self):
        """검출 서비스 호출, 검출 리스트 반환"""
        self.got_image = False

        # 이미지 대기
        timeout = time.monotonic() + 5.0
        while not self.got_image and time.monotonic() < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

        if not self.detect_client.wait_for_service(timeout_sec=5.0):
            print('[ERROR] Detection service not available')
            return None, None

        request = DetectCrossings.Request()
        request.camera_selection = self.camera_selection
        request.confidence_threshold = 0.3
        request.expected_count = 6

        future = self.detect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is None:
            print('[ERROR] Detection failed')
            return None, None

        response = future.result()
        detections = list(response.grid.detections)

        return detections, response

    def collect_round(self):
        """한 라운드 검출 + 실측값 입력"""
        print('\n' + '=' * 60)
        print('검출 수행 중...')
        detections, response = self.detect()

        if detections is None:
            return True

        if not detections:
            print('검출된 교차점 없음')
            return True

        raw_count = len(detections)

        # 중복 제거 + 매핑 좌표 기준 정렬
        mapped_list = []
        if self.x_coeffs is not None:
            detections, mapped_list = self._dedup_detections(detections, dist_mm=20.0)
            if mapped_list:
                pairs = list(zip(detections, mapped_list))
                pairs.sort(key=lambda p: p[1][0], reverse=True)
                detections, mapped_list = zip(*pairs)
                detections, mapped_list = list(detections), list(mapped_list)
        else:
            detections.sort(key=lambda d: d.x, reverse=True)

        dedup_info = f' (raw:{raw_count})' if raw_count != len(detections) else ''
        print(f'\n검출 결과: {len(detections)}개{dedup_info} ({response.detection_time_ms:.0f}ms)')
        print('-' * 60)

        for i, det in enumerate(detections):
            label = f'P{i+1}'
            line = (f'  {label}: 검출=({det.x:.1f}, {det.y:.1f}) mm  '
                    f'depth={det.depth_mm:.0f}mm  conf={det.confidence:.2f}  '
                    f'pixel=({det.pixel_u},{det.pixel_v})')
            if mapped_list:
                mx, my, cnt = mapped_list[i]
                line += f'  → mapped:({mx:.1f}, {my:.1f})'
                if cnt > 1:
                    line += f' avg:{cnt}'
            print(line)

        # 시각화 저장
        if self.camera_image is not None:
            self._save_viz(detections)

        print('-' * 60)
        print('각 포인트의 실측값을 입력하세요 (mm 단위, "X Y")')
        print("  's'=스킵  'q'=종료  'show'=데이터보기  'calc'=매핑계산  'test'=테스트")
        print()

        for i, det in enumerate(detections):
            label = f'P{i+1}'
            while True:
                raw = input(f'  {label} 검출({det.x:.1f}, {det.y:.1f}) → 실측(mm): ').strip()

                if raw == 'q':
                    return False
                if raw == 's':
                    print(f'    {label} 스킵')
                    break
                if raw == 'show':
                    self._show_data()
                    continue
                if raw == 'calc':
                    self._compute_mapping()
                    continue
                if raw == 'test':
                    self._test_mapping(detections)
                    continue
                if raw == 'reset':
                    self.cal_data = []
                    self._save_data()
                    print('    데이터 초기화됨')
                    continue

                try:
                    parts = raw.replace(',', ' ').split()
                    if len(parts) != 2:
                        print('    형식: X Y (예: 25.7 10.2)')
                        continue
                    actual_x_mm = float(parts[0])
                    actual_y_mm = float(parts[1])

                    pair = {
                        'detected_x': round(det.x, 2),
                        'detected_y': round(det.y, 2),
                        'actual_x_mm': round(actual_x_mm, 1),
                        'actual_y_mm': round(actual_y_mm, 1),
                        'depth_mm': round(det.depth_mm, 1),
                        'pixel_u': det.pixel_u,
                        'pixel_v': det.pixel_v,
                        'confidence': round(det.confidence, 3),
                    }
                    self.cal_data.append(pair)
                    self._save_data()
                    print(f'    저장: 검출({det.x:.1f}, {det.y:.1f}) → '
                          f'실측({actual_x_mm:.1f}, {actual_y_mm:.1f})mm  '
                          f'[총 {len(self.cal_data)}쌍]')

                    # 6쌍 이상이면 자동 매핑 계산
                    if len(self.cal_data) >= 6:
                        self._compute_mapping()
                    break

                except ValueError:
                    print('    숫자를 입력하세요 (예: 1.6 3)')

        return True

    def _dedup_detections(self, detections, dist_mm=20.0):
        """인접한 검출 포인트를 평균으로 병합 (캘리브레이션 매핑 기반)"""
        if self.x_coeffs is None or not detections:
            return detections, []

        mapped = []
        for det in detections:
            mx, my = self._apply_mapping(det.pixel_u, det.pixel_v, det.depth_mm)
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

    def _show_data(self):
        """수집된 데이터 표시"""
        print(f'\n  === 수집 데이터 ({len(self.cal_data)}쌍) ===')
        if not self.cal_data:
            print('  (없음)')
            return
        print(f'  {"#":>3}  {"pixel_u":>8}  {"pixel_v":>8}  {"depth":>7}  {"실측X":>7}  {"실측Y":>7}  {"검출X":>8}  {"검출Y":>8}')
        for i, d in enumerate(self.cal_data):
            print(f'  {i+1:3d}  {d["pixel_u"]:8d}  {d["pixel_v"]:8d}  {d["depth_mm"]:7.0f}  '
                  f'{d["actual_x_mm"]:7.1f}  {d["actual_y_mm"]:7.1f}  '
                  f'{d["detected_x"]:8.1f}  {d["detected_y"]:8.1f}')
        print()

    def _build_features(self, pixel_u, pixel_v, depth):
        """설계 행렬 생성: [pixel_u, pixel_v, depth, pixel_u*depth, pixel_v*depth, 1]"""
        pixel_u = np.asarray(pixel_u, dtype=np.float64)
        pixel_v = np.asarray(pixel_v, dtype=np.float64)
        depth = np.asarray(depth, dtype=np.float64)
        if pixel_u.ndim == 0:
            pixel_u, pixel_v, depth = pixel_u.reshape(1), pixel_v.reshape(1), depth.reshape(1)
        return np.column_stack([pixel_u, pixel_v, depth,
                                pixel_u * depth, pixel_v * depth,
                                np.ones(len(pixel_u))])

    def _compute_mapping_silent(self):
        """매핑 계수만 계산 (출력 없음)"""
        n = len(self.cal_data)
        if n < 6:
            return
        pixel_u = np.array([d['pixel_u'] for d in self.cal_data], dtype=np.float64)
        pixel_v = np.array([d['pixel_v'] for d in self.cal_data], dtype=np.float64)
        depth = np.array([d['depth_mm'] for d in self.cal_data], dtype=np.float64)
        actual_x = np.array([d['actual_x_mm'] for d in self.cal_data], dtype=np.float64)
        actual_y = np.array([d['actual_y_mm'] for d in self.cal_data], dtype=np.float64)
        A = self._build_features(pixel_u, pixel_v, depth)
        self.x_coeffs, _, _, _ = np.linalg.lstsq(A, actual_x, rcond=None)
        self.y_coeffs, _, _, _ = np.linalg.lstsq(A, actual_y, rcond=None)

    def _compute_mapping(self):
        """다중 선형회귀 + 교차항: (pixel_u, pixel_v, depth) → X, Y"""
        n = len(self.cal_data)
        if n < 6:
            print(f'  [매핑] 최소 6쌍 필요 (현재 {n}쌍)')
            return

        pixel_u = np.array([d['pixel_u'] for d in self.cal_data], dtype=np.float64)
        pixel_v = np.array([d['pixel_v'] for d in self.cal_data], dtype=np.float64)
        depth = np.array([d['depth_mm'] for d in self.cal_data], dtype=np.float64)
        actual_x = np.array([d['actual_x_mm'] for d in self.cal_data], dtype=np.float64)
        actual_y = np.array([d['actual_y_mm'] for d in self.cal_data], dtype=np.float64)

        A = self._build_features(pixel_u, pixel_v, depth)

        self.x_coeffs, _, _, _ = np.linalg.lstsq(A, actual_x, rcond=None)
        x_pred = A @ self.x_coeffs
        x_errors = np.abs(x_pred - actual_x)
        x_ss_res = np.sum((actual_x - x_pred)**2)
        x_ss_tot = np.sum((actual_x - np.mean(actual_x))**2)
        x_r2 = 1 - x_ss_res / x_ss_tot if x_ss_tot > 0 else 0

        self.y_coeffs, _, _, _ = np.linalg.lstsq(A, actual_y, rcond=None)
        y_pred = A @ self.y_coeffs
        y_errors = np.abs(y_pred - actual_y)
        y_ss_res = np.sum((actual_y - y_pred)**2)
        y_ss_tot = np.sum((actual_y - np.mean(actual_y))**2)
        y_r2 = 1 - y_ss_res / y_ss_tot if y_ss_tot > 0 else 0

        total_errors = np.sqrt(x_errors**2 + y_errors**2)

        print(f'\n  [매핑] 다중 선형회귀 + 교차항 ({n}쌍)')
        print(f'  X: R²={x_r2:.4f}  평균오차={np.mean(x_errors):.1f}mm  최대={np.max(x_errors):.1f}mm')
        print(f'  Y: R²={y_r2:.4f}  평균오차={np.mean(y_errors):.1f}mm  최대={np.max(y_errors):.1f}mm')
        print(f'  총합: 평균오차={np.mean(total_errors):.1f}mm, 최대={np.max(total_errors):.1f}mm')

        self._save_mapping(np.mean(total_errors), np.max(total_errors), x_r2, y_r2)
        print(f'  저장: {self.result_file}')
        print()

    def _apply_mapping(self, pixel_u, pixel_v, depth_mm):
        """(pixel_u, pixel_v, depth) → 보정 좌표 (mm)"""
        if self.x_coeffs is None or self.y_coeffs is None:
            return (None, None)
        A = self._build_features(pixel_u, pixel_v, depth_mm)
        x_mm = float(self.x_coeffs @ A[0])
        y_mm = float(self.y_coeffs @ A[0])
        return (x_mm, y_mm)

    def _test_mapping(self, detections):
        """현재 검출 결과에 매핑 적용"""
        if self.x_coeffs is None:
            print('  [테스트] 매핑 없음. 먼저 데이터 수집 후 calc 실행')
            return
        print(f'\n  === 매핑 테스트 ===')
        for i, det in enumerate(detections):
            mapped = self._apply_mapping(det.pixel_u, det.pixel_v, det.depth_mm)
            print(f'  P{i+1}: pixel_u={det.pixel_u}, pixel_v={det.pixel_v}, depth={det.depth_mm:.0f}mm → '
                  f'보정 X={mapped[0]:.1f}mm, Y={mapped[1]:.1f}mm')
        print()

    def _save_mapping(self, mean_err, max_err, x_r2, y_r2):
        """매핑 계수를 YAML로 저장"""
        data = {
            'calibration': {
                'method': 'multiple_linear_regression_with_cross_terms',
                'num_points': len(self.cal_data),
                'mean_error_mm': round(float(mean_err), 2),
                'max_error_mm': round(float(max_err), 2),
                'camera': self.camera_name,
                'x_mapping': {
                    'inputs': ['pixel_u', 'pixel_v', 'depth_mm', 'pixel_u*depth', 'pixel_v*depth', '1'],
                    'coeffs': [round(float(c), 6) for c in self.x_coeffs],
                    'formula': 'X = a*u + b*v + c*d + e*u*d + f*v*d + g',
                    'r_squared': round(float(x_r2), 4),
                },
                'y_mapping': {
                    'inputs': ['pixel_u', 'pixel_v', 'depth_mm', 'pixel_u*depth', 'pixel_v*depth', '1'],
                    'coeffs': [round(float(c), 6) for c in self.y_coeffs],
                    'formula': 'Y = a*u + b*v + c*d + e*u*d + f*v*d + g',
                    'r_squared': round(float(y_r2), 4),
                },
            }
        }
        with open(self.result_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        # right 카메라는 기존 파일에도 동기화
        if self.camera_side == 'right':
            with open(LEGACY_RESULT_FILE, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

    def _save_viz(self, detections):
        """검출 시각화 이미지 저장 (visualize_detection.py 스타일)"""
        img = self.camera_image.copy()
        bbox_half = 30

        colors = {
            'bbox': (0, 255, 255),      # 노란색
            'center': (0, 0, 255),       # 빨간색
            'text_bg': (0, 0, 0),        # 검정 배경
            'text': (255, 255, 255),     # 흰색
            'mapped': (0, 255, 0),       # 초록 (보정값)
        }
        font = cv2.FONT_HERSHEY_SIMPLEX

        for i, det in enumerate(detections):
            u, v = int(det.pixel_u), int(det.pixel_v)
            label = f'P{i+1}'

            # 바운딩 박스
            cv2.rectangle(img,
                          (u - bbox_half, v - bbox_half),
                          (u + bbox_half, v + bbox_half),
                          colors['bbox'], 2)

            # 중심점 (빨간 원)
            cv2.circle(img, (u, v), 8, colors['center'], -1)
            cv2.circle(img, (u, v), 10, colors['center'], 2)

            # 텍스트 라인 구성
            lines = [
                (label, colors['center']),
                (f'({det.x:.1f}, {det.y:.1f})', colors['text']),
                (f'conf:{det.confidence:.2f} d:{det.depth_mm:.0f}mm', colors['text']),
            ]

            # 매핑 적용 결과 추가
            if self.x_coeffs is not None:
                mapped = self._apply_mapping(det.pixel_u, det.pixel_v, det.depth_mm)
                lines.append((f'mapped:({mapped[0]:.1f}, {mapped[1]:.1f})', colors['mapped']))

            tx = u + bbox_half + 5
            ty = v - 15

            for line_idx, (text, color) in enumerate(lines):
                y_offset = ty + line_idx * 22
                (tw, th), _ = cv2.getTextSize(text, font, 0.55, 2)
                cv2.rectangle(img,
                              (tx - 2, y_offset - th - 4),
                              (tx + tw + 2, y_offset + 4),
                              colors['text_bg'], -1)
                cv2.putText(img, text, (tx, y_offset),
                            font, 0.55, color, 2)

        # 상단 요약
        summary = f'{self.camera_name} | {len(detections)} detections | cal_data: {len(self.cal_data)} pairs'
        cv2.putText(img, summary, (10, 30), font, 0.7, (0, 255, 0), 2)

        cv2.imwrite(self.viz_file, img)
        print(f'  [이미지] {self.viz_file}')


def main():
    parser = argparse.ArgumentParser(description='검출 좌표 캘리브레이션')
    parser.add_argument('--camera', choices=['left', 'right'], default='right',
                        help='카메라 선택: left(zedxmini1) / right(zedxmini2, 기본)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CalibrationCollector(camera_side=args.camera)

    print('=' * 60)
    print(f' 검출 좌표 캘리브레이션 [{node.camera_name}]')
    print('=' * 60)
    print(f' 데이터 파일: {node.data_file}')
    print(f' 결과 파일:   {node.result_file}')
    print(f' 기존 데이터: {len(node.cal_data)}쌍')
    print()
    print(' Enter=검출실행  q=종료  show=데이터  calc=매핑계산')
    print('=' * 60)

    try:
        while True:
            cmd = input('\n[Enter]=검출, [q]=종료: ').strip()
            if cmd == 'q':
                break
            if cmd == 'show':
                node._show_data()
                continue
            if cmd == 'calc':
                node._compute_mapping()
                continue
            if cmd == 'reset':
                node.cal_data = []
                node._save_data()
                print('데이터 초기화됨')
                continue

            if not node.collect_round():
                break

    except KeyboardInterrupt:
        print('\n종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f'\n최종 데이터: {len(node.cal_data)}쌍 → {node.data_file}')


if __name__ == '__main__':
    main()
