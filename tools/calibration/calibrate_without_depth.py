#!/usr/bin/env python3
"""
Depth-free 검출 좌표 캘리브레이션 스크립트

ZED depth 노이즈를 제거하고 pixel_u, pixel_v 만으로 로봇 좌표 매핑.
기존 calibration_data_{camera}.json 데이터를 공유하며,
결과는 calibration_result_nodepth_{camera}.yaml로 별도 저장.

사용법:
  python3 calibrate_without_depth.py              # 우측 카메라 (기본)
  python3 calibrate_without_depth.py --camera left
  python3 calibrate_without_depth.py --camera right

명령어:
  - 실측값 입력: "25.5 130.0" (X Y, mm 단위)
  - 's' : 해당 포인트 스킵
  - 'q' : 종료
  - 'show' : 수집 데이터 표시
  - 'calc' : 매핑 계산
  - 'test' : 현재 검출값에 매핑 적용 테스트
  - 'compare' : depth 유무 성능 비교
  - 'outlier' : 아웃라이어 분석
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
import time
import yaml

WS_DIR = '/home/koceti/ros2_ws'
DATA_DIR = os.path.join(WS_DIR, 'data', 'calibration')

CAMERA_CONFIG = {
    'right': {
        'camera_selection': 2,
        'image_topic': '/zedxmini2/zed_node/left/image_rect_color',
        'camera_name': 'Right Camera (zedxmini2)',
        'data_file': os.path.join(DATA_DIR, 'calibration_data_right.json'),
        'result_file': os.path.join(DATA_DIR, 'calibration_result_nodepth_right.yaml'),
        'viz_file': os.path.join(DATA_DIR, 'calibration_detect_right.png'),
    },
    'left': {
        'camera_selection': 1,
        'image_topic': '/zedxmini1/zed_node/left/image_rect_color',
        'camera_name': 'Left Camera (zedxmini1)',
        'data_file': os.path.join(DATA_DIR, 'calibration_data_left.json'),
        'result_file': os.path.join(DATA_DIR, 'calibration_result_nodepth_left.yaml'),
        'viz_file': os.path.join(DATA_DIR, 'calibration_detect_left.png'),
    },
}


class CalibrationNoDepth(Node):
    def __init__(self, camera_side='right'):
        super().__init__('calibration_nodepth')
        self.bridge = CvBridge()
        self.camera_image = None
        self.got_image = False

        self.camera_side = camera_side
        cfg = CAMERA_CONFIG[camera_side]
        self.camera_selection = cfg['camera_selection']
        self.camera_name = cfg['camera_name']
        self.data_file = cfg['data_file']
        self.result_file = cfg['result_file']
        self.viz_file = cfg['viz_file']

        self.image_sub = self.create_subscription(
            Image, cfg['image_topic'], self.image_callback, 1)
        self.detect_client = self.create_client(
            DetectCrossings, '/rebar/detect_crossings')

        self.cal_data = self._load_data()
        self.x_coeffs = None
        self.y_coeffs = None

        if len(self.cal_data) >= 6:
            self._compute_mapping_silent()

        self.get_logger().info(
            '[%s] 기존 데이터: %d 쌍' % (self.camera_name, len(self.cal_data)))

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

    def _build_features(self, pixel_u, pixel_v):
        """설계 행렬: [pixel_u, pixel_v, pixel_u*pixel_v, pixel_u², pixel_v², 1]"""
        pixel_u = np.asarray(pixel_u, dtype=np.float64)
        pixel_v = np.asarray(pixel_v, dtype=np.float64)
        if pixel_u.ndim == 0:
            pixel_u, pixel_v = pixel_u.reshape(1), pixel_v.reshape(1)
        return np.column_stack([
            pixel_u, pixel_v,
            pixel_u * pixel_v,
            pixel_u ** 2, pixel_v ** 2,
            np.ones(len(pixel_u)),
        ])

    def _build_features_with_depth(self, pixel_u, pixel_v, depth):
        """비교용: 기존 depth 포함 설계 행렬"""
        pixel_u = np.asarray(pixel_u, dtype=np.float64)
        pixel_v = np.asarray(pixel_v, dtype=np.float64)
        depth = np.asarray(depth, dtype=np.float64)
        if pixel_u.ndim == 0:
            pixel_u = pixel_u.reshape(1)
            pixel_v = pixel_v.reshape(1)
            depth = depth.reshape(1)
        return np.column_stack([
            pixel_u, pixel_v, depth,
            pixel_u * depth, pixel_v * depth,
            pixel_u ** 2, pixel_v ** 2,
            np.ones(len(pixel_u)),
        ])

    def _compute_mapping_silent(self):
        """매핑 계수만 계산 (출력 없음)"""
        n = len(self.cal_data)
        if n < 6:
            return
        pixel_u = np.array([d['pixel_u'] for d in self.cal_data], dtype=np.float64)
        pixel_v = np.array([d['pixel_v'] for d in self.cal_data], dtype=np.float64)
        actual_x = np.array([d['actual_x_mm'] for d in self.cal_data], dtype=np.float64)
        actual_y = np.array([d['actual_y_mm'] for d in self.cal_data], dtype=np.float64)
        A = self._build_features(pixel_u, pixel_v)
        self.x_coeffs, _, _, _ = np.linalg.lstsq(A, actual_x, rcond=None)
        self.y_coeffs, _, _, _ = np.linalg.lstsq(A, actual_y, rcond=None)

    def _compute_mapping(self):
        """다중 선형회귀 (depth 없음): (pixel_u, pixel_v) → X, Y"""
        n = len(self.cal_data)
        if n < 6:
            print('  [매핑] 최소 6쌍 필요 (현재 %d쌍)' % n)
            return

        pixel_u = np.array([d['pixel_u'] for d in self.cal_data], dtype=np.float64)
        pixel_v = np.array([d['pixel_v'] for d in self.cal_data], dtype=np.float64)
        actual_x = np.array([d['actual_x_mm'] for d in self.cal_data], dtype=np.float64)
        actual_y = np.array([d['actual_y_mm'] for d in self.cal_data], dtype=np.float64)

        A = self._build_features(pixel_u, pixel_v)

        self.x_coeffs, _, _, _ = np.linalg.lstsq(A, actual_x, rcond=None)
        x_pred = A @ self.x_coeffs
        x_errors = np.abs(x_pred - actual_x)
        x_ss_res = np.sum((actual_x - x_pred) ** 2)
        x_ss_tot = np.sum((actual_x - np.mean(actual_x)) ** 2)
        x_r2 = 1 - x_ss_res / x_ss_tot if x_ss_tot > 0 else 0

        self.y_coeffs, _, _, _ = np.linalg.lstsq(A, actual_y, rcond=None)
        y_pred = A @ self.y_coeffs
        y_errors = np.abs(y_pred - actual_y)
        y_ss_res = np.sum((actual_y - y_pred) ** 2)
        y_ss_tot = np.sum((actual_y - np.mean(actual_y)) ** 2)
        y_r2 = 1 - y_ss_res / y_ss_tot if y_ss_tot > 0 else 0

        total_errors = np.sqrt(x_errors ** 2 + y_errors ** 2)

        print('\n  [매핑] pixel-only 회귀 (%d쌍, depth 미사용)' % n)
        print('  features: [pixel_u, pixel_v, u*v, u², v², 1]')
        print('  X: R²=%.4f  평균오차=%.1fmm  최대=%.1fmm' % (
            x_r2, np.mean(x_errors), np.max(x_errors)))
        print('  Y: R²=%.4f  평균오차=%.1fmm  최대=%.1fmm' % (
            y_r2, np.mean(y_errors), np.max(y_errors)))
        print('  XY총합: 평균오차=%.1fmm, 최대=%.1fmm' % (
            np.mean(total_errors), np.max(total_errors)))

        self._save_mapping(np.mean(total_errors), np.max(total_errors), x_r2, y_r2)
        print('  저장: %s' % self.result_file)
        print()

    def _apply_mapping(self, pixel_u, pixel_v):
        """(pixel_u, pixel_v) → 보정 좌표 (mm)"""
        if self.x_coeffs is None or self.y_coeffs is None:
            return (None, None)
        A = self._build_features(pixel_u, pixel_v)
        x_mm = float(self.x_coeffs @ A[0])
        y_mm = float(self.y_coeffs @ A[0])
        return (x_mm, y_mm)

    def _test_mapping(self, detections):
        """현재 검출 결과에 매핑 적용"""
        if self.x_coeffs is None:
            print('  [테스트] 매핑 없음. 먼저 calc 실행')
            return
        print('\n  === 매핑 테스트 (pixel-only) ===')
        for i, det in enumerate(detections):
            mx, my = self._apply_mapping(det.pixel_u, det.pixel_v)
            print('  P%d: pixel=(%d, %d) → X=%.1fmm, Y=%.1fmm' % (
                i + 1, det.pixel_u, det.pixel_v, mx, my))
        print()

    def _compare_models(self):
        """depth 유무 성능 비교"""
        n = len(self.cal_data)
        if n < 6:
            print('  최소 6쌍 필요 (현재 %d쌍)' % n)
            return

        pixel_u = np.array([d['pixel_u'] for d in self.cal_data], dtype=np.float64)
        pixel_v = np.array([d['pixel_v'] for d in self.cal_data], dtype=np.float64)
        depth = np.array([d['depth_mm'] for d in self.cal_data], dtype=np.float64)
        actual_x = np.array([d['actual_x_mm'] for d in self.cal_data], dtype=np.float64)
        actual_y = np.array([d['actual_y_mm'] for d in self.cal_data], dtype=np.float64)

        A_nodep = self._build_features(pixel_u, pixel_v)
        A_dep = self._build_features_with_depth(pixel_u, pixel_v, depth)

        print('\n  === depth 유무 성능 비교 (%d쌍) ===' % n)
        print('  %-28s %7s %7s %8s %8s' % ('모델', 'X R²', 'Y R²', 'avg', 'max'))
        print('  ' + '-' * 62)

        for label, A in [('pixel only (u,v,uv,u²,v²)', A_nodep),
                         ('with depth (기존)', A_dep)]:
            xc, _, _, _ = np.linalg.lstsq(A, actual_x, rcond=None)
            yc, _, _, _ = np.linalg.lstsq(A, actual_y, rcond=None)
            xp = A @ xc
            yp = A @ yc
            xye = np.sqrt((xp - actual_x) ** 2 + (yp - actual_y) ** 2)
            xr2 = 1 - np.sum((actual_x - xp) ** 2) / np.sum((actual_x - np.mean(actual_x)) ** 2)
            yr2 = 1 - np.sum((actual_y - yp) ** 2) / np.sum((actual_y - np.mean(actual_y)) ** 2)
            print('  %-28s %7.4f %7.4f %7.1fmm %7.1fmm' % (
                label, xr2, yr2, np.mean(xye), np.max(xye)))
        print()

    def _outlier_analysis(self):
        """아웃라이어 분석"""
        n = len(self.cal_data)
        if n < 6:
            print('  최소 6쌍 필요')
            return

        pixel_u = np.array([d['pixel_u'] for d in self.cal_data], dtype=np.float64)
        pixel_v = np.array([d['pixel_v'] for d in self.cal_data], dtype=np.float64)
        actual_x = np.array([d['actual_x_mm'] for d in self.cal_data], dtype=np.float64)
        actual_y = np.array([d['actual_y_mm'] for d in self.cal_data], dtype=np.float64)

        A = self._build_features(pixel_u, pixel_v)
        xc, _, _, _ = np.linalg.lstsq(A, actual_x, rcond=None)
        yc, _, _, _ = np.linalg.lstsq(A, actual_y, rcond=None)
        xp = A @ xc
        yp = A @ yc
        xy_err = np.sqrt((xp - actual_x) ** 2 + (yp - actual_y) ** 2)

        worst = np.argsort(xy_err)[::-1][:10]
        print('\n  === 아웃라이어 상위 10개 ===')
        print('  %4s %7s %7s %8s %8s %8s %8s %7s' % (
            '#', 'pix_u', 'pix_v', '실측X', '실측Y', '예측X', '예측Y', 'err'))
        for i in worst:
            d = self.cal_data[i]
            print('  %4d %7d %7d %8.1f %8.1f %8.1f %8.1f %7.1f' % (
                i + 1, d['pixel_u'], d['pixel_v'],
                d['actual_x_mm'], d['actual_y_mm'],
                xp[i], yp[i], xy_err[i]))

        print('\n  === 오차 분포 ===')
        for th in [5, 10, 15, 20, 30]:
            cnt = int(np.sum(xy_err <= th))
            print('  <=%2dmm: %3d/%d (%.0f%%)' % (th, cnt, n, 100 * cnt / n))

        print('\n  삭제하려면 "del 번호" (예: del 115)')
        print()

    def _delete_entry(self, idx):
        """데이터 항목 삭제 (1-based index)"""
        if idx < 1 or idx > len(self.cal_data):
            print('  유효하지 않은 번호: %d (1~%d)' % (idx, len(self.cal_data)))
            return
        d = self.cal_data[idx - 1]
        print('  삭제: #%d pixel=(%d,%d) actual=(%.1f, %.1f)' % (
            idx, d['pixel_u'], d['pixel_v'], d['actual_x_mm'], d['actual_y_mm']))
        self.cal_data.pop(idx - 1)
        self._save_data()
        if len(self.cal_data) >= 6:
            self._compute_mapping_silent()
        print('  남은 데이터: %d쌍' % len(self.cal_data))

    def _save_mapping(self, mean_err, max_err, x_r2, y_r2):
        """매핑 계수를 YAML로 저장"""
        data = {
            'calibration': {
                'method': 'pixel_only_regression',
                'num_points': len(self.cal_data),
                'mean_error_mm': round(float(mean_err), 2),
                'max_error_mm': round(float(max_err), 2),
                'camera': self.camera_name,
                'x_mapping': {
                    'inputs': ['pixel_u', 'pixel_v', 'pixel_u*pixel_v',
                               'pixel_u^2', 'pixel_v^2', '1'],
                    'coeffs': [round(float(c), 6) for c in self.x_coeffs],
                    'formula': 'X = a*u + b*v + c*u*v + d*u^2 + e*v^2 + f',
                    'r_squared': round(float(x_r2), 4),
                },
                'y_mapping': {
                    'inputs': ['pixel_u', 'pixel_v', 'pixel_u*pixel_v',
                               'pixel_u^2', 'pixel_v^2', '1'],
                    'coeffs': [round(float(c), 6) for c in self.y_coeffs],
                    'formula': 'Y = a*u + b*v + c*u*v + d*u^2 + e*v^2 + f',
                    'r_squared': round(float(y_r2), 4),
                },
            }
        }
        with open(self.result_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def _dedup_detections(self, detections, dist_mm=20.0):
        """인접한 검출 포인트를 평균으로 병합"""
        if self.x_coeffs is None or not detections:
            return detections, []

        mapped = []
        for det in detections:
            mx, my = self._apply_mapping(det.pixel_u, det.pixel_v)
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

        result_dets, result_mapped = [], []
        for group in groups:
            avg_mx = sum(mapped[i][0] for i in group) / len(group)
            avg_my = sum(mapped[i][1] for i in group) / len(group)
            best = max(group, key=lambda i: detections[i].confidence)
            result_dets.append(detections[best])
            result_mapped.append((avg_mx, avg_my, len(group)))

        return result_dets, result_mapped

    def detect(self):
        """검출 서비스 호출"""
        self.got_image = False
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
        return list(response.grid.detections), response

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

        dedup_info = ' (raw:%d)' % raw_count if raw_count != len(detections) else ''
        print('\n검출 결과: %d개%s (%.0fms)' % (
            len(detections), dedup_info, response.detection_time_ms))
        print('-' * 60)

        for i, det in enumerate(detections):
            line = ('  P%d: pixel=(%d, %d)  conf=%.2f  depth=%.0fmm' % (
                i + 1, det.pixel_u, det.pixel_v, det.confidence, det.depth_mm))
            if mapped_list:
                mx, my, cnt = mapped_list[i]
                line += '  -> mapped:(%.1f, %.1f)' % (mx, my)
                if cnt > 1:
                    line += ' avg:%d' % cnt
            print(line)

        if self.camera_image is not None:
            self._save_viz(detections, mapped_list)

        print('-' * 60)
        print('실측값 입력 (mm, "X Y")  s=스킵  q=종료')
        print()

        for i, det in enumerate(detections):
            while True:
                raw = input('  P%d pixel=(%d,%d) -> 실측(mm, X Y): ' % (
                    i + 1, det.pixel_u, det.pixel_v)).strip()

                if raw == 'q':
                    return False
                if raw == 's':
                    print('    P%d 스킵' % (i + 1))
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
                if raw == 'compare':
                    self._compare_models()
                    continue
                if raw == 'outlier':
                    self._outlier_analysis()
                    continue
                if raw.startswith('del '):
                    try:
                        self._delete_entry(int(raw.split()[1]))
                    except (ValueError, IndexError):
                        print('    형식: del 번호')
                    continue

                try:
                    parts = raw.replace(',', ' ').split()
                    if len(parts) != 2:
                        print('    형식: X Y (예: 25.7 130.2)')
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
                    print('    저장: (%.1f, %.1f)mm  [총 %d쌍]' % (
                        actual_x_mm, actual_y_mm, len(self.cal_data)))

                    if len(self.cal_data) >= 6:
                        self._compute_mapping()
                    break

                except ValueError:
                    print('    숫자를 입력하세요 (예: 25.7 130.2)')

        return True

    def _show_data(self):
        """수집된 데이터 표시"""
        print('\n  === 수집 데이터 (%d쌍) ===' % len(self.cal_data))
        if not self.cal_data:
            print('  (없음)')
            return
        print('  %4s %7s %7s %8s %8s' % ('#', 'pix_u', 'pix_v', '실측X', '실측Y'))
        for i, d in enumerate(self.cal_data):
            print('  %4d %7d %7d %8.1f %8.1f' % (
                i + 1, d['pixel_u'], d['pixel_v'],
                d['actual_x_mm'], d['actual_y_mm']))
        print()

    def _save_viz(self, detections, mapped_list):
        """검출 시각화 이미지 저장"""
        img = self.camera_image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        bbox_half = 30

        for i, det in enumerate(detections):
            u, v = int(det.pixel_u), int(det.pixel_v)
            cv2.rectangle(img, (u - bbox_half, v - bbox_half),
                          (u + bbox_half, v + bbox_half), (0, 255, 255), 2)
            cv2.circle(img, (u, v), 8, (0, 0, 255), -1)

            lines = [
                ('P%d' % (i + 1), (0, 0, 255)),
                ('pixel:(%d,%d)' % (det.pixel_u, det.pixel_v), (255, 255, 255)),
                ('conf:%.2f' % det.confidence, (255, 255, 255)),
            ]
            if mapped_list and i < len(mapped_list):
                mx, my, _ = mapped_list[i]
                lines.append(('-> (%.1f, %.1f)mm' % (mx, my), (0, 255, 0)))

            tx, ty = u - bbox_half, v + bbox_half + 18
            for li, (text, color) in enumerate(lines):
                yo = ty + li * 18
                (tw, th), _ = cv2.getTextSize(text, font, 0.45, 1)
                cv2.rectangle(img, (tx - 2, yo - th - 2),
                              (tx + tw + 2, yo + 3), (0, 0, 0), -1)
                cv2.putText(img, text, (tx, yo), font, 0.45, color, 1)

        summary = '%s | %d det | pixel-only | %d pairs' % (
            self.camera_name, len(detections), len(self.cal_data))
        cv2.putText(img, summary, (10, 30), font, 0.7, (0, 255, 0), 2)
        cv2.imwrite(self.viz_file, img)
        print('  [이미지] %s' % self.viz_file)


def main():
    parser = argparse.ArgumentParser(description='Pixel-only 캘리브레이션 (depth 미사용)')
    parser.add_argument('--camera', choices=['left', 'right'], default='right',
                        help='카메라 선택: left / right (기본)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CalibrationNoDepth(camera_side=args.camera)

    print('=' * 60)
    print(' Pixel-Only 캘리브레이션 [%s]' % node.camera_name)
    print(' (depth 미사용 - ZED depth 노이즈 제거)')
    print('=' * 60)
    print(' 데이터 파일: %s' % node.data_file)
    print(' 결과 파일:   %s' % node.result_file)
    print(' 기존 데이터: %d쌍' % len(node.cal_data))
    print()
    print(' Enter=검출  calc=매핑  compare=비교  outlier=분석  q=종료')
    print('=' * 60)

    try:
        while True:
            cmd = input('\n[Enter]=검출, [q]=종료: ').strip()
            if cmd == 'q':
                break
            if cmd == 'show':
                node._show_data()
            elif cmd == 'calc':
                node._compute_mapping()
            elif cmd == 'compare':
                node._compare_models()
            elif cmd == 'outlier':
                node._outlier_analysis()
            elif cmd.startswith('del '):
                try:
                    node._delete_entry(int(cmd.split()[1]))
                except (ValueError, IndexError):
                    print('  형식: del 번호')
            elif cmd == '':
                if not node.collect_round():
                    break
    except KeyboardInterrupt:
        print('\n종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('\n최종 데이터: %d쌍 -> %s' % (len(node.cal_data), node.data_file))


if __name__ == '__main__':
    main()
