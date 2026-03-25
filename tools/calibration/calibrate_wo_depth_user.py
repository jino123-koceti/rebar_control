#!/usr/bin/env python3
"""
Depth-free 검출 좌표 캘리브레이션 스크립트 (사용자 교차점 추가 기능)

기존 calibrate_without_depth.py 기반 + 마우스 클릭으로 미검출 교차점 추가 가능.

사용법:
  python3 calibrate_wo_depth_user.py              # 우측 카메라 (기본)
  python3 calibrate_wo_depth_user.py --camera left

워크플로우:
  Enter → YOLO 검출 → 'a' 마우스로 교차점 추가 → 'p' 포인트 확정 → 'e' 실측값 입력
"""

import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rebar_base_interfaces.srv import DetectCrossings
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import json
import os
import time
import yaml
import threading

WS_DIR = '/home/koceti/ros2_ws'
DATA_DIR = os.path.join(WS_DIR, 'data', 'calibration')

CAMERA_CONFIG = {
    'right': {
        'camera_selection': 2,
        'image_topic': '/zedxmini2/zed_node/left/image_rect_color',
        'depth_topic': '/zedxmini2/zed_node/depth/depth_registered',
        'camera_name': 'Right Camera (zedxmini2)',
        'data_file': os.path.join(DATA_DIR, 'calibration_data_right.json'),
        'result_file': os.path.join(DATA_DIR, 'calibration_result_nodepth_right.yaml'),
        'viz_file': os.path.join(DATA_DIR, 'calibration_detect_right.png'),
    },
    'left': {
        'camera_selection': 1,
        'image_topic': '/zedxmini1/zed_node/left/image_rect_color',
        'depth_topic': '/zedxmini1/zed_node/depth/depth_registered',
        'camera_name': 'Left Camera (zedxmini1)',
        'data_file': os.path.join(DATA_DIR, 'calibration_data_left.json'),
        'result_file': os.path.join(DATA_DIR, 'calibration_result_nodepth_left.yaml'),
        'viz_file': os.path.join(DATA_DIR, 'calibration_detect_left.png'),
    },
}

DEPTH_BUFFER_SIZE = 15       # 누적 프레임 수
DEPTH_KERNEL_SIZE = 5        # spatial median 커널 (5x5)
DEPTH_MIN_M = 0.05           # 유효 depth 최소 (m)
DEPTH_MAX_M = 2.0            # 유효 depth 최대 (m)


class UserPoint:
    """사용자가 마우스로 추가한 포인트"""
    def __init__(self, pixel_u, pixel_v):
        self.pixel_u = pixel_u
        self.pixel_v = pixel_v
        self.x = 0.0
        self.y = 0.0
        self.confidence = 0.0
        self.depth_mm = 0.0
        self.is_user_added = True


class CalibrationUserAdd(Node):
    def __init__(self, camera_side='right'):
        super().__init__('calibration_user_add')
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
        self.depth_sub = self.create_subscription(
            Image, cfg['depth_topic'], self._depth_callback, 10)
        self.detect_client = self.create_client(
            DetectCrossings, '/rebar/detect_crossings')

        self.cal_data = self._load_data()
        self.x_coeffs = None
        self.y_coeffs = None

        # depth 누적 버퍼
        self.depth_buffer = deque(maxlen=DEPTH_BUFFER_SIZE)
        self.depth_lock = threading.Lock()

        # 마우스 클릭 상태
        self.click_points = []
        self.clicking = False

        if len(self.cal_data) >= 6:
            self._compute_mapping_silent()

        self.get_logger().info(
            '[%s] 기존 데이터: %d 쌍' % (self.camera_name, len(self.cal_data)))

    def image_callback(self, msg):
        if not self.got_image:
            self.camera_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.got_image = True

    def _depth_callback(self, msg):
        depth_np = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        with self.depth_lock:
            self.depth_buffer.append(depth_np)

    def sample_depth(self, u, v):
        """누적 depth buffer에서 spatial+temporal median 추출 (mm)

        각 프레임에서 (u,v) 주변 커널의 spatial median을 뽑고,
        프레임 간 temporal median을 취한다.
        """
        with self.depth_lock:
            frames = list(self.depth_buffer)
        if not frames:
            return None

        half = DEPTH_KERNEL_SIZE // 2
        samples = []
        for frame in frames:
            h, w = frame.shape[:2]
            y_min, y_max = max(0, v - half), min(h, v + half + 1)
            x_min, x_max = max(0, u - half), min(w, u + half + 1)
            region = frame[y_min:y_max, x_min:x_max]
            valid = region[np.isfinite(region) &
                           (region > DEPTH_MIN_M) &
                           (region < DEPTH_MAX_M)]
            if len(valid) > 0:
                samples.append(float(np.median(valid)))
        if not samples:
            return None
        return float(np.median(samples)) * 1000.0  # m → mm

    def depth_status(self):
        """현재 depth buffer 상태 출력"""
        with self.depth_lock:
            n_frames = len(self.depth_buffer)
        print('  [depth] 버퍼: %d/%d 프레임' % (n_frames, DEPTH_BUFFER_SIZE))
        return n_frames

    def _load_data(self):
        if os.path.exists(self.data_file):
            with open(self.data_file, 'r') as f:
                return json.load(f)
        return []

    def _save_data(self):
        with open(self.data_file, 'w') as f:
            json.dump(self.cal_data, f, indent=2)

    def _build_features(self, pixel_u, pixel_v):
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

    def _build_features_with_depth(self, pixel_u, pixel_v, depth_mm):
        pixel_u = np.asarray(pixel_u, dtype=np.float64)
        pixel_v = np.asarray(pixel_v, dtype=np.float64)
        depth_mm = np.asarray(depth_mm, dtype=np.float64)
        if pixel_u.ndim == 0:
            pixel_u = pixel_u.reshape(1)
            pixel_v = pixel_v.reshape(1)
            depth_mm = depth_mm.reshape(1)
        return np.column_stack([
            pixel_u, pixel_v, depth_mm,
            pixel_u * pixel_v,
            pixel_u * depth_mm,
            pixel_v * depth_mm,
            pixel_u ** 2, pixel_v ** 2, depth_mm ** 2,
            np.ones(len(pixel_u)),
        ])

    def _compute_mapping_silent(self):
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
        n = len(self.cal_data)
        if n < 6:
            print('  [매핑] 최소 6쌍 필요 (현재 %d쌍)' % n)
            return

        pixel_u = np.array([d['pixel_u'] for d in self.cal_data], dtype=np.float64)
        pixel_v = np.array([d['pixel_v'] for d in self.cal_data], dtype=np.float64)
        actual_x = np.array([d['actual_x_mm'] for d in self.cal_data], dtype=np.float64)
        actual_y = np.array([d['actual_y_mm'] for d in self.cal_data], dtype=np.float64)

        # --- pixel-only 회귀 ---
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

        print('\n  [매핑] pixel-only 회귀 (%d쌍)' % n)
        print('  features: [u, v, u*v, u², v², 1]')
        print('  X: R²=%.4f  평균오차=%.1fmm  최대=%.1fmm' % (
            x_r2, np.mean(x_errors), np.max(x_errors)))
        print('  Y: R²=%.4f  평균오차=%.1fmm  최대=%.1fmm' % (
            y_r2, np.mean(y_errors), np.max(y_errors)))
        print('  XY총합: 평균=%.1fmm, 최대=%.1fmm' % (
            np.mean(total_errors), np.max(total_errors)))

        self._save_mapping(np.mean(total_errors), np.max(total_errors), x_r2, y_r2)
        print('  저장: %s' % self.result_file)

        # --- depth 포함 회귀 비교 ---
        depth_mm = np.array(
            [d.get('depth_mm', 0.0) for d in self.cal_data], dtype=np.float64)
        depth_valid = depth_mm > 10.0  # 10mm 이상만 유효
        n_depth = int(np.sum(depth_valid))

        if n_depth >= 10:
            Ad = self._build_features_with_depth(
                pixel_u[depth_valid], pixel_v[depth_valid],
                depth_mm[depth_valid])
            ax_d = actual_x[depth_valid]
            ay_d = actual_y[depth_valid]

            xc_d, _, _, _ = np.linalg.lstsq(Ad, ax_d, rcond=None)
            xp_d = Ad @ xc_d
            xe_d = np.abs(xp_d - ax_d)
            x_r2_d = 1 - np.sum((ax_d - xp_d)**2) / np.sum((ax_d - np.mean(ax_d))**2)

            yc_d, _, _, _ = np.linalg.lstsq(Ad, ay_d, rcond=None)
            yp_d = Ad @ yc_d
            ye_d = np.abs(yp_d - ay_d)
            y_r2_d = 1 - np.sum((ay_d - yp_d)**2) / np.sum((ay_d - np.mean(ay_d))**2)

            te_d = np.sqrt(xe_d**2 + ye_d**2)

            # pixel-only (같은 서브셋)
            A_sub = self._build_features(
                pixel_u[depth_valid], pixel_v[depth_valid])
            xc_s, _, _, _ = np.linalg.lstsq(A_sub, ax_d, rcond=None)
            xp_s = A_sub @ xc_s
            yc_s, _, _, _ = np.linalg.lstsq(A_sub, ay_d, rcond=None)
            yp_s = A_sub @ yc_s
            te_s = np.sqrt((xp_s - ax_d)**2 + (yp_s - ay_d)**2)
            x_r2_s = 1 - np.sum((ax_d - xp_s)**2) / np.sum((ax_d - np.mean(ax_d))**2)
            y_r2_s = 1 - np.sum((ay_d - yp_s)**2) / np.sum((ay_d - np.mean(ay_d))**2)

            print('\n  [비교] depth 유효 서브셋 (%d/%d쌍)' % (n_depth, n))
            print('  depth 범위: %.0f ~ %.0fmm (mean=%.0f)' % (
                depth_mm[depth_valid].min(),
                depth_mm[depth_valid].max(),
                depth_mm[depth_valid].mean()))
            print('  %-20s %8s %8s %8s %8s' % (
                '', 'X_R²', 'Y_R²', 'XY평균', 'XY최대'))
            print('  %-20s %8.4f %8.4f %8.1f %8.1f' % (
                'pixel-only (subset)',
                x_r2_s, y_r2_s,
                np.mean(te_s), np.max(te_s)))
            print('  %-20s %8.4f %8.4f %8.1f %8.1f' % (
                'pixel+depth',
                x_r2_d, y_r2_d,
                np.mean(te_d), np.max(te_d)))
            improve = np.mean(te_s) - np.mean(te_d)
            print('  → depth 추가 효과: 평균오차 %.1fmm %s' % (
                abs(improve), '개선' if improve > 0 else '악화'))
        else:
            print('\n  [비교] depth 유효 데이터 부족 (%d쌍, 최소 10쌍 필요)' % n_depth)
            print('  → 검출 시 depth가 저장되면 자동 비교됩니다')
        print()

    def _apply_mapping(self, pixel_u, pixel_v):
        if self.x_coeffs is None or self.y_coeffs is None:
            return (None, None)
        A = self._build_features(pixel_u, pixel_v)
        x_mm = float(self.x_coeffs @ A[0])
        y_mm = float(self.y_coeffs @ A[0])
        return (x_mm, y_mm)

    def _save_mapping(self, mean_err, max_err, x_r2, y_r2):
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

    def _mouse_callback(self, event, x, y, flags, param):
        """마우스 클릭 콜백 - 교차점 추가"""
        if event == cv2.EVENT_LBUTTONDOWN and self.clicking:
            self.click_points.append((x, y))
            est_x, est_y = self._apply_mapping(x, y)
            if est_x is not None:
                print('    클릭: pixel=(%d, %d) → 추정 X=%.1fmm, Y=%.1fmm' % (
                    x, y, est_x, est_y))
            else:
                print('    클릭: pixel=(%d, %d) (매핑 없음)' % (x, y))

    def _user_add_points(self, detections, mapped_list):
        """마우스 클릭으로 교차점 추가 - OpenCV 윈도우"""
        if self.camera_image is None:
            print('  [오류] 이미지 없음')
            return detections, mapped_list

        self.click_points = []
        self.clicking = True

        # 이미지에 기존 검출 표시
        img = self.camera_image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        for i, det in enumerate(detections):
            u, v = int(det.pixel_u), int(det.pixel_v)
            cv2.circle(img, (u, v), 8, (0, 0, 255), -1)
            cv2.putText(img, 'P%d' % (i + 1), (u + 10, v - 10),
                        font, 0.5, (0, 0, 255), 1)

        cv2.putText(img, 'Click to add points. Press ESC or Q to finish.',
                    (10, 30), font, 0.7, (0, 255, 0), 2)

        win_name = 'Add Crossing Points'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win_name, self._mouse_callback)

        while True:
            # 클릭된 점 표시
            disp = img.copy()
            for j, (cx, cy) in enumerate(self.click_points):
                cv2.circle(disp, (cx, cy), 8, (255, 0, 0), -1)
                cv2.drawMarker(disp, (cx, cy), (255, 0, 0),
                               cv2.MARKER_CROSS, 20, 2)
                label = 'A%d' % (j + 1)
                est_x, est_y = self._apply_mapping(cx, cy)
                if est_x is not None:
                    label += ' (%.0f,%.0f)' % (est_x, est_y)
                cv2.putText(disp, label, (cx + 10, cy - 10),
                            font, 0.5, (255, 0, 0), 1)

            status = '%d YOLO + %d user added' % (
                len(detections), len(self.click_points))
            cv2.putText(disp, status, (10, img.shape[0] - 10),
                        font, 0.6, (255, 255, 0), 2)

            cv2.imshow(win_name, disp)
            key = cv2.waitKey(50) & 0xFF
            if key == 27 or key == ord('q'):  # ESC or Q
                break

        cv2.destroyWindow(win_name)
        self.clicking = False

        # 클릭된 점을 UserPoint로 변환하여 detections에 추가
        added = len(self.click_points)
        for cx, cy in self.click_points:
            up = UserPoint(cx, cy)
            detections.append(up)
            if self.x_coeffs is not None:
                mx, my = self._apply_mapping(cx, cy)
                mapped_list.append((mx, my, 1))
            else:
                mapped_list.append((0.0, 0.0, 1))

        if added > 0:
            print('  %d개 포인트 추가됨 (총 %d개)' % (added, len(detections)))

        return detections, mapped_list

    def _dedup_detections(self, detections, dist_mm=20.0):
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

    def detect_once(self):
        """단일 검출 서비스 호출"""
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

    def detect(self, n_rounds=5):
        """N회 반복 검출 + 누적 (오케스트레이터와 동일)"""
        all_detections = []
        last_response = None

        for r in range(n_rounds):
            print('  [%d/%d] 검출 중...' % (r + 1, n_rounds), end='', flush=True)
            dets, resp = self.detect_once()
            if dets is None:
                print(' 실패')
                continue
            print(' %d개 검출 (%.0fms)' % (len(dets), resp.detection_time_ms))
            all_detections.extend(dets)
            last_response = resp
            time.sleep(0.3)

        if not all_detections:
            return None, None

        print('  총 누적: %d개' % len(all_detections))
        return all_detections, last_response

    def collect_round(self):
        """한 라운드: 검출 → (선택) 사용자 추가 → 실측값 입력"""
        print('\n' + '=' * 60)
        self.depth_status()
        print('검출 수행 중...')
        detections, response = self.detect()

        if detections is None:
            return True
        if not detections:
            print('검출된 교차점 없음')

        raw_count = len(detections) if detections else 0
        mapped_list = []
        if detections and self.x_coeffs is not None:
            detections, mapped_list = self._dedup_detections(detections, dist_mm=20.0)
            if mapped_list:
                pairs = list(zip(detections, mapped_list))
                pairs.sort(key=lambda p: p[1][0], reverse=True)
                detections, mapped_list = zip(*pairs)
                detections, mapped_list = list(detections), list(mapped_list)
        elif detections:
            detections.sort(key=lambda d: d.x, reverse=True)
        else:
            detections = []

        dedup_info = ' (raw:%d)' % raw_count if raw_count != len(detections) else ''
        print('\nYOLO 검출: %d개%s' % (len(detections), dedup_info))
        if response:
            print('검출 시간: %.0fms' % response.detection_time_ms)
        print('-' * 60)

        for i, det in enumerate(detections):
            line = ('  P%d: pixel=(%d, %d)  conf=%.2f' % (
                i + 1, det.pixel_u, det.pixel_v, det.confidence))
            # 누적 depth 표시
            acc_d = self.sample_depth(int(det.pixel_u), int(det.pixel_v))
            if acc_d:
                line += '  depth=%.0fmm(acc)' % acc_d
            elif hasattr(det, 'depth_mm') and det.depth_mm > 0:
                line += '  depth=%.0fmm' % det.depth_mm
            else:
                line += '  depth=N/A'
            if mapped_list and i < len(mapped_list):
                mx, my, cnt = mapped_list[i]
                line += '  -> mapped:(%.1f, %.1f)' % (mx, my)
                if cnt > 1:
                    line += ' avg:%d' % cnt
            print(line)

        if self.camera_image is not None:
            self._save_viz(detections, mapped_list)

        print('-' * 60)
        print('  [a] 마우스로 교차점 추가')
        print('  [p] 포인트 확정 → 실측값 입력')
        print('  [e] 바로 실측값 입력 (추가 없이)')
        print('  [q] 종료')
        print()

        while True:
            cmd = input('  명령 [a/p/e/q]: ').strip().lower()

            if cmd == 'q':
                return False

            elif cmd == 'a':
                detections, mapped_list = self._user_add_points(
                    detections, mapped_list)
                # 추가 후 목록 다시 출력
                print('\n  === 전체 포인트 목록 (%d개) ===' % len(detections))
                for i, det in enumerate(detections):
                    src = '[USER]' if hasattr(det, 'is_user_added') else '[YOLO]'
                    line = '  P%d %s pixel=(%d, %d)' % (
                        i + 1, src, det.pixel_u, det.pixel_v)
                    if mapped_list and i < len(mapped_list):
                        mx, my, _ = mapped_list[i]
                        line += '  -> (%.1f, %.1f)mm' % (mx, my)
                    print(line)
                print()

            elif cmd in ('p', 'e'):
                # 실측값 입력
                return self._enter_measurements(detections, mapped_list)

            else:
                print('    a=추가, p/e=실측입력, q=종료')

        return True

    def _enter_measurements(self, detections, mapped_list):
        """각 포인트에 대해 실측값 입력"""
        print('\n  실측값 입력 (mm, "X Y")  s=스킵  q=종료')
        print()

        for i, det in enumerate(detections):
            src = '[USER]' if hasattr(det, 'is_user_added') else '[YOLO]'
            est_str = ''
            if mapped_list and i < len(mapped_list):
                mx, my, _ = mapped_list[i]
                est_str = '  추정:(%.1f, %.1f)' % (mx, my)

            while True:
                raw = input('  P%d %s pixel=(%d,%d)%s -> 실측(mm, X Y): ' % (
                    i + 1, src, det.pixel_u, det.pixel_v, est_str)).strip()

                if raw == 'q':
                    return False
                if raw == 's':
                    print('    P%d 스킵' % (i + 1))
                    break
                if raw == 'calc':
                    self._compute_mapping()
                    continue
                if raw == 'show':
                    self._show_data()
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

                    # 누적 depth 샘플링
                    acc_depth = self.sample_depth(
                        int(det.pixel_u), int(det.pixel_v))
                    det_depth = det.depth_mm if hasattr(det, 'depth_mm') else 0.0
                    # 누적 depth 우선, 없으면 검출 시 depth
                    final_depth = acc_depth if acc_depth else det_depth

                    pair = {
                        'actual_x_mm': round(actual_x_mm, 1),
                        'actual_y_mm': round(actual_y_mm, 1),
                        'depth_mm': round(final_depth, 1) if final_depth else 0.0,
                        'pixel_u': det.pixel_u,
                        'pixel_v': det.pixel_v,
                        'confidence': round(det.confidence, 3),
                        'source': 'user' if hasattr(det, 'is_user_added') else 'yolo',
                    }
                    if hasattr(det, 'x'):
                        pair['detected_x'] = round(det.x, 2)
                        pair['detected_y'] = round(det.y, 2)

                    depth_info = ''
                    if acc_depth:
                        depth_info = '  depth=%.0fmm(누적%d프레임)' % (
                            acc_depth, len(self.depth_buffer))
                    elif det_depth and det_depth > 0:
                        depth_info = '  depth=%.0fmm(단일)' % det_depth

                    self.cal_data.append(pair)
                    self._save_data()
                    print('    저장: (%.1f, %.1f)mm%s  [총 %d쌍]' % (
                        actual_x_mm, actual_y_mm, depth_info,
                        len(self.cal_data)))

                    if len(self.cal_data) >= 6:
                        self._compute_mapping_silent()
                    break

                except ValueError:
                    print('    숫자를 입력하세요 (예: 25.7 130.2)')

        return True

    def _show_data(self):
        print('\n  === 수집 데이터 (%d쌍) ===' % len(self.cal_data))
        if not self.cal_data:
            print('  (없음)')
            return
        print('  %4s %7s %7s %8s %8s %6s' % (
            '#', 'pix_u', 'pix_v', '실측X', '실측Y', 'src'))
        for i, d in enumerate(self.cal_data):
            src = d.get('source', 'yolo')[:4]
            print('  %4d %7d %7d %8.1f %8.1f %6s' % (
                i + 1, d['pixel_u'], d['pixel_v'],
                d['actual_x_mm'], d['actual_y_mm'], src))
        print()

    def _outlier_analysis(self):
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

    def _save_viz(self, detections, mapped_list):
        img = self.camera_image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        bbox_half = 30

        for i, det in enumerate(detections):
            u, v = int(det.pixel_u), int(det.pixel_v)
            is_user = hasattr(det, 'is_user_added')
            color = (255, 0, 0) if is_user else (0, 0, 255)

            cv2.rectangle(img, (u - bbox_half, v - bbox_half),
                          (u + bbox_half, v + bbox_half), (0, 255, 255), 2)
            cv2.circle(img, (u, v), 8, color, -1)

            lines = [
                ('P%d %s' % (i + 1, '[U]' if is_user else ''), color),
                ('pixel:(%d,%d)' % (det.pixel_u, det.pixel_v), (255, 255, 255)),
            ]
            if not is_user:
                lines.append(('conf:%.2f' % det.confidence, (255, 255, 255)))
            if mapped_list and i < len(mapped_list):
                mx, my, _ = mapped_list[i]
                lines.append(('-> (%.1f, %.1f)mm' % (mx, my), (0, 255, 0)))

            tx, ty = u - bbox_half, v + bbox_half + 18
            for li, (text, col) in enumerate(lines):
                yo = ty + li * 18
                (tw, th), _ = cv2.getTextSize(text, font, 0.45, 1)
                cv2.rectangle(img, (tx - 2, yo - th - 2),
                              (tx + tw + 2, yo + 3), (0, 0, 0), -1)
                cv2.putText(img, text, (tx, yo), font, 0.45, col, 1)

        summary = '%s | %d det (%d user) | %d pairs' % (
            self.camera_name, len(detections),
            sum(1 for d in detections if hasattr(d, 'is_user_added')),
            len(self.cal_data))
        cv2.putText(img, summary, (10, 30), font, 0.7, (0, 255, 0), 2)
        cv2.imwrite(self.viz_file, img)
        print('  [이미지] %s' % self.viz_file)


def main():
    parser = argparse.ArgumentParser(
        description='Pixel-only 캘리브레이션 + 사용자 교차점 추가')
    parser.add_argument('--camera', choices=['left', 'right'], default='right',
                        help='카메라 선택: left / right (기본)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CalibrationUserAdd(camera_side=args.camera)

    print('=' * 60)
    print(' Pixel-Only 캘리브레이션 + 사용자 추가 [%s]' % node.camera_name)
    print('=' * 60)
    print(' 데이터 파일: %s' % node.data_file)
    print(' 결과 파일:   %s' % node.result_file)
    print(' 기존 데이터: %d쌍' % len(node.cal_data))
    print()
    print(' Enter=검출  a=포인트추가  calc=매핑  outlier=분석')
    print(' depth=depth상태  q=종료')
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
            elif cmd == 'outlier':
                node._outlier_analysis()
            elif cmd == 'depth':
                node.depth_status()
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
