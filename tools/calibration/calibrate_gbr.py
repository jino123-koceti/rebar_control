#!/usr/bin/env python3
"""
GBR(Gradient Boosting Regressor) 기반 검출 좌표 캘리브레이션 스크립트

사용법:
  python3 calibrate_gbr.py              # 우측 카메라 (기본)
  python3 calibrate_gbr.py --camera left
  python3 calibrate_gbr.py --camera right

동작:
  1. 검출 서비스 호출 → 검출된 교차점 좌표 표시
  2. 각 포인트에 대해 실측값(mm) 입력
  3. calibration_data_{camera}.json에 누적 저장 (기존 스크립트와 공유)
  4. GBR 모델 학습 후 calibration_model_{x,y}_gbr_{camera}.joblib 저장
  5. 메타데이터를 calibration_result_gbr_{camera}.yaml로 저장

명령어:
  - 실측값 입력: "25.5 130.0" (X Y, mm 단위)
  - 's' : 해당 포인트 스킵
  - 'q' : 종료
  - 'show' : 수집 데이터 표시
  - 'calc' : GBR 모델 학습 및 저장
  - 'test' : 현재 검출값에 매핑 적용 테스트
  - 'compare' : Linear vs GBR 5-fold CV 성능 비교
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
import warnings
warnings.filterwarnings('ignore')

from sklearn.ensemble import GradientBoostingRegressor
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import KFold
from sklearn.base import clone
import joblib

WS_DIR = '/home/koceti/ros2_ws'
DATA_DIR = os.path.join(WS_DIR, 'data', 'calibration')

CAMERA_CONFIG = {
    'right': {
        'camera_selection': 2,
        'image_topic': '/zedxmini2/zed_node/left/image_rect_color',
        'camera_name': 'Right Camera (zedxmini2)',
        'data_file': os.path.join(DATA_DIR, 'calibration_data_right.json'),
        'result_file': os.path.join(DATA_DIR, 'calibration_result_gbr_right.yaml'),
        'model_x_file': os.path.join(DATA_DIR, 'calibration_model_x_gbr_right.joblib'),
        'model_y_file': os.path.join(DATA_DIR, 'calibration_model_y_gbr_right.joblib'),
        'viz_file': os.path.join(DATA_DIR, 'calibration_detect_right.png'),
    },
    'left': {
        'camera_selection': 1,
        'image_topic': '/zedxmini1/zed_node/left/image_rect_color',
        'camera_name': 'Left Camera (zedxmini1)',
        'data_file': os.path.join(DATA_DIR, 'calibration_data_left.json'),
        'result_file': os.path.join(DATA_DIR, 'calibration_result_gbr_left.yaml'),
        'model_x_file': os.path.join(DATA_DIR, 'calibration_model_x_gbr_left.joblib'),
        'model_y_file': os.path.join(DATA_DIR, 'calibration_model_y_gbr_left.joblib'),
        'viz_file': os.path.join(DATA_DIR, 'calibration_detect_left.png'),
    },
}

# GBR 하이퍼파라미터
GBR_PARAMS = dict(n_estimators=300, max_depth=4, learning_rate=0.05,
                  subsample=0.8, random_state=42)


class CalibrationGBR(Node):
    def __init__(self, camera_side='right'):
        super().__init__('calibration_gbr')
        self.bridge = CvBridge()
        self.camera_image = None
        self.got_image = False

        self.camera_side = camera_side
        cfg = CAMERA_CONFIG[camera_side]
        self.camera_selection = cfg['camera_selection']
        self.camera_name = cfg['camera_name']
        self.data_file = cfg['data_file']
        self.result_file = cfg['result_file']
        self.model_x_file = cfg['model_x_file']
        self.model_y_file = cfg['model_y_file']
        self.viz_file = cfg['viz_file']

        self.image_sub = self.create_subscription(
            Image, cfg['image_topic'], self.image_callback, 1)
        self.detect_client = self.create_client(
            DetectCrossings, '/rebar/detect_crossings')

        self.cal_data = self._load_data()
        self.model_x = None
        self.model_y = None

        # 기존 모델 자동 로드
        if os.path.exists(self.model_x_file) and os.path.exists(self.model_y_file):
            try:
                self.model_x = joblib.load(self.model_x_file)
                self.model_y = joblib.load(self.model_y_file)
                self.get_logger().info(f'[{self.camera_name}] GBR 모델 로드 완료')
            except Exception as e:
                self.get_logger().warn(f'GBR 모델 로드 실패: {e}')

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

    def _build_features(self, data_list=None, us=None, vs=None, deps=None):
        """기본 feature: [pixel_u, pixel_v, depth_mm]"""
        if data_list is not None:
            us  = np.array([d['pixel_u']  for d in data_list], dtype=np.float64)
            vs  = np.array([d['pixel_v']  for d in data_list], dtype=np.float64)
            deps = np.array([d['depth_mm'] for d in data_list], dtype=np.float64)
        return np.column_stack([us, vs, deps])

    def _apply_mapping(self, pixel_u, pixel_v, depth_mm):
        """(pixel_u, pixel_v, depth) → 보정 좌표 (mm)"""
        if self.model_x is None or self.model_y is None:
            return (None, None, None)
        feat = np.array([[pixel_u, pixel_v, depth_mm]])
        x_mm = float(self.model_x.predict(feat)[0])
        y_mm = float(self.model_y.predict(feat)[0])
        return (x_mm, y_mm, None)

    def _compute_mapping(self):
        """GBR 모델 학습 및 저장"""
        n = len(self.cal_data)
        if n < 20:
            print(f'  [GBR] 최소 20쌍 필요 (현재 {n}쌍)')
            return

        X = self._build_features(self.cal_data)
        ax = np.array([d['actual_x_mm'] for d in self.cal_data])
        ay = np.array([d['actual_y_mm'] for d in self.cal_data])

        print(f'\n  [GBR] 학습 중... ({n}쌍)')
        self.model_x = GradientBoostingRegressor(**GBR_PARAMS)
        self.model_y = GradientBoostingRegressor(**GBR_PARAMS)
        self.model_x.fit(X, ax)
        self.model_y.fit(X, ay)

        # 훈련 오차
        x_pred = self.model_x.predict(X)
        y_pred = self.model_y.predict(X)
        x_err = np.abs(x_pred - ax)
        y_err = np.abs(y_pred - ay)
        xy_err = np.sqrt(x_err**2 + y_err**2)

        print(f'  X: 평균오차={np.mean(x_err):.1f}mm  최대={np.max(x_err):.1f}mm')
        print(f'  Y: 평균오차={np.mean(y_err):.1f}mm  최대={np.max(y_err):.1f}mm')
        print(f'  XY총합: 평균오차={np.mean(xy_err):.1f}mm, 최대={np.max(xy_err):.1f}mm')

        # 5-fold CV 오차
        kf = KFold(n_splits=5, shuffle=True, random_state=42)
        cv_x, cv_y = [], []
        for tr, te in kf.split(X):
            mx = clone(self.model_x); mx.fit(X[tr], ax[tr])
            my = clone(self.model_y); my.fit(X[tr], ay[tr])
            cv_x.append(np.mean(np.abs(mx.predict(X[te]) - ax[te])))
            cv_y.append(np.mean(np.abs(my.predict(X[te]) - ay[te])))
        cv_x_mean, cv_y_mean = np.mean(cv_x), np.mean(cv_y)
        print(f'  5-fold CV: X={cv_x_mean:.2f}mm  Y={cv_y_mean:.2f}mm  XY avg={(cv_x_mean+cv_y_mean)/2:.2f}mm')

        self._save_mapping(np.mean(xy_err), np.max(xy_err), cv_x_mean, cv_y_mean)
        print(f'  모델 저장: {self.model_x_file}')
        print(f'             {self.model_y_file}')
        print(f'  메타 저장: {self.result_file}')
        print()

    def _save_mapping(self, mean_err, max_err, cv_x, cv_y):
        """GBR 모델 및 메타데이터 저장"""
        joblib.dump(self.model_x, self.model_x_file)
        joblib.dump(self.model_y, self.model_y_file)

        meta = {
            'calibration': {
                'method': 'gradient_boosting_regressor',
                'camera': self.camera_name,
                'num_points': len(self.cal_data),
                'mean_error_mm': round(float(mean_err), 2),
                'max_error_mm': round(float(max_err), 2),
                'cv_x_mae_mm': round(float(cv_x), 2),
                'cv_y_mae_mm': round(float(cv_y), 2),
                'gbr_params': GBR_PARAMS,
                'model_x_file': self.model_x_file,
                'model_y_file': self.model_y_file,
                'inputs': ['pixel_u', 'pixel_v', 'depth_mm'],
            }
        }
        with open(self.result_file, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False)

    def _compare_models(self):
        """Linear+poly vs GBR 5-fold CV 성능 비교"""
        n = len(self.cal_data)
        if n < 20:
            print(f'  최소 20쌍 필요 (현재 {n}쌍)')
            return

        X_base = self._build_features(self.cal_data)
        us  = np.array([d['pixel_u']  for d in self.cal_data], dtype=np.float64)
        vs  = np.array([d['pixel_v']  for d in self.cal_data], dtype=np.float64)
        deps = np.array([d['depth_mm'] for d in self.cal_data], dtype=np.float64)
        X_poly = np.column_stack([us, vs, deps, us*deps, vs*deps, us**2, vs**2, np.ones(n)])
        ax = np.array([d['actual_x_mm'] for d in self.cal_data])
        ay = np.array([d['actual_y_mm'] for d in self.cal_data])

        kf = KFold(n_splits=5, shuffle=True, random_state=42)

        def cv_mae(model, X, y):
            scores = []
            for tr, te in kf.split(X):
                m = clone(model); m.fit(X[tr], y[tr])
                scores.append(np.mean(np.abs(m.predict(X[te]) - y[te])))
            return np.mean(scores)

        models = [
            ('Linear+poly', LinearRegression(), X_poly),
            ('GBR(현재설정)', GradientBoostingRegressor(**GBR_PARAMS), X_base),
        ]

        print(f'\n  === 모델 성능 비교 (5-fold CV, {n}쌍) ===')
        print(f'  {"모델":<22} {"X MAE":>7} {"Y MAE":>7} {"XY avg":>7}')
        print(f'  {"-"*48}')
        for name, model, X in models:
            mx = cv_mae(model, X, ax)
            my = cv_mae(model, X, ay)
            print(f'  {name:<22} {mx:7.2f} {my:7.2f} {(mx+my)/2:7.2f}')
        print()

    def detect(self):
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

    def _dedup_detections(self, detections, dist_mm=20.0):
        if self.model_x is None or not detections:
            return detections, []

        mapped = []
        for det in detections:
            mx, my, _ = self._apply_mapping(det.pixel_u, det.pixel_v, det.depth_mm)
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
                if (dx*dx + dy*dy)**0.5 < dist_mm:
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

    def collect_round(self):
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
        if self.model_x is not None:
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
            line = (f'  P{i+1}: 검출=({det.x:.1f}, {det.y:.1f}) mm  '
                    f'depth={det.depth_mm:.0f}mm  conf={det.confidence:.2f}  '
                    f'pixel=({det.pixel_u},{det.pixel_v})')
            if mapped_list:
                mx, my, cnt = mapped_list[i]
                line += f'  → GBR:({mx:.1f}, {my:.1f})'
                if cnt > 1:
                    line += f' avg:{cnt}'
            print(line)

        if self.camera_image is not None:
            self._save_viz(detections, mapped_list)

        print('-' * 60)
        print('각 포인트의 실측값을 입력하세요 (mm 단위, "X Y")')
        print("  's'=스킵  'q'=종료  'show'=데이터보기  'calc'=GBR학습  'compare'=성능비교")
        print()

        for i, det in enumerate(detections):
            while True:
                raw = input(f'  P{i+1} 검출({det.x:.1f}, {det.y:.1f}) depth={det.depth_mm:.0f} → 실측(mm, X Y): ').strip()
                if raw == 'q':
                    return False
                if raw == 's':
                    print(f'    P{i+1} 스킵')
                    break
                if raw == 'show':
                    self._show_data()
                    continue
                if raw == 'calc':
                    self._compute_mapping()
                    continue
                if raw == 'compare':
                    self._compare_models()
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
                    print(f'    저장: ({actual_x_mm:.1f}, {actual_y_mm:.1f})mm  [총 {len(self.cal_data)}쌍]')
                    break
                except ValueError:
                    print('    숫자를 입력하세요 (예: 25.7 130.2)')

        return True

    def _show_data(self):
        print(f'\n  === 수집 데이터 ({len(self.cal_data)}쌍) ===')
        if not self.cal_data:
            print('  (없음)')
            return
        print(f'  {"#":>4}  {"pixel_u":>7}  {"pixel_v":>7}  {"depth":>7}  {"실측X":>7}  {"실측Y":>7}')
        for i, d in enumerate(self.cal_data):
            print(f'  {i+1:4d}  {d["pixel_u"]:7d}  {d["pixel_v"]:7d}  {d["depth_mm"]:7.0f}  '
                  f'{d["actual_x_mm"]:7.1f}  {d["actual_y_mm"]:7.1f}')
        print()

    def _save_viz(self, detections, mapped_list):
        img = self.camera_image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        bbox_half = 30

        for i, det in enumerate(detections):
            u, v = int(det.pixel_u), int(det.pixel_v)
            cv2.rectangle(img, (u-bbox_half, v-bbox_half), (u+bbox_half, v+bbox_half), (0,255,255), 2)
            cv2.circle(img, (u, v), 8, (0,0,255), -1)

            lines = [
                (f'P{i+1}', (0,0,255)),
                (f'({det.x:.1f}, {det.y:.1f})', (255,255,255)),
                (f'conf:{det.confidence:.2f} d:{det.depth_mm:.0f}mm', (255,255,255)),
            ]
            if mapped_list and i < len(mapped_list):
                mx, my, _ = mapped_list[i]
                lines.append((f'GBR:({mx:.1f}, {my:.1f})', (0,255,0)))

            tx, ty = u - bbox_half, v + bbox_half + 18
            for li, (text, color) in enumerate(lines):
                yo = ty + li * 18
                (tw, th), _ = cv2.getTextSize(text, font, 0.45, 1)
                cv2.rectangle(img, (tx-2, yo-th-2), (tx+tw+2, yo+3), (0,0,0), -1)
                cv2.putText(img, text, (tx, yo), font, 0.45, color, 1)

        summary = f'{self.camera_name} | {len(detections)} detections | GBR | {len(self.cal_data)} pairs'
        cv2.putText(img, summary, (10, 30), font, 0.7, (0,255,0), 2)
        cv2.imwrite(self.viz_file, img)
        print(f'  [이미지] {self.viz_file}')


def main():
    parser = argparse.ArgumentParser(description='GBR 캘리브레이션')
    parser.add_argument('--camera', choices=['left', 'right'], default='right')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CalibrationGBR(camera_side=args.camera)

    print('=' * 60)
    print(f' GBR 캘리브레이션 [{node.camera_name}]')
    print('=' * 60)
    print(f' 데이터 파일: {node.data_file}')
    print(f' 결과 파일:   {node.result_file}')
    print(f' 기존 데이터: {len(node.cal_data)}쌍')
    model_loaded = node.model_x is not None
    print(f' GBR 모델:    {"로드됨" if model_loaded else "없음 (calc로 학습 필요)"}')
    print()
    print(' Enter=검출실행  calc=GBR학습  compare=성능비교  show=데이터  q=종료')
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
            elif cmd == '':
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
