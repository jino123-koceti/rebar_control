#!/usr/bin/env python3
"""
AprilTag 개별 Pose 기반 pixel→mm 변환 테스트

호모그래피 대신 각 태그의 solvePnP pose를 이용하여
임의 pixel 좌표를 로봇 좌표(mm)로 변환합니다.

- 태그가 서로 다른 평면/깊이에 있어도 동작
- 기존 apriltag_positions_{camera}.yaml의 태그 좌표 그대로 사용
- 변환 시 가장 가까운 태그의 pose를 기준으로 변환

사용법:
  python3 test_apriltag_pose.py --camera right
  python3 test_apriltag_pose.py --camera left

키 조작:
  Enter = 캡처 + 검출
  setup = 태그 좌표 등록
  test  = 마우스 클릭 pixel→mm 변환
  acc   = 정확도 테스트 (leave-one-out)
  q     = 종료
"""

import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import yaml

WS_DIR = '/home/koceti/ros2_ws'
DATA_DIR = os.path.join(WS_DIR, 'data', 'calibration')

CAMERA_CONFIG = {
    'right': {
        'image_topic': '/zedxmini2/zed_node/left/image_rect_color',
        'info_topic': '/zedxmini2/zed_node/left/camera_info',
        'camera_name': 'Right Camera (zedxmini2)',
    },
    'left': {
        'image_topic': '/zedxmini1/zed_node/left/image_rect_color',
        'info_topic': '/zedxmini1/zed_node/left/camera_info',
        'camera_name': 'Left Camera (zedxmini1)',
    },
}

TAG_CONFIG_FILE = os.path.join(DATA_DIR, 'apriltag_positions_{camera}.yaml')
CALIB_RESULT_FILE = os.path.join(DATA_DIR, 'calibration_result_nodepth_{camera}.yaml')
TAG_WORLD_POS_FILE = os.path.join(DATA_DIR, 'apriltag_world_pos_ID{tag_id}_{camera}.yaml')

# 기본 카메라 파라미터 (camera_info 수신 전 fallback)
DEFAULT_CAMERA_MATRIX = np.array([
    [530.0, 0, 640.0],
    [0, 530.0, 360.0],
    [0, 0, 1.0]
], dtype=np.float64)
DEFAULT_DIST_COEFFS = np.zeros(5, dtype=np.float64)


def load_tag_world_data(camera_side):
    """캘리브레이션된 태그 3D 좌표 + R_cam2world 로드"""
    import glob
    pattern = os.path.join(DATA_DIR, f'apriltag_world_pos_ID*_{camera_side}.yaml')
    files = glob.glob(pattern)

    tag_world_positions = {}  # {tag_id: {x_mm, y_mm, z_mm}}
    R_cam2world = None

    for f_path in files:
        with open(f_path) as f:
            data = yaml.safe_load(f)
        if data is None:
            continue

        tag_id = data.get('target_tag_id')
        pos = data.get('position_3d', {})
        if tag_id is not None and pos:
            tag_world_positions[int(tag_id)] = {
                'x_mm': pos['x_mm'],
                'y_mm': pos['y_mm'],
                'z_mm': pos['z_mm'],
            }
            print(f'  TAG {tag_id} 3D: ({pos["x_mm"]:.1f}, {pos["y_mm"]:.1f}, {pos["z_mm"]:.1f})mm')

        # R_cam2world (가장 최근 파일에서)
        if 'R_cam2world' in data:
            R_cam2world = np.array(data['R_cam2world'])

    if R_cam2world is not None:
        print(f'  R_cam2world 로드 완료 (det={np.linalg.det(R_cam2world):.4f})')
    else:
        print(f'  [WARN] R_cam2world 없음')

    return tag_world_positions, R_cam2world


def load_linear_model(camera_side):
    """기존 nodepth 선형회귀 모델 로드"""
    path = CALIB_RESULT_FILE.format(camera=camera_side)
    if not os.path.exists(path):
        print(f'  [WARN] 선형모델 없음: {path}')
        return None

    with open(path) as f:
        data = yaml.safe_load(f)

    calib = data.get('calibration', {})
    x_coeffs = calib.get('x_mapping', {}).get('coeffs', [])
    y_coeffs = calib.get('y_mapping', {}).get('coeffs', [])

    if len(x_coeffs) != 6 or len(y_coeffs) != 6:
        print(f'  [WARN] 선형모델 계수 오류')
        return None

    return {'x_coeffs': x_coeffs, 'y_coeffs': y_coeffs}


def linear_model_predict(model, pixel_u, pixel_v):
    """선형회귀 모델로 pixel→mm 변환
    formula: a*u + b*v + c*u*v + d*u^2 + e*v^2 + f
    """
    if model is None:
        return None, None
    u, v = pixel_u, pixel_v
    features = [u, v, u * v, u**2, v**2, 1.0]
    x_mm = sum(c * f for c, f in zip(model['x_coeffs'], features))
    y_mm = sum(c * f for c, f in zip(model['y_coeffs'], features))
    return x_mm, y_mm


class AprilTagPoseCalibration(Node):
    def __init__(self, camera_side='right', tag_family='tag16h5', tag_size=0.04):
        super().__init__('apriltag_pose_calib')
        self.bridge = CvBridge()
        self.camera_image = None
        self.got_image = False

        self.camera_side = camera_side
        self.linear_model = load_linear_model(camera_side)
        self.tag_world_positions, self.R_cam2world = load_tag_world_data(camera_side)
        self.z_work = 0.0  # 작업면 Z 높이 (mm), calibrate_z_work()로 자동보정
        cfg = CAMERA_CONFIG[camera_side]
        self.camera_name = cfg['camera_name']

        # 카메라 파라미터
        self.camera_matrix = DEFAULT_CAMERA_MATRIX.copy()
        self.dist_coeffs = DEFAULT_DIST_COEFFS.copy()
        self.camera_info_received = False

        self.image_sub = self.create_subscription(
            Image, cfg['image_topic'], self.image_callback, 1)
        self.info_sub = self.create_subscription(
            CameraInfo, cfg['info_topic'], self.camera_info_callback, 1)

        # OpenCV ArUco AprilTag 검출기
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16H5)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)

        self.tag_size = tag_size  # 태그 물리적 크기 (m)

        # 태그 3D 모델 포인트 (태그 중심 원점)
        s = tag_size / 2
        self.obj_points = np.array([
            [-s,  s, 0],
            [ s,  s, 0],
            [ s, -s, 0],
            [-s, -s, 0],
        ], dtype=np.float64)

        # 태그 ID → 로봇 좌표 (mm) 로드
        self.tag_config_path = TAG_CONFIG_FILE.format(camera=camera_side)
        self.tag_positions = self._load_tag_config()

        # 검출된 태그별 pose 캐시
        self.tag_poses = {}  # {tag_id: {'rvec', 'tvec', 'rmat', 'center_px'}}

        self.get_logger().info(f'[{self.camera_name}] AprilTag Pose 캘리브레이션')
        self.get_logger().info(f'등록 태그: {list(self.tag_positions.keys())}')

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info(
                f"Camera intrinsics: fx={self.camera_matrix[0,0]:.1f} "
                f"fy={self.camera_matrix[1,1]:.1f}")

    def image_callback(self, msg):
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.got_image = True

    def _load_tag_config(self):
        if os.path.exists(self.tag_config_path):
            with open(self.tag_config_path) as f:
                return yaml.safe_load(f) or {}
        return {}

    def _save_tag_config(self):
        with open(self.tag_config_path, 'w') as f:
            yaml.dump(self.tag_positions, f, default_flow_style=False)

    def capture_image(self):
        self.got_image = False
        timeout = time.monotonic() + 5.0
        while not self.got_image and time.monotonic() < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.camera_image

    def detect_tags(self, image=None):
        """AprilTag 검출 + 개별 solvePnP"""
        if image is None:
            image = self.capture_image()
        if image is None:
            print('  [ERROR] 이미지 없음')
            return []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        tags = []
        self.tag_poses = {}

        if ids is None:
            return tags

        for i, tag_id in enumerate(ids.flatten()):
            corner = corners[i][0]
            cx, cy = np.mean(corner, axis=0)

            # solvePnP로 카메라↔태그 pose 추정
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, corner,
                self.camera_matrix, self.dist_coeffs
            )

            if not success:
                continue

            rmat, _ = cv2.Rodrigues(rvec)
            distance = float(np.linalg.norm(tvec))

            self.tag_poses[int(tag_id)] = {
                'rvec': rvec,
                'tvec': tvec,
                'rmat': rmat,
                'center_px': (float(cx), float(cy)),
                'corners': corner,
                'distance': distance,
            }

            tags.append({
                'id': int(tag_id),
                'center': (float(cx), float(cy)),
                'corners': corner.tolist(),
                'distance': distance,
            })

        return tags

    def pixel_to_mm(self, pixel_u, pixel_v):
        """
        pixel → 로봇 좌표 (mm) 변환 (R_cam2world 방식)

        1. 검출된 태그 중 하나로 카메라 월드 위치 산출
        2. pixel → 카메라 ray → R_cam2world로 월드 ray 변환
        3. 작업면 (Z=0) 평면과 교차 → 로봇 좌표 (x, y)

        어떤 태그를 사용해도 같은 결과 (R_cam2world가 정확하면)
        """
        if self.R_cam2world is None:
            return None, None, None
        if not self.tag_poses:
            return None, None, None

        # 검출된 태그 중 3D 좌표가 알려진 것을 찾아 카메라 월드 위치 산출
        # 우선순위: 고정 태그 (tag_world_positions, 캘리브레이션 완료) > EEF 태그 (위치 불확실)
        cam_world = None
        ref_tag_id = None

        # 1순위: 캘리브레이션된 고정 태그 (3D 좌표 확정)
        for tag_id, pose in self.tag_poses.items():
            if tag_id in self.tag_world_positions:
                tag_3d = self.tag_world_positions[tag_id]
                tag_world = np.array([tag_3d['x_mm'], tag_3d['y_mm'], tag_3d['z_mm']])
                tvec_m = pose['tvec'].flatten()
                cam_world = tag_world - self.R_cam2world @ tvec_m * 1000.0
                ref_tag_id = tag_id
                break

        # 2순위: EEF 태그 (tag_positions, 현재 EEF 위치=0,0,0 가정 → 부정확할 수 있음)
        if cam_world is None:
            for tag_id, pose in self.tag_poses.items():
                if str(tag_id) in self.tag_positions:
                    tag_pos = self.tag_positions[str(tag_id)]
                    tag_world = np.array([tag_pos['x_mm'], tag_pos['y_mm'], 0.0])
                    tvec_m = pose['tvec'].flatten()
                    cam_world = tag_world - self.R_cam2world @ tvec_m * 1000.0
                    ref_tag_id = tag_id
                    break

        if cam_world is None:
            return None, None, None

        # pixel → 카메라 ray
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        ray_cam = np.array([
            (pixel_u - cx) / fx,
            (pixel_v - cy) / fy,
            1.0
        ])

        # 카메라 ray → 월드 ray
        ray_world = self.R_cam2world @ ray_cam

        # 작업면 Z=z_work 평면과 교차
        # cam_world + t * ray_world 에서 Z = z_work
        if abs(ray_world[2]) < 1e-10:
            return None, None, None

        t = (self.z_work - cam_world[2]) / ray_world[2]
        if t < 0:  # 카메라 뒤쪽
            return None, None, None

        # 교차점 (월드 좌표, mm)
        hit = cam_world + t * ray_world
        return float(hit[0]), float(hit[1]), ref_tag_id

    def visualize(self, image, tags):
        """시각화"""
        img = image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX

        for tag in tags:
            tag_id = str(tag['id'])
            cx, cy = int(tag['center'][0]), int(tag['center'][1])
            corners = np.array(tag['corners'], dtype=np.int32)

            cv2.polylines(img, [corners], True, (0, 255, 0), 2)
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)

            # 축 그리기
            if int(tag['id']) in self.tag_poses:
                pose = self.tag_poses[int(tag['id'])]
                cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs,
                                  pose['rvec'], pose['tvec'], self.tag_size * 0.5)

            label = f'ID:{tag_id} d:{tag["distance"]:.2f}m'
            if tag_id in self.tag_positions:
                pos = self.tag_positions[tag_id]
                label += f' ({pos["x_mm"]:.0f},{pos["y_mm"]:.0f})mm'

            cv2.putText(img, label, (cx + 10, cy - 10), font, 0.5, (0, 0, 0), 3)
            cv2.putText(img, label, (cx + 10, cy - 10), font, 0.5, (0, 255, 0), 1)

            px_label = f'px:({cx},{cy})'
            cv2.putText(img, px_label, (cx + 10, cy + 15), font, 0.4, (200, 200, 200), 1)

        mode = 'Pose-based' if self.tag_poses else 'NO TAGS'
        registered = sum(1 for t in tags if str(t['id']) in self.tag_positions)
        cv2.putText(img, f'{mode} | {len(tags)} detected, {registered} registered',
                    (10, 30), font, 0.7, (0, 255, 0), 2)

        return img

    def setup_tag_positions(self, tags):
        """태그 로봇 좌표 등록 (인터랙티브)"""
        print('\n  === 태그 로봇 좌표 설정 ===')
        for tag in tags:
            tag_id = str(tag['id'])
            cx, cy = tag['center']
            existing = self.tag_positions.get(tag_id)

            if existing:
                print(f'  TAG {tag_id} px=({cx:.0f},{cy:.0f}) '
                      f'→ 기존: ({existing["x_mm"]:.1f}, {existing["y_mm"]:.1f})mm')
                raw = input(f'    변경? (Enter=유지): ').strip()
                if not raw:
                    continue
            else:
                raw = input(f'  TAG {tag_id} px=({cx:.0f},{cy:.0f}) → X Y (mm): ').strip()

            if raw in ('s', 'q', ''):
                continue
            try:
                parts = raw.replace(',', ' ').split()
                x_mm, y_mm = float(parts[0]), float(parts[1])
                self.tag_positions[tag_id] = {
                    'x_mm': x_mm, 'y_mm': y_mm,
                    'pixel_u': cx, 'pixel_v': cy,
                }
                print(f'    → ({x_mm}, {y_mm})mm')
            except (ValueError, IndexError):
                print('    스킵')

        self._save_tag_config()
        print(f'  저장: {self.tag_config_path}')

    def calibrate_z_work(self, tags):
        """선형모델을 기준으로 작업면 Z 높이 자동보정

        이미지 내 여러 점에서 선형모델 결과와 비교하여
        Pose 결과가 가장 일치하는 z_work를 찾습니다.
        """
        if self.linear_model is None:
            print('  선형모델 없음 → z_work 수동 입력 필요')
            raw = input('  z_work (mm, Enter=0): ').strip()
            self.z_work = float(raw) if raw else 0.0
            return

        if self.R_cam2world is None:
            print('  R_cam2world 없음')
            return

        # 이미지 내 균일 분포 샘플 점 (태그 주변 영역)
        sample_pixels = []
        for tag in tags:
            cx, cy = tag['center']
            # 태그 주변 여러 점
            for du in [-100, 0, 100]:
                for dv in [50, 100, 150]:
                    sample_pixels.append((cx + du, cy + dv))

        if not sample_pixels:
            print('  샘플 점 없음')
            return

        # 최적 z_work 탐색 (-200 ~ 200mm)
        from scipy.optimize import minimize_scalar

        def error_fn(z_w):
            old_z = self.z_work
            self.z_work = z_w
            errors = []
            for pu, pv in sample_pixels:
                pose_x, pose_y, _ = self.pixel_to_mm(pu, pv)
                lin_x, lin_y = linear_model_predict(self.linear_model, pu, pv)
                if pose_x is not None and lin_x is not None:
                    errors.append((pose_x - lin_x)**2 + (pose_y - lin_y)**2)
            self.z_work = old_z
            return np.mean(errors) if errors else 1e10

        # scipy 없으면 brute force
        try:
            result = minimize_scalar(error_fn, bounds=(-300, 300), method='bounded')
            best_z = result.x
            best_err = np.sqrt(result.fun)
        except ImportError:
            best_z = 0
            best_err = float('inf')
            for z_try in np.arange(-300, 300, 1):
                err = error_fn(z_try)
                if err < best_err**2:
                    best_err = np.sqrt(err)
                    best_z = z_try

        self.z_work = best_z
        print(f'  z_work 자동보정: {best_z:.1f}mm (평균오차: {best_err:.1f}mm)')

    def interactive_test(self, image, tags):
        """마우스 클릭 pixel→mm 변환 테스트 (선형모델 비교 포함)"""
        click_pts = []

        def mouse_cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                # AprilTag pose 변환
                rx, ry, ref_tag = self.pixel_to_mm(x, y)
                # 선형회귀 모델 변환
                lx, ly = linear_model_predict(self.linear_model, x, y)

                if rx is not None:
                    click_pts.append({
                        'px': x, 'py': y,
                        'pose_x': rx, 'pose_y': ry, 'ref_tag': ref_tag,
                        'linear_x': lx, 'linear_y': ly,
                    })
                    diff_str = ''
                    if lx is not None:
                        diff = np.sqrt((rx - lx)**2 + (ry - ly)**2)
                        diff_str = f'  diff={diff:.1f}mm'
                    print(f'    px=({x},{y})')
                    print(f'      Pose:   X={rx:.1f}, Y={ry:.1f}  [ref:T{ref_tag}]')
                    if lx is not None:
                        print(f'      Linear: X={lx:.1f}, Y={ly:.1f}{diff_str}')
                else:
                    print(f'    px=({x},{y}) → 변환 불가')

        img = self.visualize(image, tags)
        win_name = 'AprilTag Pose Test'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win_name, mouse_cb)

        has_linear = self.linear_model is not None
        print(f'\n  마우스 클릭 → pixel→mm 변환 (ESC/Q 종료)')
        print(f'  비교모델: {"nodepth 선형회귀 (녹색)" if has_linear else "없음"}')

        while True:
            disp = img.copy()
            font = cv2.FONT_HERSHEY_SIMPLEX
            for pt in click_pts:
                px, py = pt['px'], pt['py']

                # Pose 결과 (마젠타)
                cv2.circle(disp, (px, py), 6, (255, 0, 255), -1)
                cv2.putText(disp, f'P:({pt["pose_x"]:.0f},{pt["pose_y"]:.0f}) T{pt["ref_tag"]}',
                            (px + 8, py - 8), font, 0.35, (0, 0, 0), 2)
                cv2.putText(disp, f'P:({pt["pose_x"]:.0f},{pt["pose_y"]:.0f}) T{pt["ref_tag"]}',
                            (px + 8, py - 8), font, 0.35, (255, 0, 255), 1)

                # 선형모델 결과 (녹색)
                if pt['linear_x'] is not None:
                    cv2.putText(disp, f'L:({pt["linear_x"]:.0f},{pt["linear_y"]:.0f})',
                                (px + 8, py + 8), font, 0.35, (0, 0, 0), 2)
                    cv2.putText(disp, f'L:({pt["linear_x"]:.0f},{pt["linear_y"]:.0f})',
                                (px + 8, py + 8), font, 0.35, (0, 255, 0), 1)

            cv2.imshow(win_name, disp)
            key = cv2.waitKey(50) & 0xFF
            if key == 27 or key == ord('q'):
                break

        cv2.destroyWindow(win_name)

    def accuracy_test(self, tags):
        """Leave-one-out 정확도: 한 태그를 pixel→mm 변환하고 실제 좌표와 비교"""
        matched = []
        for tag in tags:
            tag_id = str(tag['id'])
            if tag_id in self.tag_positions and int(tag['id']) in self.tag_poses:
                pos = self.tag_positions[tag_id]
                matched.append({
                    'id': int(tag['id']),
                    'center': tag['center'],
                    'actual_x': pos['x_mm'],
                    'actual_y': pos['y_mm'],
                })

        if len(matched) < 2:
            print(f'  정확도 테스트: 최소 2개 매칭 필요 (현재 {len(matched)}개)')
            return

        print(f'\n  === Pose 기반 정확도 테스트 ({len(matched)}개 태그) ===')

        # 각 태그 중심을 다른 태그 pose로 변환하여 정확도 측정
        errors = []
        for left_out in matched:
            # left_out 태그의 중심 pixel을 변환
            # 가장 가까운 "다른" 등록 태그를 사용
            px, py = left_out['center']

            best_ref = None
            best_dist = float('inf')
            for other in matched:
                if other['id'] == left_out['id']:
                    continue
                pose = self.tag_poses[other['id']]
                ocx, ocy = pose['center_px']
                d = np.sqrt((px - ocx)**2 + (py - ocy)**2)
                if d < best_dist:
                    best_dist = d
                    best_ref = other['id']

            if best_ref is None:
                continue

            # best_ref 태그의 pose로 변환
            pred_x, pred_y, _ = self._pixel_to_mm_with_ref(px, py, best_ref)
            if pred_x is None:
                continue

            err_x = pred_x - left_out['actual_x']
            err_y = pred_y - left_out['actual_y']
            err = np.sqrt(err_x**2 + err_y**2)
            errors.append(err)

            print(f'  TAG {left_out["id"]}: '
                  f'actual=({left_out["actual_x"]:.1f},{left_out["actual_y"]:.1f}) '
                  f'pred=({pred_x:.1f},{pred_y:.1f}) '
                  f'err={err:.1f}mm (dX={err_x:+.1f}, dY={err_y:+.1f}) [ref:T{best_ref}]')

        if errors:
            print(f'\n  평균오차: {np.mean(errors):.1f}mm, '
                  f'최대: {np.max(errors):.1f}mm, '
                  f'최소: {np.min(errors):.1f}mm')

    def _pixel_to_mm_with_ref(self, pixel_u, pixel_v, ref_tag_id):
        """특정 태그를 기준으로 pixel→mm 변환 (R_cam2world 방식)"""
        if self.R_cam2world is None:
            return None, None, None
        if ref_tag_id not in self.tag_poses:
            return None, None, None

        pose = self.tag_poses[ref_tag_id]

        # 태그 월드 좌표 (3D 또는 2D fallback)
        if ref_tag_id in self.tag_world_positions:
            tw = self.tag_world_positions[ref_tag_id]
            tag_world = np.array([tw['x_mm'], tw['y_mm'], tw['z_mm']])
        elif str(ref_tag_id) in self.tag_positions:
            tp = self.tag_positions[str(ref_tag_id)]
            tag_world = np.array([tp['x_mm'], tp['y_mm'], 0.0])
        else:
            return None, None, None

        # 카메라 월드 위치
        tvec_m = pose['tvec'].flatten()
        cam_world = tag_world - self.R_cam2world @ tvec_m * 1000.0

        # pixel → 카메라 ray → 월드 ray
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        ray_cam = np.array([
            (pixel_u - cx) / fx,
            (pixel_v - cy) / fy,
            1.0
        ])
        ray_world = self.R_cam2world @ ray_cam

        # Z=z_work 평면 교차
        if abs(ray_world[2]) < 1e-10:
            return None, None, None

        t = (self.z_work - cam_world[2]) / ray_world[2]
        if t < 0:
            return None, None, None

        hit = cam_world + t * ray_world
        return float(hit[0]), float(hit[1]), ref_tag_id


def main():
    parser = argparse.ArgumentParser(description='AprilTag Pose 기반 캘리브레이션')
    parser.add_argument('--camera', choices=['left', 'right'], default='right')
    parser.add_argument('--tag-size', type=float, default=0.05,
                        help='태그 크기 m (default: 0.05 = 5cm)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = AprilTagPoseCalibration(
        camera_side=args.camera,
        tag_size=args.tag_size,
    )

    print('=' * 60)
    print(f' AprilTag Pose Calibration [{node.camera_name}]')
    print(f' Tag: 16h5, size: {args.tag_size}m')
    print(f' 등록 태그: {len(node.tag_positions)}개')
    print(f' 방식: 개별 solvePnP (호모그래피 아님)')
    print('=' * 60)
    print()
    print(' 명령어:')
    print('   Enter  = 캡처 + 검출 + z_work 자동보정')
    print('   setup  = 태그 좌표 등록')
    print('   test   = 마우스 클릭 변환 테스트')
    print('   acc    = 정확도 테스트')
    print('   zcal   = z_work 재보정')
    print('   q      = 종료')
    print('=' * 60)

    last_tags = []
    last_image = None

    try:
        while True:
            cmd = input('\n[Enter]=검출, [q]=종료: ').strip().lower()

            if cmd == 'q':
                break

            elif cmd in ('', 'detect'):
                image = node.capture_image()
                if image is None:
                    print('  이미지 없음')
                    continue
                last_image = image
                last_tags = node.detect_tags(image)
                print(f'\n  검출: {len(last_tags)}개')
                for tag in last_tags:
                    tid = str(tag['id'])
                    cx, cy = tag['center']
                    d = tag['distance']
                    status = '(등록됨)' if tid in node.tag_positions else '(미등록)'
                    print(f'    TAG {tid}: px=({cx:.0f},{cy:.0f}) '
                          f'd={d:.3f}m {status}')

                # 태그 검출 후 z_work 자동보정
                if node.R_cam2world is not None and node.tag_poses:
                    node.calibrate_z_work(last_tags)

                viz = node.visualize(image, last_tags)
                viz_path = os.path.join(DATA_DIR,
                                        f'apriltag_pose_{args.camera}.png')
                cv2.imwrite(viz_path, viz)
                print(f'  저장: {viz_path}')

            elif cmd == 'setup':
                if not last_tags:
                    print('  먼저 Enter로 검출')
                    continue
                node.setup_tag_positions(last_tags)

            elif cmd == 'test':
                if last_image is None or not last_tags:
                    print('  먼저 Enter로 검출')
                    continue
                node.interactive_test(last_image, last_tags)

            elif cmd == 'zcal':
                if not last_tags or not node.tag_poses:
                    print('  먼저 Enter로 검출')
                    continue
                node.calibrate_z_work(last_tags)

            elif cmd == 'acc':
                if not last_tags:
                    print('  먼저 Enter로 검출')
                    continue
                node.accuracy_test(last_tags)

            else:
                print('  Enter/setup/test/acc/q')

    except KeyboardInterrupt:
        print('\n종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
