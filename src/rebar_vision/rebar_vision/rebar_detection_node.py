#!/usr/bin/env python3
"""
철근 교차점 검출 노드 (Rebar Crossing Detection Node)

YOLO 기반 철근 교차점 검출 + 깊이 기반 3D 좌표 변환
듀얼 ZED X Mini 카메라에서 RGB+Depth를 수신하여
서비스 요청 시 교차점을 검출하고 로봇 좌표계로 변환

서비스: /rebar/detect_crossings (DetectCrossings)
발행: /rebar/detection_image (디버그), /rebar/detected_grid (모니터링)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rebar_base_interfaces.msg import RebarDetection, RebarGrid
from rebar_base_interfaces.srv import DetectCrossings
import message_filters
import numpy as np
import cv2
import time
import threading
import os
import yaml
from collections import deque


class RebarDetectionNode(Node):
    """YOLO 기반 철근 교차점 검출 서비스 노드"""

    def __init__(self):
        super().__init__('rebar_detection_node')

        # ============================================
        # 파라미터 선언
        # ============================================
        self.declare_parameter('model_path',
                               '/home/koceti/ros2_ws/src/rebar_vision/model/best_260313.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('depth_kernel_size', 5)
        self.declare_parameter('depth_buffer_size', 5)  # temporal median 프레임 수
        self.declare_parameter('max_depth_m', 2.0)
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('grid_rows', 2)
        self.declare_parameter('grid_cols', 3)
        self.declare_parameter('dedup_distance_mm', 40.0)
        self.declare_parameter('detection_strategy', 'cross')  # cross, merge, single
        self.declare_parameter('calibration_model', 'nodepth')  # gbr, nodepth, linear

        # 카메라 외부 파라미터 (position: mm, rotation: degrees)
        self.declare_parameter('left_camera.position', [-200.0, 100.0, 108.0])
        self.declare_parameter('left_camera.rotation', [40.0, 0.0, -20.0])
        self.declare_parameter('right_camera.position', [-200.0, -100.0, 108.0])
        self.declare_parameter('right_camera.rotation', [40.0, 0.0, 20.0])

        # 파라미터 가져오기
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.depth_kernel_size = self.get_parameter('depth_kernel_size').value
        self.depth_buffer_size = self.get_parameter('depth_buffer_size').value
        self.max_depth_m = self.get_parameter('max_depth_m').value
        self.min_depth_m = self.get_parameter('min_depth_m').value
        self.grid_rows = self.get_parameter('grid_rows').value
        self.grid_cols = self.get_parameter('grid_cols').value
        self.dedup_distance_mm = self.get_parameter('dedup_distance_mm').value
        self.detection_strategy = self.get_parameter('detection_strategy').value
        self.calibration_model_type = self.get_parameter('calibration_model').value

        self.camera_extrinsics = {
            'left': {
                'position': np.array(self.get_parameter('left_camera.position').value),
                'rotation': np.array(self.get_parameter('left_camera.rotation').value),
                'side': 'left',
            },
            'right': {
                'position': np.array(self.get_parameter('right_camera.position').value),
                'rotation': np.array(self.get_parameter('right_camera.rotation').value),
                'side': 'right',
            }
        }

        # ============================================
        # 캘리브레이션 회귀 모델 로드
        # ============================================
        self.calibration_models = {}
        self._load_calibration_models()

        # ============================================
        # YOLO 모델 로드
        # ============================================
        self.model = None
        self._load_model()

        # ============================================
        # CV Bridge
        # ============================================
        self.bridge = CvBridge()

        # ============================================
        # 프레임 캐시 (최신 동기화 프레임)
        # ============================================
        self.frame_lock = threading.Lock()
        self.cached_frames = None  # (left_rgb, left_depth, right_rgb, right_depth)
        self.frame_timestamp = None

        # 카메라 intrinsics
        self.left_camera_info = None
        self.right_camera_info = None

        # ============================================
        # Message Filters - 카메라별 2채널 동기화
        # 한쪽 카메라만 연결되어도 동작
        # ============================================
        # 좌측 카메라 (zedxmini1)
        self.left_rgb_sub = message_filters.Subscriber(
            self, Image, '/zedxmini1/zed_node/left/image_rect_color'
        )
        self.left_depth_sub = message_filters.Subscriber(
            self, Image, '/zedxmini1/zed_node/depth/depth_registered'
        )
        self.ts_left = message_filters.ApproximateTimeSynchronizer(
            [self.left_rgb_sub, self.left_depth_sub],
            queue_size=10, slop=0.05
        )
        self.ts_left.registerCallback(self._left_sync_callback)

        # 우측 카메라 (zedxmini2)
        self.right_rgb_sub = message_filters.Subscriber(
            self, Image, '/zedxmini2/zed_node/left/image_rect_color'
        )
        self.right_depth_sub = message_filters.Subscriber(
            self, Image, '/zedxmini2/zed_node/depth/depth_registered'
        )
        self.ts_right = message_filters.ApproximateTimeSynchronizer(
            [self.right_rgb_sub, self.right_depth_sub],
            queue_size=10, slop=0.05
        )
        self.ts_right.registerCallback(self._right_sync_callback)

        # 개별 카메라 프레임 캐시
        self.left_frames = None   # (rgb_msg, depth_msg)
        self.right_frames = None  # (rgb_msg, depth_msg)

        # Temporal depth 버퍼 (numpy array 형태로 저장)
        self.left_depth_buffer = deque(maxlen=self.depth_buffer_size)
        self.right_depth_buffer = deque(maxlen=self.depth_buffer_size)

        # Camera Info 구독
        self.left_info_sub = self.create_subscription(
            CameraInfo,
            '/zedxmini1/zed_node/left/camera_info',
            self._left_info_callback, 10
        )
        self.right_info_sub = self.create_subscription(
            CameraInfo,
            '/zedxmini2/zed_node/left/camera_info',
            self._right_info_callback, 10
        )

        # ============================================
        # 서비스 서버
        # ============================================
        self.detect_service = self.create_service(
            DetectCrossings,
            '/rebar/detect_crossings',
            self._detect_crossings_callback
        )

        # ============================================
        # 디버그 퍼블리셔
        # ============================================
        self.debug_image_pub = self.create_publisher(Image, '/rebar/detection_image', 1)
        self.grid_pub = self.create_publisher(RebarGrid, '/rebar/detected_grid', 1)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Rebar Detection Node 초기화 완료')
        self.get_logger().info(f'  모델: {self.model_path}')
        self.get_logger().info(f'  신뢰도: {self.confidence_threshold}')
        self.get_logger().info(f'  그리드: {self.grid_rows}x{self.grid_cols}')
        self.get_logger().info(f'  깊이 범위: {self.min_depth_m}~{self.max_depth_m}m')
        self.get_logger().info(f'  검출 전략: {self.detection_strategy}')
        if self.detection_strategy == 'cross':
            self.get_logger().info(
                '    Left cam → Y- (먼쪽) 3pt, Right cam → Y+ (먼쪽) 3pt'
            )
        for cam, model in self.calibration_models.items():
            mtype = model.get('type', 'linear')
            if mtype == '3d':
                self.get_logger().info(f'  캘리브레이션[{cam}]: 3d 강체변환 (R,t)')
                continue
            info = f'  캘리브레이션[{cam}]: {mtype}, {model.get("num_points", 0)}쌍'
            if mtype == 'gbr':
                info += f', 평균오차={model.get("mean_error", 0):.1f}mm'
            else:
                info += (f', X R²={model.get("x_r2", 0):.4f}, '
                         f'Y R²={model.get("y_r2", 0):.4f}')
            self.get_logger().info(info)
        if not self.calibration_models:
            self.get_logger().warn('  캘리브레이션 모델 없음 → 기하학적 변환 사용')
        self.get_logger().info('=' * 60)

    # ============================================
    # 모델 로드
    # ============================================
    def _load_model(self):
        """YOLO 모델 로드 (ultralytics)"""
        try:
            from ultralytics import YOLO
            if not os.path.exists(self.model_path):
                self.get_logger().error(f'모델 파일 없음: {self.model_path}')
                return

            self.model = YOLO(self.model_path)
            # Jetson GPU 사용 시도
            try:
                self.model.to('cuda')
                self.get_logger().info('YOLO 모델 로드 완료 (GPU)')
            except Exception:
                self.get_logger().warn('GPU 사용 불가, CPU 모드로 실행')

        except ImportError:
            self.get_logger().error(
                'ultralytics 패키지 미설치. pip install ultralytics 실행 필요'
            )
        except Exception as e:
            self.get_logger().error(f'모델 로드 실패: {e}')

    def _load_calibration_models(self):
        """캘리브레이션 모델 로드 (calibration_model 파라미터에 따라 선택)

        모델 타입:
          - gbr: GBR joblib 모델 (pixel_u, pixel_v, depth_mm → X, Y)
          - nodepth: depth 없는 다항식 회귀 (pixel_u, pixel_v → X, Y)
          - linear: 기존 다중선형회귀 (pixel_u, pixel_v, depth_mm → X, Y)
        """
        model_type = self.calibration_model_type
        self.get_logger().info(f'캘리브레이션 모델 타입: {model_type}')

        # 모델 타입별 파일 경로
        calib_dir = '/home/koceti/ros2_ws/data/calibration'
        file_map = {
            'gbr': {
                'left': os.path.join(calib_dir, 'calibration_result_gbr_left.yaml'),
                'right': os.path.join(calib_dir, 'calibration_result_gbr_right.yaml'),
            },
            'nodepth': {
                'left': os.path.join(calib_dir, 'calibration_result_nodepth_left.yaml'),
                'right': os.path.join(calib_dir, 'calibration_result_nodepth_right.yaml'),
            },
            'linear': {
                'left': os.path.join(calib_dir, 'calibration_result_left.yaml'),
                'right': os.path.join(calib_dir, 'calibration_result_right.yaml'),
            },
            '3d': {
                'left': os.path.join(calib_dir, 'calib3d_result_left.yaml'),
                'right': os.path.join(calib_dir, 'calib3d_result_right.yaml'),
            },
        }

        if model_type not in file_map:
            self.get_logger().error(
                f'알 수 없는 캘리브레이션 모델: {model_type} '
                f'(gbr, nodepth, linear, 3d 중 선택)')
            return

        calib_files = file_map[model_type]

        for cam, path in calib_files.items():
            if not os.path.exists(path):
                self.get_logger().info(f'캘리브레이션 파일 없음: {path}')
                continue
            try:
                with open(path, 'r') as f:
                    data = yaml.safe_load(f)

                if model_type == '3d':
                    self._load_3d_model(cam, data)
                    continue
                cal = data['calibration']
                if model_type == 'gbr':
                    self._load_gbr_model(cam, cal)
                elif model_type == 'nodepth':
                    self._load_nodepth_model(cam, cal)
                else:  # linear
                    self._load_linear_model(cam, cal)

            except Exception as e:
                self.get_logger().error(f'캘리브레이션 로드 실패 [{cam}]: {e}')

    def _load_3d_model(self, cam, data):
        """3D 강체변환 모델 로드 (calib3d_result_{cam}.yaml: rigid_transform R,t).
        P_tool = R @ P_cam + t  (P_cam은 최근접표면 depth로 복원한 카메라 3D, mm)"""
        rt = data['rigid_transform']
        self.calibration_models[cam] = {
            'type': '3d',
            'R': np.array(rt['R'], dtype=float),
            't': np.array(rt['t'], dtype=float),
        }
        self.get_logger().info(f'  [{cam}] 3D 강체변환 로드 (R,t)')

    def _load_gbr_model(self, cam, cal):
        """GBR joblib 모델 로드"""
        import joblib
        model_x_path = cal.get('model_x_file', '')
        model_y_path = cal.get('model_y_file', '')
        if not os.path.exists(model_x_path) or not os.path.exists(model_y_path):
            self.get_logger().error(f'GBR joblib 파일 없음 [{cam}]: {model_x_path}')
            return
        self.calibration_models[cam] = {
            'type': 'gbr',
            'model_x': joblib.load(model_x_path),
            'model_y': joblib.load(model_y_path),
            'num_points': cal.get('num_points', 0),
            'mean_error': cal.get('mean_error_mm', 0),
        }
        self.get_logger().info(
            f'  [{cam}] GBR 로드: {cal.get("num_points", 0)}쌍, '
            f'평균오차={cal.get("mean_error_mm", 0):.1f}mm')

    def _load_nodepth_model(self, cam, cal):
        """depth 없는 다항식 회귀 계수 로드"""
        self.calibration_models[cam] = {
            'type': 'nodepth',
            'x_coeffs': np.array(cal['x_mapping']['coeffs']),
            'y_coeffs': np.array(cal['y_mapping']['coeffs']),
            'x_r2': cal['x_mapping']['r_squared'],
            'y_r2': cal['y_mapping']['r_squared'],
            'num_points': cal.get('num_points', 0),
        }
        self.get_logger().info(
            f'  [{cam}] nodepth 로드: {cal.get("num_points", 0)}쌍, '
            f'X R²={cal["x_mapping"]["r_squared"]:.4f}, '
            f'Y R²={cal["y_mapping"]["r_squared"]:.4f}')

    def _load_linear_model(self, cam, cal):
        """기존 다중선형회귀 계수 로드"""
        self.calibration_models[cam] = {
            'type': 'linear',
            'x_coeffs': np.array(cal['x_mapping']['coeffs']),
            'y_coeffs': np.array(cal['y_mapping']['coeffs']),
            'x_r2': cal['x_mapping']['r_squared'],
            'y_r2': cal['y_mapping']['r_squared'],
            'num_points': cal.get('num_points', 0),
        }
        self.get_logger().info(
            f'  [{cam}] linear 로드: {cal.get("num_points", 0)}쌍, '
            f'X R²={cal["x_mapping"]["r_squared"]:.4f}, '
            f'Y R²={cal["y_mapping"]["r_squared"]:.4f}')

    def _apply_calibration(self, pixel_u, pixel_v, depth_mm, camera_side):
        """캘리브레이션 모델로 로봇 좌표 (X, Y) mm 계산

        모델 타입에 따라 자동 분기:
          - gbr: GBR predict([[u, v, depth_mm]])
          - nodepth: X = a*u + b*v + c*u*v + d*u² + e*v² + f
          - linear: 6/8계수 다중선형회귀
        """
        model = self.calibration_models[camera_side]
        model_type = model.get('type', 'linear')

        if model_type == 'gbr':
            features = np.array([[pixel_u, pixel_v, depth_mm]])
            x_mm = float(model['model_x'].predict(features)[0])
            y_mm = float(model['model_y'].predict(features)[0])

        elif model_type == 'nodepth':
            features = np.array([
                pixel_u, pixel_v,
                pixel_u * pixel_v,
                pixel_u ** 2, pixel_v ** 2, 1.0
            ])
            x_mm = float(np.dot(model['x_coeffs'], features))
            y_mm = float(np.dot(model['y_coeffs'], features))

        else:  # linear
            n_coeffs = len(model['x_coeffs'])
            if n_coeffs == 8:
                features = np.array([
                    pixel_u, pixel_v, depth_mm,
                    pixel_u * depth_mm, pixel_v * depth_mm,
                    pixel_u ** 2, pixel_v ** 2, 1.0
                ])
            else:  # 6계수
                features = np.array([
                    pixel_u, pixel_v, depth_mm,
                    pixel_u * depth_mm, pixel_v * depth_mm, 1.0
                ])
            x_mm = float(np.dot(model['x_coeffs'], features))
            y_mm = float(np.dot(model['y_coeffs'], features))

        return x_mm, y_mm

    # ============================================
    # 프레임 캐싱 콜백
    # ============================================
    def _left_sync_callback(self, rgb_msg, depth_msg):
        """좌측 카메라 프레임 캐싱"""
        with self.frame_lock:
            self.left_frames = (rgb_msg, depth_msg)
            depth_np = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            self.left_depth_buffer.append(depth_np)
            self._update_cached_frames()

    def _right_sync_callback(self, rgb_msg, depth_msg):
        """우측 카메라 프레임 캐싱"""
        with self.frame_lock:
            self.right_frames = (rgb_msg, depth_msg)
            depth_np = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            self.right_depth_buffer.append(depth_np)
            self._update_cached_frames()

    def _update_cached_frames(self):
        """개별 프레임으로 cached_frames 구성 (lock 내에서 호출)"""
        left_rgb = self.left_frames[0] if self.left_frames else None
        left_depth = self.left_frames[1] if self.left_frames else None
        right_rgb = self.right_frames[0] if self.right_frames else None
        right_depth = self.right_frames[1] if self.right_frames else None
        self.cached_frames = (left_rgb, left_depth, right_rgb, right_depth)
        self.frame_timestamp = self.get_clock().now()

    def _left_info_callback(self, msg):
        """좌측 카메라 intrinsics 저장"""
        if self.left_camera_info is None:
            self.left_camera_info = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5],
                'width': msg.width, 'height': msg.height
            }
            self.get_logger().info(
                f'Left camera info: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}'
            )

    def _right_info_callback(self, msg):
        """우측 카메라 intrinsics 저장"""
        if self.right_camera_info is None:
            self.right_camera_info = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5],
                'width': msg.width, 'height': msg.height
            }
            self.get_logger().info(
                f'Right camera info: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}'
            )

    # ============================================
    # 서비스 핸들러
    # ============================================
    def _detect_crossings_callback(self, request, response):
        """교차점 검출 서비스 핸들러"""
        start_time = time.monotonic()

        # 모델 확인
        if self.model is None:
            response.success = False
            response.message = 'YOLO 모델이 로드되지 않음'
            return response

        # 캐시된 프레임 확인
        with self.frame_lock:
            frames = self.cached_frames
            left_depth_buf = list(self.left_depth_buffer)
            right_depth_buf = list(self.right_depth_buffer)

        if frames is None:
            response.success = False
            response.message = '카메라 프레임 수신 대기 중'
            return response

        # 신뢰도 설정
        conf_threshold = request.confidence_threshold
        if conf_threshold <= 0.0:
            conf_threshold = self.confidence_threshold

        expected_count = request.expected_count
        if expected_count == 0:
            expected_count = self.grid_rows * self.grid_cols

        try:
            left_rgb_msg, left_depth_msg, right_rgb_msg, right_depth_msg = frames

            # CV 이미지 변환 (카메라가 없으면 None)
            left_rgb = self.bridge.imgmsg_to_cv2(left_rgb_msg, 'bgr8') if left_rgb_msg else None
            left_depth = self.bridge.imgmsg_to_cv2(left_depth_msg, 'passthrough') if left_depth_msg else None
            right_rgb = self.bridge.imgmsg_to_cv2(right_rgb_msg, 'bgr8') if right_rgb_msg else None
            right_depth = self.bridge.imgmsg_to_cv2(right_depth_msg, 'passthrough') if right_depth_msg else None

            all_detections = []
            strategy = self.detection_strategy

            # 단일 카메라 요청 시 strategy 무관하게 처리
            if request.camera_selection in (1, 2):
                strategy = 'single'

            if strategy == 'cross':
                # ===== Cross Detection 전략 =====
                # far_side 필터링 제거: orchestrator가 Y범위 필터링 수행
                left_dets = []
                right_dets = []

                if self.left_camera_info and left_rgb is not None:
                    left_dets = self._detect_single_camera(
                        left_rgb, left_depth,
                        self.left_camera_info,
                        self.camera_extrinsics['left'],
                        camera_id=0,
                        conf_threshold=conf_threshold,
                        depth_buffer=left_depth_buf
                    )

                if self.right_camera_info and right_rgb is not None:
                    right_dets = self._detect_single_camera(
                        right_rgb, right_depth,
                        self.right_camera_info,
                        self.camera_extrinsics['right'],
                        camera_id=1,
                        conf_threshold=conf_threshold,
                        depth_buffer=right_depth_buf
                    )

                self.get_logger().info(
                    f'  dedup후: L={len(left_dets)}개, R={len(right_dets)}개')
                all_detections = left_dets + right_dets

            else:
                # ===== Merge / Single 전략 =====
                if request.camera_selection in (0, 1):  # 좌측 또는 양쪽
                    if self.left_camera_info and left_rgb is not None:
                        dets = self._detect_single_camera(
                            left_rgb, left_depth,
                            self.left_camera_info,
                            self.camera_extrinsics['left'],
                            camera_id=0,
                            conf_threshold=conf_threshold,
                            depth_buffer=left_depth_buf
                        )
                        all_detections.extend(dets)

                if request.camera_selection in (0, 2):  # 우측 또는 양쪽
                    if self.right_camera_info and right_rgb is not None:
                        dets = self._detect_single_camera(
                            right_rgb, right_depth,
                            self.right_camera_info,
                            self.camera_extrinsics['right'],
                            camera_id=1,
                            conf_threshold=conf_threshold,
                            depth_buffer=right_depth_buf
                        )
                        all_detections.extend(dets)

            total_detected = len(all_detections)

            # 2x3 그리드 정렬
            grid = self._sort_into_grid(
                all_detections, self.grid_rows, self.grid_cols
            )

            # 디버그 이미지 발행 (사용 가능한 이미지 선택)
            debug_rgb = right_rgb if right_rgb is not None else left_rgb
            if debug_rgb is not None:
                self._publish_debug_image(debug_rgb, all_detections)

            # 응답 구성
            elapsed_ms = (time.monotonic() - start_time) * 1000.0

            response.success = True
            response.grid = grid
            response.grid.total_detected = total_detected
            response.detection_time_ms = elapsed_ms
            response.message = (
                f'{len(grid.detections)}개 교차점 검출 '
                f'(총 {total_detected}개 중, {elapsed_ms:.1f}ms)'
            )

            # 모니터링 토픽 발행
            self.grid_pub.publish(grid)

            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'검출 오류: {e}'
            self.get_logger().error(response.message)

        return response

    # ============================================
    # 단일 카메라 검출
    # ============================================
    def _detect_single_camera(self, rgb, depth, camera_info, extrinsics,
                              camera_id, conf_threshold, depth_buffer=None):
        """단일 카메라에서 교차점 검출 + 3D 변환"""
        detections = []

        # YOLO 추론 (낮은 threshold로 전체 후보 확인, 이후 수동 필터링)
        yolo_min_conf = 0.1
        results = self.model(rgb, conf=yolo_min_conf, verbose=False)

        if not results or len(results[0].boxes) == 0:
            self.get_logger().info(
                f'  [{extrinsics.get("side", "?")}] YOLO: 0개 검출 (conf>={yolo_min_conf})')
            return detections

        all_boxes = results[0].boxes
        all_confs = [float(b.conf[0]) for b in all_boxes]
        above = sum(1 for c in all_confs if c >= conf_threshold)
        below = len(all_confs) - above
        conf_str = ', '.join(f'{c:.2f}' for c in sorted(all_confs, reverse=True))
        self.get_logger().info(
            f'  [{extrinsics.get("side", "?")}] YOLO: {len(all_boxes)}개 검출 '
            f'(>={conf_threshold}: {above}개, <{conf_threshold}: {below}개) '
            f'[{conf_str}]')

        camera_side = extrinsics.get('side', 'right')
        use_calibration = camera_side in self.calibration_models

        for box in all_boxes:
            # 바운딩 박스 중심
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            confidence = float(box.conf[0])

            # confidence threshold 미만은 스킵
            if confidence < conf_threshold:
                continue

            model_type_cam = (self.calibration_models[camera_side]['type']
                              if use_calibration else None)

            if model_type_cam == '3d':
                # 3D 강체변환: 최근접표면 depth → 카메라3D → R·P_cam+t (calibrate_3d 동일)
                z_mm = self._near_surface_depth_mm(depth_buffer, depth, cx, cy)
                if z_mm is None:
                    continue
                ppx, ppy = camera_info['cx'], camera_info['cy']
                fx, fy = camera_info['fx'], camera_info['fy']
                p_cam = np.array([(cx - ppx) * z_mm / fx,
                                  (cy - ppy) * z_mm / fy, z_mm])
                m = self.calibration_models[camera_side]
                rob = m['R'] @ p_cam + m['t']
                robot_x, robot_y, robot_z = float(rob[0]), float(rob[1]), 0.0
                depth_mm = z_mm
            else:
                # 깊이 샘플링 (공간 median + temporal median)
                depth_m = self._sample_depth(depth, cx, cy, depth_buffer)
                if depth_m is None:
                    continue
                depth_mm = depth_m * 1000.0

                if use_calibration:
                    # 캘리브레이션 회귀 모델로 직접 변환
                    robot_x, robot_y = self._apply_calibration(
                        cx, cy, depth_mm, camera_side
                    )
                    robot_z = 0.0  # Z는 스테이지에서 별도 제어
                else:
                    # fallback: 기하학적 변환
                    point_cam = self._pixel_to_camera_3d(
                        cx, cy, depth_m, camera_info
                    )
                    point_robot = self._camera_to_robot(point_cam, extrinsics)
                    robot_x = float(point_robot[0])
                    robot_y = float(point_robot[1])
                    robot_z = float(point_robot[2])

            det = RebarDetection()
            det.x = robot_x
            det.y = robot_y
            det.z = robot_z
            det.confidence = confidence
            det.depth_mm = depth_mm
            det.pixel_u = cx
            det.pixel_v = cy
            det.camera_id = camera_id
            detections.append(det)

        # 같은 카메라 내 근접 포인트 중복 제거
        if len(detections) > 1:
            detections = self._deduplicate(detections)

        return detections

    # ============================================
    # 깊이 샘플링
    # ============================================
    def _sample_depth_frame(self, depth_image, u, v):
        """단일 프레임에서 공간 median depth 추출 (미터)"""
        h, w = depth_image.shape[:2]
        half = self.depth_kernel_size // 2

        y_min = max(0, v - half)
        y_max = min(h, v + half + 1)
        x_min = max(0, u - half)
        x_max = min(w, u + half + 1)

        region = depth_image[y_min:y_max, x_min:x_max]
        valid = region[np.isfinite(region) &
                       (region > self.min_depth_m) &
                       (region < self.max_depth_m)]

        if len(valid) == 0:
            return None

        return float(np.median(valid))

    def _near_surface_depth_mm(self, depth_buffer, depth_image, u, v, win=9):
        """철근 교차점=최근접표면 depth(mm). calibrate_3d.rebar_depth_mm와 동일.
        윈도우+버퍼 풀 → 5%분위 근접밴드(+25mm) 평균 → 철판 섞여도 철근면 고정."""
        r = win // 2
        frames = list(depth_buffer) if depth_buffer else (
            [depth_image] if depth_image is not None else [])
        pool = []
        for frame in frames:
            h, w = frame.shape[:2]
            if not (0 <= u < w and 0 <= v < h):
                continue
            patch = frame[max(0, v-r):v+r+1, max(0, u-r):u+r+1].ravel()
            valid = patch[np.isfinite(patch) &
                          (patch > self.min_depth_m) & (patch < self.max_depth_m)]
            pool.extend((valid * 1000.0).tolist())
        if len(pool) < 10:
            return None
        pool = np.array(pool)
        lo = np.percentile(pool, 5)
        band = pool[pool < lo + 25.0]
        if band.size < 3:
            band = pool
        return float(band.mean())

    def _sample_depth(self, depth_image, u, v, depth_buffer=None):
        """깊이 추출: 공간 median + temporal median (미터)

        depth_buffer가 있으면 최근 N프레임에서 각각 공간 median을 뽑고
        그 값들의 median을 취해 순간 depth 스파이크를 제거한다.
        """
        if depth_buffer and len(depth_buffer) > 1:
            samples = []
            for frame in depth_buffer:
                val = self._sample_depth_frame(frame, u, v)
                if val is not None:
                    samples.append(val)
            if samples:
                return float(np.median(samples))
            return None

        return self._sample_depth_frame(depth_image, u, v)

    # ============================================
    # 좌표 변환
    # ============================================
    def _pixel_to_camera_3d(self, u, v, depth_m, camera_info):
        """픽셀 (u, v) + 깊이 → 카메라 좌표계 3D (미터)"""
        fx = camera_info['fx']
        fy = camera_info['fy']
        cx = camera_info['cx']
        cy = camera_info['cy']

        x_cam = (u - cx) * depth_m / fx
        y_cam = (v - cy) * depth_m / fy
        z_cam = depth_m

        return np.array([x_cam, y_cam, z_cam])

    def _camera_to_robot(self, point_cam, extrinsics):
        """카메라 3D → 로봇 좌표계 (mm)

        카메라 프레임 (OpenCV): X=우, Y=하, Z=전방
        로봇 프레임: X=전방(+), Y=우(+), Z=상(+)
        원점: EEF_origin (스테이지 홈 위치 공구 끝단)

        변환 순서:
        1. 카메라 프레임에서 pitch 적용 (하향 틸트)
        2. 카메라→로봇 프레임 축 정렬 (카메라 바라보는 방향에 따라)
        3. 로봇 프레임에서 yaw 적용 (내향 각도)
        4. 카메라 위치(translation) 더하기
        """
        pitch_deg = extrinsics['rotation'][0]  # 하향 각도 (양수 = 아래로)
        yaw_deg = extrinsics['rotation'][2]    # 내향 각도
        side = extrinsics.get('side', 'right')
        t = extrinsics['position']  # mm 단위

        R = self._build_cam_to_robot_rotation(side, pitch_deg, yaw_deg)

        # 카메라 좌표 (미터) → mm 변환 후 회전+이동
        point_cam_mm = point_cam * 1000.0
        point_robot = R @ point_cam_mm + t

        return point_robot

    @staticmethod
    def _build_cam_to_robot_rotation(side, pitch_deg, yaw_deg):
        """카메라→로봇 회전 행렬 구성

        카메라 설치:
        - right (zedxmini_right): -Y 방향 바라봄 (오른쪽에서 왼쪽으로)
        - left (zedxmini_left): +Y 방향 바라봄 (왼쪽에서 오른쪽으로)

        변환: R_full = Rz_robot(yaw) @ T_base(side) @ Rx_cam(-pitch)
        """
        # Step 1: 카메라 프레임에서 하향 틸트 (Camera X축 기준)
        p = np.radians(-pitch_deg)  # 하향 = 음수 회전
        Rx_cam = np.array([
            [1, 0, 0],
            [0, np.cos(p), -np.sin(p)],
            [0, np.sin(p), np.cos(p)]
        ])

        # Step 2: 카메라→로봇 프레임 축 정렬
        if side == 'right':
            # Camera Z(forward) → -Y_robot, Camera X(right) → -X_robot
            T_base = np.array([
                [-1,  0,  0],
                [ 0,  0, -1],
                [ 0, -1,  0]
            ], dtype=float)
        else:
            # Camera Z(forward) → +Y_robot, Camera X(right) → +X_robot
            T_base = np.array([
                [ 1,  0,  0],
                [ 0,  0,  1],
                [ 0, -1,  0]
            ], dtype=float)

        # Step 3: 로봇 프레임에서 yaw 회전 (Z축 기준)
        y = np.radians(yaw_deg)
        Rz = np.array([
            [np.cos(y), -np.sin(y), 0],
            [np.sin(y),  np.cos(y), 0],
            [0, 0, 1]
        ])

        return Rz @ T_base @ Rx_cam

    # ============================================
    # Cross Detection: 먼 쪽 포인트 필터링
    # ============================================
    def _filter_far_side(self, detections, camera_side, max_points):
        """카메라 반대쪽(먼 쪽) 포인트만 필터링

        Left 카메라 (Y=+100mm): Y가 작은(Y-) 포인트 = 카메라 기준 먼 쪽
        Right 카메라 (Y=-100mm): Y가 큰(Y+) 포인트 = 카메라 기준 먼 쪽

        깊이 최소 측정거리 문제를 회피하고, 결속 공구 가림 문제도 감소
        """
        if not detections:
            return detections

        if camera_side == 'left':
            # Left 카메라: Y가 작은 순서 (먼 쪽)로 정렬 → 상위 max_points 선택
            sorted_dets = sorted(detections, key=lambda d: d.y)
        else:
            # Right 카메라: Y가 큰 순서 (먼 쪽)로 정렬 → 상위 max_points 선택
            sorted_dets = sorted(detections, key=lambda d: d.y, reverse=True)

        selected = sorted_dets[:max_points]
        dropped = sorted_dets[max_points:]

        self.get_logger().info(
            f'  [{camera_side}] far_side: {len(detections)}개 → {len(selected)}개 선택, '
            f'{len(dropped)}개 제거')
        for d in selected:
            self.get_logger().info(
                f'    ✓ x={d.x:.1f} y={d.y:.1f} conf={d.confidence:.2f}')
        for d in dropped:
            self.get_logger().info(
                f'    ✗ x={d.x:.1f} y={d.y:.1f} conf={d.confidence:.2f} (제거)')
        return selected

    # ============================================
    # 중복 제거
    # ============================================
    def _deduplicate(self, detections):
        """양쪽 카메라 검출 결과 중복 제거 (거리 기반)"""
        if not detections:
            return detections

        kept = []
        for det in detections:
            is_dup = False
            for existing in kept:
                dist = np.sqrt(
                    (det.x - existing.x) ** 2 +
                    (det.y - existing.y) ** 2
                )
                if dist < self.dedup_distance_mm:
                    # 높은 신뢰도 유지
                    if det.confidence > existing.confidence:
                        kept.remove(existing)
                        kept.append(det)
                    is_dup = True
                    break
            if not is_dup:
                kept.append(det)

        return kept

    # ============================================
    # 그리드 정렬
    # ============================================
    def _sort_into_grid(self, detections, grid_rows, grid_cols):
        """검출 결과를 2x3 그리드 순서로 정렬

        그리드 레이아웃 (로봇 기준, X=전방, Y=횡방향):
          Row 0 (Y- 쪽, Right 카메라가 담당): [col0] [col1] [col2]  X 기준 정렬
          Row 1 (Y+ 쪽, Left 카메라가 담당):  [col0] [col1] [col2]  X 기준 정렬

        Cross 전략에서는 camera_id로 자연 분리 가능
        """
        grid = RebarGrid()
        grid.grid_rows = grid_rows
        grid.grid_cols = grid_cols
        expected = grid_rows * grid_cols

        if len(detections) < expected:
            grid.valid = False
            grid.error_message = (
                f'{expected}개 필요, {len(detections)}개만 검출됨'
            )
            sorted_dets = sorted(detections, key=lambda d: (d.y, d.x))
            grid.detections = sorted_dets
            return grid

        # 신뢰도 높은 순으로 expected개 선택
        sorted_by_conf = sorted(detections, key=lambda d: d.confidence, reverse=True)
        selected = sorted_by_conf[:expected]

        if self.detection_strategy == 'cross':
            # Cross 전략: camera_id로 행 분리
            # camera_id=1 (Right) → Y- 먼 쪽 = Row 0
            # camera_id=0 (Left) → Y+ 먼 쪽 = Row 1
            row_0 = [d for d in selected if d.camera_id == 1]  # Right → Y-
            row_1 = [d for d in selected if d.camera_id == 0]  # Left → Y+
        else:
            # Merge/Single: Y 기준 정렬 → 행 분리
            sorted_by_y = sorted(selected, key=lambda d: d.y)
            row_0 = sorted_by_y[:grid_cols]  # Y가 작은 행
            row_1 = sorted_by_y[grid_cols:]  # Y가 큰 행

        # 각 행을 X 기준 정렬 (전방 방향 순서)
        row_0.sort(key=lambda d: d.x)
        row_1.sort(key=lambda d: d.x)

        grid.detections = row_0 + row_1
        grid.valid = True
        grid.error_message = ''

        return grid

    # ============================================
    # 디버그 이미지 발행
    # ============================================
    def _publish_debug_image(self, rgb_image, detections):
        """검출 결과가 표시된 디버그 이미지 발행"""
        debug_img = rgb_image.copy()

        for i, det in enumerate(detections):
            # 중심점 표시
            color = (0, 255, 0) if det.camera_id == 0 else (255, 0, 0)
            cv2.circle(debug_img, (det.pixel_u, det.pixel_v), 8, color, 2)

            # 인덱스 및 좌표 표시
            label = f'#{i} ({det.x:.0f},{det.y:.0f}) {det.confidence:.2f}'
            cv2.putText(debug_img, label,
                        (det.pixel_u + 10, det.pixel_v - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'디버그 이미지 발행 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RebarDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
