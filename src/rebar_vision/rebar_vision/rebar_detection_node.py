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


class RebarDetectionNode(Node):
    """YOLO 기반 철근 교차점 검출 서비스 노드"""

    def __init__(self):
        super().__init__('rebar_detection_node')

        # ============================================
        # 파라미터 선언
        # ============================================
        self.declare_parameter('model_path',
                               '/home/koceti/ros2_ws/src/rebar_vision/model/best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('depth_kernel_size', 5)
        self.declare_parameter('max_depth_m', 2.0)
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('grid_rows', 2)
        self.declare_parameter('grid_cols', 3)
        self.declare_parameter('dedup_distance_mm', 20.0)
        self.declare_parameter('detection_strategy', 'cross')  # cross, merge, single

        # 카메라 외부 파라미터 (position: mm, rotation: degrees)
        self.declare_parameter('left_camera.position', [-200.0, 100.0, 108.0])
        self.declare_parameter('left_camera.rotation', [40.0, 0.0, -20.0])
        self.declare_parameter('right_camera.position', [-200.0, -100.0, 108.0])
        self.declare_parameter('right_camera.rotation', [40.0, 0.0, 20.0])

        # 파라미터 가져오기
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.depth_kernel_size = self.get_parameter('depth_kernel_size').value
        self.max_depth_m = self.get_parameter('max_depth_m').value
        self.min_depth_m = self.get_parameter('min_depth_m').value
        self.grid_rows = self.get_parameter('grid_rows').value
        self.grid_cols = self.get_parameter('grid_cols').value
        self.dedup_distance_mm = self.get_parameter('dedup_distance_mm').value
        self.detection_strategy = self.get_parameter('detection_strategy').value

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

    # ============================================
    # 프레임 캐싱 콜백
    # ============================================
    def _left_sync_callback(self, rgb_msg, depth_msg):
        """좌측 카메라 프레임 캐싱"""
        with self.frame_lock:
            self.left_frames = (rgb_msg, depth_msg)
            self._update_cached_frames()

    def _right_sync_callback(self, rgb_msg, depth_msg):
        """우측 카메라 프레임 캐싱"""
        with self.frame_lock:
            self.right_frames = (rgb_msg, depth_msg)
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
                # Left 카메라(Y=+100mm) → 먼 쪽(Y-) 포인트 검출
                # Right 카메라(Y=-100mm) → 먼 쪽(Y+) 포인트 검출
                # 각 카메라가 3개씩, 총 6개 → 중복 제거 불필요
                left_dets = []
                right_dets = []

                if self.left_camera_info and left_rgb is not None:
                    left_dets = self._detect_single_camera(
                        left_rgb, left_depth,
                        self.left_camera_info,
                        self.camera_extrinsics['left'],
                        camera_id=0,
                        conf_threshold=conf_threshold
                    )
                    # Left 카메라: 먼 쪽 = Y가 작은 포인트 (Y-)
                    left_dets = self._filter_far_side(
                        left_dets, camera_side='left',
                        max_points=self.grid_cols
                    )

                if self.right_camera_info and right_rgb is not None:
                    right_dets = self._detect_single_camera(
                        right_rgb, right_depth,
                        self.right_camera_info,
                        self.camera_extrinsics['right'],
                        camera_id=1,
                        conf_threshold=conf_threshold
                    )
                    # Right 카메라: 먼 쪽 = Y가 큰 포인트 (Y+)
                    right_dets = self._filter_far_side(
                        right_dets, camera_side='right',
                        max_points=self.grid_cols
                    )

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
                            conf_threshold=conf_threshold
                        )
                        all_detections.extend(dets)

                if request.camera_selection in (0, 2):  # 우측 또는 양쪽
                    if self.right_camera_info and right_rgb is not None:
                        dets = self._detect_single_camera(
                            right_rgb, right_depth,
                            self.right_camera_info,
                            self.camera_extrinsics['right'],
                            camera_id=1,
                            conf_threshold=conf_threshold
                        )
                        all_detections.extend(dets)

                # merge 전략: 양쪽 카메라 중복 제거
                if strategy == 'merge' and request.camera_selection == 0:
                    if len(all_detections) > expected_count:
                        all_detections = self._deduplicate(all_detections)

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
                              camera_id, conf_threshold):
        """단일 카메라에서 교차점 검출 + 3D 변환"""
        detections = []

        # YOLO 추론
        results = self.model(rgb, conf=conf_threshold, verbose=False)

        if not results or len(results[0].boxes) == 0:
            return detections

        boxes = results[0].boxes

        for box in boxes:
            # 바운딩 박스 중심
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            confidence = float(box.conf[0])

            # 깊이 샘플링 (median 필터)
            depth_m = self._sample_depth(depth, cx, cy)
            if depth_m is None:
                continue

            # 픽셀 → 카메라 3D
            point_cam = self._pixel_to_camera_3d(
                cx, cy, depth_m, camera_info
            )

            # 카메라 3D → 로봇 좌표
            point_robot = self._camera_to_robot(point_cam, extrinsics)

            det = RebarDetection()
            det.x = float(point_robot[0])
            det.y = float(point_robot[1])
            det.z = float(point_robot[2])
            det.confidence = confidence
            det.depth_mm = float(depth_m * 1000.0)
            det.pixel_u = cx
            det.pixel_v = cy
            det.camera_id = camera_id
            detections.append(det)

        return detections

    # ============================================
    # 깊이 샘플링
    # ============================================
    def _sample_depth(self, depth_image, u, v):
        """깊이 이미지에서 중앙값 기반 깊이 추출 (미터)"""
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

        self.get_logger().debug(
            f'[{camera_side}] 전체 {len(detections)}개 중 '
            f'먼 쪽 {len(selected)}개 선택'
        )
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
