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

        self.camera_extrinsics = {
            'left': {
                'position': np.array(self.get_parameter('left_camera.position').value),
                'rotation': np.array(self.get_parameter('left_camera.rotation').value),
            },
            'right': {
                'position': np.array(self.get_parameter('right_camera.position').value),
                'rotation': np.array(self.get_parameter('right_camera.rotation').value),
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
        # Message Filters - 4채널 동기화
        # (dual_camera_recorder_node.py 패턴 재사용)
        # ============================================
        self.left_rgb_sub = message_filters.Subscriber(
            self, Image, '/zedxmini1/zedxmini1_node/left/image_rect_color'
        )
        self.left_depth_sub = message_filters.Subscriber(
            self, Image, '/zedxmini1/zedxmini1_node/depth/depth_registered'
        )
        self.right_rgb_sub = message_filters.Subscriber(
            self, Image, '/zedxmini2/zedxmini2_node/left/image_rect_color'
        )
        self.right_depth_sub = message_filters.Subscriber(
            self, Image, '/zedxmini2/zedxmini2_node/depth/depth_registered'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_rgb_sub, self.left_depth_sub,
             self.right_rgb_sub, self.right_depth_sub],
            queue_size=10,
            slop=0.05  # 50ms
        )
        self.ts.registerCallback(self._sync_callback)

        # Camera Info 구독
        self.left_info_sub = self.create_subscription(
            CameraInfo,
            '/zedxmini1/zedxmini1_node/left/camera_info',
            self._left_info_callback, 10
        )
        self.right_info_sub = self.create_subscription(
            CameraInfo,
            '/zedxmini2/zedxmini2_node/left/camera_info',
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
    def _sync_callback(self, left_rgb, left_depth, right_rgb, right_depth):
        """동기화된 4채널 프레임 캐싱 (추론 없이 저장만)"""
        with self.frame_lock:
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

            # CV 이미지 변환
            left_rgb = self.bridge.imgmsg_to_cv2(left_rgb_msg, 'bgr8')
            left_depth = self.bridge.imgmsg_to_cv2(left_depth_msg, 'passthrough')
            right_rgb = self.bridge.imgmsg_to_cv2(right_rgb_msg, 'bgr8')
            right_depth = self.bridge.imgmsg_to_cv2(right_depth_msg, 'passthrough')

            all_detections = []

            # 카메라별 검출
            if request.camera_selection in (0, 1):  # 좌측 또는 양쪽
                if self.left_camera_info:
                    dets = self._detect_single_camera(
                        left_rgb, left_depth,
                        self.left_camera_info,
                        self.camera_extrinsics['left'],
                        camera_id=0,
                        conf_threshold=conf_threshold
                    )
                    all_detections.extend(dets)

            if request.camera_selection in (0, 2):  # 우측 또는 양쪽
                if self.right_camera_info:
                    dets = self._detect_single_camera(
                        right_rgb, right_depth,
                        self.right_camera_info,
                        self.camera_extrinsics['right'],
                        camera_id=1,
                        conf_threshold=conf_threshold
                    )
                    all_detections.extend(dets)

            total_detected = len(all_detections)

            # 양쪽 카메라 사용 시 중복 제거
            if request.camera_selection == 0 and len(all_detections) > expected_count:
                all_detections = self._deduplicate(all_detections)

            # 2x3 그리드 정렬
            grid = self._sort_into_grid(
                all_detections, self.grid_rows, self.grid_cols
            )

            # 디버그 이미지 발행
            self._publish_debug_image(left_rgb, all_detections)

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
        로봇 프레임: X=전방, Y=좌, Z=상
        """
        pitch = np.radians(extrinsics['rotation'][0])  # 40도 하향
        roll = np.radians(extrinsics['rotation'][1])
        yaw = np.radians(extrinsics['rotation'][2])

        R = self._rotation_matrix(roll, pitch, yaw)
        t = extrinsics['position']  # mm 단위

        # 카메라 좌표 (미터) → mm 변환 후 회전+이동
        point_cam_mm = point_cam * 1000.0
        point_robot = R @ point_cam_mm + t

        return point_robot

    @staticmethod
    def _rotation_matrix(roll, pitch, yaw):
        """ZYX 오일러 각도 → 회전 행렬

        카메라→로봇 변환:
        1. Rz(yaw): 좌우 카메라의 내향 각도
        2. Ry(pitch): 하향 각도 (40도)
        3. Rx(roll): 기울기 (0)
        """
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

        return Rz @ Ry @ Rx

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

        그리드 레이아웃 (로봇 X=전방):
          Row 0 (작은 X, 가까운 쪽): [col0] [col1] [col2]  Y 기준 정렬
          Row 1 (큰 X, 먼 쪽):      [col0] [col1] [col2]  Y 기준 정렬
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
            # 검출된 것이라도 정렬해서 반환
            sorted_dets = sorted(detections, key=lambda d: (d.x, d.y))
            grid.detections = sorted_dets
            return grid

        # 신뢰도 높은 순으로 expected개 선택
        sorted_by_conf = sorted(detections, key=lambda d: d.confidence, reverse=True)
        selected = sorted_by_conf[:expected]

        # X 기준 정렬 → 행 분리
        sorted_by_x = sorted(selected, key=lambda d: d.x)
        row_0 = sorted_by_x[:grid_cols]
        row_1 = sorted_by_x[grid_cols:]

        # 각 행을 Y 기준 정렬
        row_0.sort(key=lambda d: d.y)
        row_1.sort(key=lambda d: d.y)

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
