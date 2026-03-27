"""
장애물/사람 감지 노드 (YOLO + ZED Depth)

전방(zed_front)/후방(zed_back) 카메라로 사람 및 장애물을 감지하고,
일정 거리 이내이면 /obstacle_pause 토픽을 발행하여 주행/결속을 일시정지시킨다.

토픽:
  구독:
    - /zed_front/zed_node/left/image_rect_color (RGB)
    - /zed_front/zed_node/depth/depth_registered (Depth)
    - /zed_back/zed_node/left/image_rect_color (RGB)
    - /zed_back/zed_node/depth/depth_registered (Depth)
  발행:
    - /obstacle_pause (Bool) - True: 장애물 감지, False: 해제
    - /obstacle_status (String) - JSON 상태 정보

파라미터:
    - enabled: 감지 활성화 (default: true)
    - distance_threshold_m: 감지 거리 임계값 (default: 0.5m)
    - monitor_while_tying: 결속 중 감시 여부 (default: true)
    - detection_interval_sec: 검출 주기 (default: 0.5초)
    - yolo_model: YOLO 모델 이름 (default: yolov8n.pt)
    - confidence_threshold: YOLO 검출 신뢰도 (default: 0.5)
    - target_classes: 감지 대상 클래스 ID 목록 (default: [0] = person)
    - pause_hold_sec: 장애물 사라진 후 PAUSE 유지 시간 (default: 1.0초)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import numpy as np
import json
import time


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # 파라미터 선언
        self.declare_parameter('enabled', True)
        self.declare_parameter('distance_threshold_m', 0.5)
        self.declare_parameter('monitor_while_tying', True)
        self.declare_parameter('detection_interval_sec', 0.5)
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_classes', [0])  # 0=person
        self.declare_parameter('pause_hold_sec', 1.0)

        # 파라미터 로드
        self.enabled = self.get_parameter('enabled').value
        self.distance_threshold = self.get_parameter('distance_threshold_m').value
        self.monitor_while_tying = self.get_parameter('monitor_while_tying').value
        self.detection_interval = self.get_parameter('detection_interval_sec').value
        self.yolo_model_name = self.get_parameter('yolo_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.target_classes = self.get_parameter('target_classes').value
        self.pause_hold_sec = self.get_parameter('pause_hold_sec').value

        # YOLO 모델 로드
        self.model = None
        self._load_yolo_model()

        # CvBridge
        self.bridge = CvBridge()

        # 상태 변수
        self.obstacle_detected = False
        self.last_detection_time = 0.0
        self.last_obstacle_time = 0.0
        self.pause_published = False

        # 카메라별 최신 이미지/뎁스 저장
        self.front_rgb = None
        self.front_depth = None
        self.back_rgb = None
        self.back_depth = None
        self.front_rgb_stamp = 0.0
        self.back_rgb_stamp = 0.0

        # 구독: 전방 카메라 (zed_front = 전진 시 전방)
        self.front_rgb_sub = self.create_subscription(
            Image,
            '/zed_front/zed_node/left/image_rect_color',
            self._front_rgb_callback,
            10
        )
        self.front_depth_sub = self.create_subscription(
            Image,
            '/zed_front/zed_node/depth/depth_registered',
            self._front_depth_callback,
            10
        )

        # 구독: 후방 카메라 (zed_back = 후진 시 전방)
        self.back_rgb_sub = self.create_subscription(
            Image,
            '/zed_back/zed_node/left/image_rect_color',
            self._back_rgb_callback,
            10
        )
        self.back_depth_sub = self.create_subscription(
            Image,
            '/zed_back/zed_node/depth/depth_registered',
            self._back_depth_callback,
            10
        )

        # 발행
        self.pause_pub = self.create_publisher(Bool, '/obstacle_pause', 10)
        self.status_pub = self.create_publisher(String, '/obstacle_status', 10)

        # 검출 타이머
        self.timer = self.create_timer(
            self.detection_interval, self._detection_loop)

        # 클래스 이름 매핑
        self.class_names = {}
        if self.model is not None:
            self.class_names = self.model.names

        self.get_logger().info('=== Obstacle Detector 초기화 ===')
        self.get_logger().info(f'  enabled: {self.enabled}')
        self.get_logger().info(f'  distance_threshold: {self.distance_threshold}m')
        self.get_logger().info(f'  monitor_while_tying: {self.monitor_while_tying}')
        self.get_logger().info(f'  detection_interval: {self.detection_interval}s')
        self.get_logger().info(f'  target_classes: {self.target_classes}')
        self.get_logger().info(f'  yolo_model: {self.yolo_model_name}')

    def _load_yolo_model(self):
        """YOLO COCO 모델 로드"""
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.yolo_model_name)
            self.get_logger().info(
                f'YOLO 모델 로드: {self.yolo_model_name} '
                f'({len(self.model.names)} classes)')
        except Exception as e:
            self.get_logger().error(f'YOLO 모델 로드 실패: {e}')
            self.model = None

    # ===== 카메라 콜백 =====

    def _front_rgb_callback(self, msg):
        self.front_rgb = msg
        self.front_rgb_stamp = time.time()

    def _front_depth_callback(self, msg):
        self.front_depth = msg

    def _back_rgb_callback(self, msg):
        self.back_rgb = msg
        self.back_rgb_stamp = time.time()

    def _back_depth_callback(self, msg):
        self.back_depth = msg

    # ===== 메인 검출 루프 =====

    def _detection_loop(self):
        """주기적 검출 루프"""
        if not self.enabled or self.model is None:
            return

        now = time.time()
        detections = []

        # 전방 카메라 검출
        if self.front_rgb is not None and (now - self.front_rgb_stamp) < 2.0:
            front_dets = self._detect_camera(
                self.front_rgb, self.front_depth, 'front')
            detections.extend(front_dets)

        # 후방 카메라 검출
        if self.back_rgb is not None and (now - self.back_rgb_stamp) < 2.0:
            back_dets = self._detect_camera(
                self.back_rgb, self.back_depth, 'back')
            detections.extend(back_dets)

        # 임계거리 이내 장애물 확인
        close_obstacles = [
            d for d in detections if d['distance_m'] < self.distance_threshold
        ]

        if close_obstacles:
            self.last_obstacle_time = now
            if not self.obstacle_detected:
                self.obstacle_detected = True
                closest = min(close_obstacles, key=lambda d: d['distance_m'])
                cls_name = self.class_names.get(
                    closest['class_id'], str(closest['class_id']))
                self.get_logger().warn(
                    f'장애물 감지! {cls_name} @ {closest["distance_m"]:.2f}m '
                    f'({closest["camera"]})')
            self._publish_pause(True)
        else:
            # pause_hold 시간 경과 후 해제
            if self.obstacle_detected:
                if (now - self.last_obstacle_time) > self.pause_hold_sec:
                    self.obstacle_detected = False
                    self.get_logger().info('장애물 해제 → RESUME')
                    self._publish_pause(False)

        # 상태 발행
        self._publish_status(detections)

    def _detect_camera(self, rgb_msg, depth_msg, camera_name):
        """한 카메라에서 YOLO 검출 + depth 거리 계산"""
        detections = []

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(
                f'{camera_name} RGB 변환 실패: {e}',
                throttle_duration_sec=5.0)
            return detections

        # depth 이미지 변환
        depth_arr = None
        if depth_msg is not None:
            try:
                depth_arr = self.bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding='passthrough')
            except Exception:
                pass

        # YOLO 추론
        results = self.model(
            rgb_img,
            conf=self.confidence_threshold,
            classes=self.target_classes,
            verbose=False
        )

        if not results or len(results[0].boxes) == 0:
            return detections

        boxes = results[0].boxes
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # depth에서 bbox 중앙 영역의 거리 계산
            distance_m = float('inf')
            if depth_arr is not None:
                distance_m = self._get_bbox_distance(
                    depth_arr, x1, y1, x2, y2)

            detections.append({
                'camera': camera_name,
                'class_id': cls_id,
                'class_name': self.class_names.get(cls_id, str(cls_id)),
                'confidence': conf,
                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                'distance_m': distance_m,
            })

        return detections

    def _get_bbox_distance(self, depth_arr, x1, y1, x2, y2):
        """bbox 중앙 영역의 median depth를 반환 (m)"""
        h, w = depth_arr.shape[:2]

        # bbox 중앙 50% 영역
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        rw = max((x2 - x1) // 4, 2)
        rh = max((y2 - y1) // 4, 2)

        roi_x1 = max(0, cx - rw)
        roi_x2 = min(w, cx + rw)
        roi_y1 = max(0, cy - rh)
        roi_y2 = min(h, cy + rh)

        roi = depth_arr[roi_y1:roi_y2, roi_x1:roi_x2]

        # NaN, inf, 0 제거
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if len(valid) == 0:
            return float('inf')

        # ZED depth는 meters 단위
        return float(np.median(valid))

    # ===== 발행 =====

    def _publish_pause(self, pause: bool):
        """장애물 PAUSE/RESUME 발행"""
        msg = Bool()
        msg.data = pause
        self.pause_pub.publish(msg)

    def _publish_status(self, detections):
        """장애물 감지 상태 JSON 발행"""
        status = {
            'obstacle_detected': self.obstacle_detected,
            'num_detections': len(detections),
            'closest_distance_m': min(
                (d['distance_m'] for d in detections),
                default=float('inf')),
            'detections': detections[:5],  # 상위 5개만
        }
        msg = String()
        msg.data = json.dumps(status, default=str)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
