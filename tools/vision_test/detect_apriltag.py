#!/usr/bin/env python3
"""
AprilTag 검출 및 Pose 추정 테스트

zed_front 카메라에서 AprilTag를 검출하고 위치/자세를 추정합니다.
- 태그 ID, 거리, 위치(X,Y,Z), 회전(roll,pitch,yaw) 표시
- 첫 검출 시 자동 저장 (이미지 + JSON)

사용법:
    # 실시간 카메라 (ROS2 토픽)
    python3 detect_apriltag.py

    # 다른 카메라
    python3 detect_apriltag.py --topic /zed_back/zed_node/left/image_rect_color

    # 저장된 이미지로 테스트
    python3 detect_apriltag.py --image /path/to/image.png

    # 태그 크기 변경 (기본 0.19m = 19cm)
    python3 detect_apriltag.py --tag-size 0.15

    # 태그 family 변경
    python3 detect_apriltag.py --family 25h9

키 조작:
    q: 종료
    s: 현재 프레임 저장
"""

import cv2
import numpy as np
import argparse
import json
import os
import time
from datetime import datetime


# AprilTag family 매핑
FAMILY_MAP = {
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '36h10': cv2.aruco.DICT_APRILTAG_36h10,
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}

# ZED X 기본 카메라 내부 파라미터 (HD720 기준, 실제 값은 camera_info에서 가져와야 함)
# 추후 /camera_info 토픽에서 자동 취득하도록 개선
DEFAULT_CAMERA_MATRIX = np.array([
    [530.0, 0, 640.0],
    [0, 530.0, 360.0],
    [0, 0, 1.0]
], dtype=np.float64)
DEFAULT_DIST_COEFFS = np.zeros(5, dtype=np.float64)


class AprilTagDetector:
    """OpenCV aruco 기반 AprilTag 검출 + Pose 추정"""

    def __init__(self, family='36h11', tag_size=0.19,
                 camera_matrix=None, dist_coeffs=None):
        self.tag_size = tag_size  # 태그 크기 (m)

        # ArUco 딕셔너리
        if family not in FAMILY_MAP:
            raise ValueError(f"Unsupported family: {family}. Use: {list(FAMILY_MAP.keys())}")
        self.dictionary = cv2.aruco.getPredefinedDictionary(FAMILY_MAP[family])
        self.family = family

        # 검출 파라미터
        self.params = cv2.aruco.DetectorParameters()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

        # 카메라 파라미터
        self.camera_matrix = camera_matrix if camera_matrix is not None else DEFAULT_CAMERA_MATRIX
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else DEFAULT_DIST_COEFFS
        self.camera_info_received = camera_matrix is not None

    def update_camera_info(self, camera_matrix, dist_coeffs):
        """camera_info 토픽에서 카메라 파라미터 업데이트"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.camera_info_received = True

    def detect(self, frame):
        """태그 검출 + pose 추정"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 검출
        corners, ids, rejected = self.detector.detectMarkers(gray)

        results = []
        if ids is None:
            return results

        # 각 태그에 대해 pose 추정
        for i, tag_id in enumerate(ids.flatten()):
            corner = corners[i]

            # solvePnP로 pose 추정
            obj_points = np.array([
                [-self.tag_size / 2,  self.tag_size / 2, 0],
                [ self.tag_size / 2,  self.tag_size / 2, 0],
                [ self.tag_size / 2, -self.tag_size / 2, 0],
                [-self.tag_size / 2, -self.tag_size / 2, 0],
            ], dtype=np.float64)

            success, rvec, tvec = cv2.solvePnP(
                obj_points, corner[0], self.camera_matrix, self.dist_coeffs
            )

            if not success:
                continue

            # 회전 벡터 → 회전 행렬 → 오일러 각도
            rmat, _ = cv2.Rodrigues(rvec)
            roll, pitch, yaw = self._rotation_matrix_to_euler(rmat)

            # 거리 계산
            distance = np.linalg.norm(tvec)

            results.append({
                'id': int(tag_id),
                'corners': corner[0].tolist(),
                'center': np.mean(corner[0], axis=0).tolist(),
                'tvec': tvec.flatten().tolist(),  # [x, y, z] in meters
                'rvec': rvec.flatten().tolist(),
                'distance': float(distance),
                'roll': float(roll),
                'pitch': float(pitch),
                'yaw': float(yaw),
            })

        # ID 순 정렬
        results.sort(key=lambda r: r['id'])
        return results

    def _rotation_matrix_to_euler(self, R):
        """회전 행렬 → 오일러 각도 (도)"""
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        if sy > 1e-6:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def draw_result(self, frame, results):
        """검출 결과 시각화"""
        vis = frame.copy()
        h, w = vis.shape[:2]

        for r in results:
            corners = np.array(r['corners'], dtype=np.int32)

            # 태그 윤곽선
            cv2.polylines(vis, [corners], True, (0, 255, 0), 2)

            # 중심점
            cx, cy = int(r['center'][0]), int(r['center'][1])
            cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

            # 축 그리기
            rvec = np.array(r['rvec'], dtype=np.float64).reshape(3, 1)
            tvec = np.array(r['tvec'], dtype=np.float64).reshape(3, 1)
            cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, self.tag_size * 0.5)

            # 정보 텍스트
            tx, ty, tz = r['tvec']
            info_lines = [
                f"ID:{r['id']}  dist:{r['distance']:.2f}m",
                f"X:{tx:.3f} Y:{ty:.3f} Z:{tz:.3f}",
                f"R:{r['roll']:.1f} P:{r['pitch']:.1f} Y:{r['yaw']:.1f}",
            ]

            text_x = cx + 15
            text_y = cy - 30
            for line in info_lines:
                cv2.putText(vis, line, (text_x, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # 검은 배경으로 가독성 향상
                cv2.putText(vis, line, (text_x, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                text_y += 20

        # 상단 정보 패널
        panel_h = 40
        overlay = vis[:panel_h, :].copy()
        cv2.rectangle(vis, (0, 0), (w, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, vis[:panel_h, :], 0.7, 0, vis[:panel_h, :])

        cam_status = "camera_info OK" if self.camera_info_received else "DEFAULT intrinsics"
        cv2.putText(vis, f"AprilTag {self.family} | size:{self.tag_size}m | "
                    f"detected:{len(results)} | {cam_status}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        return vis


def get_save_dir():
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', 'data', 'vision_test', 'apriltag'
    )


def save_frame(frame, results, detector, tag='manual'):
    """프레임과 검출 결과 저장"""
    save_dir = get_save_dir()
    os.makedirs(save_dir, exist_ok=True)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = f'{timestamp}_{tag}'

    # 원본
    cv2.imwrite(os.path.join(save_dir, f'{prefix}_original.png'), frame)

    # 검출 결과 오버레이
    vis = detector.draw_result(frame, results)
    cv2.imwrite(os.path.join(save_dir, f'{prefix}_detected.png'), vis)

    # JSON
    log = {
        'timestamp': timestamp,
        'tag': tag,
        'family': detector.family,
        'tag_size_m': detector.tag_size,
        'camera_info_from_topic': detector.camera_info_received,
        'detections': results,
    }
    with open(os.path.join(save_dir, f'{prefix}_result.json'), 'w') as f:
        json.dump(log, f, indent=2, ensure_ascii=False)

    print(f"Saved: {save_dir}/{prefix}_*")


def run_with_ros(detector, args):
    """ROS2 토픽에서 실시간 검출"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge

    class AprilTagTestNode(Node):
        def __init__(self):
            super().__init__('apriltag_detect_test')
            self.bridge = CvBridge()
            self.detector = detector
            self.latest_frame = None
            self.auto_saved = False
            self.frame_count = 0

            topic = args.topic
            self.get_logger().info(f"Subscribing to: {topic}")
            self.get_logger().info(f"Tag family: {args.family}, size: {args.tag_size}m")

            self.sub = self.create_subscription(
                Image, topic, self.image_callback, 1)

            # camera_info 구독 (토픽 이름 추론)
            info_topic = topic.rsplit('/', 1)[0] + '/camera_info'
            self.info_sub = self.create_subscription(
                CameraInfo, info_topic, self.camera_info_callback, 1)
            self.get_logger().info(f"Camera info: {info_topic}")

            self.timer = self.create_timer(1.0 / 30, self.display_loop)
            self.get_logger().info("Waiting for image...")

        def camera_info_callback(self, msg):
            """camera_info에서 실제 카메라 파라미터 취득"""
            if not self.detector.camera_info_received:
                K = np.array(msg.k).reshape(3, 3)
                D = np.array(msg.d)
                self.detector.update_camera_info(K, D)
                self.get_logger().info(
                    f"Camera intrinsics received: fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
                    f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

        def image_callback(self, msg):
            try:
                self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f"cv_bridge error: {e}")

        def display_loop(self):
            if self.latest_frame is None:
                return

            frame = self.latest_frame
            self.frame_count += 1
            results = self.detector.detect(frame)

            # 첫 검출 시 자동 저장
            if not self.auto_saved and len(results) > 0 and self.frame_count >= 3:
                self.auto_saved = True
                save_frame(frame, results, self.detector, tag='auto_first')
                self.get_logger().info(
                    f"Auto-saved: {len(results)} tag(s) detected")
                for r in results:
                    self.get_logger().info(
                        f"  ID:{r['id']} dist:{r['distance']:.3f}m "
                        f"X:{r['tvec'][0]:.3f} Y:{r['tvec'][1]:.3f} Z:{r['tvec'][2]:.3f}")

            vis = self.detector.draw_result(frame, results)
            cv2.imshow('AprilTag Detection', vis)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                rclpy.shutdown()
            elif key == ord('s'):
                save_frame(frame, results, self.detector, tag='manual')

    rclpy.init()
    node = AprilTagTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


def run_with_image(detector, image_path):
    """이미지 파일로 테스트"""
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Cannot read image: {image_path}")
        return

    print(f"Image: {frame.shape[1]}x{frame.shape[0]}")
    results = detector.detect(frame)

    print(f"\nDetected: {len(results)} tag(s)")
    for r in results:
        print(f"  ID:{r['id']}  dist:{r['distance']:.3f}m  "
              f"X:{r['tvec'][0]:.3f} Y:{r['tvec'][1]:.3f} Z:{r['tvec'][2]:.3f}  "
              f"R:{r['roll']:.1f} P:{r['pitch']:.1f} Y:{r['yaw']:.1f}")

    save_frame(frame, results, detector, tag='image_test')

    vis = detector.draw_result(frame, results)
    cv2.imshow('AprilTag Detection', vis)
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_frame(frame, results, detector, tag='manual')
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='AprilTag 검출 테스트')
    parser.add_argument('--image', '-i', type=str, default=None,
                        help='테스트 이미지 (없으면 ROS2 토픽)')
    parser.add_argument('--topic', '-t', type=str,
                        default='/zed_back/zed_node/left/image_rect_color',
                        help='ROS2 이미지 토픽 (default: zed_back, 후진용)')
    parser.add_argument('--tag-size', type=float, default=0.19,
                        help='태그 전체 크기 m (default: 0.19)')
    parser.add_argument('--family', type=str, default='25h9',
                        choices=list(FAMILY_MAP.keys()),
                        help='AprilTag family (default: 25h9)')

    args = parser.parse_args()

    detector = AprilTagDetector(
        family=args.family,
        tag_size=args.tag_size,
    )

    if args.image:
        run_with_image(detector, args.image)
    else:
        run_with_ros(detector, args)


if __name__ == '__main__':
    main()
