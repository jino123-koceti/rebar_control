#!/usr/bin/env python3
"""
철근 라인 검출 테스트 (OpenCV 기반)

zed_front 카메라 이미지에서 가로/세로 배근 철근을 구분 검출합니다.
- 가로 철근 (horizontal): 주행 방향에 수직 → 카운팅으로 종방향 거리 보정 가능
- 세로 철근 (vertical):   주행 방향에 평행 → 추종으로 횡방향 정렬 가능

사용법:
    # 실시간 카메라 (ROS2 토픽)
    python3 detect_rebar_lines.py

    # 저장된 이미지 파일로 테스트
    python3 detect_rebar_lines.py --image /path/to/image.png

    # 파라미터 조정
    python3 detect_rebar_lines.py --canny-low 30 --canny-high 90 --hough-thresh 80

키 조작:
    q: 종료
    s: 현재 프레임 저장
    1: 원본 + 검출 결과 (기본)
    2: Canny edge 보기
    3: 가로 철근만
    4: 세로 철근만
    +/-: Hough threshold 조정
"""

import cv2
import numpy as np
import argparse
import json
import os
import time
from datetime import datetime


class RebarLineDetector:
    """OpenCV 기반 철근 라인 검출기"""

    def __init__(self, canny_low=30, canny_high=90, hough_threshold=80,
                 min_line_length=80, max_line_gap=15,
                 angle_tolerance=20.0):
        # Canny edge 파라미터
        self.canny_low = canny_low
        self.canny_high = canny_high

        # HoughLinesP 파라미터
        self.hough_threshold = hough_threshold
        self.min_line_length = min_line_length
        self.max_line_gap = max_line_gap

        # 가로/세로 판별 각도 허용 범위 (도)
        # 0° ± tolerance = 가로, 90° ± tolerance = 세로
        self.angle_tolerance = angle_tolerance

        # 전처리 파라미터
        self.blur_ksize = 5
        self.clahe_clip = 2.0
        self.clahe_grid = (8, 8)

    def preprocess(self, frame):
        """전처리: 그레이스케일 → CLAHE → 블러 → Canny"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # CLAHE로 대비 향상 (야외 조명 변화 대응)
        clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip,
            tileGridSize=self.clahe_grid
        )
        enhanced = clahe.apply(gray)

        # 가우시안 블러로 노이즈 제거
        blurred = cv2.GaussianBlur(enhanced, (self.blur_ksize, self.blur_ksize), 0)

        # Canny edge
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)

        # 모폴로지 연산으로 끊긴 엣지 연결
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)

        return gray, enhanced, edges

    def detect_lines(self, edges):
        """HoughLinesP로 직선 검출 후 가로/세로 분류"""
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )

        horizontal = []  # 가로 철근
        vertical = []    # 세로 철근
        other = []       # 기타 (사선)

        if lines is None:
            return horizontal, vertical, other

        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = self._line_angle(x1, y1, x2, y2)
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            line_info = {
                'pts': (x1, y1, x2, y2),
                'angle': angle,
                'length': length,
                'center': ((x1 + x2) // 2, (y1 + y2) // 2),
            }

            if abs(angle) <= self.angle_tolerance:
                horizontal.append(line_info)
            elif abs(angle - 90) <= self.angle_tolerance or abs(angle + 90) <= self.angle_tolerance:
                vertical.append(line_info)
            else:
                other.append(line_info)

        # 중복 라인 병합
        horizontal = self._merge_nearby_lines(horizontal, axis='horizontal')
        vertical = self._merge_nearby_lines(vertical, axis='vertical')

        return horizontal, vertical, other

    def _line_angle(self, x1, y1, x2, y2):
        """직선 각도 계산 (-90° ~ +90°), 0°=수평, ±90°=수직"""
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0:
            return 90.0
        return np.degrees(np.arctan2(dy, dx))

    def _merge_nearby_lines(self, lines, axis='horizontal', dist_thresh=20):
        """가까운 라인들을 병합하여 중복 제거

        같은 철근에서 여러 엣지가 검출되는 것을 방지
        """
        if len(lines) <= 1:
            return lines

        # 정렬: 가로 → y좌표순, 세로 → x좌표순
        if axis == 'horizontal':
            lines.sort(key=lambda l: l['center'][1])
        else:
            lines.sort(key=lambda l: l['center'][0])

        merged = [lines[0]]
        for line in lines[1:]:
            prev = merged[-1]
            if axis == 'horizontal':
                dist = abs(line['center'][1] - prev['center'][1])
            else:
                dist = abs(line['center'][0] - prev['center'][0])

            if dist < dist_thresh:
                # 더 긴 라인으로 대체
                if line['length'] > prev['length']:
                    merged[-1] = line
            else:
                merged.append(line)

        return merged

    def detect(self, frame):
        """전체 파이프라인: 전처리 → 검출 → 분류"""
        gray, enhanced, edges = self.preprocess(frame)
        horizontal, vertical, other = self.detect_lines(edges)

        return {
            'horizontal': horizontal,
            'vertical': vertical,
            'other': other,
            'edges': edges,
            'enhanced': enhanced,
        }

    def draw_result(self, frame, result, show_mode='all'):
        """검출 결과를 프레임에 시각화"""
        vis = frame.copy()
        h, w = vis.shape[:2]

        horizontal = result['horizontal']
        vertical = result['vertical']

        # 가로 철근 (파란색)
        if show_mode in ('all', 'horizontal'):
            for line in horizontal:
                x1, y1, x2, y2 = line['pts']
                cv2.line(vis, (x1, y1), (x2, y2), (255, 100, 0), 2)
                cx, cy = line['center']
                cv2.circle(vis, (cx, cy), 4, (255, 100, 0), -1)

        # 세로 철근 (녹색)
        if show_mode in ('all', 'vertical'):
            for line in vertical:
                x1, y1, x2, y2 = line['pts']
                cv2.line(vis, (x1, y1), (x2, y2), (0, 255, 100), 2)
                cx, cy = line['center']
                cv2.circle(vis, (cx, cy), 4, (0, 255, 100), -1)

        # 정보 패널
        panel_h = 120
        overlay = vis[:panel_h, :].copy()
        cv2.rectangle(vis, (0, 0), (w, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, vis[:panel_h, :], 0.7, 0, vis[:panel_h, :])

        y_text = 25
        cv2.putText(vis, f"Horizontal (blue): {len(horizontal)}",
                    (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)
        y_text += 30
        cv2.putText(vis, f"Vertical (green): {len(vertical)}",
                    (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 100), 2)
        y_text += 30
        cv2.putText(vis, f"Canny: {self.canny_low}/{self.canny_high}  "
                    f"Hough: {self.hough_threshold}  "
                    f"MinLen: {self.min_line_length}",
                    (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_text += 25
        cv2.putText(vis, "Keys: q=quit s=save 1=all 2=edge 3=horiz 4=vert +/-=threshold",
                    (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        return vis


def run_with_ros(detector, args):
    """ROS2 토픽에서 실시간 이미지 수신하여 검출"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    class RebarLineTestNode(Node):
        def __init__(self):
            super().__init__('rebar_line_detect_test')
            self.bridge = CvBridge()
            self.detector = detector
            self.latest_frame = None
            self.show_mode = 'all'
            self.auto_saved = False  # 첫 프레임 자동 저장 플래그
            self.frame_count = 0

            topic = args.topic
            self.get_logger().info(f"Subscribing to: {topic}")

            self.sub = self.create_subscription(
                Image, topic, self.image_callback, 1
            )
            # 표시용 타이머 (30Hz)
            self.timer = self.create_timer(1.0 / 30, self.display_loop)
            self.get_logger().info("Waiting for image...")
            self.get_logger().info(f"Auto-save dir: {get_save_dir()}")

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
            result = self.detector.detect(frame)

            # 첫 프레임 자동 저장 (3번째 프레임 — 카메라 안정화 대기)
            if not self.auto_saved and self.frame_count >= 3:
                self.auto_saved = True
                save_frame(frame, result, self.detector, tag='auto_first')
                self.get_logger().info(
                    f"Auto-saved first frame: "
                    f"H={len(result['horizontal'])}, V={len(result['vertical'])}"
                )

            if self.show_mode == 'edge':
                vis = cv2.cvtColor(result['edges'], cv2.COLOR_GRAY2BGR)
            else:
                vis = self.detector.draw_result(frame, result, self.show_mode)

            cv2.imshow('Rebar Line Detection', vis)
            key = cv2.waitKey(1) & 0xFF
            self.handle_key(key, frame, result)

        def handle_key(self, key, frame, result):
            if key == ord('q'):
                self.get_logger().info("Quit requested")
                rclpy.shutdown()
            elif key == ord('s'):
                save_frame(frame, result, self.detector)
            elif key == ord('1'):
                self.show_mode = 'all'
            elif key == ord('2'):
                self.show_mode = 'edge'
            elif key == ord('3'):
                self.show_mode = 'horizontal'
            elif key == ord('4'):
                self.show_mode = 'vertical'
            elif key == ord('+') or key == ord('='):
                self.detector.hough_threshold = min(200, self.detector.hough_threshold + 5)
                self.get_logger().info(f"Hough threshold: {self.detector.hough_threshold}")
            elif key == ord('-'):
                self.detector.hough_threshold = max(10, self.detector.hough_threshold - 5)
                self.get_logger().info(f"Hough threshold: {self.detector.hough_threshold}")

    rclpy.init()
    node = RebarLineTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


def run_with_image(detector, image_path):
    """저장된 이미지 파일로 테스트"""
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Cannot read image: {image_path}")
        return

    print(f"Image loaded: {frame.shape[1]}x{frame.shape[0]}")
    show_mode = 'all'

    # 이미지 모드: 즉시 자동 저장
    result = detector.detect(frame)
    save_frame(frame, result, detector, tag='image_test')
    print(f"Auto-saved: H={len(result['horizontal'])}, V={len(result['vertical'])}")

    while True:
        result = detector.detect(frame)

        if show_mode == 'edge':
            vis = cv2.cvtColor(result['edges'], cv2.COLOR_GRAY2BGR)
        else:
            vis = detector.draw_result(frame, result, show_mode)

        cv2.imshow('Rebar Line Detection', vis)
        key = cv2.waitKey(0) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            save_frame(frame, result, detector)
        elif key == ord('1'):
            show_mode = 'all'
        elif key == ord('2'):
            show_mode = 'edge'
        elif key == ord('3'):
            show_mode = 'horizontal'
        elif key == ord('4'):
            show_mode = 'vertical'
        elif key == ord('+') or key == ord('='):
            detector.hough_threshold = min(200, detector.hough_threshold + 5)
            print(f"Hough threshold: {detector.hough_threshold}")
        elif key == ord('-'):
            detector.hough_threshold = max(10, detector.hough_threshold - 5)
            print(f"Hough threshold: {detector.hough_threshold}")

    cv2.destroyAllWindows()

    # 최종 결과 출력
    print(f"\n=== Detection Result ===")
    print(f"Horizontal lines: {len(result['horizontal'])}")
    for i, line in enumerate(result['horizontal']):
        print(f"  [{i}] y={line['center'][1]:4d}  angle={line['angle']:+5.1f}°  len={line['length']:.0f}px")
    print(f"Vertical lines: {len(result['vertical'])}")
    for i, line in enumerate(result['vertical']):
        print(f"  [{i}] x={line['center'][0]:4d}  angle={line['angle']:+5.1f}°  len={line['length']:.0f}px")


def get_save_dir():
    """저장 디렉토리 경로 반환"""
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', 'data', 'vision_test', 'rebar_lines'
    )


def save_frame(frame, result, detector, tag='manual'):
    """현재 프레임과 검출 결과 저장

    저장 항목:
    - 원본 이미지
    - 검출 결과 오버레이 이미지
    - Canny edge 이미지
    - JSON 로그 (검출 파라미터 + 라인 정보)
    """
    save_dir = get_save_dir()
    os.makedirs(save_dir, exist_ok=True)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = f'{timestamp}_{tag}'

    # 원본 저장
    orig_path = os.path.join(save_dir, f'{prefix}_original.png')
    cv2.imwrite(orig_path, frame)

    # 검출 결과 저장
    vis = detector.draw_result(frame, result, 'all')
    vis_path = os.path.join(save_dir, f'{prefix}_detected.png')
    cv2.imwrite(vis_path, vis)

    # 엣지 저장
    edge_path = os.path.join(save_dir, f'{prefix}_edges.png')
    cv2.imwrite(edge_path, result['edges'])

    # JSON 로그 저장 (파라미터 + 검출 결과 요약)
    def line_to_dict(line):
        return {
            'pts': [int(v) for v in line['pts']],
            'angle': round(float(line['angle']), 2),
            'length': round(float(line['length']), 1),
            'center': [int(v) for v in line['center']],
        }

    log = {
        'timestamp': timestamp,
        'tag': tag,
        'image_size': [frame.shape[1], frame.shape[0]],
        'params': {
            'canny_low': detector.canny_low,
            'canny_high': detector.canny_high,
            'hough_threshold': detector.hough_threshold,
            'min_line_length': detector.min_line_length,
            'max_line_gap': detector.max_line_gap,
            'angle_tolerance': detector.angle_tolerance,
        },
        'result': {
            'horizontal_count': len(result['horizontal']),
            'vertical_count': len(result['vertical']),
            'other_count': len(result['other']),
            'horizontal': [line_to_dict(l) for l in result['horizontal']],
            'vertical': [line_to_dict(l) for l in result['vertical']],
        },
    }
    log_path = os.path.join(save_dir, f'{prefix}_result.json')
    with open(log_path, 'w') as f:
        json.dump(log, f, indent=2, ensure_ascii=False)

    print(f"Saved: {save_dir}/{prefix}_*.png + .json")


def main():
    parser = argparse.ArgumentParser(description='철근 라인 검출 테스트')
    parser.add_argument('--image', '-i', type=str, default=None,
                        help='테스트할 이미지 파일 경로 (없으면 ROS2 토픽 사용)')
    parser.add_argument('--topic', '-t', type=str,
                        default='/zed_back/zed_node/left/image_rect_color',
                        help='ROS2 이미지 토픽 (default: zed_back, 전진용)')
    parser.add_argument('--canny-low', type=int, default=30,
                        help='Canny edge 하한 (default: 30)')
    parser.add_argument('--canny-high', type=int, default=90,
                        help='Canny edge 상한 (default: 90)')
    parser.add_argument('--hough-thresh', type=int, default=80,
                        help='HoughLinesP threshold (default: 80)')
    parser.add_argument('--min-line-length', type=int, default=80,
                        help='최소 라인 길이 (default: 80px)')
    parser.add_argument('--max-line-gap', type=int, default=15,
                        help='라인 내 최대 갭 (default: 15px)')
    parser.add_argument('--angle-tolerance', type=float, default=20.0,
                        help='가로/세로 판별 각도 허용범위 (default: 20도)')

    args = parser.parse_args()

    detector = RebarLineDetector(
        canny_low=args.canny_low,
        canny_high=args.canny_high,
        hough_threshold=args.hough_thresh,
        min_line_length=args.min_line_length,
        max_line_gap=args.max_line_gap,
        angle_tolerance=args.angle_tolerance,
    )

    if args.image:
        run_with_image(detector, args.image)
    else:
        run_with_ros(detector, args)


if __name__ == '__main__':
    main()
