#!/usr/bin/env python3
"""
AprilTag 기반 실시간 Homography 캘리브레이션 테스트

작업영역 모서리 4곳에 AprilTag (16h5, 40mm) 부착 후,
태그 위치 → 로봇 좌표 매핑으로 pixel→mm 변환 정확도 테스트.

사용법:
  1. 태그 부착 후 이 스크립트 실행
  2. 각 태그의 로봇 좌표(mm) 입력 (최초 1회)
  3. 실시간 homography로 임의 pixel → mm 변환 테스트

  python3 test_apriltag_homography.py --camera right
  python3 test_apriltag_homography.py --camera left
"""

import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import time
import yaml
from dt_apriltags import Detector

WS_DIR = '/home/koceti/ros2_ws'
DATA_DIR = os.path.join(WS_DIR, 'data', 'calibration')

CAMERA_CONFIG = {
    'right': {
        'image_topic': '/zedxmini2/zed_node/left/image_rect_color',
        'camera_name': 'Right Camera (zedxmini2)',
    },
    'left': {
        'image_topic': '/zedxmini1/zed_node/left/image_rect_color',
        'camera_name': 'Left Camera (zedxmini1)',
    },
}

# AprilTag 로봇 좌표 설정 파일
TAG_CONFIG_FILE = os.path.join(DATA_DIR, 'apriltag_positions_{camera}.yaml')


class AprilTagHomography(Node):
    def __init__(self, camera_side='right'):
        super().__init__('apriltag_homography')
        self.bridge = CvBridge()
        self.camera_image = None
        self.got_image = False

        self.camera_side = camera_side
        cfg = CAMERA_CONFIG[camera_side]
        self.camera_name = cfg['camera_name']

        self.image_sub = self.create_subscription(
            Image, cfg['image_topic'], self.image_callback, 1)

        # AprilTag detector (16h5)
        self.detector = Detector(
            families='tag16h5',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
        )

        # 태그 ID → 로봇 좌표 (mm)
        self.tag_config_path = TAG_CONFIG_FILE.format(camera=camera_side)
        self.tag_positions = self._load_tag_config()

        # Homography 행렬
        self.H = None

        self.get_logger().info(f'[{self.camera_name}] AprilTag Homography 테스트')

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
        """카메라 이미지 캡처"""
        self.got_image = False
        timeout = time.monotonic() + 5.0
        while not self.got_image and time.monotonic() < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.camera_image

    def detect_tags(self, image=None, min_margin=15.0):
        """AprilTag 검출 (decision_margin 필터링)"""
        if image is None:
            image = self.capture_image()
        if image is None:
            print('  [ERROR] 이미지 없음')
            return []

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        tags = []
        filtered = 0
        for r in results:
            if r.decision_margin < min_margin:
                filtered += 1
                continue
            tag_id = r.tag_id
            cx, cy = r.center
            corners = r.corners  # 4 corners
            tags.append({
                'id': tag_id,
                'center': (float(cx), float(cy)),
                'corners': corners.tolist(),
                'decision_margin': r.decision_margin,
            })

        if filtered > 0:
            print(f'  (margin<{min_margin} 필터링: {filtered}개 제거)')

        return tags

    def compute_homography(self, tags):
        """검출된 태그 + 로봇 좌표로 homography 계산"""
        src_pts = []  # pixel 좌표
        dst_pts = []  # 로봇 좌표 (mm)

        for tag in tags:
            tag_id = str(tag['id'])
            if tag_id in self.tag_positions:
                pos = self.tag_positions[tag_id]
                src_pts.append(tag['center'])
                dst_pts.append((pos['x_mm'], pos['y_mm']))

        if len(src_pts) < 4:
            print(f'  Homography 불가: {len(src_pts)}개 매칭 (최소 4개)')
            if len(src_pts) >= 2:
                return self._compute_affine(src_pts, dst_pts)
            return None

        src = np.array(src_pts, dtype=np.float32)
        dst = np.array(dst_pts, dtype=np.float32)

        self.H, status = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if self.H is not None:
            print(f'  Homography 계산 완료 ({len(src_pts)}점)')
        return self.H

    def _compute_affine(self, src_pts, dst_pts):
        """2~3점일 때 affine/linear 변환"""
        n = len(src_pts)
        src = np.array(src_pts, dtype=np.float64)
        dst = np.array(dst_pts, dtype=np.float64)

        if n == 2:
            # 2점: 스케일 + 오프셋 (X, Y 각각 선형)
            A = np.column_stack([src, np.ones(n)])
            self.x_linear = np.linalg.lstsq(A, dst[:, 0], rcond=None)[0]
            self.y_linear = np.linalg.lstsq(A, dst[:, 1], rcond=None)[0]
            print(f'  2점 선형 보정 계산 완료')
            self.H = 'linear'
            return self.H
        elif n == 3:
            # 3점: affine
            src_3 = np.array(src_pts, dtype=np.float32)
            dst_3 = np.array(dst_pts, dtype=np.float32)
            self.affine_M = cv2.getAffineTransform(src_3, dst_3)
            print(f'  Affine 변환 계산 완료 (3점)')
            self.H = 'affine'
            return self.H

    def pixel_to_mm(self, pixel_u, pixel_v):
        """pixel → mm 변환"""
        if self.H is None:
            return None, None

        if isinstance(self.H, str) and self.H == 'linear':
            feat = np.array([pixel_u, pixel_v, 1.0])
            x_mm = float(feat @ self.x_linear)
            y_mm = float(feat @ self.y_linear)
            return x_mm, y_mm

        if isinstance(self.H, str) and self.H == 'affine':
            pt = np.array([[[pixel_u, pixel_v]]], dtype=np.float32)
            result = cv2.transform(pt, self.affine_M)
            return float(result[0][0][0]), float(result[0][0][1])

        # Full homography
        pt = np.array([[[pixel_u, pixel_v]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, self.H)
        return float(result[0][0][0]), float(result[0][0][1])

    def visualize(self, image, tags):
        """태그 검출 결과 시각화"""
        img = image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX

        for tag in tags:
            tag_id = str(tag['id'])
            cx, cy = int(tag['center'][0]), int(tag['center'][1])
            corners = np.array(tag['corners'], dtype=np.int32)

            # 태그 외곽선
            cv2.polylines(img, [corners], True, (0, 255, 0), 2)
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)

            # 태그 ID
            label = f'ID:{tag_id}'
            if tag_id in self.tag_positions:
                pos = self.tag_positions[tag_id]
                label += f' ({pos["x_mm"]:.0f},{pos["y_mm"]:.0f})mm'
            cv2.putText(img, label, (cx + 10, cy - 10), font, 0.5, (0, 255, 0), 1)

            # pixel 좌표
            cv2.putText(img, f'px:({cx},{cy})', (cx + 10, cy + 10),
                        font, 0.4, (255, 255, 255), 1)

        # Homography 상태
        if self.H is not None:
            mode = 'Homography' if not isinstance(self.H, str) else self.H
            cv2.putText(img, f'{mode} ACTIVE | {len(tags)} tags',
                        (10, 30), font, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(img, f'NO TRANSFORM | {len(tags)} tags',
                        (10, 30), font, 0.7, (0, 0, 255), 2)

        return img

    def setup_tag_positions(self, tags):
        """검출된 태그에 로봇 좌표 할당 (인터랙티브)"""
        print('\n  === 태그 로봇 좌표 설정 ===')
        print('  검출된 태그에 실제 로봇 좌표(mm)를 입력하세요.')
        print('  이미 설정된 태그는 Enter로 스킵합니다.')
        print()

        for tag in tags:
            tag_id = str(tag['id'])
            cx, cy = tag['center']
            existing = self.tag_positions.get(tag_id)

            if existing:
                print(f'  TAG {tag_id} pixel=({cx:.0f},{cy:.0f}) '
                      f'→ 기존: ({existing["x_mm"]:.1f}, {existing["y_mm"]:.1f})mm')
                raw = input(f'    변경? (Enter=유지, "X Y"=변경): ').strip()
                if not raw:
                    continue
            else:
                raw = input(f'  TAG {tag_id} pixel=({cx:.0f},{cy:.0f}) '
                            f'→ 로봇 좌표 (X Y mm): ').strip()

            if raw == 's' or raw == 'q':
                continue

            try:
                parts = raw.replace(',', ' ').split()
                x_mm = float(parts[0])
                y_mm = float(parts[1])
                self.tag_positions[tag_id] = {
                    'x_mm': x_mm,
                    'y_mm': y_mm,
                    'pixel_u': cx,
                    'pixel_v': cy,
                }
                print(f'    설정: TAG {tag_id} → ({x_mm}, {y_mm})mm')
            except (ValueError, IndexError):
                print('    스킵')

        self._save_tag_config()
        print(f'  저장: {self.tag_config_path}')
        print(f'  설정된 태그: {len(self.tag_positions)}개')

    def interactive_test(self, image, tags):
        """마우스 클릭으로 pixel→mm 변환 테스트"""
        if self.H is None:
            print('  변환 행렬 없음')
            return

        click_pts = []

        def mouse_cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                x_mm, y_mm = self.pixel_to_mm(x, y)
                if x_mm is not None:
                    click_pts.append((x, y, x_mm, y_mm))
                    print(f'    pixel=({x},{y}) → X={x_mm:.1f}mm, Y={y_mm:.1f}mm')

        img = self.visualize(image, tags)
        # 클릭 포인트 표시
        win_name = 'AprilTag Homography Test'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win_name, mouse_cb)

        print('\n  마우스 클릭으로 pixel→mm 변환 테스트')
        print('  ESC/Q 종료')

        while True:
            disp = img.copy()
            font = cv2.FONT_HERSHEY_SIMPLEX
            for px, py, mx, my in click_pts:
                cv2.circle(disp, (px, py), 6, (255, 0, 255), -1)
                cv2.putText(disp, f'({mx:.1f},{my:.1f})',
                            (px + 8, py - 8), font, 0.4, (255, 0, 255), 1)

            cv2.imshow(win_name, disp)
            key = cv2.waitKey(50) & 0xFF
            if key == 27 or key == ord('q'):
                break

        cv2.destroyWindow(win_name)

    def accuracy_test(self, tags):
        """태그 중심점으로 변환 정확도 계산 (leave-one-out)"""
        matched = []
        for tag in tags:
            tag_id = str(tag['id'])
            if tag_id in self.tag_positions:
                pos = self.tag_positions[tag_id]
                matched.append({
                    'id': tag_id,
                    'pixel': tag['center'],
                    'actual': (pos['x_mm'], pos['y_mm']),
                })

        if len(matched) < 3:
            print('  정확도 테스트: 최소 3개 매칭 필요')
            return

        print('\n  === Leave-One-Out 정확도 테스트 ===')
        errors = []
        for i, left_out in enumerate(matched):
            # 나머지로 homography 계산
            src = np.array([m['pixel'] for j, m in enumerate(matched) if j != i],
                           dtype=np.float32)
            dst = np.array([m['actual'] for j, m in enumerate(matched) if j != i],
                           dtype=np.float32)

            if len(src) >= 4:
                H, _ = cv2.findHomography(src, dst, 0)
            elif len(src) == 3:
                M = cv2.getAffineTransform(src, dst)
                H = np.vstack([M, [0, 0, 1]])
            else:
                continue

            # left-out 포인트 변환
            pt = np.array([[[left_out['pixel'][0], left_out['pixel'][1]]]],
                          dtype=np.float32)
            pred = cv2.perspectiveTransform(pt, H)
            pred_x, pred_y = float(pred[0][0][0]), float(pred[0][0][1])
            actual_x, actual_y = left_out['actual']

            err_x = pred_x - actual_x
            err_y = pred_y - actual_y
            err = (err_x**2 + err_y**2)**0.5
            errors.append(err)

            print(f'  TAG {left_out["id"]}: '
                  f'actual=({actual_x:.1f},{actual_y:.1f}) '
                  f'pred=({pred_x:.1f},{pred_y:.1f}) '
                  f'err={err:.1f}mm (dX={err_x:+.1f}, dY={err_y:+.1f})')

        if errors:
            print(f'\n  LOO 평균오차: {np.mean(errors):.1f}mm, '
                  f'최대: {np.max(errors):.1f}mm')


def main():
    parser = argparse.ArgumentParser(description='AprilTag Homography 캘리브레이션 테스트')
    parser.add_argument('--camera', choices=['left', 'right'], default='right')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = AprilTagHomography(camera_side=args.camera)

    print('=' * 60)
    print(f' AprilTag Homography Test [{node.camera_name}]')
    print(f' Tag family: 16h5, size: 40mm')
    print(f' 설정 파일: {node.tag_config_path}')
    print(f' 설정된 태그: {len(node.tag_positions)}개')
    print('=' * 60)
    print()
    print(' 명령어:')
    print('   Enter  = 이미지 캡처 + 태그 검출')
    print('   setup  = 태그 로봇 좌표 설정')
    print('   calc   = homography 계산')
    print('   test   = 마우스 클릭 변환 테스트')
    print('   acc    = leave-one-out 정확도')
    print('   q      = 종료')
    print('=' * 60)

    last_tags = []
    last_image = None

    try:
        while True:
            cmd = input('\n[Enter]=검출, [q]=종료: ').strip().lower()

            if cmd == 'q':
                break

            elif cmd == '' or cmd == 'detect':
                image = node.capture_image()
                if image is None:
                    print('  이미지 캡처 실패')
                    continue
                last_image = image
                last_tags = node.detect_tags(image)
                print(f'\n  검출: {len(last_tags)}개 태그')
                for tag in last_tags:
                    tag_id = str(tag['id'])
                    cx, cy = tag['center']
                    margin = tag['decision_margin']
                    status = '(설정됨)' if tag_id in node.tag_positions else '(미설정)'
                    print(f'    TAG {tag_id}: pixel=({cx:.1f},{cy:.1f}) '
                          f'margin={margin:.1f} {status}')

                # 시각화 저장
                viz = node.visualize(image, last_tags)
                viz_path = os.path.join(DATA_DIR,
                                        f'apriltag_detect_{args.camera}.png')
                cv2.imwrite(viz_path, viz)
                print(f'  이미지 저장: {viz_path}')

            elif cmd == 'setup':
                if not last_tags:
                    print('  먼저 Enter로 태그 검출하세요')
                    continue
                node.setup_tag_positions(last_tags)

            elif cmd == 'calc':
                if not last_tags:
                    print('  먼저 Enter로 태그 검출하세요')
                    continue
                node.compute_homography(last_tags)

            elif cmd == 'test':
                if last_image is None:
                    print('  먼저 Enter로 이미지 캡처하세요')
                    continue
                node.interactive_test(last_image, last_tags)

            elif cmd == 'acc':
                if not last_tags:
                    print('  먼저 Enter로 태그 검출하세요')
                    continue
                node.accuracy_test(last_tags)

            else:
                print('  Enter/setup/calc/test/acc/q')

    except KeyboardInterrupt:
        print('\n종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
