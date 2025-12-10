#!/usr/bin/env python3
"""
듀얼 ZED X mini 카메라 데이터 수집 노드

기능:
- 좌우 카메라 RGB + Depth 동기화 저장
- 메타데이터 기록 (철근 간격, 패턴, 카메라 설정)
- 수동/자동 캡처 모드

Author: test
Date: 2025-12-03
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
import os
import json
from datetime import datetime
from pathlib import Path


class DualCameraRecorderNode(Node):
    """듀얼 ZED X mini 카메라 데이터 수집 노드"""
    
    def __init__(self):
        super().__init__('dual_camera_recorder')
        
        # ============================================
        # 파라미터 선언
        # ============================================
        self.declare_parameter('save_path', '/home/test/dataset')
        self.declare_parameter('save_mode', 'manual')  # manual, auto, continuous
        self.declare_parameter('auto_interval', 2.0)   # seconds (auto mode)
        self.declare_parameter('fps_limit', 10.0)      # Hz (continuous mode)
        self.declare_parameter('rebar_spacing', 200)   # mm
        self.declare_parameter('rebar_pattern', 'orthogonal')  # orthogonal, diagonal
        self.declare_parameter('session_name', '')     # optional custom name
        self.declare_parameter('save_depth', True)     # 깊이맵 저장 여부
        
        # 파라미터 가져오기
        self.save_path = self.get_parameter('save_path').value
        self.save_mode = self.get_parameter('save_mode').value
        self.auto_interval = self.get_parameter('auto_interval').value
        self.fps_limit = self.get_parameter('fps_limit').value
        self.rebar_spacing = self.get_parameter('rebar_spacing').value
        self.rebar_pattern = self.get_parameter('rebar_pattern').value
        self.session_name = self.get_parameter('session_name').value
        self.save_depth = self.get_parameter('save_depth').value
        
        # ============================================
        # CV Bridge
        # ============================================
        self.bridge = CvBridge()
        
        # ============================================
        # 세션 정보
        # ============================================
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        if self.session_name:
            self.session_id = f"{self.session_name}_{timestamp}"
        else:
            self.session_id = f"rebar_{timestamp}"
        
        self.session_dir = os.path.join(self.save_path, self.session_id)
        self.frame_count = 0
        self.recording = False
        
        # ============================================
        # 디렉토리 생성
        # ============================================
        self.create_directories()
        
        # ============================================
        # 카메라 설정 (대향 배치)
        # ============================================
        self.camera_config = {
            'left': {
                'serial_number': '56755054',
                'position': [-200, 100, 108],  # mm [x, y, z]
                'rotation': [40, 0, -20],      # degrees [pitch, roll, yaw]
                'facing': 'right_inward',
                'height': 108,                 # mm
                'angle': 40                    # degrees (downward)
            },
            'right': {
                'serial_number': '54946194',
                'position': [-200, -100, 108],
                'rotation': [40, 0, 20],
                'facing': 'left_inward',
                'height': 108,
                'angle': 40
            }
        }
        
        # ============================================
        # Message Filters - 4채널 동기화
        # ============================================
        self.get_logger().info('Setting up message filters...')
        self.left_rgb_sub = message_filters.Subscriber(
            self, Image, '/zedxmini1/zedxmini1_node/left/image_rect_color'
        )
        self.get_logger().info('  - Left RGB subscriber created')
        
        self.left_depth_sub = message_filters.Subscriber(
            self, Image, '/zedxmini1/zedxmini1_node/depth/depth_registered'
        )
        self.get_logger().info('  - Left Depth subscriber created')
        
        self.right_rgb_sub = message_filters.Subscriber(
            self, Image, '/zedxmini2/zedxmini2_node/left/image_rect_color'
        )
        self.get_logger().info('  - Right RGB subscriber created')
        
        self.right_depth_sub = message_filters.Subscriber(
            self, Image, '/zedxmini2/zedxmini2_node/depth/depth_registered'
        )
        self.get_logger().info('  - Right Depth subscriber created')
        
        # 동기화 (50ms 허용 오차)
        self.get_logger().info('Creating ApproximateTimeSynchronizer (slop=0.05s)...')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_rgb_sub, self.left_depth_sub,
             self.right_rgb_sub, self.right_depth_sub],
            queue_size=10,
            slop=0.05  # 50ms
        )
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info('Message synchronization setup complete!')
        
        # ============================================
        # Camera Info 구독 (intrinsic 파라미터)
        # ============================================
        self.left_camera_info = None
        self.right_camera_info = None
        
        self.left_info_sub = self.create_subscription(
            CameraInfo,
            '/zedxmini1/zedxmini1_node/left/camera_info',
            self.left_info_callback,
            10
        )
        self.right_info_sub = self.create_subscription(
            CameraInfo,
            '/zedxmini2/zedxmini2_node/left/camera_info',
            self.right_info_callback,
            10
        )
        
        # ============================================
        # 수동 캡처 트리거
        # ============================================
        self.capture_sub = self.create_subscription(
            Bool,
            '/rebar/recorder/trigger',
            self.trigger_callback,
            10
        )
        
        # ============================================
        # 서비스
        # ============================================
        self.start_service = self.create_service(
            Trigger,
            '/rebar/recorder/start',
            self.start_recording_callback
        )
        self.stop_service = self.create_service(
            Trigger,
            '/rebar/recorder/stop',
            self.stop_recording_callback
        )
        self.save_session_service = self.create_service(
            Trigger,
            '/rebar/recorder/save_session',
            self.save_session_callback
        )
        
        # ============================================
        # 타이머 (자동/연속 모드)
        # ============================================
        if self.save_mode == 'auto':
            self.auto_timer = self.create_timer(
                self.auto_interval,
                self.auto_capture_callback
            )
            self.recording = True
            self.get_logger().info(f'Auto mode: capturing every {self.auto_interval}s')
        
        elif self.save_mode == 'continuous':
            timer_period = 1.0 / self.fps_limit
            self.continuous_timer = self.create_timer(
                timer_period,
                self.continuous_capture_callback
            )
            self.recording = True
            self.get_logger().info(f'Continuous mode: {self.fps_limit} FPS')
        
        # ============================================
        # 상태 출력
        # ============================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('Dual Camera Recorder Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Session ID: {self.session_id}')
        self.get_logger().info(f'Save Path: {self.session_dir}')
        self.get_logger().info(f'Mode: {self.save_mode}')
        self.get_logger().info(f'Rebar Spacing: {self.rebar_spacing} mm')
        self.get_logger().info(f'Rebar Pattern: {self.rebar_pattern}')
        self.get_logger().info(f'Save Depth: {self.save_depth}')
        self.get_logger().info('=' * 60)
        
        if self.save_mode == 'manual':
            self.get_logger().info('Waiting for trigger...')
            self.get_logger().info('  ros2 topic pub /rebar/recorder/trigger std_msgs/Bool "data: true" --once')
    
    def create_directories(self):
        """디렉토리 구조 생성"""
        dirs = [
            os.path.join(self.session_dir, 'left_camera', 'rgb'),
            os.path.join(self.session_dir, 'left_camera', 'depth'),
            os.path.join(self.session_dir, 'right_camera', 'rgb'),
            os.path.join(self.session_dir, 'right_camera', 'depth'),
            os.path.join(self.session_dir, 'metadata'),
        ]
        
        for d in dirs:
            Path(d).mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info(f'Created directory: {self.session_dir}')
    
    def left_info_callback(self, msg):
        """좌측 카메라 info 저장"""
        if self.left_camera_info is None:
            self.left_camera_info = msg
            self.save_camera_info('left', msg)
            self.get_logger().info('Left camera info received')
    
    def right_info_callback(self, msg):
        """우측 카메라 info 저장"""
        if self.right_camera_info is None:
            self.right_camera_info = msg
            self.save_camera_info('right', msg)
            self.get_logger().info('Right camera info received')
    
    def save_camera_info(self, side, camera_info):
        """카메라 intrinsic 파라미터 저장"""
        info_dict = {
            'width': camera_info.width,
            'height': camera_info.height,
            'camera_matrix': {
                'fx': camera_info.k[0],
                'fy': camera_info.k[4],
                'cx': camera_info.k[2],
                'cy': camera_info.k[5],
                'matrix': [
                    [camera_info.k[0], 0, camera_info.k[2]],
                    [0, camera_info.k[4], camera_info.k[5]],
                    [0, 0, 1]
                ]
            },
            'distortion': list(camera_info.d),
            'camera_setup': self.camera_config[side],
            'depth_info': {
                'format': 'uint16',
                'unit': 'millimeters',
                'encoding': 'PNG 16-bit grayscale',
                'camera_depth_range': '100-8000mm (0.1-8.0m)',
                'storage_range': '0-65535mm (0-65.535m)',
                'note': 'ZED X Mini depth range: 0.1m~8m. Values stored in millimeters.'
            }
        }
        
        filepath = os.path.join(
            self.session_dir,
            f'{side}_camera',
            'camera_info.json'
        )
        
        with open(filepath, 'w') as f:
            json.dump(info_dict, f, indent=2)
        
        self.get_logger().info(f'Saved {side} camera info to {filepath}')
    
    def sync_callback(self, left_rgb, left_depth, right_rgb, right_depth):
        """동기화된 4채널 이미지 콜백"""
        # DEBUG: 콜백 호출 확인
        if self.frame_count == 0:
            self.get_logger().info('sync_callback: First synchronized message received!')
            self.get_logger().info(f'  Mode: {self.save_mode}, Recording: {self.recording}')
        
        # recording이 False면 무시 (모든 모드 공통)
        if not self.recording:
            return
        
        # recording이 True일 때 이미지 저장
        self.save_frame(left_rgb, left_depth, right_rgb, right_depth)
    
    def trigger_callback(self, msg):
        """수동 캡처 트리거"""
        if msg.data and self.save_mode == 'manual':
            self.recording = True  # 일시적으로 활성화
            self.get_logger().info(f'Manual capture triggered (frame {self.frame_count})')
    
    def auto_capture_callback(self):
        """자동 캡처 타이머"""
        pass  # sync_callback에서 자동 처리
    
    def continuous_capture_callback(self):
        """연속 캡처 타이머"""
        pass  # sync_callback에서 자동 처리
    
    def save_frame(self, left_rgb, left_depth, right_rgb, right_depth):
        """프레임 저장"""
        try:
            # CV 이미지로 변환
            left_rgb_cv = self.bridge.imgmsg_to_cv2(left_rgb, 'bgr8')
            right_rgb_cv = self.bridge.imgmsg_to_cv2(right_rgb, 'bgr8')
            
            # 파일명
            frame_id = f'{self.frame_count:04d}'
            timestamp = self.get_clock().now().to_msg()
            
            # RGB 저장
            left_rgb_path = os.path.join(
                self.session_dir, 'left_camera', 'rgb', f'frame_{frame_id}.png'
            )
            right_rgb_path = os.path.join(
                self.session_dir, 'right_camera', 'rgb', f'frame_{frame_id}.png'
            )
            cv2.imwrite(left_rgb_path, left_rgb_cv)
            cv2.imwrite(right_rgb_path, right_rgb_cv)
            
            # Depth 저장
            if self.save_depth:
                # ZED depth는 32FC1 (float32, 단위: 미터)
                left_depth_cv = self.bridge.imgmsg_to_cv2(left_depth, 'passthrough')
                right_depth_cv = self.bridge.imgmsg_to_cv2(right_depth, 'passthrough')

                left_depth_path = os.path.join(
                    self.session_dir, 'left_camera', 'depth', f'frame_{frame_id}.png'
                )
                right_depth_path = os.path.join(
                    self.session_dir, 'right_camera', 'depth', f'frame_{frame_id}.png'
                )

                # float32 (미터) -> uint16 (밀리미터) 변환
                # ZED X Mini depth 범위: 0.1m ~ 8.0m (100mm ~ 8000mm)
                # NaN/Inf 처리: 0으로 설정
                left_depth_mm = np.nan_to_num(left_depth_cv * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
                right_depth_mm = np.nan_to_num(right_depth_cv * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)

                # uint16 범위 클리핑 (0 ~ 65535mm)
                # ZED X Mini는 최대 8000mm이므로 uint16으로 충분
                left_depth_mm = np.clip(left_depth_mm, 0, 65535).astype(np.uint16)
                right_depth_mm = np.clip(right_depth_mm, 0, 65535).astype(np.uint16)

                # 16-bit PNG로 저장
                cv2.imwrite(left_depth_path, left_depth_mm)
                cv2.imwrite(right_depth_path, right_depth_mm)

                # 시각화 이미지도 함께 저장 (8-bit, 사람이 볼 수 있게)
                self.save_depth_visualization(left_depth_mm, left_depth_path)
                self.save_depth_visualization(right_depth_mm, right_depth_path)
            
            # 메타데이터 저장
            self.save_frame_metadata(frame_id, timestamp)
            
            self.frame_count += 1
            
            # 로그
            depth_str = " + Depth" if self.save_depth else ""
            self.get_logger().info(
                f'✅ Saved frame {self.frame_count} (RGB{depth_str})'
            )
            
            # 수동 모드는 한 번만 저장 후 비활성화
            if self.save_mode == 'manual':
                self.recording = False
            
        except Exception as e:
            self.get_logger().error(f'Failed to save frame: {e}')
    
    def save_depth_visualization(self, depth_mm, depth_path):
        """Depth 이미지 시각화 저장 (사람이 볼 수 있는 8-bit 이미지)"""
        try:
            # 유효한 depth만 추출
            valid_mask = depth_mm > 0

            if not valid_mask.any():
                return  # 유효한 depth가 없으면 시각화 안함

            # 컬러맵 적용 (0-2000mm 범위로 정규화)
            depth_clipped = np.clip(depth_mm.astype(float), 0, 2000)
            depth_normalized = (depth_clipped / 2000 * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)

            # 무효 픽셀은 검은색
            depth_color[~valid_mask] = [0, 0, 0]

            # 시각화 이미지 저장
            vis_path = depth_path.replace('.png', '_visualization.png')
            cv2.imwrite(vis_path, depth_color)

        except Exception as e:
            self.get_logger().warn(f'Failed to save depth visualization: {e}')

    def save_frame_metadata(self, frame_id, timestamp):
        """프레임별 메타데이터 저장"""
        metadata = {
            'frame_id': int(frame_id),
            'timestamp': {
                'sec': timestamp.sec,
                'nanosec': timestamp.nanosec
            },
            'rebar_spacing': self.rebar_spacing,
            'rebar_pattern': self.rebar_pattern,
        }
        
        # 기존 메타데이터 로드
        metadata_path = os.path.join(self.session_dir, 'metadata', 'frame_info.json')
        
        if os.path.exists(metadata_path):
            with open(metadata_path, 'r') as f:
                all_metadata = json.load(f)
        else:
            all_metadata = {'frames': []}
        
        all_metadata['frames'].append(metadata)
        
        with open(metadata_path, 'w') as f:
            json.dump(all_metadata, f, indent=2)
    
    def start_recording_callback(self, request, response):
        """녹화 시작 서비스"""
        self.recording = True
        response.success = True
        response.message = f'Recording started (mode: {self.save_mode})'
        self.get_logger().info(response.message)
        return response
    
    def stop_recording_callback(self, request, response):
        """녹화 중지 서비스"""
        self.recording = False
        response.success = True
        response.message = f'Recording stopped. Total frames: {self.frame_count}'
        self.get_logger().info(response.message)
        return response
    
    def save_session_callback(self, request, response):
        """세션 정보 저장 서비스"""
        try:
            session_info = {
                'session_id': self.session_id,
                'total_frames': self.frame_count,
                'rebar_config': {
                    'spacing': self.rebar_spacing,
                    'pattern': self.rebar_pattern,
                    'crossing_points': 6  # 기본값
                },
                'camera_setup': self.camera_config,
                'save_mode': self.save_mode,
                'save_depth': self.save_depth,
                'depth_format': {
                    'format': 'uint16',
                    'unit': 'millimeters',
                    'encoding': 'PNG 16-bit grayscale',
                    'camera_depth_range': '100-8000mm (0.1-8.0m)',
                    'storage_range': '0-65535mm (0-65.535m)',
                    'conversion': 'ZED 32FC1 (meters) -> uint16 (millimeters)',
                    'note': 'ZED X Mini operational range: 0.1m~8m. Invalid/out-of-range depths stored as 0.'
                },
                'created_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            
            filepath = os.path.join(self.session_dir, 'metadata', 'session_info.json')
            
            with open(filepath, 'w') as f:
                json.dump(session_info, f, indent=2)
            
            response.success = True
            response.message = f'Session info saved: {filepath}'
            self.get_logger().info('=' * 60)
            self.get_logger().info('Session Summary:')
            self.get_logger().info(f'  Total frames: {self.frame_count}')
            self.get_logger().info(f'  Saved to: {self.session_dir}')
            self.get_logger().info('=' * 60)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to save session info: {e}'
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = DualCameraRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        # 종료 시 세션 정보 자동 저장
        request = Trigger.Request()
        node.save_session_callback(request, Trigger.Response())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
