#!/usr/bin/env python3
"""
EZI-IO ROS2 노드
FASTECH EZI-IO-EN-L16O16N-T I/O 모듈에서 리미트 센서 읽기
FASTECH Plus-E 라이브러리 사용
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# FASTECH Plus-E 라이브러리 경로 설정
# 환경 변수 또는 기본 경로 사용
import os
library_path = os.environ.get('FASTECH_LIBRARY_PATH', '/home/koceti/python/PE/Library')

# 라이브러리 경로 추가
if os.path.exists(library_path):
    sys.path.append(library_path)
else:
    print(f"⚠️ WARNING: FASTECH 라이브러리 경로가 없습니다: {library_path}")
    print(f"   환경 변수로 설정: export FASTECH_LIBRARY_PATH=/path/to/PE/Library")
    print(f"   또는 코드에서 library_path를 수정하세요.")
    sys.exit(1)

try:
    from FAS_EziMOTIONPlusE import *
    from MOTION_DEFINE import *
    from ReturnCodes_Define import *
except ImportError as e:
    print(f"❌ ERROR: FASTECH 라이브러리를 불러올 수 없습니다!")
    print(f"   경로: {library_path}")
    print(f"   에러: {e}")
    print(f"   해결 방법:")
    print(f"   1. FASTECH Plus-E 라이브러리를 {library_path}에 설치하세요")
    print(f"   2. 또는 환경 변수 설정: export FASTECH_LIBRARY_PATH=/실제/경로")
    sys.exit(1)


class EziIoNode(Node):
    """EZI-IO 리미트 센서 모니터링 노드"""
    
    def __init__(self):
        super().__init__('ezi_io_node')
        
        # 파라미터 선언
        self.declare_parameter('ip_address', '192.168.0.3')
        self.declare_parameter('board_id', 0)
        self.declare_parameter('update_rate', 20.0)  # Hz
        
        # 리미트 센서 매핑 (입력 채널 번호)
        self.declare_parameter('limit_x_min_channel', 2)  # IN02
        self.declare_parameter('limit_x_max_channel', 3)  # IN03
        self.declare_parameter('limit_y_min_channel', 1)  # IN01
        self.declare_parameter('limit_y_max_channel', 0)  # IN00
        self.declare_parameter('limit_z_min_channel', 5)  # IN05
        self.declare_parameter('limit_z_max_channel', 6)  # IN06
        self.declare_parameter('limit_yaw_min_channel', 4)  # IN04 (Yaw 원점)
        
        # 파라미터 가져오기
        ip_str = self.get_parameter('ip_address').value
        self.board_id = self.get_parameter('board_id').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # IP 주소를 4개의 숫자로 변환
        ip_parts = ip_str.split('.')
        self.ip_address = [int(x) for x in ip_parts]
        
        # 리미트 채널
        self.limit_channels = {
            'x_min': self.get_parameter('limit_x_min_channel').value,
            'x_max': self.get_parameter('limit_x_max_channel').value,
            'y_min': self.get_parameter('limit_y_min_channel').value,
            'y_max': self.get_parameter('limit_y_max_channel').value,
            'z_min': self.get_parameter('limit_z_min_channel').value,
            'z_max': self.get_parameter('limit_z_max_channel').value,
            'yaw_min': self.get_parameter('limit_yaw_min_channel').value,
        }
        
        # FASTECH Plus-E 연결
        self.connected = False
        self.connect_device()
        
        # 발행자 생성 - 개별 리미트 센서
        self.limit_publishers = {}
        for name in self.limit_channels.keys():
            topic = f'/limit_sensors/{name}'
            self.limit_publishers[name] = self.create_publisher(Bool, topic, 10)
        
        # 전체 입력 상태 발행
        self.all_inputs_pub = self.create_publisher(
            DiagnosticArray,
            '/ezi_io/diagnostics',
            10
        )
        
        # 타이머 - 주기적으로 입력 읽기
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.read_inputs_callback)
        
        # 이전 상태 저장 (변화 감지용)
        self.previous_limits = {}
        
        ip_str_display = '.'.join(map(str, self.ip_address))
        self.get_logger().info(f'EZI-IO node started: {ip_str_display}')
    
    def connect_device(self):
        """FASTECH Plus-E 연결"""
        try:
            # TCP 연결 시도
            result = FAS_ConnectTCP(
                self.ip_address[0], 
                self.ip_address[1], 
                self.ip_address[2], 
                self.ip_address[3], 
                self.board_id
            )
            
            if result == 0:
                self.get_logger().error(f'Failed to connect to EZI-IO')
                return False
            
            ip_str = '.'.join(map(str, self.ip_address))
            self.get_logger().info(f'Connected to EZI-IO at {ip_str}')
            self.connected = True
            return True
                
        except Exception as e:
            self.get_logger().error(f'EZI-IO connection error: {e}')
            return False
    
    def read_inputs_callback(self):
        """입력 상태 읽기 및 발행"""
        if not self.connected:
            if not self.connect_device():
                return
        
        try:
            # FAS_GetInput으로 입력 읽기
            status_result, input_status, latch_status = FAS_GetInput(self.board_id)
            
            if status_result != FMM_OK:
                self.get_logger().warn(f'Failed to read inputs (error: {status_result})')
                return
            
            # 리미트 센서 상태 발행
            for name, channel in self.limit_channels.items():
                if channel < 16:  # 16 inputs total
                    # 비트 체크
                    state = bool(input_status & (1 << channel))
                    
                    # 상태 변화시에만 로그 (TRIGGERED만 info, CLEAR는 debug)
                    if name not in self.previous_limits or self.previous_limits[name] != state:
                        if state:  # TRIGGERED
                            self.get_logger().info(f'Limit {name} (IN{channel:02d}): TRIGGERED')
                        else:  # CLEAR
                            self.get_logger().debug(f'Limit {name} (IN{channel:02d}): CLEAR')
                        self.previous_limits[name] = state
                    
                    # 발행
                    msg = Bool()
                    msg.data = state
                    self.limit_publishers[name].publish(msg)
            
            # 진단 메시지 발행
            self.publish_diagnostics(input_status)
            
        except Exception as e:
            self.get_logger().error(f'Error reading inputs: {e}')
            # 재연결 시도
            self.connected = False
            self.connect_device()
    
    def publish_diagnostics(self, input_status):
        """진단 정보 발행"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = "EZI-IO Limit Sensors"
        ip_str = '.'.join(map(str, self.ip_address))
        status.hardware_id = ip_str
        
        # 리미트 센서 상태
        any_triggered = False
        for name, channel in self.limit_channels.items():
            if channel < 16:
                state = bool(input_status & (1 << channel))
                status.values.append(KeyValue(
                    key=name,
                    value="TRIGGERED" if state else "CLEAR"
                ))
                if state:
                    any_triggered = True
        
        # 전체 상태 레벨
        if any_triggered:
            status.level = DiagnosticStatus.WARN
            status.message = "Some limits triggered"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "All limits clear"
        
        diag_array.status.append(status)
        self.all_inputs_pub.publish(diag_array)
    
    def destroy_node(self):
        """노드 종료"""
        if self.connected:
            FAS_Close(self.board_id)
            self.get_logger().info('EZI-IO connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EziIoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
