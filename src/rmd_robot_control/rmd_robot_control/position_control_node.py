#!/usr/bin/env python3
"""
통합 제어 노드
- 0x141, 0x142: cmd_vel 속도 제어
- 0x143, 0x144, 0x145, 0x146, 0x147: 위치 제어
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float32, Int32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import struct
import time
import threading
from typing import Dict, Optional, List

from .can_manager import CANManager
from .rmd_x4_protocol import RMDX4Protocol, CommandType


class PositionControlNode(Node):
    """통합 제어 노드 (위치제어 + cmd_vel 속도제어)"""

    def __init__(self):
        super().__init__('unified_control_node')

        # 파일 로그 설정 (INFO 이상 모든 로그 기록)
        import logging
        import os
        self.debug_logger = logging.getLogger('unified_control_debug')
        self.debug_logger.setLevel(logging.INFO)  # INFO 이상 기록
        # 기존 핸들러 제거 (중복 방지)
        self.debug_logger.handlers.clear()
        # 파일 핸들러 추가
        log_file = '/tmp/unified_control_debug.log'
        fh = logging.FileHandler(log_file, mode='a', encoding='utf-8', delay=False)
        fh.setLevel(logging.INFO)  # INFO 이상 기록
        # ms 단위 타임스탬프 출력
        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S.%f')
        fh.setFormatter(formatter)
        self.debug_logger.addHandler(fh)
        self.debug_log_file_handler = fh  # 나중에 flush를 위해 저장
        self.debug_logger.info("="*60)
        self.debug_logger.info("Unified Control Node 시작")
        self.debug_logger.info("="*60)
        self.debug_log_file_handler.flush()  # 즉시 파일에 기록

        # 파라미터 선언
        self.declare_parameter('can_interface', 'can2')
        self.declare_parameter('motor_ids', [0x141, 0x142, 0x143, 0x144, 0x145, 0x146, 0x147])
        self.declare_parameter('joint_names', ['drive_left', 'drive_right', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'])
        self.declare_parameter('max_position', 36000.0)  # 최대 위치 (도, 100바퀴)
        self.declare_parameter('min_position', -36000.0)  # 최소 위치 (도, -100바퀴)
        self.declare_parameter('max_velocity', 100.0)  # 최대 속도 (도/초)
        self.declare_parameter('position_tolerance', 1.0)  # 위치 허용 오차 (도)
        
        # CMD_VEL 파라미터 선언
        self.declare_parameter('left_motor_id', 0x141)
        self.declare_parameter('right_motor_id', 0x142)
        self.declare_parameter('wheel_radius', 0.1)  # 바퀴 반지름 (m)
        self.declare_parameter('wheel_base', 0.5)     # 바퀴 간 거리 (m)
        self.declare_parameter('max_linear_vel', 10.0)  # 최대 선속도 (m/s) - 1.0 → 10.0 (10배)
        self.declare_parameter('max_angular_vel', 2.0) # 최대 각속도 (rad/s) - 1.0 → 10.0 → 2.0 (10배→5분의1로 감소)
        
        # 파라미터 가져오기
        self.can_interface = self.get_parameter('can_interface').value
        self.motor_ids = self.get_parameter('motor_ids').value
        self.joint_names = self.get_parameter('joint_names').value
        self.max_position = self.get_parameter('max_position').value
        self.min_position = self.get_parameter('min_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        
        # CMD_VEL 파라미터 가져오기
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # CAN 매니저 및 프로토콜 초기화
        self.can_manager = CANManager(self.can_interface)
        self.protocol = RMDX4Protocol()

        # 주행 모터 가감속 상한 상승 (RAM+ROM 저장)
        self.apply_drive_accel_limits(accel_dpss=20000)

        # 모터 상태 저장 (위치제어 모터)
        self.motor_states: Dict[int, Dict] = {}
        self.target_positions: Dict[int, float] = {}
        self.current_positions: Dict[int, float] = {}

        for motor_id in self.motor_ids:
            self.motor_states[motor_id] = {
                'position': 0.0,
                'velocity': 0.0,
                'torque': 0.0,
                'target_position': 0.0,
                'is_moving': False,
                'last_position_read_time': 0.0  # 마지막 위치 읽기 시간 (과부하 방지)
            }
            self.target_positions[motor_id] = 0.0
            self.current_positions[motor_id] = 0.0
        
        # CMD_VEL 모터 상태 저장 (S20 모드에서 위치 제어도 가능하도록 확장)
        # 0x141, 0x142도 위치 제어 시 is_moving, target_position 등을 사용할 수 있도록 초기화
        if self.left_motor_id not in self.motor_states:
            self.motor_states[self.left_motor_id] = {'position': 0.0, 'velocity': 0.0, 'torque': 0.0}
        if self.right_motor_id not in self.motor_states:
            self.motor_states[self.right_motor_id] = {'position': 0.0, 'velocity': 0.0, 'torque': 0.0}
        
        # 0x141, 0x142의 위치 제어용 상태 초기화 (기존 위치 제어 모터들과 동일한 구조)
        if 'target_position' not in self.motor_states[self.left_motor_id]:
            self.motor_states[self.left_motor_id]['target_position'] = 0.0
            self.motor_states[self.left_motor_id]['is_moving'] = False
            self.motor_states[self.left_motor_id]['last_position_read_time'] = 0.0
            self.motor_states[self.left_motor_id]['position_command_time'] = 0.0  # 위치 명령 시간 (타임아웃용)
        if 'target_position' not in self.motor_states[self.right_motor_id]:
            self.motor_states[self.right_motor_id]['target_position'] = 0.0
            self.motor_states[self.right_motor_id]['is_moving'] = False
            self.motor_states[self.right_motor_id]['last_position_read_time'] = 0.0
            self.motor_states[self.right_motor_id]['position_command_time'] = 0.0  # 위치 명령 시간 (타임아웃용)
        
        # 0x141, 0x142의 target_positions 초기화
        if self.left_motor_id not in self.target_positions:
            self.target_positions[self.left_motor_id] = 0.0
        if self.right_motor_id not in self.target_positions:
            self.target_positions[self.right_motor_id] = 0.0
        if self.left_motor_id not in self.current_positions:
            self.current_positions[self.left_motor_id] = 0.0
        if self.right_motor_id not in self.current_positions:
            self.current_positions[self.right_motor_id] = 0.0

        # 주행 모터 동기화 변수 (종료 시점 동기화)
        self.drive_sync_mode = False  # 주행 모터 동기화 모드 활성화 플래그
        self.drive_left_near_target = False  # 좌측 모터 목표 근접 플래그
        self.drive_right_near_target = False  # 우측 모터 목표 근접 플래그
        self.drive_sync_tolerance = 5.0  # 목표 근접 판정 허용 오차 (도)
        self.drive_sync_final_tolerance = 1.0  # 최종 정지 허용 오차 (도)
        self.drive_both_commands_received = False  # 양쪽 모터 명령 모두 수신 플래그
        self.drive_last_left_command_time = 0.0  # 좌측 모터 마지막 명령 시간
        self.drive_last_right_command_time = 0.0  # 우측 모터 마지막 명령 시간
        self.drive_command_sync_window = 0.2  # 명령 동기화 시간 윈도우 (초)

        # 모터 전류 보호 시스템
        self.motor_current_limits = {
            # X4-10 모터들 (주행, Z축, Yaw, Y축)
            0x141: {'name': '좌측주행', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 100},
            0x142: {'name': '우측주행', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 100},
            0x144: {'name': 'Z축', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 75},
            0x145: {'name': 'Yaw', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 67},
            0x146: {'name': 'X축', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 200},
            0x147: {'name': 'Y축', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 200},
            # X4-36 모터 (횡이동)
            0x143: {'name': '횡이동', 'rated': 6.1, 'warning': 8.0, 'danger': 12.0, 'emergency': 18.0, 'speed_limit': 100},
        }

        # 전류 모니터링 상태
        self.motor_protection_state = {}
        for motor_id in self.motor_current_limits.keys():
            self.motor_protection_state[motor_id] = {
                'current': 0.0,
                'temperature': 0,
                'high_current_count': 0,  # 고전류 연속 감지 횟수
                'emergency_count': 0,  # 긴급 상황 연속 감지 횟수
                'last_warning_time': 0.0,
                'protection_active': False,  # 보호 모드 활성화 여부
                'speed_limited': False,  # 속도 제한 활성화 여부
                'stopped': False,  # 긴급 정지 상태
            }

        # 토픽 구독/발행
        # 위치제어 토픽
        self.trajectory_subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # CMD_VEL 토픽 추가
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        self.motor_status_publisher = self.create_publisher(
            Float64MultiArray,
            'motor_status',
            10
        )

        # 개별 모터 위치 발행용 퍼블리셔
        from std_msgs.msg import Float32
        self.motor_position_publishers = {}
        for motor_id in self.motor_ids:
            topic_name = f'motor_{hex(motor_id)}_position'
            self.motor_position_publishers[motor_id] = self.create_publisher(
                Float32, topic_name, 10
            )
        
        # 주행 모터(0x141, 0x142) 위치 발행용 퍼블리셔 (S20 모드용)
        self.motor_position_publishers[self.left_motor_id] = self.create_publisher(
            Float32, f'motor_{hex(self.left_motor_id)}_position', 10
        )
        self.motor_position_publishers[self.right_motor_id] = self.create_publisher(
            Float32, f'motor_{hex(self.right_motor_id)}_position', 10
        )
        
        # CMD_VEL 모터 RPM 발행용 퍼블리셔
        self.left_rpm_publisher = self.create_publisher(
            Float32,
            f'motor_{hex(self.left_motor_id)}_rpm',
            10
        )
        self.right_rpm_publisher = self.create_publisher(
            Float32,
            f'motor_{hex(self.right_motor_id)}_rpm',
            10
        )
        
        # 목표 위치 도달 완료 알림 퍼블리셔 (motor_id 발행)
        self.goal_reached_publisher = self.create_publisher(
            Int32,
            'motor_goal_reached',
            10
        )

        # 개별 관절 제어용 토픽들 (위치)
        self.joint_position_subscriptions = []
        for i, joint_name in enumerate(self.joint_names):
            topic_name = f'{joint_name}/position'
            subscription = self.create_subscription(
                Float64MultiArray,
                topic_name,
                lambda msg, idx=i: self.single_joint_callback(msg, idx),
                10
            )
            self.joint_position_subscriptions.append(subscription)
        
        # 개별 관절 속도 제어용 토픽들 (0x144, 0x145용)
        self.joint_speed_subscriptions = []
        for i, joint_name in enumerate(self.joint_names):
            topic_name = f'{joint_name}/speed'
            subscription = self.create_subscription(
                Float32,
                topic_name,
                lambda msg, idx=i: self.single_joint_speed_callback(msg, idx),
                10
            )
            self.joint_speed_subscriptions.append(subscription)
        
        # 주행 모터(0x141, 0x142) 위치 제어 토픽 구독 (S20 모드용)
        self.left_wheel_position_sub = self.create_subscription(
            Float64MultiArray,
            '/motor_0x141/position',
            self.left_wheel_position_callback,
            10
        )
        self.right_wheel_position_sub = self.create_subscription(
            Float64MultiArray,
            '/motor_0x142/position',
            self.right_wheel_position_callback,
            10
        )
        
        # 타이머 설정 (bus-off 방지: 모터 상태 읽기 비활성화)
        self.status_timer = self.create_timer(0.1, self.publish_status)  # 10Hz
        # self.motor_status_timer = self.create_timer(0.1, self.read_motor_status)  # 비활성화 (CAN 부하 감소)
        self.position_control_timer = self.create_timer(0.1, self.position_control_loop)  # 10Hz로 변경 (과부하 방지)
        
        # 서비스 생성
        self.brake_release_service = self.create_service(
            Trigger,
            'safe_brake_release',
            self.brake_release_service_callback
        )

        self.brake_lock_service = self.create_service(
            Trigger,
            'safe_brake_lock',
            self.brake_lock_service_callback
        )
        
        # CAN 연결 및 콜백 등록
        if self.can_manager.connect():
            self.get_logger().info(f"✅ CAN {self.can_interface} 연결 성공 (통합 노드)")
            
            # 위치제어 모터 응답 콜백 등록 (0x143~0x147)
            for motor_id in self.motor_ids:
                motor_number = motor_id - 0x140
                response_id = 0x240 + motor_number
                self.can_manager.register_callback(
                    response_id,
                    lambda can_id, data, mid=motor_id: self.motor_response_callback(mid, data)
                )
            
            # CMD_VEL 모터 응답 콜백 등록 (0x141, 0x142 -> 0x241, 0x242로 응답)
            left_motor_number = self.left_motor_id - 0x140  # 0x141 - 0x140 = 1
            left_response_id = 0x240 + left_motor_number  # 0x240 + 1 = 0x241

            right_motor_number = self.right_motor_id - 0x140  # 0x142 - 0x140 = 2
            right_response_id = 0x240 + right_motor_number  # 0x240 + 2 = 0x242

            self.can_manager.register_callback(
                left_response_id,  # 0x241
                self.left_motor_response_callback
            )

            self.can_manager.register_callback(
                right_response_id,  # 0x242
                self.right_motor_response_callback
            )

            self.get_logger().info(f"CMD_VEL callback registered: 0x{left_response_id:03X}, 0x{right_response_id:03X}")
            
            # 백그라운드 수신 스레드 시작
            self.can_manager.start_receive_thread()

            # 수신 스레드가 완전히 시작될 때까지 약간 대기
            time.sleep(0.1)

            # ✨ 노드 시작 시 모든 모터의 엔코더 위치 읽기 (브레이크 해제 전에도 위치 확보)
            self.get_logger().info("📍 [초기화] 모든 모터 엔코더 위치 읽기 시작...")
            self.read_all_encoder_positions()
            self.get_logger().info("✅ [초기화] 엔코더 위치 읽기 완료")

            # 모터 활성화 (자동 브레이크 해제 비활성화 - GUI에서 수동 제어)
            # self.enable_motors()  # GUI에서 수동으로 브레이크 해제하므로 주석 처리
            self.get_logger().info("✅ 통합 노드 초기화 완료 (7개 모터)")
        else:
            self.get_logger().error(f"❌ CAN {self.can_interface} 연결 실패")
    
    def enable_motors(self):
        """모터 활성화 (브레이크 릴리즈 - 순차적 전송으로 CAN 버스 보호)"""
        self.get_logger().info("🔓 브레이크 해제 시작...")
        
        # 브레이크 릴리즈 명령
        brake_release_cmd = self.protocol.create_system_command(CommandType.BRAKE_RELEASE)

        # 순차적으로 브레이크 릴리즈 (CAN 버스 보호)
        for i, motor_id in enumerate(self.motor_ids):
            # 브레이크 릴리즈 명령 전송
            success = self.can_manager.send_frame(motor_id, brake_release_cmd)
            
            if not success:
                self.debug_logger.warning(f"❌ 모터 0x{motor_id:03X} 브레이크 해제 실패")
                self.get_logger().warning(f"❌ M{motor_id:03X} 브레이크 해제 실패")

            # 마지막 모터가 아니면 지연 시간 추가
            if i < len(self.motor_ids) - 1:
                time.sleep(0.1)  # 100ms 지연으로 CAN 버스 보호

        self.get_logger().info("✅ 브레이크 해제 완료")
        time.sleep(0.5)  # 모든 명령 처리 대기

    def read_all_encoder_positions(self):
        """모든 모터의 현재 엔코더 위치 읽기 (초기화 및 브레이크 해제 시 사용)"""
        multi_turn_cmd = self.protocol.create_system_command(CommandType.READ_MULTI_TURN_ANGLE)

        # 모든 모터 (위치 제어 모터 + 주행 모터)의 위치 읽기
        all_motors = list(self.motor_ids) + [self.left_motor_id, self.right_motor_id]

        for i, motor_id in enumerate(all_motors):
            if i > 0:
                time.sleep(0.2)  # 200ms 간격으로 CAN 버스 보호

            # 멀티턴 각도 읽기 (0x92)
            self.can_manager.send_frame(motor_id, multi_turn_cmd)
            self.get_logger().info(f"  → 0x{motor_id:03X} 위치 읽기 요청")

        # 응답 대기 (모터 개수 * 0.2초 + 여유 시간)
        time.sleep(1.5)

        # 읽은 위치 로그 출력
        self.get_logger().info("📊 모터 현재 위치:")
        for motor_id in all_motors:
            pos = self.motor_states[motor_id]['position']
            self.get_logger().info(f"  0x{motor_id:03X}: {pos:.1f}°")

    def safe_brake_release(self):
        """안전한 브레이크 해제 (모터 7개 대응)"""
        self.get_logger().info("🔓 브레이크 해제 중...")
        
        brake_release_cmd = self.protocol.create_system_command(CommandType.BRAKE_RELEASE)
        
        # 모터 7개에 대해 매우 안전하게 순차 처리
        failed_motors = []
        for i, motor_id in enumerate(self.motor_ids):
            # 각 모터마다 충분한 지연 시간 확보
            if i > 0:
                time.sleep(0.5)  # 500ms 지연으로 증가
            
            # 브레이크 해제 명령 전송
            success = self.can_manager.send_frame(motor_id, brake_release_cmd)
            
            if not success:
                failed_motors.append(motor_id)
                self.debug_logger.error(f"❌ 모터 0x{motor_id:03X} 브레이크 해제 실패")
        
        # 모든 명령 완료 후 충분한 대기 시간
        time.sleep(2.0)  # 2초 대기

        if failed_motors:
            self.get_logger().warning(f"WARNING: Brake release failed: {[hex(m) for m in failed_motors]}")
        else:
            self.get_logger().info("✅ 브레이크 해제 완료")

        # 브레이크 해제 후 현재 위치 읽기 (매우 중요!)
        self.get_logger().info("📍 [브레이크 해제] 모터 현재 위치 읽는 중...")
        self.read_all_encoder_positions()
        self.get_logger().info("✅ [브레이크 해제] 위치 읽기 완료")
    
    def safe_brake_lock(self):
        """안전한 브레이크 잠금 (모터 5개)"""
        self.get_logger().info("🔒 브레이크 잠금 중...")

        brake_lock_cmd = self.protocol.create_system_command(CommandType.BRAKE_LOCK)

        # 모터 5개에 대해 순차 처리
        failed_motors = []
        for i, motor_id in enumerate(self.motor_ids):
            # 각 모터마다 지연 시간 확보
            if i > 0:
                time.sleep(0.2)  # 200ms 지연

            # 브레이크 잠금 명령 전송
            success = self.can_manager.send_frame(motor_id, brake_lock_cmd)

            if not success:
                failed_motors.append(motor_id)
                self.debug_logger.error(f"❌ 모터 0x{motor_id:03X} 브레이크 잠금 실패")

        # 모든 명령 완료 후 충분한 대기 시간
        time.sleep(1.0)  # 1초 대기

        if failed_motors:
            self.get_logger().warning(f"WARNING: Brake lock failed: {[hex(m) for m in failed_motors]}")
        else:
            self.get_logger().info("✅ 브레이크 잠금 완료")

    def brake_release_service_callback(self, request, response):
        """브레이크 해제 서비스 콜백"""
        try:
            self.get_logger().info("🔓 브레이크 해제 서비스 호출됨")
            self.safe_brake_release()

            response.success = True
            response.message = "브레이크 해제가 안전하게 완료되었습니다"
            return response

        except Exception as e:
            self.get_logger().error(f"❌ 브레이크 해제 서비스 오류: {e}")
            response.success = False
            response.message = f"브레이크 해제 실패: {e}"
            return response

    def brake_lock_service_callback(self, request, response):
        """브레이크 잠금 서비스 콜백"""
        try:
            self.get_logger().info("🔒 브레이크 잠금 서비스 호출됨")
            self.safe_brake_lock()

            response.success = True
            response.message = "브레이크 잠금이 안전하게 완료되었습니다"
            return response

        except Exception as e:
            self.get_logger().error(f"❌ 브레이크 잠금 서비스 오류: {e}")
            response.success = False
            response.message = f"브레이크 잠금 실패: {e}"
            return response
    
    def trajectory_callback(self, msg: JointTrajectory):
        """궤적 명령 콜백"""
        self.debug_logger.info(f"📥 Trajectory 메시지 수신: joint_names={msg.joint_names}, points={len(msg.points)}")
        self.get_logger().info(f"📥 Trajectory 메시지 수신: {msg.joint_names}")

        if not msg.points:
            self.debug_logger.warning("❌ Trajectory 포인트가 없습니다")
            return

        # 첫 번째 포인트 사용 (간단한 구현)
        point = msg.points[0]
        self.debug_logger.info(f"📍 첫 번째 포인트: positions={point.positions}")

        # 관절 이름과 위치 매핑
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in self.joint_names:
                joint_index = self.joint_names.index(joint_name)
                if joint_index < len(self.motor_ids) and i < len(point.positions):
                    motor_id = self.motor_ids[joint_index]
                    target_position = point.positions[i] * 180.0 / 3.14159  # 라디안 -> 도

                    # 위치 제한
                    target_position = max(self.min_position, min(self.max_position, target_position))

                    self.target_positions[motor_id] = target_position
                    self.motor_states[motor_id]['target_position'] = target_position
                    self.motor_states[motor_id]['is_moving'] = True

                    self.debug_logger.info(
                        f"🎯 모터 0x{motor_id:03X} 목표 위치 설정: {target_position:.1f}도"
                    )

                    # 직접 위치 명령 전송
                    self.send_position_command(motor_id, target_position)

                    self.get_logger().info(
                        f"관절 {joint_name} (ID: 0x{motor_id:03X}) 목표 위치: {target_position:.1f}도"
                    )
    
    def single_joint_callback(self, msg: Float64MultiArray, joint_index: int):
        """단일 관절 위치 명령 콜백"""
        if joint_index >= len(self.motor_ids) or not msg.data:
            return
        
        motor_id = self.motor_ids[joint_index]
        
        # 로그: 수신한 위치 명령 (degree 단위 그대로)
        received_position = msg.data[0]
        
        # 속도 파라미터 (옵션, 기본값은 모터별 설정 사용)
        custom_speed = None
        if len(msg.data) > 1:
            custom_speed = int(msg.data[1])  # dps 단위
        
        self.get_logger().info(
            f'📥 [ROS2] /joint_{joint_index+1}/position 수신: {received_position:.2f}° (motor 0x{motor_id:03X})'
            + (f', speed={custom_speed}dps' if custom_speed else '')
        )
        
        # degree 단위 그대로 사용 (라디안 변환 제거)
        target_position = received_position
        
        # 위치 제한
        target_position = max(self.min_position, min(self.max_position, target_position))
        
        self.target_positions[motor_id] = target_position
        self.motor_states[motor_id]['target_position'] = target_position
        self.motor_states[motor_id]['is_moving'] = True
        
        # 주행 모터(0x141, 0x142)의 경우 motor_states에 is_moving 키가 없을 수 있으므로 확인
        if 'is_moving' not in self.motor_states[motor_id]:
            self.motor_states[motor_id]['is_moving'] = True
        
        self.get_logger().info(
            f'🎯 [목표설정] {self.joint_names[joint_index]} (0x{motor_id:03X}): {target_position:.1f}°'
        )
        
        # 위치 명령 전송 (속도 옵션 포함)
        self.send_position_command(motor_id, target_position, custom_speed)
    
    def single_joint_speed_callback(self, msg: Float32, joint_index: int):
        """단일 관절 속도 명령 콜백 (0x144, 0x145용)"""
        if joint_index >= len(self.motor_ids):
            return
        
        motor_id = self.motor_ids[joint_index]
        speed_dps = msg.data  # degree per second
        
        # 로그: 수신한 속도 명령
        self.get_logger().info(
            f'📥 [ROS2] /joint_{joint_index+1}/speed 수신: {speed_dps:.1f} dps (motor 0x{motor_id:03X})'
        )
        
        # 속도 명령 전송 (0xA2)
        speed_control = int(speed_dps * 100)  # 0.01 dps/LSB
        
        self.get_logger().info(
            f'CAN2: 0x{motor_id:03X} 속도 명령: {speed_dps:.1f} dps (제어값={speed_control})'
        )
        
        self.send_speed_command_single(motor_id, speed_control)
    
    def position_control_loop(self):
        """위치 제어 루프 - 목표 도달 확인 (과부하 방지: 위치 읽기 주기 제한)"""
        import time as time_module
        current_time = time_module.time()

        # 이동 중인 모터들의 위치를 읽기 위해 0x92 명령 전송
        # 단, 위치 읽기 주기를 제한하여 과부하 방지 (50Hz -> 10Hz, 100ms 간격)
        multi_turn_cmd = self.protocol.create_system_command(CommandType.READ_MULTI_TURN_ANGLE)
        position_read_interval = 0.1  # 100ms 간격 (10Hz)

        # 위치 제어 모터들 위치 읽기 (모든 motor_ids: 0x141~0x147)
        # 0x141, 0x142도 이제 motor_ids에 포함되어 있음
        motors_to_read = []
        
        for motor_id in self.motor_ids:
            if motor_id not in self.motor_states:
                continue

            is_moving = self.motor_states[motor_id].get('is_moving', False)
            if not is_moving:
                continue

            # 마지막 위치 읽기 시간 확인 (과부하 방지)
            last_read_time = self.motor_states[motor_id].get('last_position_read_time', 0.0)
            time_since_last_read = current_time - last_read_time

            # 위치 읽기 주기 제한 (100ms 간격)
            if time_since_last_read >= position_read_interval:
                motors_to_read.append(motor_id)
        
        # 0x92 명령 전송 (모든 이동 중인 모터)
        for motor_id in motors_to_read:
            self.can_manager.send_frame(motor_id, multi_turn_cmd)
            self.motor_states[motor_id]['last_position_read_time'] = current_time
            self.debug_logger.debug(f"📤 [0x92] 0x{motor_id:03X} 멀티턴 각도 읽기 명령 전송")

        # CAN 응답을 위한 짧은 대기 (응답은 motor_response_callback에서 비동기 처리)
        time.sleep(0.005)  # 5ms 대기

        # 목표 도달 확인 (모든 위치 제어 모터들)
        # 단, 주행 모터(0x141, 0x142)는 동기화 모드일 때 별도 처리
        for motor_id in self.motor_ids:
            if motor_id not in self.motor_states:
                continue
            if not self.motor_states[motor_id].get('is_moving', False):
                continue
            if motor_id not in self.target_positions:
                continue

            # 주행 모터는 동기화 모드일 때 별도 처리 (아래에서 동기화 종료 로직 실행)
            if self.drive_sync_mode and motor_id in [self.left_motor_id, self.right_motor_id]:
                continue

            current_pos = self.motor_states[motor_id]['position']
            target_pos = self.target_positions[motor_id]

            # 위치 오차 계산
            position_error = abs(target_pos - current_pos)

            # 모터별 허용 오차 설정
            if motor_id == 0x143:  # 횡이동 모터 (떨림 방지)
                tolerance = 0.5  # 0.5도 허용 (더 엄격)
            elif motor_id == 0x146:  # Z축 모터
                tolerance = 5.0  # 5도 허용
            else:
                tolerance = self.position_tolerance  # 기본 1도

            # 목표 위치에 도달했는지 확인
            if position_error < tolerance:
                log_msg = (
                    f"🎯 모터 0x{motor_id:03X} 목표 도달: {current_pos:.1f}° "
                    f"(목표: {target_pos:.1f}°, 오차: {position_error:.1f}°)"
                )
                self.get_logger().info(log_msg)
                self.debug_logger.info(log_msg)

                # is_moving=False 설정
                self.motor_states[motor_id]['is_moving'] = False

                # 정지 시 속도=0 + 브레이크 적용으로 잔류 토크 제거
                if motor_id in [self.left_motor_id, self.right_motor_id, 0x143]:
                    self.apply_stop_damping(motor_id)

                # 목표 도달 토픽 발행
                goal_msg = Int32()
                goal_msg.data = motor_id
                self.goal_reached_publisher.publish(goal_msg)

        # ✨ 주행 모터 동기화 종료 로직 (0x141, 0x142)
        if self.drive_sync_mode:
            left_moving = self.motor_states[self.left_motor_id].get('is_moving', False)
            right_moving = self.motor_states[self.right_motor_id].get('is_moving', False)

            # 둘 다 이동 중일 때만 동기화 로직 실행
            if left_moving and right_moving:
                left_pos = self.motor_states[self.left_motor_id]['position']
                right_pos = self.motor_states[self.right_motor_id]['position']
                left_target = self.target_positions[self.left_motor_id]
                right_target = self.target_positions[self.right_motor_id]

                left_error = abs(left_target - left_pos)
                right_error = abs(right_target - right_pos)

                # 1단계: 각 모터가 목표에 근접했는지 확인 (5도 이내)
                if left_error < self.drive_sync_tolerance:
                    if not self.drive_left_near_target:
                        self.drive_left_near_target = True
                        self.debug_logger.info(f'🔄 [동기화] 0x141 목표 근접: {left_pos:.1f}° (목표: {left_target:.1f}°, 오차: {left_error:.1f}°)')
                        self.debug_log_file_handler.flush()

                if right_error < self.drive_sync_tolerance:
                    if not self.drive_right_near_target:
                        self.drive_right_near_target = True
                        self.debug_logger.info(f'🔄 [동기화] 0x142 목표 근접: {right_pos:.1f}° (목표: {right_target:.1f}°, 오차: {right_error:.1f}°)')
                        self.debug_log_file_handler.flush()

                # 2단계: 양쪽 모두 목표에 근접하고, 최종 허용 오차 이내면 동시 정지
                if self.drive_left_near_target and self.drive_right_near_target:
                    if left_error < self.drive_sync_final_tolerance and right_error < self.drive_sync_final_tolerance:
                        # 양쪽 모두 최종 허용 오차 이내 도달 → 동시 정지
                        self.debug_logger.info(
                            f'🎯 [동기화] 주행 모터 동기화 정지!\n'
                            f'  0x141: {left_pos:.1f}° (목표: {left_target:.1f}°, 오차: {left_error:.1f}°)\n'
                            f'  0x142: {right_pos:.1f}° (목표: {right_target:.1f}°, 오차: {right_error:.1f}°)'
                        )
                        self.debug_log_file_handler.flush()  # 즉시 파일에 기록

                        # 동시 정지 처리
                        self.motor_states[self.left_motor_id]['is_moving'] = False
                        self.motor_states[self.right_motor_id]['is_moving'] = False

                        # 목표 도달 토픽 발행
                        goal_msg_left = Int32()
                        goal_msg_left.data = self.left_motor_id
                        self.goal_reached_publisher.publish(goal_msg_left)

                        goal_msg_right = Int32()
                        goal_msg_right.data = self.right_motor_id
                        self.goal_reached_publisher.publish(goal_msg_right)

                        # 정지 시 속도=0 후 브레이크로 떨림 억제
                        self.apply_stop_damping(self.left_motor_id)
                        self.apply_stop_damping(self.right_motor_id)

                        # 동기화 모드 해제
                        self.drive_sync_mode = False
                        self.drive_left_near_target = False
                        self.drive_right_near_target = False
                        self.drive_both_commands_received = False
            else:
                # 한쪽만 이동 중이거나 둘 다 정지 → 동기화 모드 해제
                if not left_moving and not right_moving:
                    if self.drive_sync_mode:
                        self.debug_logger.info(f'🔄 [동기화] 주행 모터 둘 다 정지 → 동기화 모드 해제')
                        self.debug_log_file_handler.flush()
                        self.drive_sync_mode = False
                        self.drive_left_near_target = False
                        self.drive_right_near_target = False
                        self.drive_both_commands_received = False
    
    def read_motor_error(self, motor_id: int):
        """모터 에러 읽기 (0x9A 명령)"""
        error_read_cmd = bytes([0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.can_manager.send_frame(motor_id, error_read_cmd)
        self.get_logger().warning(f"🔍 0x{motor_id:03X} 에러 읽기 명령 전송")
    
    def send_speed_command(self, motor_id: int, velocity: float):
        """속도 명령 전송 (도/초)"""
        # RMD-X4 속도 명령 형식: [0xA2][0x00][speed(4bytes float, dps)]
        # speed: 도/초 단위
        command_data = bytes([self.COMMANDS['SET_MOTOR_SPEED'], 0x00]) + \
                      struct.pack('<f', velocity) + \
                      b'\x00\x00'  # 패딩

        self.can_manager.send_frame(motor_id, command_data)

        self.get_logger().debug(
            f"모터 0x{motor_id:03X} 속도 명령: {velocity:.1f} dps"
        )

    def check_motor_current_safety(self, motor_id: int, current: float, temperature: float) -> str:
        """
        모터 전류 안전성 검사 및 보호 조치

        Args:
            motor_id: 모터 ID
            current: 현재 전류 (A)
            temperature: 온도 (°C)

        Returns:
            'normal' / 'warning' / 'danger' / 'emergency'
        """
        if motor_id not in self.motor_current_limits:
            return 'normal'

        limits = self.motor_current_limits[motor_id]
        state = self.motor_protection_state[motor_id]
        abs_current = abs(current)
        current_time = time.time()

        # 상태 업데이트
        state['current'] = abs_current
        state['temperature'] = temperature

        # 단계 판정
        if abs_current < limits['warning']:
            level = 'normal'
            state['high_current_count'] = 0
            state['emergency_count'] = 0

            # 보호 모드 해제 조건: 전류가 정상으로 돌아오고 5초 경과
            if state['protection_active'] and (current_time - state['last_warning_time']) > 5.0:
                self.debug_logger.info(
                    f"✅ [보호해제] 모터 0x{motor_id:03X}({limits['name']}): "
                    f"전류 {abs_current:.2f}A (정상 범위)"
                )
                self.debug_log_file_handler.flush()
                state['protection_active'] = False
                state['speed_limited'] = False
                state['stopped'] = False

        elif abs_current < limits['danger']:
            level = 'warning'
            state['high_current_count'] += 1
            state['emergency_count'] = 0

        elif abs_current < limits['emergency']:
            level = 'danger'
            state['high_current_count'] += 1
            state['emergency_count'] = 0

        else:
            level = 'emergency'
            state['high_current_count'] += 1
            state['emergency_count'] += 1

        # 단계별 조치
        if level == 'warning':
            # 5회 이상 연속 경고 시 로그 및 속도 제한 준비
            if state['high_current_count'] >= 5:
                if current_time - state['last_warning_time'] > 3.0:
                    self.debug_logger.warning(
                        f"⚠️ [전류경고] 모터 0x{motor_id:03X}({limits['name']}): "
                        f"{abs_current:.2f}A (정격: {limits['rated']:.1f}A, 온도: {temperature}°C)"
                    )
                    self.debug_log_file_handler.flush()
                    state['last_warning_time'] = current_time
                    state['protection_active'] = True

        elif level == 'danger':
            # 3회 이상 연속 위험 → 경고만 (자동 제한은 나중에)
            if state['high_current_count'] >= 3:
                if current_time - state['last_warning_time'] > 2.0:
                    self.debug_logger.error(
                        f"🔴 [전류위험] 모터 0x{motor_id:03X}({limits['name']}): "
                        f"{abs_current:.2f}A (위험: {limits['danger']:.1f}A, 온도: {temperature}°C) "
                        f"- 부하 감소 권장!"
                    )
                    self.debug_log_file_handler.flush()
                    state['protection_active'] = True
                    state['last_warning_time'] = current_time

                # 에러 상태 읽기
                self.read_motor_error(motor_id)

        elif level == 'emergency':
            # 2회 이상 연속 긴급 → 긴급 경고만 (자동 정지는 나중에)
            if state['emergency_count'] >= 2:
                if current_time - state['last_warning_time'] > 1.0:
                    self.debug_logger.critical(
                        f"⛔ [긴급] 모터 0x{motor_id:03X}({limits['name']}): "
                        f"{abs_current:.2f}A (한계: {limits['emergency']:.1f}A, 온도: {temperature}°C) "
                        f"- 모터 손상 위험! 즉시 확인 필요!"
                    )
                    self.debug_log_file_handler.flush()
                    state['protection_active'] = True
                    state['last_warning_time'] = current_time

                # 에러 상태 읽기
                self.read_motor_error(motor_id)

        return level

    def apply_current_protection_action(self, motor_id: int, level: str):
        """보호 단계에 따른 즉시 조치 (우선 긴급 정지 위주)"""
        if motor_id not in self.motor_protection_state:
            return

        state = self.motor_protection_state[motor_id]
        limits = self.motor_current_limits[motor_id]

        # 긴급 단계에서 2회 연속 감지 시 정지 명령 전송
        if level == 'emergency' and state['emergency_count'] >= 2 and not state['stopped']:
            self.debug_logger.critical(
                f"⛔ [긴급정지] 모터 0x{motor_id:03X}({limits['name']}): "
                f"전류 {state['current']:.2f}A, 온도 {state['temperature']}°C"
            )
            self.debug_log_file_handler.flush()
            self.send_motor_stop(motor_id)
            state['stopped'] = True

        # 향후 speed_limited 동작은 실제 테스트 후 활성화 예정

    def send_motor_stop(self, motor_id: int):
        """모터 긴급 정지 (0xA2 속도 제어로 속도 0 전송)"""
        try:
            command_data = struct.pack('<B', 0xA2)  # 속도 제어 명령
            command_data += struct.pack('<i', 0)  # 속도 = 0
            command_data += b'\x00\x00\x00'  # 패딩

            self.can_manager.send_frame(motor_id, command_data)
            self.debug_logger.info(f"🛑 모터 0x{motor_id:03X} 정지 명령 전송")
            self.debug_log_file_handler.flush()

            # 이동 상태 업데이트
            if motor_id in self.motor_states:
                self.motor_states[motor_id]['is_moving'] = False

        except Exception as e:
            self.get_logger().error(f"모터 0x{motor_id:03X} 정지 실패: {e}")

    def get_safe_speed_limit(self, motor_id: int, requested_speed: int) -> int:
        """
        모터 보호 상태에 따른 안전 속도 제한 적용

        Args:
            motor_id: 모터 ID
            requested_speed: 요청된 속도 (dps)

        Returns:
            조정된 안전 속도 (dps)
        """
        if motor_id not in self.motor_protection_state:
            return requested_speed

        state = self.motor_protection_state[motor_id]
        limits = self.motor_current_limits[motor_id]

        # 긴급 정지 상태면 속도 0
        if state['stopped']:
            return 0

        # 속도 제한 상태면 50% 제한
        if state['speed_limited']:
            limited_speed = int(requested_speed * 0.5)
            max_allowed = limits.get('speed_limit', 200)
            return min(limited_speed, max_allowed // 2)

        # 정상 상태
        return requested_speed

    def send_position_command(self, motor_id: int, target_position: float, custom_speed: Optional[int] = None):
        """
        위치 명령 전송 (0xA4 Absolute Position Control 사용)
        절대 위치로 이동하여 누적 오차 방지 및 전원 재시작 후에도 정확한 위치 제어
        
        Args:
            motor_id: 모터 ID
            target_position: 목표 위치 (degree)
            custom_speed: 사용자 지정 속도 (dps), None이면 모터별 기본값 사용
        """
        # 현재 위치 가져오기
        current_position = self.motor_states[motor_id]['position']

        # 로그: 위치 명령 계산 (DEBUG 레벨)
        self.get_logger().debug(
            f'Calc: 0x{motor_id:03X}: 현재={current_position:.2f}°, 목표={target_position:.2f}°'
        )

        # 이동 거리가 너무 작으면 무시 (노이즈 방지)
        # 0x143 횡이동 모터는 더 엄격한 기준 적용 (떨림 방지)
        min_distance = 0.05 if motor_id == 0x143 else 0.1
        position_diff = abs(target_position - current_position)
        if position_diff < min_distance:
            self.get_logger().debug(
                f"⏭️  모터 0x{motor_id:03X} 이동 거리가 너무 작아 무시: {position_diff:.2f}도"
            )
            self.motor_states[motor_id]['is_moving'] = False
            return

        # 0xA4 명령 형식:
        # DATA[0] = 0xA4 (명령 바이트)
        # DATA[1] = 0x00 (NULL)
        # DATA[2:3] = maxSpeed (uint16_t, dps)
        # DATA[4:7] = angleControl (int32_t, 0.01 degree/LSB) - 절대 위치

        # 속도 설정 (사용자 지정 또는 모터별 기본값)
        if custom_speed is not None:
            max_speed_dps = custom_speed
        elif motor_id in (0x141, 0x142):
            # 주행 모터 가감속 성능 확보: 내부 프로파일 상한을 높여 더 빠르게 목표 속도에 도달하도록 허용
            max_speed_dps = 800  # 기존 400→800 dps
        elif motor_id == 0x143:  # 횡이동 모터 (과부하 방지를 위해 속도 낮춤)
            max_speed_dps = 200  # 횡이동 속도 200 dps (과부하 방지)
        elif motor_id == 0x147:  # Yaw 회전 모터
            max_speed_dps = 134  # Yaw 회전 속도 134 dps (67 dps의 2배)
        elif motor_id == 0x146:  # Z축 모터
            max_speed_dps = 150  # Z축 속도 150 dps (75 dps의 2배)
        elif motor_id == 0x144:  # X축 모터
            max_speed_dps = 400  # X축 속도 400 dps (200 dps의 2배)
        elif motor_id == 0x145:  # Y축 모터
            max_speed_dps = 400  # Y축 속도 400 dps (200 dps의 2배)
        else:
            max_speed_dps = 400  # 기본 속도 400 dps

        # 보호 상태 기반 안전 속도 제한 적용
        max_speed_dps = self.get_safe_speed_limit(motor_id, int(max_speed_dps))

        # 절대 각도를 프로토콜 단위로 변환 (0.01 degree/LSB)
        # 주의: 명령 시 부호를 반전하여 보냄 → 수신 시에도 반전 처리
        angle_control = int(-target_position * 100)

        # int32로 변환 (부호 있는 32비트 정수)
        command_data = struct.pack(
            '<BBHi',
            CommandType.SET_MOTOR_POSITION,  # 0xA4
            0x00,  # NULL
            max_speed_dps,  # 최대 속도 (uint16_t)
            angle_control,  # 절대 각도 제어 (int32_t, 0.01도 단위, 부호 반전)
        )

        # 로그: CAN 명령 전송 (중요한 위치 제어만 INFO)
        self.get_logger().info(
            f'🚀 Motor 0x{motor_id:03X} 위치 제어 명령 전송: 목표={target_position:.1f}°, 속도={max_speed_dps}dps, 현재={current_position:.1f}°'
        )

        self.can_manager.send_frame(motor_id, command_data)

        # 모터 상태 업데이트
        self.motor_states[motor_id]['is_moving'] = True
        self.target_positions[motor_id] = target_position

    def write_acceleration_limits(self, motor_id: int, accel_dpss: int):
        """Set acceleration/deceleration limits via 0x43 (range 100-60000 dps/s)."""
        accel_clamped = max(100, min(60000, int(accel_dpss)))
        for index in (0x00, 0x01, 0x02, 0x03):
            data = bytearray(8)
            data[0] = 0x43  # Write Acceleration to RAM+ROM
            data[1] = index  # 0: pos accel, 1: pos decel, 2: vel accel, 3: vel decel
            data[4:8] = accel_clamped.to_bytes(4, byteorder='little', signed=True)
            sent = self.can_manager.send_frame(motor_id, data)
            if sent:
                self.get_logger().info(
                    f"    ✓ 0x{motor_id:03X} accel idx {index:#04x} set to {accel_clamped} dps/s"
                )
            else:
                self.get_logger().warning(
                    f"    ❌ 0x{motor_id:03X} accel idx {index:#04x} set failed"
                )

    def apply_drive_accel_limits(self, accel_dpss: int = 20000):
        """Raise drive motors' accel/decel limits to improve step response."""
        for mid in (self.left_motor_id, self.right_motor_id):
            self.write_acceleration_limits(mid, accel_dpss)
    
    def wait_for_position_reached(self, motor_id: int, target_position: float, max_speed_dps: int, timeout: float = 30.0):
        """
        모터가 목표 위치에 도달할 때까지 대기

        Args:
            motor_id: 모터 ID
            target_position: 목표 위치 (도)
            max_speed_dps: 최대 속도 (dps)
            timeout: 타임아웃 (초, 기본 30초)
        """
        start_time = time.time()
        position_tolerance = 5.0  # 위치 허용 오차 (도)

        # 예상 도달 시간 계산 (여유 시간 포함)
        current_position = self.motor_states[motor_id]['position']
        distance = abs(target_position - current_position)
        estimated_time = (distance / max_speed_dps) * 1.5  # 1.5배 여유
        wait_time = min(estimated_time, timeout)

        self.get_logger().info(
            f"⏳ 0x{motor_id:03X} 위치 도달 대기 중... (예상: {estimated_time:.1f}초, 최대: {wait_time:.1f}초)"
        )

        # 대기 루프
        check_interval = 0.1  # 100ms마다 확인
        last_check_time = start_time

        while (time.time() - start_time) < wait_time:
            # 주기적으로 위치 읽기 (100ms 간격)
            if (time.time() - last_check_time) >= check_interval:
                # 멀티턴 각도 읽기 (0x92)
                multi_turn_cmd = self.protocol.create_system_command(CommandType.READ_MULTI_TURN_ANGLE)
                self.can_manager.send_frame(motor_id, multi_turn_cmd)
                last_check_time = time.time()
                time.sleep(0.02)  # CAN 응답 대기

                # 현재 위치 확인
                current_pos = self.motor_states[motor_id]['position']
                position_error = abs(target_position - current_pos)

                # 목표 위치 도달 확인
                if position_error < position_tolerance:
                    elapsed = time.time() - start_time
                    self.get_logger().info(
                        f"✅ 0x{motor_id:03X} 목표 도달: {current_pos:.1f}° (목표: {target_position:.1f}°, "
                        f"오차: {position_error:.1f}°, 소요: {elapsed:.2f}초)"
                    )
                    self.motor_states[motor_id]['is_moving'] = False
                    return True

            time.sleep(0.05)  # 50ms 대기

        # 타임아웃
        final_position = self.motor_states[motor_id]['position']
        final_error = abs(target_position - final_position)
        self.get_logger().warning(
            f"WARNING: 0x{motor_id:03X} 위치 도달 타임아웃: 현재={final_position:.1f}°, "
            f"목표={target_position:.1f}°, 오차={final_error:.1f}°"
        )
        self.motor_states[motor_id]['is_moving'] = False
        return False

    def send_stop_command(self, motor_id: int):
        """정지 명령 전송"""
        stop_cmd = self.protocol.create_system_command(CommandType.MOTOR_STOP)
        self.can_manager.send_frame(motor_id, stop_cmd)

    def read_motor_status(self):
        """모터 상태 읽기 (순차적 전송으로 CAN 버스 보호)"""
        status_cmd = self.protocol.create_system_command(CommandType.READ_MOTOR_STATUS)

        # 순차적으로 모터 상태 읽기 (CAN 버스 보호)
        for i, motor_id in enumerate(self.motor_ids):
            self.debug_logger.debug(f"0x9C cmd sent: 모터 0x{motor_id:03X}")
            success = self.can_manager.send_frame(motor_id, status_cmd)
            
            if success:
                self.debug_logger.debug(f"Motor 0x{motor_id:03X} 상태 읽기 명령 전송 성공")
            else:
                self.debug_logger.warning(f"❌ 모터 0x{motor_id:03X} 상태 읽기 명령 전송 실패")
            
            # 마지막 모터가 아니면 지연 시간 추가
            if i < len(self.motor_ids) - 1:
                time.sleep(0.05)  # 50ms 지연으로 CAN 버스 보호 강화
    
    def motor_response_callback(self, motor_id: int, data: bytes):
        """모터 응답 콜백"""
        try:
            if len(data) < 1:
                self.debug_logger.warning(f"❌ 모터 0x{motor_id:03X} 응답 데이터 없음")
                return

            command = data[0]

            # RAW 응답은 debug 레벨로 (터미널 로그 방지)
            self.get_logger().debug(
                f"📥 [RAW] 모터 0x{motor_id:03X} 응답: 명령=0x{command:02X}, "
                f"데이터={data.hex().upper()}, 길이={len(data)}"
            )

            if command == CommandType.READ_MULTI_TURN_ANGLE:
                # 0x92 멀티턴 각도 응답 파싱: [cmd][reserved(3)][angle(4)] = 8바이트
                # angle: int32, 0.01도/LSB, 누적 각도
                if len(data) >= 8:
                    # CAN 로그 분석 결과: 멀티턴 각도는 data[4:8]에 위치
                    # 예: 920000004A1F0000 -> [4A, 1F, 00, 00] = 0x00001F4A = 7946 (0.01도) = 79.46도
                    angle_raw = struct.unpack('<i', data[4:8])[0]  # int32, 0.01도 단위
                    angle_degrees = angle_raw * 0.01  # 도 단위로 변환

                    # 부호 반전: 명령 시 부호를 반전했으므로 응답도 반전해야 함
                    angle_degrees = -angle_degrees

                    # 이전 위치 저장 (변화량 계산용)
                    previous_position = self.motor_states[motor_id].get('position', angle_degrees)
                    position_delta = angle_degrees - previous_position

                    # 상태 업데이트
                    self.motor_states[motor_id]['position'] = float(angle_degrees)
                    self.current_positions[motor_id] = float(angle_degrees)

                    # 0x141, 0x142 위치 정보 발행 (S20 모드용)
                    if motor_id in [self.left_motor_id, self.right_motor_id]:
                        if motor_id in self.motor_position_publishers:
                            try:
                                position_msg = Float32()
                                position_msg.data = float(angle_degrees)
                                self.motor_position_publishers[motor_id].publish(position_msg)
                            except Exception as pub_error:
                                # Publisher context가 invalid한 경우 (노드 종료 중) 무시
                                if "context is invalid" in str(pub_error) or "publisher" in str(pub_error).lower():
                                    self.debug_logger.debug(f"0x{motor_id:03X} 위치 발행 실패 (노드 종료 중): {pub_error}")
                                else:
                                    self.get_logger().warning(f"0x{motor_id:03X} 위치 발행 오류: {pub_error}")

                    # 0x92 응답은 debug 레벨로 (터미널 로그 방지)
                    self.get_logger().debug(
                        f"모터 0x{motor_id:03X} 0x92 멀티턴: 위치={angle_degrees:.1f}°, RAW={angle_raw}"
                    )
                else:
                    self.get_logger().warning(f"모터 0x{motor_id:03X} 0x92 응답 데이터 길이 부족: {len(data)} bytes")
            elif command == CommandType.READ_MOTOR_STATUS:
                # 상태 응답 파싱: [cmd][temp(1)][current(2)][speed(2)][angle(2)] = 8바이트
                if len(data) >= 8:
                    temperature = struct.unpack('<b', data[1:2])[0]  # 온도 (°C)
                    current = struct.unpack('<h', data[2:4])[0] * 0.01  # 전류 (A)
                    speed = struct.unpack('<h', data[4:6])[0]  # 속도 (dps)
                    angle = struct.unpack('<h', data[6:8])[0]  # 각도 (도, -180~+180)

                    # 0x9C는 단일 회전 각도이므로 위치 업데이트 안 함
                    self.motor_states[motor_id]['velocity'] = float(speed)
                    self.motor_states[motor_id]['torque'] = current

                    # 전류 보호 체크
                    safety_level = self.check_motor_current_safety(motor_id, current, temperature)
                    self.apply_current_protection_action(motor_id, safety_level)

                    # 0x9C 상태 응답은 모두 debug 레벨로 (터미널 로그 방지)
                    self.get_logger().debug(
                        f"모터 0x{motor_id:03X} 0x9C 상태: 단일각도={angle}°, 속도={speed} dps, "
                        f"전류={current:.2f}A, 온도={temperature}°C"
                    )
                else:
                    self.get_logger().warning(f"모터 0x{motor_id:03X} 응답 데이터 길이 부족: {len(data)} bytes")
            elif command == CommandType.SET_MOTOR_POSITION:
                # 0xA4 명령 응답 파싱: [cmd][temp(1)][current(2)][speed(2)][angle(2)] = 8바이트
                # 주의: angle은 단일 회전 각도(-180~+180)이므로 절대 위치 업데이트에 사용하지 않음!
                if len(data) >= 8:
                    temperature = struct.unpack('<b', data[1:2])[0]  # 온도 (°C)
                    current = struct.unpack('<h', data[2:4])[0] * 0.01  # 전류 (A)
                    speed = struct.unpack('<h', data[4:6])[0]  # 속도 (dps)
                    angle = struct.unpack('<h', data[6:8])[0]  # 단일 회전 각도 (도, -180~+180)

                    # 속도와 전류만 업데이트 (위치는 0x92 멀티턴 응답으로 업데이트)
                    # 0x141, 0x142 위치 정보 발행 (S20 모드용)
                    if motor_id in [self.left_motor_id, self.right_motor_id]:
                        if motor_id in self.motor_position_publishers:
                            try:
                                position_msg = Float32()
                                position_msg.data = self.motor_states[motor_id]['position']
                                self.motor_position_publishers[motor_id].publish(position_msg)
                            except Exception as pub_error:
                                # Publisher context가 invalid한 경우 (노드 종료 중) 무시
                                if "context is invalid" in str(pub_error) or "publisher" in str(pub_error).lower():
                                    self.debug_logger.debug(f"0x{motor_id:03X} 위치 발행 실패 (노드 종료 중): {pub_error}")
                                else:
                                    self.get_logger().warning(f"0x{motor_id:03X} 위치 발행 오류: {pub_error}")
                    self.motor_states[motor_id]['velocity'] = float(speed)
                    self.motor_states[motor_id]['torque'] = current

                    # 전류 보호 체크
                    safety_level = self.check_motor_current_safety(motor_id, current, temperature)
                    self.apply_current_protection_action(motor_id, safety_level)

                    # 0xA4 응답은 debug 레벨로 (터미널 로그 방지)
                    self.get_logger().debug(
                        f"모터 0x{motor_id:03X} 0xA4 응답: 속도={speed} dps, "
                        f"전류={current:.2f}A, 온도={temperature}°C, 단일각도={angle}°"
                    )
            elif command == CommandType.SET_MOTOR_INCREMENTAL_POSITION:
                # 0xA8 명령 응답 파싱: [cmd][temp(1)][current(2)][speed(2)][angle(2)] = 8바이트
                # 주의: angle은 단일 회전 각도(-180~+180)이므로 절대 위치 업데이트에 사용하지 않음!
                if len(data) >= 8:
                    temperature = struct.unpack('<b', data[1:2])[0]  # 온도 (°C)
                    current = struct.unpack('<h', data[2:4])[0] * 0.01  # 전류 (A)
                    speed = struct.unpack('<h', data[4:6])[0]  # 속도 (dps)
                    angle = struct.unpack('<h', data[6:8])[0]  # 단일 회전 각도 (도, -180~+180)

                    # 속도와 전류만 업데이트 (위치는 업데이트하지 않음!)
                    self.motor_states[motor_id]['velocity'] = float(speed)
                    self.motor_states[motor_id]['torque'] = current

                    # 전류 보호 체크
                    safety_level = self.check_motor_current_safety(motor_id, current, temperature)
                    self.apply_current_protection_action(motor_id, safety_level)

                    # 0xA8 응답도 debug 레벨로 (터미널 로그 방지)
                    self.get_logger().debug(
                        f"모터 0x{motor_id:03X} 0xA8 응답: 속도={speed} dps, "
                        f"전류={current:.2f}A, 온도={temperature}°C, 단일각도={angle}°"
                    )
            elif command in [CommandType.BRAKE_RELEASE, CommandType.MOTOR_ENABLE,
                           CommandType.MOTOR_STOP, CommandType.MOTOR_SHUTDOWN,
                           CommandType.SET_MOTOR_SPEED, CommandType.SET_MOTOR_POSITION]:
                # 이러한 명령들은 단순 확인 응답만 함
                self.get_logger().debug(f"모터 0x{motor_id:03X} 명령 0x{command:02X} 응답 수신")
            elif command == 0x9A:
                # 에러 상태 읽기 응답 (0x9A)
                if len(data) >= 8:
                    error_state = data[7]  # Error byte
                    if error_state != 0:
                        self.get_logger().error(
                            f'MOTOR ERROR 0x{motor_id:03X}: Error State=0x{error_state:02X} '
                            f'(Low Voltage={error_state&0x01}, Overcurrent={error_state&0x02}, '
                            f'Overtemp={error_state&0x04}, Encoder={error_state&0x08}, '
                            f'Overload={error_state&0x10}, Other={error_state&0xE0})'
                        )
                    else:
                        self.get_logger().info(f'모터 0x{motor_id:03X} 에러 없음')
            else:
                self.get_logger().debug(f"모터 0x{motor_id:03X} 알 수 없는 명령 응답: 0x{command:02X}")

        except Exception as e:
            self.get_logger().error(f"모터 0x{motor_id:03X} 응답 파싱 오류: {e}")
    
    def publish_status(self):
        """상태 발행"""
        from std_msgs.msg import Float32

        # JointState 메시지 생성
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'

        joint_state.name = self.joint_names
        joint_state.position = []
        joint_state.velocity = []

        for motor_id in self.motor_ids:
            # 도를 라디안으로 변환
            position_rad = self.motor_states[motor_id]['position'] * 3.14159 / 180.0
            velocity_rad = self.motor_states[motor_id]['velocity'] * 2 * 3.14159 / 60.0  # dps -> rad/s

            joint_state.position.append(position_rad)
            joint_state.velocity.append(velocity_rad)

            # 개별 모터 위치 발행 (도 단위)
            if motor_id in self.motor_position_publishers:
                try:
                    position_msg = Float32()
                    position_msg.data = self.motor_states[motor_id]['position']
                    self.motor_position_publishers[motor_id].publish(position_msg)

                    # 위치 발행 로그는 debug 레벨로 (터미널 로그 방지)
                    self.get_logger().debug(
                        f"위치 발행: 모터 0x{motor_id:03X} -> {self.motor_states[motor_id]['position']:.1f}° "
                        f"(토픽: motor_{hex(motor_id)}_position)"
                    )
                except Exception as pub_error:
                    # Publisher context가 invalid한 경우 (노드 종료 중) 무시
                    if "context is invalid" in str(pub_error) or "publisher" in str(pub_error).lower():
                        self.debug_logger.debug(f"0x{motor_id:03X} 위치 발행 실패 (노드 종료 중): {pub_error}")
                    else:
                        self.get_logger().warning(f"0x{motor_id:03X} 위치 발행 오류: {pub_error}")

        self.joint_state_publisher.publish(joint_state)

        # 모터 상태 발행
        motor_status = Float64MultiArray()
        motor_status.data = []

        for motor_id in self.motor_ids:
            motor_status.data.extend([
                self.motor_states[motor_id]['position'],
                self.motor_states[motor_id]['velocity'],
                self.motor_states[motor_id]['torque'],
                self.motor_states[motor_id]['target_position'],
                1.0 if self.motor_states[motor_id]['is_moving'] else 0.0
            ])

        self.motor_status_publisher.publish(motor_status)
    
    def stop_all_motors(self):
        """모든 모터 정지 (순차적 전송으로 CAN 버스 보호)"""
        self.get_logger().info("⏹ 모든 모터 정지 명령 시작...")
        stop_cmd = self.protocol.create_system_command(CommandType.MOTOR_STOP)

        # 순차적으로 모터 정지 (CAN 버스 보호)
        for i, motor_id in enumerate(self.motor_ids):
            success = self.can_manager.send_frame(motor_id, stop_cmd)
            self.motor_states[motor_id]['is_moving'] = False
            
            if success:
                self.get_logger().debug(f"Motor 0x{motor_id:03X} 정지 명령 전송 성공")
            else:
                self.get_logger().warning(f"❌ 모터 0x{motor_id:03X} 정지 명령 전송 실패")
            
            # 마지막 모터가 아니면 지연 시간 추가
            if i < len(self.motor_ids) - 1:
                time.sleep(0.1)  # 100ms 지연으로 CAN 버스 보호

        self.get_logger().info("✅ 모든 모터 정지 명령 전송 완료")

    def shutdown_all_motors(self):
        """모든 모터 셧다운 (순차적 전송으로 CAN 버스 보호)"""
        self.get_logger().info("🔌 모든 모터 셧다운 명령 시작...")
        shutdown_cmd = self.protocol.create_system_command(CommandType.MOTOR_SHUTDOWN)

        # 순차적으로 모터 셧다운 (CAN 버스 보호)
        for i, motor_id in enumerate(self.motor_ids):
            success = self.can_manager.send_frame(motor_id, shutdown_cmd)
            
            if success:
                self.get_logger().debug(f"Motor 0x{motor_id:03X} 셧다운 명령 전송 성공")
            else:
                self.get_logger().warning(f"❌ 모터 0x{motor_id:03X} 셧다운 명령 전송 실패")
            
            # 마지막 모터가 아니면 지연 시간 추가
            if i < len(self.motor_ids) - 1:
                time.sleep(0.1)  # 100ms 지연으로 CAN 버스 보호

        self.get_logger().info("✅ 모든 모터 셧다운 명령 전송 완료")
    
    # ============================================================
    # CMD_VEL 제어 함수들
    # ============================================================
    
    def cmd_vel_callback(self, msg: Twist):
        """CMD_VEL 콜백 (0x141, 0x142 모터 속도 제어)"""
        # 로그: 수신한 cmd_vel
        self.get_logger().info(
            f'📥 [ROS2] /cmd_vel 수신: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}'
        )
        
        # 선속도와 각속도 제한
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

        # 정지 명령 확인
        if abs(linear_vel) < 0.001 and abs(angular_vel) < 0.001:
            # 속도 0 명령으로 정지 (0xA2)
            self.get_logger().info(f"Motors 0x141, 0x142 stopped")
            self.debug_logger.debug(f"STOP command: 0x141, 0x142 (speed=0)")
            
            # 0x141 속도 0
            self.send_speed_command(self.left_motor_id, 0)
            time.sleep(0.01)
            
            # 0x142 속도 0
            self.send_speed_command(self.right_motor_id, 0)
            
            return

        # 차동 구동 계산
        left_wheel_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_wheel_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # RPM 계산 (rad/s -> RPM)
        left_rpm = (left_wheel_vel / self.wheel_radius) * 60.0 / (2 * 3.14159)
        right_rpm = (right_wheel_vel / self.wheel_radius) * 60.0 / (2 * 3.14159)

        # dps (도/초) 계산
        left_dps = left_rpm * 6.0
        right_dps = right_rpm * 6.0

        # 로그: 반전 전
        # 로그: 계산 과정 (DEBUG 레벨)
        self.get_logger().debug(
            f'Calc: left_dps={left_dps:.1f}, right_dps={right_dps:.1f}'
        )

        # 방향 반전 제거 - 둘 다 같은 방향으로 회전
        # (모터가 같은 방향으로 장착되어 있는 경우)

        # 속도 제어값 변환 (0.01dps/LSB)
        left_speed_control = int(left_dps * 100)
        right_speed_control = int(right_dps * 100)
        
        # 좌우 모터 180도 반대 장착으로 인한 방향 보정:
        # - 0x141(좌)과 0x142(우)는 서로 마주보고 장착됨
        # - 전진: 좌측은 반시계, 우측은 시계 (또는 그 반대)
        # - 차등 구동 계산 후 한쪽만 반전 필요
        # FIX: 오른쪽 모터만 반전 (또는 왼쪽만 반전)
        right_speed_control = -right_speed_control

        # 로그: CAN 명령 전송 전 (DEBUG 레벨)
        self.get_logger().debug(
            f'CAN2: 0x141={left_speed_control}, 0x142={right_speed_control}'
        )

        # 속도 명령 전송 (동기 제어를 위해 간격 최소화)
        self.send_speed_command(self.left_motor_id, left_speed_control)
        # FIX: 동기 제어를 위해 time.sleep 제거 (응답 ID가 다르므로 충돌 없음: 0x241, 0x242)
        # time.sleep(0.01)  # 제거됨
        self.send_speed_command(self.right_motor_id, right_speed_control)
    
    def send_speed_command(self, motor_id: int, speed_control: int, log_stop: bool = False):
        """속도 명령 전송 (0xA2 - Speed Control Command)"""
        data = bytearray(8)
        data[0] = 0xA2  # Speed Control Command

        # 토크 100% (토크 부족 방지)
        data[1] = 0x64  # 100% of rated current

        data[2] = 0x00
        data[3] = 0x00
        data[4] = speed_control & 0xFF
        data[5] = (speed_control >> 8) & 0xFF
        data[6] = (speed_control >> 16) & 0xFF
        data[7] = (speed_control >> 24) & 0xFF

        success = self.can_manager.send_frame(motor_id, data)

        if log_stop and speed_control == 0:
            if success:
                self.get_logger().info(f"    ✓ 0x{motor_id:03X} 속도=0 전송 성공")
            else:
                self.get_logger().warning(f"    ❌ 0x{motor_id:03X} 속도=0 전송 실패")
        elif not success:
            self.get_logger().warning(f"❌ 모터 0x{motor_id:03X} 속도 명령 전송 실패")
    
    def send_speed_command_single(self, motor_id: int, speed_control: int):
        """단일 모터 속도 명령 전송 (0xA2 - Speed Control Command)"""
        data = bytearray(8)
        data[0] = 0xA2  # Speed Control Command

        # 토크 100% (토크 부족 방지)
        data[1] = 0x64  # 100% of rated current

        data[2] = 0x00
        data[3] = 0x00
        data[4] = speed_control & 0xFF
        data[5] = (speed_control >> 8) & 0xFF
        data[6] = (speed_control >> 16) & 0xFF
        data[7] = (speed_control >> 24) & 0xFF

        success = self.can_manager.send_frame(motor_id, data)

        if not success:
            self.get_logger().warning(f"❌ 모터 0x{motor_id:03X} 속도 명령 전송 실패")

    def apply_stop_damping(self, motor_id: int):
        """정지 후 속도=0 전송 및 브레이크 잠금으로 잔류 토크/진동 억제"""
        try:
            # 속도 0 (0xA2)
            self.send_speed_command(motor_id, 0, log_stop=True)

            # 브레이크 잠금 (0x78)
            brake_lock_cmd = self.protocol.create_system_command(CommandType.BRAKE_LOCK)
            sent = self.can_manager.send_frame(motor_id, brake_lock_cmd)
            if sent:
                self.get_logger().info(f"    ✓ 0x{motor_id:03X} 브레이크 잠금 전송")
            else:
                self.get_logger().warning(f"    ❌ 0x{motor_id:03X} 브레이크 잠금 전송 실패")
        except Exception as e:
            self.get_logger().warning(f"정지 감쇠 처리 오류 (0x{motor_id:03X}): {e}")
    
    def left_motor_response_callback(self, can_id: int, data: bytes):
        """왼쪽 모터 (0x141) 응답 콜백"""
        if len(data) >= 8:
            try:
                cmd = data[0]
                
                # 멀티턴 각도 읽기 응답 (0x92) - S20 모드 위치 제어용
                if cmd == 0x92:
                    # motor_response_callback과 동일한 로직 사용
                    self.debug_logger.info(f"🔍 [0x141] 0x92 응답 수신, motor_response_callback 호출")
                    self.motor_response_callback(self.left_motor_id, data)
                    return
                
                # 에러 읽기 응답 (0x9A)
                if cmd == 0x9A:
                    error_code = data[7]
                    error_msgs = {
                        0x00: "정상",
                        0x01: "과전류",
                        0x02: "과전압",
                        0x03: "엔코더 에러",
                        0x04: "과열",
                        0x08: "저전압",
                        0x10: "홀센서 에러",
                        0x20: "과부하"
                    }
                    error_msg = error_msgs.get(error_code, f"알 수 없는 에러 (0x{error_code:02X})")
                    self.get_logger().error(f"🔴 0x141 에러: {error_msg} (코드: 0x{error_code:02X})")
                    self.debug_logger.error(f"🔴 0x141 에러: {error_msg}, RAW={data.hex().upper()}")
                    return
                
                # 속도 명령 응답 감지 (0xA2)
                if cmd == 0xA2:
                    temperature = int.from_bytes([data[1]], byteorder='little', signed=True)
                    torque_raw = int.from_bytes(data[2:4], byteorder='little', signed=True)
                    speed_raw = int.from_bytes(data[4:6], byteorder='little', signed=True)
                    torque = torque_raw * 0.01
                    speed = speed_raw

                    # ✨ 전류 안전성 모니터링
                    safety_level = self.check_motor_current_safety(self.left_motor_id, torque, temperature)
                    self.apply_current_protection_action(self.left_motor_id, safety_level)

                    # 상세 로그 (debug 레벨로 변경하여 스팸 방지)
                    self.debug_logger.debug(f"✓ 0x141 속도 응답: speed={speed:.1f}dps, torque={torque:.2f}A, level={safety_level}, RAW={data.hex().upper()}")
                
                temperature = int.from_bytes([data[1]], byteorder='little', signed=True)
                torque_raw = int.from_bytes(data[2:4], byteorder='little', signed=True)
                speed_raw = int.from_bytes(data[4:6], byteorder='little', signed=True)
                
                torque = torque_raw * 0.01  # A
                speed = speed_raw  # dps
                
                self.motor_states[self.left_motor_id]['velocity'] = speed
                self.motor_states[self.left_motor_id]['torque'] = torque
                
                # RPM 발행 (노드 종료 중이면 무시)
                try:
                    from std_msgs.msg import Float32
                    rpm_msg = Float32()
                    rpm_msg.data = speed / 6.0  # dps -> RPM
                    self.left_rpm_publisher.publish(rpm_msg)
                except Exception as pub_error:
                    # Publisher context가 invalid한 경우 (노드 종료 중) 무시
                    if "context is invalid" in str(pub_error) or "publisher" in str(pub_error).lower():
                        self.debug_logger.debug(f"0x141 RPM 발행 실패 (노드 종료 중): {pub_error}")
                    else:
                        self.get_logger().warning(f"0x141 RPM 발행 오류: {pub_error}")
                
            except Exception as e:
                self.get_logger().error(f"0x141 응답 파싱 오류: {e}")
                self.debug_logger.error(f"0x141 응답 파싱 오류: {e}")
    
    def right_motor_response_callback(self, can_id: int, data: bytes):
        """오른쪽 모터 (0x142) 응답 콜백"""
        if len(data) >= 8:
            try:
                cmd = data[0]
                
                # 멀티턴 각도 읽기 응답 (0x92) - S20 모드 위치 제어용
                if cmd == 0x92:
                    # motor_response_callback과 동일한 로직 사용
                    self.debug_logger.info(f"🔍 [0x142] 0x92 응답 수신, motor_response_callback 호출")
                    self.motor_response_callback(self.right_motor_id, data)
                    return
                
                # 에러 읽기 응답 (0x9A)
                if cmd == 0x9A:
                    error_code = data[7]
                    error_msgs = {
                        0x00: "정상",
                        0x01: "과전류",
                        0x02: "과전압",
                        0x03: "엔코더 에러",
                        0x04: "과열",
                        0x08: "저전압",
                        0x10: "홀센서 에러",
                        0x20: "과부하"
                    }
                    error_msg = error_msgs.get(error_code, f"알 수 없는 에러 (0x{error_code:02X})")
                    self.get_logger().error(f"🔴 0x142 에러: {error_msg} (코드: 0x{error_code:02X})")
                    self.debug_logger.error(f"🔴 0x142 에러: {error_msg}, RAW={data.hex().upper()}")
                    return
                
                # 속도 명령 응답 감지 (0xA2) - 0x142 특별 추적
                if cmd == 0xA2:
                    temperature = int.from_bytes([data[1]], byteorder='little', signed=True)
                    torque_raw = int.from_bytes(data[2:4], byteorder='little', signed=True)
                    speed_raw = int.from_bytes(data[4:6], byteorder='little', signed=True)
                    torque = torque_raw * 0.01
                    speed = speed_raw
                    
                    # 비정상 값 감지 시 에러 읽기
                    if abs(torque) > 10.0 or temperature > 80:
                        self.get_logger().error(f"WARNING: 0x142 비정상: torque={torque:.2f}A, temp={temperature}°C")
                        self.read_motor_error(self.right_motor_id)
                    
                    safety_level = self.check_motor_current_safety(self.right_motor_id, torque, temperature)
                    self.apply_current_protection_action(self.right_motor_id, safety_level)

                    self.get_logger().warning(f"★ 0x142 속도 응답: speed={speed:.1f}dps, torque={torque:.2f}A")
                    self.debug_logger.warning(f"★ 0x142 속도 응답: speed={speed:.1f}dps, torque={torque:.2f}A, level={safety_level}, RAW={data.hex().upper()}")
                    
                    # 속도가 0이 아니면 경고
                    if abs(speed) > 1.0:
                        self.get_logger().warning(f"⚠⚠⚠ 0x142 정지 실패! 여전히 회전 중: {speed:.1f}dps")
                        self.debug_logger.warning(f"⚠⚠⚠ 0x142 정지 실패! 여전히 회전 중: {speed:.1f}dps")
                
                temperature = int.from_bytes([data[1]], byteorder='little', signed=True)
                torque_raw = int.from_bytes(data[2:4], byteorder='little', signed=True)
                speed_raw = int.from_bytes(data[4:6], byteorder='little', signed=True)
                
                torque = torque_raw * 0.01  # A
                speed = speed_raw  # dps
                
                self.motor_states[self.right_motor_id]['velocity'] = speed
                self.motor_states[self.right_motor_id]['torque'] = torque
                
                # RPM 발행 (노드 종료 중이면 무시)
                try:
                    from std_msgs.msg import Float32
                    rpm_msg = Float32()
                    rpm_msg.data = speed / 6.0  # dps -> RPM
                    self.right_rpm_publisher.publish(rpm_msg)
                except Exception as pub_error:
                    # Publisher context가 invalid한 경우 (노드 종료 중) 무시
                    if "context is invalid" in str(pub_error) or "publisher" in str(pub_error).lower():
                        self.debug_logger.debug(f"0x142 RPM 발행 실패 (노드 종료 중): {pub_error}")
                    else:
                        self.get_logger().warning(f"0x142 RPM 발행 오류: {pub_error}")
                
            except Exception as e:
                self.get_logger().error(f"0x142 응답 파싱 오류: {e}")
                self.debug_logger.error(f"0x142 응답 파싱 오류: {e}")
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        # 모든 모터 정지 (위치제어 + 속도제어)
        self.get_logger().info("🛑 모든 모터 정지 중...")
        
        # CMD_VEL 모터 정지
        self.send_speed_command(self.left_motor_id, 0)
        time.sleep(0.05)
        self.send_speed_command(self.right_motor_id, 0)
        time.sleep(0.1)
    
    def left_wheel_position_callback(self, msg: Float64MultiArray):
        """좌측 주행 모터(0x141) 위치 제어 콜백 (S20 모드용) - 동기화 지원"""
        if len(msg.data) >= 1:
            target_position = msg.data[0]  # 목표 위치 (도)
            speed = msg.data[1] if len(msg.data) >= 2 else 704.0   # 속도 (dps, 기본값 704 ≈ 117rpm)

            current_time = time.time()
            self.drive_last_left_command_time = current_time

            self.debug_logger.info(f'📍 [동기화] 모터 0x141 위치 명령: 목표={target_position:.1f}°, 속도={speed:.1f}dps')
            self.debug_log_file_handler.flush()
            self.target_positions[self.left_motor_id] = target_position
            self.motor_states[self.left_motor_id]['is_moving'] = True
            self.motor_states[self.left_motor_id]['position_command_time'] = current_time

            # 양쪽 명령이 짧은 시간 내에 들어왔는지 확인 (동기화 모드 활성화)
            time_diff = abs(self.drive_last_left_command_time - self.drive_last_right_command_time)
            if time_diff < self.drive_command_sync_window:
                self.drive_sync_mode = True
                self.drive_left_near_target = False
                self.drive_right_near_target = False
                self.drive_both_commands_received = True
                self.debug_logger.info(f'🔄 [동기화] 주행 모터 동기화 모드 활성화 (시간차: {time_diff:.3f}초)')
                self.debug_log_file_handler.flush()
            else:
                self.drive_sync_mode = False
                self.debug_logger.info(f'⚠️  [동기화] 단독 명령 모드 (시간차: {time_diff:.3f}초 > {self.drive_command_sync_window}초)')
                self.debug_log_file_handler.flush()

            self.send_position_command(self.left_motor_id, target_position, int(speed))

    def right_wheel_position_callback(self, msg: Float64MultiArray):
        """우측 주행 모터(0x142) 위치 제어 콜백 (S20 모드용) - 동기화 지원"""
        if len(msg.data) >= 1:
            target_position = msg.data[0]  # 목표 위치 (도)
            speed = msg.data[1] if len(msg.data) >= 2 else 704.0   # 속도 (dps, 기본값 704 ≈ 117rpm)

            current_time = time.time()
            self.drive_last_right_command_time = current_time

            self.debug_logger.info(f'📍 [동기화] 모터 0x142 위치 명령: 목표={target_position:.1f}°, 속도={speed:.1f}dps')
            self.debug_log_file_handler.flush()
            self.target_positions[self.right_motor_id] = target_position
            self.motor_states[self.right_motor_id]['is_moving'] = True
            self.motor_states[self.right_motor_id]['position_command_time'] = current_time

            # 양쪽 명령이 짧은 시간 내에 들어왔는지 확인 (동기화 모드 활성화)
            time_diff = abs(self.drive_last_left_command_time - self.drive_last_right_command_time)
            if time_diff < self.drive_command_sync_window:
                self.drive_sync_mode = True
                self.drive_left_near_target = False
                self.drive_right_near_target = False
                self.drive_both_commands_received = True
                self.debug_logger.info(f'🔄 [동기화] 주행 모터 동기화 모드 활성화 (시간차: {time_diff:.3f}초)')
                self.debug_log_file_handler.flush()
            else:
                self.drive_sync_mode = False
                self.debug_logger.info(f'⚠️  [동기화] 단독 명령 모드 (시간차: {time_diff:.3f}초 > {self.drive_command_sync_window}초)')
                self.debug_log_file_handler.flush()

            self.send_position_command(self.right_motor_id, target_position, int(speed))

    def destroy_node(self):
        """노드 종료"""
        # 위치제어 모터 정지
        self.stop_all_motors()
        time.sleep(0.1)
        self.shutdown_all_motors()

        self.can_manager.disconnect()
        super().destroy_node()


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    node = PositionControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
