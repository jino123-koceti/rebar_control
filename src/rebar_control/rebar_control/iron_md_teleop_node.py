#!/usr/bin/env python3
"""
철근 결속 로봇 텔레옵 노드 (Iron-MD CAN 조종기)
Iron-MD 무선 조종기 CAN 메시지를 수신하여 로봇 제어

CAN 통신:
- 0x1E4 (484): 조이스틱 아날로그 데이터 (50ms)
- 0x2E4 (740): 스위치 및 상태 (50ms)
- 0x764 (1892): Heartbeat (300ms)

조종기 매핑 (실제 리모콘 조작 기준):
[아날로그 조이스틱 - 연속 제어]
- Joystick_3 (AN3): 하부체 전후진 (0x141, 0x142) - linear 변수 사용 (좌우 모터가 180도 반대로 설치되어있음)
  - AN3- -> 전진, AN3+ -> 후진
- Joystick_4 (AN4): 하부체 좌우회전 (0x141, 0x142) - angular 변수 사용 (좌우 모터가 180도 반대로 설치되어 있어서)
  - AN4+ -> CCW, AN4- -> CW 로 로봇이 움직임
- Joystick_1 (AN1): 상부체 X축 속도 제어 (0x144)
- Joystick_2 (AN2): 상부체 Y축 속도 제어 (0x145)

[3단 스위치 - 토글형]
- S19-S20: 모드 선택 (S19=Remote, S20=Automatic)
- S17-S18: 
  * S19(Remote 모드): 횡이동 (S17=+360도, S18=-360도) 0x143 (+-50mm씩 이동함)
  * S20(Auto 모드): 그리퍼 제어 (S17=열림 g 5 600, S18=닫힘 g 5 -600)
- S21-S22: 
  * S19(Remote 모드): 작업 시퀀스 (S21=하강→닫힘, S22=트리거→상승→열림)
  * S20(Auto 모드): 횡이동 회전 (S21=+2880도, S22=-2880도) 0x143 (+-400mm씩 이동함)
- S23-S24: 
  * S19(Remote 모드): Yaw 회전 (S23=+5도, S24=-5도) 0x147
  * S20(Auto 모드): S23=XYZ호밍, S24=X +1181.85°, Y -189.4° 상대이동

[일반 스위치]
- S13: 브레이크 해제/잠금
- S14: 위치 리셋
- Emergency_Stop: 비상 정지
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64MultiArray, Int32, Bool
from std_srvs.srv import Trigger
import can
import struct
import threading
import math
import time


class IronMDTeleopNode(Node):
    """Iron-MD CAN 조종기로 로봇 단동 제어"""
    
    def __init__(self):
        super().__init__('iron_md_teleop')
        
        # 파일 로그 설정 (INFO 이상 모든 로그 기록)
        import logging
        import os
        self.file_logger = logging.getLogger('iron_md_teleop_file')
        self.file_logger.setLevel(logging.INFO)  # INFO 이상 기록
        # 기존 핸들러 제거 (중복 방지)
        self.file_logger.handlers.clear()
        # 파일 핸들러 추가
        log_file = '/tmp/unified_control_debug.log'
        fh = logging.FileHandler(log_file, mode='a', encoding='utf-8', delay=False)
        fh.setLevel(logging.INFO)  # INFO 이상 기록
        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        fh.setFormatter(formatter)
        self.file_logger.addHandler(fh)
        self.file_log_file_handler = fh  # 나중에 flush를 위해 저장
        self.file_logger.info("="*60)
        self.file_logger.info("Iron-MD Teleop Node 시작")
        self.file_logger.info("="*60)
        self.file_log_file_handler.flush()  # 즉시 파일에 기록
        
        # 파라미터 선언
        self.declare_parameter('can_interface', 'can3')  # Iron-MD 조종기용 can3
        self.declare_parameter('can_baudrate', 250000)
        # 최대 속도 제한 (Speed PID 게인 동기화 후 10배 증가, 각속도는 2배로 조정)
        self.declare_parameter('max_linear_speed', 3.0)  # 0.3 → 3.0 m/s (10배)
        self.declare_parameter('max_angular_speed', 2.0)  # 1.0 → 10.0 → 2.0 rad/s (10배→5분의1로 감소)
        self.declare_parameter('xyz_step_size', 10.0)  # degree, 10도 per command (조이스틱 연속 제어)
        self.declare_parameter('lateral_move_distance', 50.0)  # degree, 50도 per step (횡이동)
        self.declare_parameter('z_work_distance', 100.0)  # degree, 100도 for work sequence
        self.declare_parameter('yaw_rotation_angle', 5.0)  # degrees, 5도 (fine control)
        self.declare_parameter('trigger_duration', 0.5)  # seconds
        self.declare_parameter('gripper_open_position', 0)  # 그리퍼 열림
        self.declare_parameter('gripper_close_position', 2000)  # 그리퍼 닫힘
        self.declare_parameter('joystick_center', 127)  # 중립값
        self.declare_parameter('joystick_deadzone', 20)  # 데드존
        self.declare_parameter('debug_mode', False)  # 디버그 모드 (터미널 출력 상세화)
        
        self.can_interface = self.get_parameter('can_interface').value
        self.can_baudrate = self.get_parameter('can_baudrate').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.xyz_step = self.get_parameter('xyz_step_size').value
        self.lateral_distance = self.get_parameter('lateral_move_distance').value
        self.z_work_distance = self.get_parameter('z_work_distance').value
        self.yaw_angle = self.get_parameter('yaw_rotation_angle').value
        self.trigger_duration = self.get_parameter('trigger_duration').value
        self.gripper_open = self.get_parameter('gripper_open_position').value
        self.gripper_close = self.get_parameter('gripper_close_position').value
        self.joy_center = self.get_parameter('joystick_center').value
        self.joy_deadzone = self.get_parameter('joystick_deadzone').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # ROS2 발행자들
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint1_pub = self.create_publisher(Float64MultiArray, '/joint_1/position', 10)  # 0x143 Lateral (횡이동)
        self.joint2_pub = self.create_publisher(Float32, '/joint_2/speed', 10)  # 0x144 X-axis 속도
        self.joint2_position_pub = self.create_publisher(Float64MultiArray, '/joint_2/position', 10)  # 0x144 X-axis 위치
        self.joint3_pub = self.create_publisher(Float32, '/joint_3/speed', 10)  # 0x145 Y-axis 속도
        self.joint3_position_pub = self.create_publisher(Float64MultiArray, '/joint_3/position', 10)  # 0x145 Y-axis 위치
        self.joint4_pub = self.create_publisher(Float64MultiArray, '/joint_4/position', 10)  # 0x146 Z-axis (상하)
        self.joint5_pub = self.create_publisher(Float64MultiArray, '/joint_5/position', 10)  # 0x147 Yaw (회전)
        # 주행 모터 위치 제어 퍼블리셔 (0x141, 0x142) - S20 모드에서 사용
        self.left_wheel_position_pub = self.create_publisher(Float64MultiArray, '/motor_0x141/position', 10)
        self.right_wheel_position_pub = self.create_publisher(Float64MultiArray, '/motor_0x142/position', 10)
        self.trigger_pub = self.create_publisher(Float32, '/motor_0/vel', 10)
        self.gripper_pos_pub = self.create_publisher(Float32, '/gripper/position', 10)
        self.gripper_cmd_pub = self.create_publisher(Int32, '/gripper/command', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # 조이스틱 및 스위치 상태
        self.joystick_data = {
            'AN1': 127,  # X축 (상부체)
            'AN2': 127,  # Y축 (상부체)
            'AN3': 127,  # 리모콘: 전후진 조작 → 코드: linear (좌우 모터 180도 반대 설치, AN3- = 전진, AN3+ = 후진)
            'AN4': 127,  # 리모콘: 좌우회전 조작 → 코드: angular (좌우 모터 180도 반대 설치, AN4+ = CCW, AN4- = CW)
        }
        
        self.switch_data = {
            'S00': 0, 'S01': 0, 'S02': 0, 'S03': 0,
            'S06': 0, 'S07': 0, 'S08': 0, 'S09': 0,
            'S13': 0,  # 브레이크 해제 버튼
            'S14': 0,  # 드라이브 모터 홈잉
            'S17': 0, 'S18': 0, 'S19': 0, 'S20': 0,
            'S21': 0, 'S22': 0, 'S23': 0, 'S24': 0,
            'Emergency_Stop_Active': 0,
            'Emergency_Stop_Release': 1,
            'TX_Connected': 0,
        }
        
        # 이전 스위치 상태 (엣지 감지)
        self.prev_switches = self.switch_data.copy()
        
        # 현재 위치 (degree 단위)
        self.current_positions = {
            'lateral': 0.0,  # 횡이동 (0x143) - degree
            'x': 0.0,  # X축 (0x144) - degree
            'y': 0.0,  # Y축 (0x145) - degree
            'z': 0.0,  # Z축 (0x146) - degree
            'yaw': 0.0,  # Yaw (0x147) - degree
            'left_wheel': 0.0,  # 좌측 주행 모터 (0x141) - degree
            'right_wheel': 0.0,  # 우측 주행 모터 (0x142) - degree
        }
        default_speed_dps = 704.0
        # 제어 모드
        self.control_mode = 'remote'  # 'remote' or 'automatic'
        
        # 작업 시퀀스 상태
        self.work_sequence_active = False
        
        # 트리거 타이머
        self.trigger_timer = None
        
        # 안전 플래그
        self.emergency_stopped = False

        # 브레이크 상태 (True=해제됨, False=잠김)
        self.brake_released = False
        
        # 마지막 발행 값 (중복 방지)
        self.last_cmd_sent = {'linear': 0.0, 'angular': 0.0}
        
        # 횡이동 명령 간격 제어 (과부하 방지)
        self.last_lateral_command_time = 0.0
        self.lateral_command_min_interval = 0.5  # 최소 0.5초 간격 (과부하 방지)
        self.lateral_target_position = None  # 목표 위치 (이동 중 체크용)
        
        # 제어 루프 타이머 (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # 디버그 출력용 타이머 (1Hz)
        if self.debug_mode:
            self.debug_timer = self.create_timer(1.0, self.print_status)

        # 브레이크 제어 서비스 클라이언트 생성
        self.brake_release_client = self.create_client(Trigger, 'safe_brake_release')
        self.brake_lock_client = self.create_client(Trigger, 'safe_brake_lock')

        # 모터 위치 구독 (0x143 = joint_1, 0x146 = joint_4, 0x147 = joint_5)
        self.motor_position_sub = self.create_subscription(
            Float32,
            '/motor_0x143_position',
            lambda msg: self.motor_position_callback(msg, 'lateral'),
            10
        )
        
        self.motor_z_position_sub = self.create_subscription(
            Float32,
            '/motor_0x146_position',
            lambda msg: self.motor_position_callback(msg, 'z'),
            10
        )
        
        self.motor_yaw_position_sub = self.create_subscription(
            Float32,
            '/motor_0x147_position',
            lambda msg: self.motor_position_callback(msg, 'yaw'),
            10
        )
        
        # XY 스테이지 모터 위치 구독 (0x144 = X축, 0x145 = Y축)
        self.motor_x_position_sub = self.create_subscription(
            Float32,
            '/motor_0x144_position',
            lambda msg: self.motor_position_callback(msg, 'x'),
            10
        )
        
        self.motor_y_position_sub = self.create_subscription(
            Float32,
            '/motor_0x145_position',
            lambda msg: self.motor_position_callback(msg, 'y'),
            10
        )

        # 주행 모터 위치 구독 (0x141, 0x142) - S20 모드에서 사용
        self.motor_0x141_position_sub = self.create_subscription(
            Float32,
            '/motor_0x141_position',
            lambda msg: self.motor_position_callback(msg, 'left_wheel'),
            10
        )
        
        self.motor_0x142_position_sub = self.create_subscription(
            Float32,
            '/motor_0x142_position',
            lambda msg: self.motor_position_callback(msg, 'right_wheel'),
            10
        )

        # EZI-IO 리미트 센서 구독
        self.limit_sensor_in00 = False  # IN00 상태 (Y축 원점 리미트, 0x145)
        self.limit_sensor_in01 = False  # IN01 상태 (Y축 최대 리미트, 0x145)
        self.limit_sensor_in02 = False  # IN02 상태 (X축 홈 리미트, 0x144)
        self.limit_sensor_in03 = False  # IN03 상태 (X축 최대 리미트, 0x144)
        self.limit_sensor_in04 = False  # IN04 상태 (Yaw 홈 리미트, 0x147)
        self.limit_sensor_in05 = False  # IN05 상태 (Z축 상단 리미트)
        self.limit_sensor_in06 = False  # IN06 상태 (Z축 하단 리미트)

        self.limit_sensor_in00_sub = self.create_subscription(
            Bool,
            '/limit_sensors/y_max',  # EZI-IO IN00 (Y축 원점)
            self.limit_sensor_in00_callback,
            10
        )

        self.limit_sensor_in01_sub = self.create_subscription(
            Bool,
            '/limit_sensors/y_min',  # EZI-IO IN01 (Y축 최대)
            self.limit_sensor_in01_callback,
            10
        )

        self.limit_sensor_in02_sub = self.create_subscription(
            Bool,
            '/limit_sensors/x_min',  # EZI-IO IN02 (X축 홈)
            self.limit_sensor_in02_callback,
            10
        )

        self.limit_sensor_in03_sub = self.create_subscription(
            Bool,
            '/limit_sensors/x_max',  # EZI-IO IN03 (X축 최대)
            self.limit_sensor_in03_callback,
            10
        )

        self.limit_sensor_in04_sub = self.create_subscription(
            Bool,
            '/limit_sensors/yaw_min',  # EZI-IO IN04 (Yaw 홈)
            self.limit_sensor_in04_callback,
            10
        )

        self.limit_sensor_in05_sub = self.create_subscription(
            Bool,
            '/limit_sensors/z_min',  # EZI-IO IN05 (Z축 상단)
            self.limit_sensor_in05_callback,
            10
        )

        self.limit_sensor_in06_sub = self.create_subscription(
            Bool,
            '/limit_sensors/z_max',  # EZI-IO IN06 (Z축 하단)
            self.limit_sensor_in06_callback,
            10
        )

        # 모터 목표 도달 알림 구독
        self.motor_goal_reached_sub = self.create_subscription(
            Int32,
            '/motor_goal_reached',
            self.motor_goal_reached_callback,
            10
        )

        # 초기 위치 읽기 완료 플래그
        self.initial_position_read = {'lateral': False, 'z': False, 'yaw': False, 'x': False, 'y': False, 'left_wheel': False, 'right_wheel': False}
        
        # S20 모드에서 AN3 조이스틱 엣지 감지용
        self.an3_prev_value = 127
        self.an3_last_command_time = 0.0
        self.an3_command_min_interval = 0.5  # 최소 명령 간격 (초)
        self.an3_command_active = False  # 명령 실행 중 플래그 (중립 복귀까지 추가 명령 무시)
        
        # Z축 동작 상태
        self.z_moving_to_limit = False  # 상단 리미트까지 이동 중
        self.z_moving_down = False  # 하강 중
        self.z_target_position = 0.0  # 목표 위치
        
        # 작업 시퀀스 상태
        self.s21_sequence_active = False  # S21 시퀀스 진행 중
        self.s21_sequence_timer = None  # S21 시퀀스 타이머
        self.s22_sequence_active = False  # S22 시퀀스 진행 중
        self.s22_sequence_timer = None  # S22 시퀀스 타이머
        self.s22_sequence_step = 0  # S22 시퀀스 단계
        
        # 드라이브 모터(0x141/0x142) 홈잉 상태
        self.homing_active = False  # 홈잉 진행 중
        self.homing_state = 'idle'  # idle, releasing_sensor, searching_sensor, final_approach
        self.homing_initial_in02 = False  # 초기 IN02 상태 저장

        # 홈잉 속도 설정
        self.homing_speed_slow = 0.02  # 저속: 0.02 m/s (정밀 제어)
        self.homing_speed_medium = 0.05  # 중속: 0.05 m/s (빠른 이동)

        # 스테이지 모터(0x144/0x145/0x147) 홈잉 상태
        self.stage_homing_active = False  # 스테이지 홈잉 진행 중
        self.stage_homing_phase = 1  # 호밍 단계: 1=1차 고속, 2=센서 이탈, 3=2차 저속
        self.stage_x_homing_done = False  # X축 홈 도달 완료
        self.stage_y_homing_done = False  # Y축 홈 도달 완료
        self.stage_yaw_homing_done = False  # Yaw 홈 도달 완료
        self.stage_x_homing_phase2_done = False  # X축 2차 호밍 완료
        self.stage_y_homing_phase2_done = False  # Y축 2차 호밍 완료
        self.stage_yaw_homing_phase2_done = False  # Yaw 2차 호밍 완료
        self.stage_xy_homing_speed = 200.0  # XY 스테이지 1차 호밍 속도 (dps, 고속)
        self.stage_xy_homing_speed_slow = 50.0  # XY 스테이지 2차 호밍 속도 (dps, 저속)
        self.stage_yaw_homing_speed = 100.0  # Yaw 1차 호밍 속도 (dps)
        self.stage_yaw_homing_speed_slow = 20.0  # Yaw 2차 호밍 속도 (dps, 저속)
        
        # 스테이지 홈 위치 저장 (엔코더 절대값)
        self.home_x_encoder_position = None  # 홈잉 완료 시 X축 엔코더값 저장
        self.home_y_encoder_position = None  # 홈잉 완료 시 Y축 엔코더값 저장
        self.home_yaw_encoder_position = None  # 홈잉 완료 시 Yaw 엔코더값 저장
        
        # 자동 작업 시퀀스 상태 (S20→S24→Z축 작업)
        self.work_sequence_active = False  # 작업 시퀀스 진행 중
        self.work_sequence_step = 0  # 0: 대기, 1: XY이동완료, 2: Z하강중, 3: 트리거, 4: Z상승중
        self.z_work_start_position = None  # Z축 작업 시작 위치 저장
        
        # 타이머 변수 초기화
        self.encoder_display_timer = None  # 브레이크 해제 후 엔코더 출력용
        self.homing_save_timer = None  # 홈잉 완료 후 홈 위치 저장용
        self.stage_encoder_timer = None  # 스테이지 엔코더 출력용
        self.work_sequence_timer = None  # 작업 시퀀스용

        # CAN 버스 초기화 (모든 데이터 구조 초기화 후)
        try:
            # CAN3: Iron-MD 조종기 수신용
            self.can_bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=self.can_baudrate
            )
            self.get_logger().info(f'CAN bus opened: {self.can_interface} @ {self.can_baudrate} bps')
        except Exception as e:
            self.get_logger().error(f'Failed to open CAN bus: {e}')
            raise

        # CAN 수신 스레드 시작 (마지막에)
        self.can_thread = threading.Thread(target=self.can_receiver_thread, daemon=True)
        self.can_thread.start()

        self.get_logger().info(f'Iron-MD Teleop node started (CAN: {self.can_interface})')
        if self.debug_mode:
            self.get_logger().info('DEBUG MODE: Verbose logging enabled')
        self.print_help()
    
    def print_help(self):
        """조종기 매핑 도움말 (실제 리모콘 조작 기준)"""
        self.get_logger().info('Iron-MD Remote Controller Mapping (실제 조작):')
        self.get_logger().info('  AN3: Forward/Backward (전후진, 좌우 모터 180도 반대 설치: AN3- = 전진, AN3+ = 후진)')
        self.get_logger().info('  AN4: Left/Right turn (좌우회전, 좌우 모터 180도 반대 설치: AN4+ = CCW, AN4- = CW)')
        self.get_logger().info('  AN1: X-axis (0x144), AN2: Y-axis (0x145)')
        self.get_logger().info('  S17/S18: Lateral move ±360deg (0x143)')
        self.get_logger().info('  S21/S22: Work sequence (Z-axis + gripper)')
        self.get_logger().info('  S23/S24: Yaw rotation ±30deg (0x147)')
        self.get_logger().info('  S13: Brake toggle, S14: Homing')
        self.get_logger().info('  Note: AN3 전후진 방향 반전 적용됨') 
    
    def can_receiver_thread(self):
        """CAN 메시지 수신 스레드"""
        while rclpy.ok():
            try:
                msg = self.can_bus.recv(timeout=1.0)
                if msg is not None:
                    self.process_can_message(msg)
            except Exception as e:
                self.get_logger().error(f'CAN receive error: {e}')
    
    def process_can_message(self, msg):
        """CAN 메시지 파싱"""
        can_id = msg.arbitration_id
        data = msg.data
        
        if can_id == 0x1E4:  # 484: Joystick Data
            self.parse_joystick_data(data)
        
        elif can_id == 0x2E4:  # 740: Switch Status
            self.parse_switch_status(data)
        
        elif can_id == 0x764:  # 1892: Heartbeat
            pass  # Heartbeat는 연결 상태 확인용
    
    def parse_joystick_data(self, data):
        """조이스틱 데이터 파싱 (0x1E4)"""
        if len(data) >= 4:
            self.joystick_data['AN1'] = data[0]  # Joystick 1
            self.joystick_data['AN2'] = data[1]  # Joystick 2
            self.joystick_data['AN3'] = data[2]  # Joystick 3
            self.joystick_data['AN4'] = data[3]  # Joystick 4
            
            if self.debug_mode:
                self.get_logger().debug(
                    f'📊 조이스틱: AN1={data[0]:3d} AN2={data[1]:3d} '
                    f'AN3={data[2]:3d}(전후진) AN4={data[3]:3d}(좌우회전)'
                )
    
    def parse_switch_status(self, data):
        """스위치 상태 파싱 (0x2E4)"""
        if len(data) < 8:
            return
        
        # Byte 0: Start, Power, Engine, Emergency, S13
        byte0 = data[0]
        self.switch_data['S13'] = (byte0 >> 2) & 0x01  # 브레이크 해제 버튼
        self.switch_data['Emergency_Stop_Release'] = (byte0 >> 6) & 0x01
        self.switch_data['Emergency_Stop_Active'] = (byte0 >> 7) & 0x01
        
        # Byte 1: S00-S07
        byte1 = data[1]
        self.switch_data['S06'] = (byte1 >> 0) & 0x01
        self.switch_data['S07'] = (byte1 >> 1) & 0x01
        self.switch_data['S02'] = (byte1 >> 4) & 0x01
        self.switch_data['S03'] = (byte1 >> 5) & 0x01
        self.switch_data['S01'] = (byte1 >> 6) & 0x01
        self.switch_data['S00'] = (byte1 >> 7) & 0x01
        
        # Byte 2: S08-S09, S14
        byte2 = data[2]
        prev_s14 = self.switch_data.get('S14', 0)

        self.switch_data['S08'] = (byte2 >> 0) & 0x01
        self.switch_data['S09'] = (byte2 >> 1) & 0x01
        self.switch_data['S14'] = (byte2 >> 3) & 0x01  # 홈잉 버튼 (0x62→0x6A)

        # S14 변화 감지 시 디버그 로그 (모든 비트 출력)
        if prev_s14 != self.switch_data['S14']:
            self.get_logger().info(
                f'🔍 [CAN DEBUG] S14 변화: {prev_s14} → {self.switch_data["S14"]}, '
                f'byte2=0x{byte2:02X} (binary: {bin(byte2)[2:].zfill(8)}, '
                f'bit0={byte2&0x01}, bit1={(byte2>>1)&0x01}, bit2={(byte2>>2)&0x01}, '
                f'bit3={(byte2>>3)&0x01}, bit4={(byte2>>4)&0x01}, bit5={(byte2>>5)&0x01}, '
                f'bit6={(byte2>>6)&0x01}, bit7={(byte2>>7)&0x01})'
            )
        
        # Byte 3: S17-S24
        byte3 = data[3]
        prev_s21 = self.switch_data.get('S21', 0)
        prev_s22 = self.switch_data.get('S22', 0)
        
        self.switch_data['S23'] = (byte3 >> 0) & 0x01
        self.switch_data['S24'] = (byte3 >> 1) & 0x01
        self.switch_data['S21'] = (byte3 >> 2) & 0x01
        self.switch_data['S22'] = (byte3 >> 3) & 0x01
        self.switch_data['S19'] = (byte3 >> 4) & 0x01
        self.switch_data['S20'] = (byte3 >> 5) & 0x01
        self.switch_data['S17'] = (byte3 >> 6) & 0x01
        self.switch_data['S18'] = (byte3 >> 7) & 0x01
        
        # S21/S22 변화 감지 시 디버그 로그
        if prev_s21 != self.switch_data['S21']:
            self.get_logger().info(
                f'[CAN DEBUG] S21 변화: {prev_s21} -> {self.switch_data["S21"]}, '
                f'byte3=0x{byte3:02X} (binary: {bin(byte3)[2:].zfill(8)})'
            )
        if prev_s22 != self.switch_data['S22']:
            self.get_logger().info(
                f'[CAN DEBUG] S22 변화: {prev_s22} -> {self.switch_data["S22"]}, '
                f'byte3=0x{byte3:02X} (binary: {bin(byte3)[2:].zfill(8)})'
            )
        
        # Byte 6: TX Connected
        if len(data) >= 7:
            byte6 = data[6]
            self.switch_data['TX_Connected'] = (byte6 >> 6) & 0x01
    
    def switch_pressed(self, switch_name):
        """스위치 엣지 감지 (Rising Edge)"""
        current = self.switch_data.get(switch_name, 0)
        previous = self.prev_switches.get(switch_name, 0)
        return current == 1 and previous == 0
    
    def normalize_joystick(self, value):
        """조이스틱 값 정규화 (0-255 -> -1.0 to 1.0)"""
        centered = value - self.joy_center
        
        # 데드존 적용
        if abs(centered) < self.joy_deadzone:
            return 0.0
        
        # 정규화
        if centered > 0:
            return centered / (255 - self.joy_center)
        else:
            return centered / self.joy_center
    
    def control_loop(self):
        """제어 루프 (20Hz)"""
        # 비상 정지 체크
        if self.switch_data['Emergency_Stop_Active'] == 1:
            if not self.emergency_stopped:
                self.emergency_stop()
            return
        elif self.emergency_stopped and self.switch_data['Emergency_Stop_Release'] == 1:
            self.emergency_stopped = False
            self.get_logger().info('Emergency stop released (hardware)')
        
        if self.emergency_stopped:
            return
        
        # 연결 상태 확인
        if self.switch_data['TX_Connected'] == 0:
            # 송신기 연결 안됨 - 모든 모터 정지
            self.publish_zero_velocity()
            return
        
        # 제어 모드 확인
        self.update_control_mode()

        if self.control_mode != 'remote':
            # Automatic 모드일 경우 호밍 등의 특수 명령 처리
            self.handle_auto_mode()
            return
        
        # Remote Control 모드
        # 1. 주행 제어 (연속) - 리모콘: AN3=전후진, AN4=좌우회전 (좌우 모터 180도 반대 설치)
        self.handle_driving()
        
        # 2. XYZ 스테이지 (연속) - AN1, AN2
        self.handle_xyz_stage()
        
        # 3. 횡이동 (스텝) - S17, S18
        self.handle_lateral_move()
        
        # 4. 작업 시퀀스 - S21, S22
        self.handle_work_sequence()
        
        # 5. Yaw 회전 - S23, S24
        self.handle_yaw_rotation()
        
        # 6. 브레이크 해제 - S13
        self.handle_brake_release()

        # 7. 홈잉 - S14
        self.handle_homing()

        # 이전 상태 저장
        self.prev_switches = self.switch_data.copy()
    
    def update_control_mode(self):
        """제어 모드 업데이트 (S19-S20)"""
        if self.switch_data['S19'] == 1:
            if self.control_mode != 'remote':
                old_mode = self.control_mode
                self.control_mode = 'remote'
                self.get_logger().info(f'🔄 모드 전환: {old_mode} → Remote Control mode')
                self.file_logger.info(f'🔄 모드 전환: {old_mode} → Remote Control mode')
                self.file_log_file_handler.flush()
                # S20에서 S19로 전환 시 주행 모터 위치 제어 중지
                if old_mode == 'automatic':
                    # 위치 제어 중지 (속도 제어 모드로 전환)
                    # 위치는 실제 엔코더 값으로 유지 (리셋하지 않음)
                    # self.current_positions['left_wheel'] = 0.0  # 제거: 모드 전환 시에도 실제 위치 유지
                    # self.current_positions['right_wheel'] = 0.0  # 제거: 모드 전환 시에도 실제 위치 유지
                    self.an3_command_active = False
                    self.an3_prev_value = 127
                    self.get_logger().info('🛑 S20→S19 전환: 주행 모터 위치 제어 중지 (위치 유지)')
                    self.file_logger.info('🛑 S20→S19 전환: 주행 모터 위치 제어 중지 (위치 유지)')
                    self.file_log_file_handler.flush()
        elif self.switch_data['S20'] == 1:
            if self.control_mode != 'automatic':
                old_mode = self.control_mode
                self.control_mode = 'automatic'
                self.get_logger().info(f'🔄 모드 전환: {old_mode} → 🤖 Automatic Control 모드 (상위제어 대기)')
                self.file_logger.info(f'🔄 모드 전환: {old_mode} → 🤖 Automatic Control 모드 (상위제어 대기)')
                self.file_log_file_handler.flush()

    def handle_auto_mode(self):
        """S20 Automatic 모드에서 호밍 및 작업 시퀀스 처리"""
        # AN3: 주행 모터(0x141, 0x142) ±1200도 회전 제어
        self.handle_an3_drive_motor_rotation()
        
        # S17: 그리퍼 열기
        if self.switch_pressed('S17'):
            self.get_logger().info('[S17] 그리퍼 열기')
            # 그리퍼 열기 명령: g 5 600 -> (5 << 12) | 600 = 0x5258 = 21080
            from std_msgs.msg import Int32
            open_cmd = Int32()
            open_cmd.data = 21080  # Command 5, Speed +600 (g 5 600)
            self.gripper_cmd_pub.publish(open_cmd)
            self.get_logger().info('  -> Published /gripper/command: 21080 (g 5 600 - OPEN with speed)')

        # S18: 그리퍼 닫기
        if self.switch_pressed('S18'):
            self.get_logger().info('[S18] 그리퍼 닫기')
            # 그리퍼 닫기 명령: g 5 -600 -> (5 << 12) | (-600 & 0x0FFF) = 0x5DA8 = 23976
            from std_msgs.msg import Int32
            close_cmd = Int32()
            close_cmd.data = 23976  # Command 5, Speed -600 (g 5 -600)
            self.gripper_cmd_pub.publish(close_cmd)
            self.get_logger().info('  -> Published /gripper/command: 23976 (g 5 -600 - CLOSE with speed)')

        # S21: 횡이동 +2880도 회전 (0x143) - 과부하 방지를 위해 속도 낮춤 및 간격 체크
        if self.switch_pressed('S21'):
            # 명령 간 최소 간격 체크 (과부하 방지)
            current_time = self.get_clock().now().nanoseconds / 1e9
            time_since_last = current_time - self.last_lateral_command_time
            
            if time_since_last < self.lateral_command_min_interval:
                self.get_logger().warning(f'횡이동 명령 간격 너무 짧음 ({time_since_last:.2f}s < {self.lateral_command_min_interval}s), 무시')
                return
            
            # 이동 중인지 체크
            if self.lateral_target_position is not None:
                position_diff = abs(self.lateral_target_position - self.current_positions['lateral'])
                if position_diff > 50.0:  # 50도 이상 차이면 이동 중으로 간주
                    self.get_logger().warning(f'횡이동 모터 이동 중 (차이: {position_diff:.1f}도), 새 명령 무시')
                    return
            
            prev_lateral = self.current_positions['lateral']
            self.current_positions['lateral'] += 2880.0  # +2880도 (8회전)
            self.lateral_target_position = self.current_positions['lateral']
            self.last_lateral_command_time = current_time
            self.get_logger().info(f'[S21] 횡이동 +2880° (0x143, {prev_lateral:.1f}° → {self.current_positions["lateral"]:.1f}°)')
            self.publish_joint_position('lateral', self.joint1_pub, speed=100.0)  # 0x143 속도 낮춤 (과부하 방지: 200→100 dps)

        # S22: 횡이동 -2880도 회전 (0x143) - 과부하 방지를 위해 속도 낮춤 및 간격 체크
        if self.switch_pressed('S22'):
            # 명령 간 최소 간격 체크 (과부하 방지)
            current_time = self.get_clock().now().nanoseconds / 1e9
            time_since_last = current_time - self.last_lateral_command_time
            
            if time_since_last < self.lateral_command_min_interval:
                self.get_logger().warning(f'횡이동 명령 간격 너무 짧음 ({time_since_last:.2f}s < {self.lateral_command_min_interval}s), 무시')
                return
            
            # 이동 중인지 체크
            if self.lateral_target_position is not None:
                position_diff = abs(self.lateral_target_position - self.current_positions['lateral'])
                if position_diff > 50.0:  # 50도 이상 차이면 이동 중으로 간주
                    self.get_logger().warning(f'횡이동 모터 이동 중 (차이: {position_diff:.1f}도), 새 명령 무시')
                    return
            
            prev_lateral = self.current_positions['lateral']
            self.current_positions['lateral'] -= 2880.0  # -2880도 (8회전)
            self.lateral_target_position = self.current_positions['lateral']
            self.last_lateral_command_time = current_time
            self.get_logger().info(f'[S22] 횡이동 -2880° (0x143, {prev_lateral:.1f}° → {self.current_positions["lateral"]:.1f}°)')
            self.publish_joint_position('lateral', self.joint1_pub, speed=100.0)  # 0x143 속도 낮춤 (과부하 방지: 200→100 dps)

        # S23: 스테이지 XY 호밍 (IN01 & IN02까지 이동)
        if self.switch_pressed('S23'):
            if not self.stage_homing_active:
                self.start_stage_homing()
            else:
                self.get_logger().warning('⚠️  스테이지 홈잉이 이미 진행 중입니다')

        # S24: 작업 시퀀스 시작 (XY 이동 → Z 하강 → 트리거 → Z 상승)
        if self.switch_pressed('S24'):
            # 홈잉 완료 여부 확인
            if self.home_x_encoder_position is None or self.home_y_encoder_position is None:
                self.get_logger().warning('⚠️  XY축 홈잉을 먼저 완료해주세요 (S23)')
            elif self.work_sequence_active:
                self.get_logger().warning('⚠️  작업 시퀀스가 이미 진행 중입니다')
            else:
                self.start_work_sequence()

        # 이전 스위치 상태 저장
        self.prev_switches = self.switch_data.copy()
    
    def handle_an3_drive_motor_rotation(self):
        """S20 모드에서 AN3로 주행 모터(0x141, 0x142) ±1200도 회전 제어 (엣지 트리거, 1회만 실행)"""
        current_an3 = self.joystick_data['AN3']
        current_time = self.get_clock().now().nanoseconds / 1e9
        rotation_degrees = 1200.0
        
        # 명령 실행 중이면 AN3가 중립(127)으로 돌아올 때까지 대기
        if self.an3_command_active:
            # 중립 복귀 확인 (데드존: 120~134)
            if 120 <= current_an3 <= 134:
                self.an3_command_active = False
                log_msg = f'[S20/AN3] ✅ 중립 복귀 감지 (AN3={current_an3}), 다음 명령 허용'
                self.get_logger().info(log_msg)
                self.file_logger.info(log_msg)  # 파일에도 저장
                self.file_log_file_handler.flush()  # 즉시 파일에 기록
            # 아직 명령 실행 중이므로 AN3 추가 명령만 무시 (다른 스위치는 계속 처리)
            # FIX: return 제거 - 다른 스위치(S17, S19, S21 등)가 처리되도록 함
            self.an3_prev_value = current_an3
            return  # 엣지 감지는 건너뛰고 handle_auto_mode()의 다음 코드로 진행

        # AN3 값 변화 감지 (엣지 트리거: 중립 → AN3- 또는 AN3+)
        an3_changed = False
        rotation_direction = 0  # 0: 없음, 1: +1200도, -1: -1200도

        # AN3- 감지 (중립에서 앞으로 기울임: 127 → <100)
        if 120 <= self.an3_prev_value <= 134 and current_an3 < 100:  # 중립에서 AN3-로 변화
            an3_changed = True
            rotation_direction = 1  # +1200도
            log_msg = f'[S20/AN3] 🔍 엣지 감지: 중립({self.an3_prev_value}) → AN3-({current_an3}), +{rotation_degrees:.0f}° 회전 명령'
            self.get_logger().info(log_msg)
            self.file_logger.info(log_msg)  # 파일에도 저장
            self.file_log_file_handler.flush()  # 즉시 파일에 기록
        # AN3+ 감지 (중립에서 아래로 기울임: 127 → >154)
        elif 120 <= self.an3_prev_value <= 134 and current_an3 > 154:  # 중립에서 AN3+로 변화
            an3_changed = True
            rotation_direction = -1  # -1200도
            log_msg = f'[S20/AN3] 🔍 엣지 감지: 중립({self.an3_prev_value}) → AN3+({current_an3}), -{rotation_degrees:.0f}° 회전 명령'
            self.get_logger().info(log_msg)
            self.file_logger.info(log_msg)  # 파일에도 저장
            self.file_log_file_handler.flush()  # 즉시 파일에 기록
        
        if an3_changed:
            # 명령 간 최소 간격 체크
            time_since_last = current_time - self.an3_last_command_time
            if time_since_last < self.an3_command_min_interval:
                self.get_logger().warning(f'[S20/AN3] ⚠️  명령 간격 너무 짧음 ({time_since_last:.2f}s < {self.an3_command_min_interval}s), 무시')
                self.an3_prev_value = current_an3
                return
            
            # 현재 위치에서 ±1200도 계산
            left_current = self.current_positions['left_wheel']
            right_current = self.current_positions['right_wheel']
            left_target = left_current - (rotation_direction * rotation_degrees)  # 좌측 모터: 반대 방향
            right_target = right_current + (rotation_direction * rotation_degrees)  # 우측 모터: 정방향

            direction_str = f"+{rotation_degrees:.0f}°" if rotation_direction > 0 else f"-{rotation_degrees:.0f}°"
            log_msg = (
                f'[S20/AN3] 🎯 주행 모터 {direction_str} 회전 명령 (1회): '
                f'0x141 {left_current:.1f}° → {left_target:.1f}° (변화: {-rotation_direction * rotation_degrees:.1f}°), '
                f'0x142 {right_current:.1f}° → {right_target:.1f}° (변화: {rotation_direction * rotation_degrees:.1f}°)'
            )
            self.get_logger().info(log_msg)
            self.file_logger.info(log_msg)  # 파일에도 저장
            self.file_log_file_handler.flush()  # 즉시 파일에 기록
            
            # 위치 제어 명령 발행
            msg_left = Float64MultiArray()
            msg_left.data = [left_target, 704.0]  # [위치(도), 속도(dps)]
            self.left_wheel_position_pub.publish(msg_left)
            log_msg = f'[S20/AN3] 📤 0x141 위치 제어 명령 발행: 목표={left_target:.1f}°, 속도=704.0dps'
            self.get_logger().info(log_msg)
            self.file_logger.info(log_msg)  # 파일에도 저장
            self.file_log_file_handler.flush()  # 즉시 파일에 기록
            
            msg_right = Float64MultiArray()
            msg_right.data = [right_target, 704.0]  # [위치(도), 속도(dps)]
            self.right_wheel_position_pub.publish(msg_right)
            log_msg = f'[S20/AN3] 📤 0x142 위치 제어 명령 발행: 목표={right_target:.1f}°, 속도=704.0dps'
            self.get_logger().info(log_msg)
            self.file_logger.info(log_msg)  # 파일에도 저장
            self.file_log_file_handler.flush()  # 즉시 파일에 기록

            # 명령 실행 플래그 설정
            # 주의: current_positions는 motor_position_callback에서 실제 엔코더 값으로 업데이트됨
            # 목표값을 저장하지 않음 (실제 위치와 목표 위치가 다르면 다음 명령 계산이 틀어짐)
            self.an3_last_command_time = current_time
            self.an3_command_active = True  # 명령 실행 중 플래그 설정 (중립 복귀까지 추가 명령 무시)
            log_msg = f'[S20/AN3] ✅ 명령 실행 완료, AN3 중립 복귀 대기 중... (현재 AN3={current_an3})'
            self.get_logger().info(log_msg)
            self.file_logger.info(log_msg)  # 파일에도 저장
            self.file_log_file_handler.flush()  # 즉시 파일에 기록
        
        self.an3_prev_value = current_an3

    def start_work_sequence(self):
        """작업 시퀀스 시작: XY 이동 → Z 하강 → 트리거 → Z 상승"""
        self.work_sequence_active = True
        self.work_sequence_step = 1
        
        # Step 1: XY 이동
        prev_x_pos = self.current_positions['x']
        self.current_positions['x'] += 1181.85
        
        prev_y_pos = self.current_positions['y']
        self.current_positions['y'] -= 189.4
        
        self.get_logger().info('🔧 ===== 작업 시퀀스 시작 =====')
        self.get_logger().info(f'📍 Step 1: XY 스테이지 작업 위치로 이동')
        self.get_logger().info(f'   X: {prev_x_pos:.2f}° → {self.current_positions["x"]:.2f}° (+1181.85°)')
        self.get_logger().info(f'   Y: {prev_y_pos:.2f}° → {self.current_positions["y"]:.2f}° (-189.4°)')
        
        # 위치 제어 명령 전송
        self.publish_joint_position('x', self.joint2_position_pub)
        self.publish_joint_position('y', self.joint3_position_pub)
        
        # 3초 후 Z축 하강 시작 (XY 이동 완료 대기)
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
        self.work_sequence_timer = self.create_timer(3.0, self._work_sequence_step2_wrapper)
    
    def _work_sequence_step2_wrapper(self):
        """작업 시퀀스 Step 2: Z축 하강"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None
        
        if not self.work_sequence_active or self.work_sequence_step != 1:
            return
        
        self.work_sequence_step = 2
        
        # 현재 Z축 위치 저장
        self.z_work_start_position = self.current_positions['z']
        
        # Z축 하강: 현재 위치 - 900° (2.5회전)
        self.current_positions['z'] -= 900.0
        
        self.get_logger().info(f'📍 Step 2: Z축 하강 시작 (400dps)')
        self.get_logger().info(f'   Z: {self.z_work_start_position:.2f}° → {self.current_positions["z"]:.2f}° (-900°)')
        
        # Z축 위치 제어 명령 전송 (속도 400dps 지정)
        msg = Float64MultiArray()
        msg.data = [self.current_positions['z'], 400.0]  # [위치, 속도]
        self.joint4_pub.publish(msg)
        
        # Z축 하강 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 Step 3 트리거 동작 시작)
    
    def _work_sequence_step3_wrapper(self):
        """작업 시퀀스 Step 3: 트리거 동작"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None
        
        if not self.work_sequence_active or self.work_sequence_step != 2:
            return
        
        self.work_sequence_step = 3
        
        self.get_logger().info(f'📍 Step 3: 트리거 동작 시작')
        # trigger_pull() 완료 시 trigger_release()에서 자동으로 Step 4 호출됨
        self.trigger_pull()
    
    def _work_sequence_step4_wrapper(self):
        """작업 시퀀스 Step 4: Z축 상승 (원위치)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 3:
            return

        self.work_sequence_step = 4

        # Z축 상승: +900° (원래 위치로)
        self.current_positions['z'] += 900.0

        self.get_logger().info(f'📍 Step 4: Z축 상승 (원위치, 400dps)')
        self.get_logger().info(f'   Z: {self.current_positions["z"] - 900.0:.2f}° → {self.current_positions["z"]:.2f}° (+900°)')

        # Z축 위치 제어 명령 전송 (속도 400dps 지정)
        msg = Float64MultiArray()
        msg.data = [self.current_positions['z'], 400.0]  # [위치, 속도]
        self.joint4_pub.publish(msg)

        # Z축 상승 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 Step 5 X축 이동 시작)

    def _work_sequence_step5_wrapper(self):
        """작업 시퀀스 Step 5: X축 원점방향 이동"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 4:
            return

        self.work_sequence_step = 5

        # X축 원점방향으로 -920.59° 이동
        prev_x_pos = self.current_positions['x']
        self.current_positions['x'] -= 920.59

        self.get_logger().info(f'📍 Step 5: X축 원점방향 이동 (400dps)')
        self.get_logger().info(f'   X: {prev_x_pos:.2f}° → {self.current_positions["x"]:.2f}° (-920.59°)')

        # X축 위치 제어 명령 전송
        self.publish_joint_position('x', self.joint2_position_pub)

        # X축 이동 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 Step 6 Z축 하강 시작)

    def _work_sequence_step6_wrapper(self):
        """작업 시퀀스 Step 6: Z축 하강 (2차)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 5:
            return

        self.work_sequence_step = 6

        # 현재 Z축 위치 저장
        self.z_work_start_position = self.current_positions['z']

        # Z축 하강: 현재 위치 - 900° (2.5회전)
        self.current_positions['z'] -= 900.0

        self.get_logger().info(f'📍 Step 6: Z축 하강 (2차, 400dps)')
        self.get_logger().info(f'   Z: {self.z_work_start_position:.2f}° → {self.current_positions["z"]:.2f}° (-900°)')

        # Z축 위치 제어 명령 전송 (속도 400dps 지정)
        msg = Float64MultiArray()
        msg.data = [self.current_positions['z'], 400.0]  # [위치, 속도]
        self.joint4_pub.publish(msg)

        # Z축 하강 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 Step 7 트리거 동작 시작)

    def _work_sequence_step7_wrapper(self):
        """작업 시퀀스 Step 7: 트리거 동작 (2차)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 6:
            return

        self.work_sequence_step = 7

        self.get_logger().info(f'📍 Step 7: 트리거 동작 (2차)')
        # trigger_pull() 완료 시 trigger_release()에서 자동으로 Step 8 호출됨
        self.trigger_pull()

    def _work_sequence_step8_wrapper(self):
        """작업 시퀀스 Step 8: Z축 상승 (2차, 완료)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 7:
            return

        self.work_sequence_step = 8

        # Z축 상승: +900° (원래 위치로)
        self.current_positions['z'] += 900.0

        self.get_logger().info(f'📍 Step 8: Z축 상승 (2차, 400dps)')
        self.get_logger().info(f'   Z: {self.current_positions["z"] - 900.0:.2f}° → {self.current_positions["z"]:.2f}° (+900°)')

        # Z축 위치 제어 명령 전송 (속도 400dps 지정)
        msg = Float64MultiArray()
        msg.data = [self.current_positions['z'], 400.0]  # [위치, 속도]
        self.joint4_pub.publish(msg)

        # Z축 상승 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 Step 9 XYYaw 이동 시작)

    def _work_sequence_step9_wrapper(self):
        """작업 시퀀스 Step 9: X/Y/Yaw 3축 동시 이동 (3번째 작업 위치)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 8:
            return

        self.work_sequence_step = 9

        # X축: 현재 위치에서 -91.39° (홈 방향)
        prev_x_pos = self.current_positions['x']
        self.current_positions['x'] -= 91.39

        # Y축: 현재 위치에서 -1036.58° (호밍 반대 방향)
        prev_y_pos = self.current_positions['y']
        self.current_positions['y'] -= 1036.58

        # Yaw축: 현재 위치에서 -260°
        prev_yaw_pos = self.current_positions['yaw']
        self.current_positions['yaw'] -= 260.0

        self.get_logger().info(f'📍 Step 9: X/Y/Yaw 3축 동시 이동 (3번째 작업 위치, 400dps)')
        self.get_logger().info(f'   X: {prev_x_pos:.2f}° → {self.current_positions["x"]:.2f}° (-91.39°)')
        self.get_logger().info(f'   Y: {prev_y_pos:.2f}° → {self.current_positions["y"]:.2f}° (-1036.58°)')
        self.get_logger().info(f'   Yaw: {prev_yaw_pos:.2f}° → {self.current_positions["yaw"]:.2f}° (-260.0°)')

        # 위치 제어 명령 전송 (3축 동시, 속도 400dps 명시)
        # X축
        msg_x = Float64MultiArray()
        msg_x.data = [self.current_positions['x'], 400.0]
        self.joint2_position_pub.publish(msg_x)
        self.get_logger().info(f'📍 X: {self.current_positions["x"]:.2f}° (400dps)')

        # Y축
        msg_y = Float64MultiArray()
        msg_y.data = [self.current_positions['y'], 400.0]
        self.joint3_position_pub.publish(msg_y)
        self.get_logger().info(f'📍 Y: {self.current_positions["y"]:.2f}° (400dps)')

        # Yaw축
        msg_yaw = Float64MultiArray()
        msg_yaw.data = [self.current_positions['yaw'], 400.0]
        self.joint5_pub.publish(msg_yaw)
        self.get_logger().info(f'📍 Yaw: {self.current_positions["yaw"]:.2f}° (400dps)')

        # 3축 이동 완료 확인을 위한 플래그 초기화
        if not hasattr(self, 'step9_completed_motors'):
            self.step9_completed_motors = set()
        self.step9_completed_motors.clear()

        # 모든 축 이동 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 3축 모두 완료 시 Step 10 시작)

    def _work_sequence_step10_wrapper(self):
        """작업 시퀀스 Step 10: Z축 하강 (3차)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 9:
            return

        self.work_sequence_step = 10

        # 현재 Z축 위치 저장
        self.z_work_start_position = self.current_positions['z']

        # Z축 하강: 현재 위치 - 900° (2.5회전)
        self.current_positions['z'] -= 900.0

        self.get_logger().info(f'📍 Step 10: Z축 하강 (3차, 400dps)')
        self.get_logger().info(f'   Z: {self.z_work_start_position:.2f}° → {self.current_positions["z"]:.2f}° (-900°)')

        # Z축 위치 제어 명령 전송 (속도 400dps 지정)
        msg = Float64MultiArray()
        msg.data = [self.current_positions['z'], 400.0]  # [위치, 속도]
        self.joint4_pub.publish(msg)

        # Z축 하강 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 Step 11 트리거 동작 시작)

    def _work_sequence_step11_wrapper(self):
        """작업 시퀀스 Step 11: 트리거 동작 (3차)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 10:
            return

        self.work_sequence_step = 11

        self.get_logger().info(f'📍 Step 11: 트리거 동작 (3차)')
        # trigger_pull() 완료 시 trigger_release()에서 자동으로 Step 12 호출됨
        self.trigger_pull()

    def _work_sequence_step12_wrapper(self):
        """작업 시퀀스 Step 12: Z축 상승 (3차, 완료)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 11:
            return

        self.work_sequence_step = 12

        # Z축 상승: +900° (원래 위치로)
        self.current_positions['z'] += 900.0

        self.get_logger().info(f'📍 Step 12: Z축 상승 (3차, 400dps)')
        self.get_logger().info(f'   Z: {self.current_positions["z"] - 900.0:.2f}° → {self.current_positions["z"]:.2f}° (+900°)')

        # Z축 위치 제어 명령 전송 (속도 400dps 지정)
        msg = Float64MultiArray()
        msg.data = [self.current_positions['z'], 400.0]  # [위치, 속도]
        self.joint4_pub.publish(msg)

        # Z축 상승 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 Step 13으로 진행)

    def _work_sequence_step13_wrapper(self):
        """작업 시퀀스 Step 13: X/Y 2축 동시 이동 (4번째 작업 위치, Yaw 변화 없음)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 12:
            return

        self.work_sequence_step = 13

        # X축: 현재 위치에서 +884.22° (홈 방향)
        prev_x_pos = self.current_positions['x']
        self.current_positions['x'] += 884.22

        # Y축: 현재 위치에서 -11.54° (호밍 반대 방향)
        prev_y_pos = self.current_positions['y']
        self.current_positions['y'] -= 11.54

        self.get_logger().info(f'📍 Step 13: X/Y 2축 동시 이동 (4번째 작업 위치, 400dps)')
        self.get_logger().info(f'   X: {prev_x_pos:.2f}° → {self.current_positions["x"]:.2f}° (+884.22°)')
        self.get_logger().info(f'   Y: {prev_y_pos:.2f}° → {self.current_positions["y"]:.2f}° (-11.54°)')
        self.get_logger().info(f'   Yaw: 변화 없음')

        # 위치 제어 명령 전송 (2축 동시, 속도 400dps 명시)
        # X축
        msg_x = Float64MultiArray()
        msg_x.data = [self.current_positions['x'], 400.0]
        self.joint2_position_pub.publish(msg_x)
        self.get_logger().info(f'📍 X: {self.current_positions["x"]:.2f}° (400dps)')

        # Y축
        msg_y = Float64MultiArray()
        msg_y.data = [self.current_positions['y'], 400.0]
        self.joint3_position_pub.publish(msg_y)
        self.get_logger().info(f'📍 Y: {self.current_positions["y"]:.2f}° (400dps)')

        # 2축 이동 완료 확인을 위한 플래그 초기화
        if not hasattr(self, 'step13_completed_motors'):
            self.step13_completed_motors = set()
        self.step13_completed_motors.clear()

        # 모든 축 이동 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 2축 모두 완료 시 Step 14 시작)

    def _work_sequence_step14_wrapper(self):
        """작업 시퀀스 Step 14: Z축 하강 (4차)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 13:
            return

        self.work_sequence_step = 14

        # 현재 Z축 위치 저장
        self.z_work_start_position = self.current_positions['z']

        # Z축 하강: 현재 위치 - 900° (2.5회전)
        self.current_positions['z'] -= 900.0

        self.get_logger().info(f'📍 Step 14: Z축 하강 (4차, 400dps)')
        self.get_logger().info(f'   Z: {self.z_work_start_position:.2f}° → {self.current_positions["z"]:.2f}° (-900°)')

        # Z축 위치 제어 명령 전송 (속도 400dps 지정)
        msg = Float64MultiArray()
        msg.data = [self.current_positions['z'], 400.0]  # [위치, 속도]
        self.joint4_pub.publish(msg)

        # Z축 하강 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 트리거 동작 시작)

    def _work_sequence_step15_wrapper(self):
        """작업 시퀀스 Step 15: 트리거 동작 (4차)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 14:
            return

        self.work_sequence_step = 15

        self.get_logger().info(f'📍 Step 15: 트리거 동작 (4차)')

        # 트리거 당기기
        self.trigger_pull()

        # 트리거 완료 후 자동으로 Step 16 (Z축 상승)으로 진행
        # (trigger_release()에서 처리)

    def _work_sequence_step16_wrapper(self):
        """작업 시퀀스 Step 16: Z축 상승 (4차, 완료)"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None

        if not self.work_sequence_active or self.work_sequence_step != 15:
            return

        self.work_sequence_step = 16

        # Z축 상승: +900° (원래 위치로)
        self.current_positions['z'] += 900.0

        self.get_logger().info(f'📍 Step 16: Z축 상승 (4차, 완료, 400dps)')
        self.get_logger().info(f'   Z: {self.current_positions["z"] - 900.0:.2f}° → {self.current_positions["z"]:.2f}° (+900°)')

        # Z축 위치 제어 명령 전송 (속도 400dps 지정)
        msg = Float64MultiArray()
        msg.data = [self.current_positions['z'], 400.0]  # [위치, 속도]
        self.joint4_pub.publish(msg)

        # Z축 상승 완료는 motor_goal_reached 토픽으로 자동 감지됨
        # (motor_goal_reached_callback에서 작업 완료 처리)

    def _work_sequence_complete_wrapper(self):
        """작업 시퀀스 완료"""
        if hasattr(self, 'work_sequence_timer') and self.work_sequence_timer:
            self.work_sequence_timer.cancel()
            self.work_sequence_timer = None
        
        if not self.work_sequence_active:
            return
        
        self.get_logger().info('✅ ===== 작업 시퀀스 완료 =====')
        self.work_sequence_active = False
        self.work_sequence_step = 0
        self.z_work_start_position = None
    
    def handle_driving(self):
        """주행 제어 (AN3: 전후진, AN4: 좌우회전) - 좌우 모터 180도 반대 설치로 인한 방향 보정"""
        # 홈잉 중일 때는 조이스틱 제어 무시
        if self.homing_active:
            return

        # AN3: 전후진 (좌우 모터가 180도 반대로 설치되어있음)
        # AN3- -> 전진, AN3+ -> 후진
        linear = self.normalize_joystick(self.joystick_data['AN4'])

        # AN4: 좌우회전 (좌우 모터가 180도 반대로 설치되어 있어서)
        # AN4+ -> CCW, AN4- -> CW 로 로봇이 움직임
        angular = -self.normalize_joystick(self.joystick_data['AN3'])
        
        # 로그: 조이스틱 RAW 값 (DEBUG 모드만)
        if self.debug_mode and (abs(linear) > 0.01 or abs(angular) > 0.01):
            self.get_logger().debug(
                f'Joystick: AN3={self.joystick_data["AN3"]}(linear, 전후진: AN3-=전진), AN4={self.joystick_data["AN4"]}(angular, 좌우회전: AN4+=CCW), '
                f'linear={linear:.3f}, angular={angular:.3f}'
            )
        
        # 값 변화가 있을 때만 발행 (중복 방지)
        if (abs(linear - self.last_cmd_sent['linear']) > 0.01 or
            abs(angular - self.last_cmd_sent['angular']) > 0.01):
            twist = Twist()
            # AN3(전후진): 전체 속도 (12.5% 감속) - AN3- = 전진, AN3+ = 후진 (좌우 모터 180도 반대 설치)
            # 실제로는 AN4 값을 사용하지만 주석은 AN3로 표기 (좌우 모터 180도 반대 설치)
            twist.linear.x = linear * self.max_linear * 0.125
            # AN4(좌우회전): 각속도 - AN4+ = CCW, AN4- = CW (좌우 모터 180도 반대 설치)
            # 실제로는 AN3 값을 사용하지만 주석은 AN4로 표기 (좌우 모터 180도 반대 설치)
            twist.angular.z = angular * self.max_angular
            
            # 로그: ROS2 토픽 발행 (DEBUG 모드만)
            if self.debug_mode and (abs(linear) > 0.01 or abs(angular) > 0.01):
                self.get_logger().debug(
                    f'cmd_vel pub: linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}'
                )
            
            self.cmd_vel_pub.publish(twist)
            self.last_cmd_sent['linear'] = linear
            self.last_cmd_sent['angular'] = angular
    
    def handle_xyz_stage(self):
        """XYZ 스테이지 제어 (AN1: X축 0x144, AN2: Y축 0x145) - 속도 제어"""
        # 홈잉 중일 때는 XYZ 스테이지 제어 무시
        if self.homing_active:
            return

        import can
        import struct

        # X축: AN1 -> 0x144 속도 제어 (방향 반전)
        x_value = self.normalize_joystick(self.joystick_data['AN1'])

        # 안전: 리미트 센서 체크 및 방향 제한
        if abs(x_value) > 0.1:
            # IN02 (홈) ON이면 양수(+) 방향 차단 (반전 고려)
            if self.limit_sensor_in02 and x_value > 0:
                self.get_logger().warning('IN02 sensor ON: blocking positive direction')
                x_value = 0.0

            # IN03 (최대) ON이면 음수(-) 방향 차단 (반전 고려)
            if self.limit_sensor_in03 and x_value < 0:
                self.get_logger().warning('IN03 sensor ON: blocking negative direction')
                x_value = 0.0

        if abs(x_value) > 0.1:
            try:
                x_speed_dps = x_value * 200.0  # 최대 200 dps
                speed_control = int(x_speed_dps * 100)  # 0.01 dps/LSB

                can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                msg = can.Message(
                    arbitration_id=0x144,
                    data=[
                        0xA2,  # Speed Control Command
                        0x64,  # 100% max torque
                        0x00,
                        0x00,
                        speed_control & 0xFF,
                        (speed_control >> 8) & 0xFF,
                        (speed_control >> 16) & 0xFF,
                        (speed_control >> 24) & 0xFF
                    ],
                    is_extended_id=False
                )
                can_bus.send(msg)
                can_bus.shutdown()

                if self.debug_mode:
                    self.get_logger().debug(f'X-axis (0x144) speed: {x_speed_dps:.1f} dps')
            except Exception as e:
                self.get_logger().error(f'X축 속도 제어 실패: {e}')
        else:
            # 조이스틱이 중립이면 정지
            try:
                can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                msg = can.Message(
                    arbitration_id=0x144,
                    data=[0xA2, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    is_extended_id=False
                )
                can_bus.send(msg)
                can_bus.shutdown()
            except:
                pass
        
        # Y축: AN2 -> 0x145 속도 제어 (반전 없음)
        y_value = -self.normalize_joystick(self.joystick_data['AN2'])

        # 안전: 리미트 센서 체크 및 방향 제한
        if abs(y_value) > 0.1:
            # IN00 (원점) ON이면 양수(+) 방향 차단 (반대로 수정)
            if self.limit_sensor_in00 and y_value > 0:
                self.get_logger().warning('IN00 (Y-axis home) sensor ON: blocking positive direction')
                y_value = 0.0

            # IN01 (최대) ON이면 음수(-) 방향 차단 (반대로 수정)
            if self.limit_sensor_in01 and y_value < 0:
                self.get_logger().warning('IN01 (Y-axis max) sensor ON: blocking negative direction')
                y_value = 0.0

        if abs(y_value) > 0.1:
            try:
                y_speed_dps = y_value * 200.0  # 최대 200 dps
                speed_control = int(y_speed_dps * 100)  # 0.01 dps/LSB

                can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                msg = can.Message(
                    arbitration_id=0x145,
                    data=[
                        0xA2,  # Speed Control Command
                        0x64,  # 100% max torque
                        0x00,
                        0x00,
                        speed_control & 0xFF,
                        (speed_control >> 8) & 0xFF,
                        (speed_control >> 16) & 0xFF,
                        (speed_control >> 24) & 0xFF
                    ],
                    is_extended_id=False
                )
                can_bus.send(msg)
                can_bus.shutdown()

                if self.debug_mode:
                    self.get_logger().debug(f'Y-axis (0x145) speed: {y_speed_dps:.1f} dps')
            except Exception as e:
                self.get_logger().error(f'Y축 속도 제어 실패: {e}')
        else:
            # 조이스틱이 중립이면 정지
            try:
                can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                msg = can.Message(
                    arbitration_id=0x145,
                    data=[0xA2, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    is_extended_id=False
                )
                can_bus.send(msg)
                can_bus.shutdown()
            except:
                pass
    
    def handle_lateral_move(self):
        """횡이동 제어 (S17/S18: ±360도 회전) 0x143"""
        # 홈잉 중일 때는 횡이동 제어 무시
        if self.homing_active:
            return
        
        if self.switch_pressed('S17'):
            # 양의 방향 360도 회전
            self.current_positions['lateral'] += 360.0  # degree
            self.publish_joint_position('lateral', self.joint1_pub, speed=150.0)  # 0x143 속도 낮춤 (과부하 방지)
            self.get_logger().info(f'>> 횡이동 +360도 (0x143, 누적: {self.current_positions["lateral"]:.1f}도)')

        elif self.switch_pressed('S18'):
            # 음의 방향 360도 회전
            self.current_positions['lateral'] -= 360.0  # degree
            self.publish_joint_position('lateral', self.joint1_pub, speed=150.0)  # 0x143 속도 낮춤 (과부하 방지)
            self.get_logger().info(f'<< 횡이동 -360도 (0x143, 누적: {self.current_positions["lateral"]:.1f}도)')

    def handle_work_sequence(self):
        """Z축 작업 시퀀스 (S21: 하강→닫힘, S22: 트리거→상승→열림) 0x146 - S19(Remote) 모드 전용"""
        # 홈잉 중일 때는 Z축 제어 무시
        if self.homing_active:
            return

        # S19(Remote) 모드에서만 S21/S22로 작업 시퀀스 제어
        # S20(Auto) 모드에서는 S21/S22가 횡이동 회전으로 사용되므로 여기서 처리 안 함

        if self.switch_pressed('S21'):
            self.get_logger().info('[DEBUG] S21 button pressed detected!')
            # S21 시퀀스: Z축 하강 → 그리퍼 닫기 (그리퍼는 이미 열려있다고 가정)
            # 주의: S21 실행 전 그리퍼를 미리 열어놓아야 함!
            if not self.s21_sequence_active:
                self.s21_sequence_active = True
                self.s21_sequence_step = 0

                self.get_logger().info('[S21 Sequence] Z축 하강 시작 (그리퍼 열린 상태 가정)')
                self.get_logger().info('  ⚠️  그리퍼가 닫혀있다면 먼저 열어주세요!')
                
                # Z축 하강만 실행 (그리퍼는 건드리지 않음)
                self.s21_sequence_move_down()
            else:
                self.get_logger().warning('WARNING: S21 sequence already in progress')

        elif self.switch_pressed('S22'):
            self.get_logger().info('[DEBUG] S22 button pressed detected!')
            # S22 시퀀스: 트리거 → Z축 상승 → 그리퍼 열기(유지)
            if not self.s22_sequence_active:
                self.s22_sequence_active = True
                self.s22_sequence_step = 0

                # 1. 트리거 동작
                self.get_logger().info('[S22 Sequence] 1단계: 트리거 동작 시작')
                self.trigger_pull()

                # 3초 후 Z축 상승 (트리거 완료 대기: 1초 당김 + 1초 되돌림 + 1초 여유)
                self.s22_sequence_timer = self.create_timer(3.0, self.s22_sequence_move_up)
            else:
                self.get_logger().warning('WARNING: S22 sequence already in progress')
    
    def s21_sequence_move_down(self):
        """S21 시퀀스: Z축 하강만 실행 (그리퍼는 건드리지 않음)"""
        if self.s21_sequence_timer:
            self.s21_sequence_timer.cancel()
            self.s21_sequence_timer = None

        # Z축 하강 시작 (그리퍼는 이미 열려있다고 가정, 추가 명령 없음)
        self.z_moving_down = True
        self.z_moving_to_limit = False
        self.current_positions['z'] -= 900.0  # degree
        self.publish_joint_position('z', self.joint4_pub)  # joint_4 = 0x146
        self.get_logger().info(f'[S21 Sequence] Z축 -약 2.5회전(900°) 하강 시작 (0x146, 누적: {self.current_positions["z"]:.1f}도)')
        self.get_logger().info('  그리퍼는 열린 상태 유지 (명령 없음)')

        # 6.0초 후 그리퍼 닫기 (하강 완료 대기: 900도 하강 시간 고려, 150dps 속도 기준 약 6초)
        self.s21_sequence_timer = self.create_timer(6.0, self.s21_sequence_gripper_close)

    def s21_sequence_gripper_close(self):
        """S21 시퀀스: 하강 완료 후 그리퍼 닫기"""
        if self.s21_sequence_timer:
            self.s21_sequence_timer.cancel()
            self.s21_sequence_timer = None

        self.get_logger().info('[S21 Sequence] 하강 완료 → 그리퍼 닫기')

        # 그리퍼 닫기 명령: g 5 -600 -> 0x5DA8 = 23976
        close_cmd = Int32()
        close_cmd.data = 23976  # Command 5, Speed -600 (g 5 -600)
        self.gripper_cmd_pub.publish(close_cmd)
        self.get_logger().info('  -> Published /gripper/command: 23976 (g 5 -600 - CLOSE)')

        # 시퀀스 완료
        self.s21_sequence_active = False
        self.z_moving_down = False
        self.get_logger().info('[S21 Sequence] ✅ Complete')

    def s22_sequence_move_up(self):
        """S22 시퀀스 2단계: Z축 상승"""
        if self.s22_sequence_timer:
            self.s22_sequence_timer.cancel()
            self.s22_sequence_timer = None

        # 2단계: Z축 원점 리미트까지 상승
        self.z_moving_down = False
        self.z_moving_to_limit = True
        self.current_positions['z'] += 3600.0  # degree (충분히 큰 값)
        self.publish_joint_position('z', self.joint4_pub)  # joint_4 = 0x146
        self.get_logger().info(f'[S22 Sequence] 2단계: Z축 상승 시작 (0x146, IN05 감지 대기)')

        # 8초 후 그리퍼 열기 (Z축 상승 완료 대기)
        self.s22_sequence_timer = self.create_timer(8.0, self.s22_sequence_open_gripper)

    def s22_sequence_open_gripper(self):
        """S22 시퀀스 3단계: 그리퍼 열기 (열린 상태 유지)"""
        if self.s22_sequence_timer:
            self.s22_sequence_timer.cancel()
            self.s22_sequence_timer = None

        self.get_logger().info('[S22 Sequence] 3단계: 그리퍼 열기')

        # 그리퍼 열기 명령: g 5 600 -> 0x5258 = 21080
        open_cmd = Int32()
        open_cmd.data = 21080  # Command 5, Speed +600 (g 5 600)
        self.gripper_cmd_pub.publish(open_cmd)
        self.get_logger().info('  -> Published /gripper/command: 21080 (g 5 600 - OPEN)')

        # 시퀀스 완료 (그리퍼는 열린 상태 유지)
        self.s22_sequence_active = False
        self.get_logger().info('[S22 Sequence] ✅ Complete - 그리퍼 열린 상태 유지')

    
    def handle_yaw_rotation(self):
        """Yaw 회전 제어 (S23: +yaw_angle, S24: -yaw_angle) 0x147 - S19(Manual) 모드 전용"""
        # 홈잉 중일 때는 Yaw 제어 무시
        if self.homing_active:
            return

        # S19(Remote) 모드에서만 S23/S24로 Yaw 제어
        # S20(Auto) 모드에서는 S23이 호밍으로 사용되므로 여기서 처리 안 함

        if self.switch_pressed('S23'):
            # 양의 방향 30도 회전
            prev_yaw = self.current_positions['yaw']
            self.current_positions['yaw'] += self.yaw_angle  # degree 단위 직접 처리
            self.get_logger().info(f'↻  Yaw +{self.yaw_angle:.1f}° (0x147, {prev_yaw:.1f}° → {self.current_positions["yaw"]:.1f}°)')
            self.publish_joint_position('yaw', self.joint5_pub, show_log=False)  # joint_5 = 0x147

        elif self.switch_pressed('S24'):
            # 음의 방향 30도 회전
            prev_yaw = self.current_positions['yaw']
            self.current_positions['yaw'] -= self.yaw_angle  # degree 단위 직접 처리
            self.get_logger().info(f'↺  Yaw -{self.yaw_angle:.1f}° (0x147, {prev_yaw:.1f}° → {self.current_positions["yaw"]:.1f}°)')
            self.publish_joint_position('yaw', self.joint5_pub, show_log=False)  # joint_5 = 0x147
    
    def handle_brake_release(self):
        """브레이크 해제/잠금 토글 (S13) - 위치제어 모터 0x143-0x147"""
        if self.switch_pressed('S13'):
            # 현재 상태 반전
            self.brake_released = not self.brake_released

            if self.brake_released:
                # 브레이크 해제 서비스 호출
                self.get_logger().info('🔓 브레이크 해제 서비스 호출...')
                self.call_brake_service(self.brake_release_client, '해제')
            else:
                # 브레이크 잠금 서비스 호출
                self.get_logger().info('🔒 브레이크 잠금 서비스 호출...')
                self.call_brake_service(self.brake_lock_client, '잠금')

            # LCD 디스플레이에 브레이크 상태 표시
            self.send_lcd_brake_status()

    def call_brake_service(self, client, action_name):
        """브레이크 서비스 호출 (비동기)"""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'WARNING: Brake {action_name} 서비스를 사용할 수 없습니다')
            return

        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.brake_service_callback(f, action_name))

    def brake_service_callback(self, future, action_name):
        """브레이크 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 브레이크 {action_name} 완료!')

                # 브레이크 해제 시 초기 위치 읽기 플래그 리셋
                if action_name == '해제':
                    self.initial_position_read = {'lateral': False, 'z': False, 'yaw': False, 'x': False, 'y': False, 'left_wheel': False, 'right_wheel': False}
                    self.get_logger().info('📍 모터 초기 위치 읽는 중... (1.5초 대기)')

                    # 1.5초 후 엔코더 위치 출력 (수동 취소 방식)
                    if hasattr(self, 'encoder_display_timer') and self.encoder_display_timer:
                        self.encoder_display_timer.cancel()
                    self.encoder_display_timer = self.create_timer(1.5, self._display_encoder_positions_wrapper)
            else:
                self.get_logger().warning(f'WARNING: Brake {action_name} 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ 브레이크 {action_name} 서비스 오류: {e}')

    def motor_position_callback(self, msg: Float32, motor_type: str):
        """모터 위치 콜백 (0x143: lateral, 0x144: x, 0x145: y, 0x146: z, 0x147: yaw, 0x141: left_wheel, 0x142: right_wheel)"""
        # 초기 위치만 읽고, 이후에는 피드백으로 위치를 덮어쓰지 않음 (누적 오류 방지)
        if motor_type == 'left_wheel':
            # 주행 모터 0x141 위치 업데이트 (S20 모드에서 사용)
            prev_pos = self.current_positions['left_wheel']
            self.current_positions['left_wheel'] = msg.data
            log_msg = f'🔄 [S20/AN3] 0x141 위치 업데이트: {prev_pos:.1f}° → {msg.data:.1f}° (변화: {msg.data - prev_pos:.1f}°)'
            self.get_logger().info(log_msg)
            self.file_logger.info(log_msg)
            self.file_log_file_handler.flush()
        elif motor_type == 'right_wheel':
            # 주행 모터 0x142 위치 업데이트 (S20 모드에서 사용)
            prev_pos = self.current_positions['right_wheel']
            self.current_positions['right_wheel'] = msg.data
            log_msg = f'🔄 [S20/AN3] 0x142 위치 업데이트: {prev_pos:.1f}° → {msg.data:.1f}° (변화: {msg.data - prev_pos:.1f}°)'
            self.get_logger().info(log_msg)
            self.file_logger.info(log_msg)
            self.file_log_file_handler.flush()
        elif motor_type == 'lateral':
            # 초기 위치 읽기 (브레이크 해제 직후 한 번만)
            if not self.initial_position_read[motor_type] and self.brake_released:
                self.current_positions['lateral'] = msg.data
                self.get_logger().info(f'✅ 0x143 (lateral) 초기 위치 읽기 완료: {msg.data:.1f}°')
                self.initial_position_read[motor_type] = True
        elif motor_type == 'x':
            # 초기 위치 읽기 (브레이크 해제 or 호밍 완료 직후)
            if not self.initial_position_read[motor_type]:
                self.current_positions['x'] = msg.data
                self.get_logger().info(f'✅ 0x144 (X축) 초기 위치 읽기 완료: {msg.data:.1f}°')
                self.initial_position_read[motor_type] = True
        elif motor_type == 'y':
            # 초기 위치 읽기 (브레이크 해제 or 호밍 완료 직후)
            if not self.initial_position_read[motor_type]:
                self.current_positions['y'] = msg.data
                self.get_logger().info(f'✅ 0x145 (Y축) 초기 위치 읽기 완료: {msg.data:.1f}°')
                self.initial_position_read[motor_type] = True
        elif motor_type == 'z':
            # 초기 위치 읽기 (브레이크 해제 or 호밍 완료 직후)
            if not self.initial_position_read[motor_type]:
                self.current_positions['z'] = msg.data
                self.get_logger().info(f'✅ 0x146 (Z축) 초기 위치 읽기 완료: {msg.data:.1f}°')
                self.initial_position_read[motor_type] = True
        elif motor_type == 'yaw':
            # 초기 위치 읽기 (브레이크 해제 or 호밍 완료 직후)
            if not self.initial_position_read[motor_type]:
                self.current_positions['yaw'] = msg.data
                self.get_logger().info(f'✅ 0x147 (yaw) 초기 위치 읽기 완료: {msg.data:.1f}°')
                self.initial_position_read[motor_type] = True

    def _display_encoder_positions_wrapper(self):
        """엔코더 위치 출력 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'encoder_display_timer') and self.encoder_display_timer:
            self.encoder_display_timer.cancel()
            self.encoder_display_timer = None
        self.display_encoder_positions()

    def display_encoder_positions(self):
        """현재 엔코더 위치 출력 (브레이크 해제 후 자동 호출)"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('📍 ===== 현재 모터 엔코더 위치 =====')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  0x143 (횡이동): {self.current_positions["lateral"]:.2f}°')
        self.get_logger().info(f'  0x144 (X축): {self.current_positions["x"]:.2f}°')
        self.get_logger().info(f'  0x145 (Y축): {self.current_positions["y"]:.2f}°')
        self.get_logger().info(f'  0x146 (Z축): {self.current_positions["z"]:.2f}°')
        self.get_logger().info(f'  0x147 (Yaw): {self.current_positions["yaw"]:.2f}°')
        self.get_logger().info('=' * 60)

    def limit_sensor_in05_callback(self, msg: Bool):
        """EZI-IO IN05 리미트 센서 콜백 (Z축 상단 리미트)"""
        prev_state = self.limit_sensor_in05
        self.limit_sensor_in05 = msg.data
        
        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in05:
            status = "ON (감지됨)" if self.limit_sensor_in05 else "OFF"
            self.get_logger().info(f'🔴 IN05 리미트 센서 (상단): {status}')
        
        # 상단 리미트까지 이동 중이고 센서가 ON되면 정지
        if self.z_moving_to_limit and self.limit_sensor_in05:
            self.get_logger().info('✅ 상단 리미트 감지! Z축 긴급 정지')
            self.z_moving_to_limit = False
            # 0x146 모터에 긴급 정지 명령 전송 (CAN2를 통해)
            self.send_motor_emergency_stop(0x146)
            # Z축 위치를 원점(0도)으로 리셋
            self.current_positions['z'] = 0.0
            self.get_logger().info('🏠 Z축 위치 원점(0°)으로 리셋')
    
    def limit_sensor_in06_callback(self, msg: Bool):
        """EZI-IO IN06 리미트 센서 콜백 (Z축 하단 리미트)"""
        prev_state = self.limit_sensor_in06
        self.limit_sensor_in06 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in06:
            status = "ON (감지됨)" if self.limit_sensor_in06 else "OFF"
            self.get_logger().info(f'🔴 IN06 리미트 센서 (하단): {status}')

        # 하강 중이고 센서가 ON되면 정지
        if self.z_moving_down and self.limit_sensor_in06:
            self.get_logger().info('✅ 하단 리미트 감지! Z축 긴급 정지')
            self.z_moving_down = False
            # 0x146 모터에 긴급 정지 명령 전송 (CAN2를 통해)
            self.send_motor_emergency_stop(0x146)

    def limit_sensor_in00_callback(self, msg: Bool):
        """EZI-IO IN00 리미트 센서 콜백 (Y축 원점 리미트, 0x145 음수 제한)"""
        prev_state = self.limit_sensor_in00
        self.limit_sensor_in00 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in00:
            status = "ON (감지됨)" if self.limit_sensor_in00 else "OFF"
            self.get_logger().info(f'🟢 IN00 리미트 센서 (Y축 원점): {status}')

        # 센서 ON되면 0x145 긴급 정지
        if self.limit_sensor_in00 and not prev_state:
            self.get_logger().warning('WARNING: IN00 detected! 0x145 긴급 정지')
            self.send_motor_emergency_stop(0x145)

    def limit_sensor_in01_callback(self, msg: Bool):
        """EZI-IO IN01 리미트 센서 콜백 (Y축 최대 리미트, 0x145 양수 제한)"""
        prev_state = self.limit_sensor_in01
        self.limit_sensor_in01 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in01:
            status = "ON (감지됨)" if self.limit_sensor_in01 else "OFF"
            self.get_logger().info(f'🟡 IN01 리미트 센서 (Y축 최대): {status}')

        # 스테이지 호밍 중이면 호밍 완료 체크
        if self.stage_homing_active:
            self.check_stage_homing_complete()
            return

        # 일반 동작 중 센서 ON되면 0x145 긴급 정지
        if self.limit_sensor_in01 and not prev_state:
            self.get_logger().warning('WARNING: IN01 detected! 0x145 긴급 정지')
            self.send_motor_emergency_stop(0x145)

    def limit_sensor_in02_callback(self, msg: Bool):
        """EZI-IO IN02 리미트 센서 콜백 (X축 홈 리미트, 0x144)"""
        prev_state = self.limit_sensor_in02
        self.limit_sensor_in02 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in02:
            status = "ON (감지됨)" if self.limit_sensor_in02 else "OFF"
            self.get_logger().info(f'🟢 IN02 리미트 센서 (X축 홈): {status}')

        # 스테이지 호밍 중이면 호밍 완료 체크
        if self.stage_homing_active:
            self.check_stage_homing_complete()
            return

        # 일반 동작 중 센서 ON되면 0x144 긴급 정지
        if self.limit_sensor_in02 and not prev_state:
            self.get_logger().warning('WARNING: IN02 detected! 0x144 긴급 정지')
            self.send_motor_emergency_stop(0x144)

        # 드라이브 모터 홈잉 중일 때 센서 상태에 따라 처리
        if not self.homing_active:
            return

        # 상태별 처리
        if self.homing_state == 'releasing_sensor':
            # 센서 해제 중 → IN02 OFF되면 다음 단계로
            if not self.limit_sensor_in02:
                self.get_logger().info('✅ IN02 센서 해제 완료!')
                # 긴급 정지
                self.send_motor_emergency_stop(0x141)
                self.send_motor_emergency_stop(0x142)
                # 다음 단계로: 최종 접근 (수동 취소 방식)
                if hasattr(self, 'homing_timer') and self.homing_timer:
                    self.homing_timer.cancel()
                self.homing_timer = self.create_timer(0.5, self._homing_final_approach_wrapper)

        elif self.homing_state == 'searching_sensor':
            # 센서 탐색 중 → IN02 ON되면 긴급정지
            if self.limit_sensor_in02:
                self.get_logger().info('✅ IN02 센서 감지!')
                # 긴급 정지
                self.send_motor_emergency_stop(0x141)
                self.send_motor_emergency_stop(0x142)
                # 다음 단계로: 센서 해제 (수동 취소 방식)
                if hasattr(self, 'homing_timer') and self.homing_timer:
                    self.homing_timer.cancel()
                self.homing_timer = self.create_timer(0.5, self._homing_release_sensor_wrapper)

        elif self.homing_state == 'final_approach':
            # 최종 접근 중 → IN02 ON되면 홈잉 완료
            if self.limit_sensor_in02:
                self.get_logger().info('✅ 홈 리미트 최종 감지! 드라이브 모터 긴급 정지')
                self.homing_active = False
                self.homing_state = 'idle'
                # 긴급 정지
                self.send_motor_emergency_stop(0x141)
                self.send_motor_emergency_stop(0x142)
                self.get_logger().info('🏠 홈잉 완료!')

    def limit_sensor_in03_callback(self, msg: Bool):
        """EZI-IO IN03 리미트 센서 콜백 (X축 최대 리미트, 0x144)"""
        prev_state = self.limit_sensor_in03
        self.limit_sensor_in03 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in03:
            status = "ON (감지됨)" if self.limit_sensor_in03 else "OFF"
            self.get_logger().info(f'🟡 IN03 리미트 센서 (X축 최대): {status}')

        # 일반 동작 중 센서 ON되면 0x144 긴급 정지
        if self.limit_sensor_in03 and not prev_state:
            self.get_logger().warning('WARNING: IN03 detected! 0x144 긴급 정지')
            self.send_motor_emergency_stop(0x144)

    def limit_sensor_in04_callback(self, msg: Bool):
        """EZI-IO IN04 리미트 센서 콜백 (Yaw 홈 리미트, 0x147)"""
        prev_state = self.limit_sensor_in04
        self.limit_sensor_in04 = msg.data

        # 센서 상태 변화 시 로그 출력
        if prev_state != self.limit_sensor_in04:
            status = "ON (감지됨)" if self.limit_sensor_in04 else "OFF"
            self.get_logger().info(f'🔵 IN04 리미트 센서 (Yaw 홈): {status}')

        # 스테이지 호밍 중이면 호밍 완료 체크
        if self.stage_homing_active:
            self.check_stage_homing_complete()
            return

        # 일반 동작 중 센서 ON되면 0x147 긴급 정지
        if self.limit_sensor_in04 and not prev_state:
            self.get_logger().warning('WARNING: IN04 detected! 0x147 긴급 정지')
            self.send_motor_emergency_stop(0x147)
    
    def motor_goal_reached_callback(self, msg):
        """모터 목표 도달 콜백 - 작업 시퀀스 다음 단계 진행"""
        motor_id = msg.data

        # 작업 시퀀스 진행 중이 아니면 무시
        if not self.work_sequence_active:
            return

        # Step 2 (Z축 하강 1차) 완료 → Step 3 (트리거 1차) 시작
        if self.work_sequence_step == 2 and motor_id == 0x146:
            self.get_logger().info('✅ Z축 하강 완료! 트리거 동작 시작')
            self._work_sequence_step3_wrapper()

        # Step 4 (Z축 상승 1차) 완료 → Step 5 (X축 이동) 시작
        elif self.work_sequence_step == 4 and motor_id == 0x146:
            self.get_logger().info('✅ Z축 상승 완료! X축 원점방향 이동 시작')
            self._work_sequence_step5_wrapper()

        # Step 5 (X축 이동) 완료 → Step 6 (Z축 하강 2차) 시작
        elif self.work_sequence_step == 5 and motor_id == 0x144:
            self.get_logger().info('✅ X축 이동 완료! Z축 하강 시작 (2차)')
            self._work_sequence_step6_wrapper()

        # Step 6 (Z축 하강 2차) 완료 → Step 7 (트리거 2차) 시작
        elif self.work_sequence_step == 6 and motor_id == 0x146:
            self.get_logger().info('✅ Z축 하강 완료 (2차)! 트리거 동작 시작 (2차)')
            self._work_sequence_step7_wrapper()

        # Step 8 (Z축 상승 2차) 완료 → Step 9 (XYYaw 3축 이동) 시작
        elif self.work_sequence_step == 8 and motor_id == 0x146:
            self.get_logger().info('✅ Z축 상승 완료 (2차)! X/Y/Yaw 3축 이동 시작')
            self._work_sequence_step9_wrapper()

        # Step 9 (XYYaw 3축 이동) - 3축 모두 완료 확인
        elif self.work_sequence_step == 9:
            if motor_id in [0x144, 0x145, 0x147]:  # X, Y, Yaw 모터
                self.step9_completed_motors.add(motor_id)
                self.get_logger().info(f'✅ 모터 0x{motor_id:03X} 이동 완료 ({len(self.step9_completed_motors)}/3)')

                # 3축 모두 완료되면 Step 10으로 진행
                if len(self.step9_completed_motors) == 3:
                    self.get_logger().info('✅ X/Y/Yaw 3축 이동 완료! Z축 하강 시작 (3차)')
                    self._work_sequence_step10_wrapper()

        # Step 10 (Z축 하강 3차) 완료 → Step 11 (트리거 3차) 시작
        elif self.work_sequence_step == 10 and motor_id == 0x146:
            self.get_logger().info('✅ Z축 하강 완료 (3차)! 트리거 동작 시작 (3차)')
            self._work_sequence_step11_wrapper()

        # Step 12 (Z축 상승 3차) 완료 → Step 13 (XY 2축 이동) 시작
        elif self.work_sequence_step == 12 and motor_id == 0x146:
            self.get_logger().info('✅ Z축 상승 완료 (3차)! X/Y 2축 이동 시작')
            self._work_sequence_step13_wrapper()

        # Step 13 (XY 2축 이동) - 2축 모두 완료 확인
        elif self.work_sequence_step == 13:
            if motor_id in [0x144, 0x145]:  # X, Y 모터
                self.step13_completed_motors.add(motor_id)
                self.get_logger().info(f'✅ 모터 0x{motor_id:03X} 이동 완료 ({len(self.step13_completed_motors)}/2)')

                # 2축 모두 완료되면 Step 14로 진행
                if len(self.step13_completed_motors) == 2:
                    self.get_logger().info('✅ X/Y 2축 이동 완료! Z축 하강 시작 (4차)')
                    self._work_sequence_step14_wrapper()

        # Step 14 (Z축 하강 4차) 완료 → Step 15 (트리거 4차) 시작
        elif self.work_sequence_step == 14 and motor_id == 0x146:
            self.get_logger().info('✅ Z축 하강 완료 (4차)! 트리거 동작 시작 (4차)')
            self._work_sequence_step15_wrapper()

        # Step 16 (Z축 상승 4차) 완료 → 전체 작업 완료
        elif self.work_sequence_step == 16 and motor_id == 0x146:
            self.get_logger().info('✅ Z축 상승 완료 (4차)! 전체 작업 완료')
            self._work_sequence_complete_wrapper()
    
    def send_motor_emergency_stop(self, motor_id):
        """모터 긴급 정지 명령 전송 (0x81)"""
        try:
            import can
            # CAN2 버스로 긴급 정지 명령 전송
            can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
            
            # RMD 긴급 정지 명령: 0x81
            msg = can.Message(
                arbitration_id=motor_id,
                data=[0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            can_bus.send(msg)
            can_bus.shutdown()
            
            self.get_logger().info(f'🛑 모터 0x{motor_id:03X} 긴급 정지 명령 전송')
        except Exception as e:
            self.get_logger().error(f'❌ 긴급 정지 명령 전송 실패: {e}')
    
    def request_encoder_read(self, motor_id):
        """모터 멀티턴 엔코더 읽기 명령 전송 (0x92)"""
        try:
            import can
            # CAN2 버스로 멀티턴 엔코더 읽기 명령 전송
            can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
            
            # RMD 멀티턴 엔코더 읽기: 0x92
            msg = can.Message(
                arbitration_id=motor_id,
                data=[0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            can_bus.send(msg)
            can_bus.shutdown()
            
            self.get_logger().info(f'📍 모터 0x{motor_id:03X} 엔코더 읽기 명령 전송')
        except Exception as e:
            self.get_logger().error(f'❌ 엔코더 읽기 명령 전송 실패: {e}')
    
    def start_homing_sequence(self):
        """드라이브 모터 홈잉 시퀀스 시작"""
        if self.homing_active:
            self.get_logger().warn('WARNING: Homing already in progress')
            return

        self.get_logger().info('🏁 드라이브 모터(0x141/0x142) 홈잉 시작')
        self.homing_active = True

        # 초기 IN02 상태 저장
        self.homing_initial_in02 = self.limit_sensor_in02

        if self.homing_initial_in02:
            # IN02 ON인 경우: 센서 해제 단계부터 시작
            self.get_logger().info('📍 초기 상태: IN02 ON → 센서 해제 단계 시작')
            self.homing_release_sensor()
        else:
            # IN02 OFF인 경우: 센서 탐색 단계부터 시작
            self.get_logger().info('📍 초기 상태: IN02 OFF → 센서 탐색 단계 시작')
            self.homing_search_sensor()

    def homing_release_sensor(self):
        """홈잉: 센서 해제 단계 (저속 전진)"""
        if not self.homing_active:
            return

        self.homing_state = 'releasing_sensor'
        self.get_logger().info(f'➡️  [1단계] 센서 해제: 저속({self.homing_speed_slow} m/s) 전진 중...')

        # 저속 전진
        cmd_vel = Twist()
        cmd_vel.linear.x = self.homing_speed_slow
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def homing_search_sensor(self):
        """홈잉: 센서 탐색 단계 (중속 후진)"""
        if not self.homing_active:
            return

        self.homing_state = 'searching_sensor'
        self.get_logger().info(f'⬅️  [2단계] 센서 탐색: 중속({self.homing_speed_medium} m/s) 후진 중...')

        # 중속 후진
        cmd_vel = Twist()
        cmd_vel.linear.x = -self.homing_speed_medium
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def _homing_release_sensor_wrapper(self):
        """홈잉: 센서 해제 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'homing_timer') and self.homing_timer:
            self.homing_timer.cancel()
            self.homing_timer = None
        self.homing_release_sensor()

    def _homing_final_approach_wrapper(self):
        """홈잉: 최종 접근 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'homing_timer') and self.homing_timer:
            self.homing_timer.cancel()
            self.homing_timer = None
        self.homing_final_approach()

    def homing_final_approach(self):
        """홈잉: 최종 접근 단계 (저속 후진)"""
        if not self.homing_active:
            return

        self.homing_state = 'final_approach'
        self.get_logger().info(f'⬅️  [3단계] 최종 접근: 저속({self.homing_speed_slow} m/s) 후진 중...')

        # 저속 후진
        cmd_vel = Twist()
        cmd_vel.linear.x = -self.homing_speed_slow
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def clear_motor_error(self, motor_id):
        """모터 에러 클리어 (0x9B)"""
        try:
            import can

            can_bus = can.interface.Bus(channel='can2', bustype='socketcan')

            # RMD 에러 클리어 명령: 0x9B
            msg = can.Message(
                arbitration_id=motor_id,
                data=[0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            can_bus.send(msg)
            can_bus.shutdown()

            self.get_logger().info(f'🔧 모터 0x{motor_id:03X} 에러 클리어')
        except Exception as e:
            self.get_logger().error(f'❌ 에러 클리어 실패: {e}')

    def start_stage_homing(self):
        """스테이지 XY + Yaw 2단계 호밍 시작 (S23 트리거)"""
        self.get_logger().info('🏁 스테이지 XYZ 호밍 시작 (2단계)')
        self.get_logger().info('  [1차] 고속 호밍: X축(200 dps), Y축(200 dps), Yaw(100 dps)')
        self.get_logger().info('  → X축(0x144): +방향 이동 (IN02 감지까지)')
        self.get_logger().info('  → Y축(0x145): -방향 이동 (IN01 감지까지)')
        self.get_logger().info('  → Yaw(0x147): -방향 이동 (IN04 감지까지)')

        self.stage_homing_active = True
        self.stage_homing_phase = 1  # 1차 고속 호밍
        self.stage_x_homing_done = False
        self.stage_y_homing_done = False
        self.stage_yaw_homing_done = False
        self.stage_x_homing_phase2_done = False
        self.stage_y_homing_phase2_done = False
        self.stage_yaw_homing_phase2_done = False
        
        # 호밍 중 엔코더 피드백을 받기 위해 초기 위치 읽기 플래그 리셋
        self.initial_position_read['x'] = False
        self.initial_position_read['y'] = False
        self.initial_position_read['yaw'] = False
        self.initial_position_read['z'] = False

        # 1차 고속 호밍: X축 +방향, Y축 -방향, Yaw -방향
        self.move_stage_motor(0x144, +self.stage_xy_homing_speed)   # X축 +방향 (100 dps)
        self.move_stage_motor(0x145, -self.stage_xy_homing_speed)   # Y축 -방향 (100 dps)
        self.move_stage_motor(0x147, -self.stage_yaw_homing_speed)  # Yaw -방향 (10 dps)

    def move_stage_motor(self, motor_id, speed_dps):
        """스테이지 모터 속도 제어 (0xA2 Speed Control)"""
        try:
            import can
            import struct

            # 속도 제어 값 계산 (0.01 dps/LSB)
            speed_control = int(speed_dps * 100)

            can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
            msg = can.Message(
                arbitration_id=motor_id,
                data=[
                    0xA2,  # Speed Control Command
                    0x64,  # 100% max torque
                    0x00,
                    0x00,
                    speed_control & 0xFF,
                    (speed_control >> 8) & 0xFF,
                    (speed_control >> 16) & 0xFF,
                    (speed_control >> 24) & 0xFF
                ],
                is_extended_id=False
            )
            can_bus.send(msg)
            can_bus.shutdown()

            if self.debug_mode:
                self.get_logger().debug(f'Motor 0x{motor_id:03X} speed: {speed_dps:.1f} dps')
        except Exception as e:
            self.get_logger().error(f'❌ 모터 0x{motor_id:03X} 속도 제어 실패: {e}')

    def check_stage_homing_complete(self):
        """스테이지 2단계 호밍 완료 확인"""
        if not self.stage_homing_active:
            return

        # ========== 1단계: 고속 호밍 ==========
        if self.stage_homing_phase == 1:
            # X축 1차 홈 도달 확인 (IN02 ON)
            if self.limit_sensor_in02 and not self.stage_x_homing_done:
                self.get_logger().info('✅ [1차] X축(0x144) 홈 리미트(IN02) 감지!')
                self.send_motor_emergency_stop(0x144)
                self.stage_x_homing_done = True

            # Y축 1차 홈 도달 확인 (IN01 ON)
            if self.limit_sensor_in01 and not self.stage_y_homing_done:
                self.get_logger().info('✅ [1차] Y축(0x145) 홈 리미트(IN01) 감지!')
                self.send_motor_emergency_stop(0x145)
                self.stage_y_homing_done = True

            # Yaw 1차 홈 도달 확인 (IN04 ON)
            if self.limit_sensor_in04 and not self.stage_yaw_homing_done:
                self.get_logger().info('✅ [1차] Yaw(0x147) 홈 리미트(IN04) 감지!')
                self.send_motor_emergency_stop(0x147)
                self.stage_yaw_homing_done = True

            # 3축 모두 1차 완료되면 → 2단계: 센서 이탈
            if self.stage_x_homing_done and self.stage_y_homing_done and self.stage_yaw_homing_done:
                self.get_logger().info('🔄 [2단계] 센서 이탈 중...')
                self.stage_homing_phase = 2
                
                # 센서에서 벗어나기 (반대 방향으로 살짝 이동)
                self.move_stage_motor(0x144, -30.0)  # X축 -방향 (30 dps, 0.5초)
                self.move_stage_motor(0x145, +30.0)  # Y축 +방향 (30 dps)
                self.move_stage_motor(0x147, +5.0)   # Yaw +방향 (5 dps)
                
                return

        # ========== 2단계: 센서 이탈 확인 ==========
        elif self.stage_homing_phase == 2:
            # 각 축 센서 이탈 확인
            x_cleared = not self.limit_sensor_in02
            y_cleared = not self.limit_sensor_in01
            yaw_cleared = not self.limit_sensor_in04
            
            # 모든 센서 이탈했으면 3단계 진입
            if x_cleared and y_cleared and yaw_cleared:
                self.get_logger().info('✅ [2단계] 모든 센서 이탈 완료')
                
                # 모터 정지
                self.send_motor_emergency_stop(0x144)
                self.send_motor_emergency_stop(0x145)
                self.send_motor_emergency_stop(0x147)
                
                # 0.2초 후 3단계 시작
                self.phase3_start_timer = self.create_timer(0.2, self._start_phase3_homing_wrapper)
                self.stage_homing_phase = 2.5  # 대기 상태
                
            return

        # ========== 3단계: 저속 정밀 호밍 ==========
        elif self.stage_homing_phase == 3:
            # X축 2차 홈 도달 확인 (IN02 ON)
            if self.limit_sensor_in02 and not self.stage_x_homing_phase2_done:
                self.get_logger().info('✅ [2차] X축(0x144) 홈 리미트(IN02) 정밀 감지!')
                self.send_motor_emergency_stop(0x144)
                self.stage_x_homing_phase2_done = True

            # Y축 2차 홈 도달 확인 (IN01 ON)
            if self.limit_sensor_in01 and not self.stage_y_homing_phase2_done:
                self.get_logger().info('✅ [2차] Y축(0x145) 홈 리미트(IN01) 정밀 감지!')
                self.send_motor_emergency_stop(0x145)
                self.stage_y_homing_phase2_done = True

            # Yaw 2차 홈 도달 확인 (IN04 ON)
            if self.limit_sensor_in04 and not self.stage_yaw_homing_phase2_done:
                self.get_logger().info('✅ [2차] Yaw(0x147) 홈 리미트(IN04) 정밀 감지!')
                self.send_motor_emergency_stop(0x147)
                self.stage_yaw_homing_phase2_done = True

            # 3축 모두 2차 완료되면 → 호밍 완료
            if self.stage_x_homing_phase2_done and self.stage_y_homing_phase2_done and self.stage_yaw_homing_phase2_done:
                self.get_logger().info('🏠 스테이지 XY + Yaw 2단계 호밍 완료!')
                self.stage_homing_active = False
                
                # 0.3초 대기 후 엔코더 읽기 시작 (모터가 안정화될 시간)
                if hasattr(self, 'encoder_read_timer') and self.encoder_read_timer:
                    self.encoder_read_timer.cancel()
                self.encoder_read_timer = self.create_timer(0.3, self._start_encoder_read_wrapper)

    def start_phase3_homing(self):
        """3단계: 저속 정밀 호밍 시작"""
        # 모터 정지
        self.send_motor_emergency_stop(0x144)
        self.send_motor_emergency_stop(0x145)
        self.send_motor_emergency_stop(0x147)
        time.sleep(0.1)
        
        self.get_logger().info('🔍 [3단계] 저속 정밀 호밍 시작')
        self.get_logger().info('  → X축(50 dps), Y축(50 dps), Yaw(20 dps)')
        
        self.stage_homing_phase = 3
        
        # 저속으로 다시 센서 방향으로 이동
        self.move_stage_motor(0x144, +self.stage_xy_homing_speed_slow)   # X축 +방향 (20 dps, 저속)
        self.move_stage_motor(0x145, -self.stage_xy_homing_speed_slow)   # Y축 -방향 (20 dps, 저속)
        self.move_stage_motor(0x147, -self.stage_yaw_homing_speed_slow)  # Yaw -방향 (5 dps, 저속)

    def _start_phase3_homing_wrapper(self):
        """3단계 호밍 시작 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'phase3_start_timer') and self.phase3_start_timer:
            self.phase3_start_timer.cancel()
            self.phase3_start_timer = None
        self.start_phase3_homing()

    def _save_home_positions_wrapper(self):
        """홈 위치 저장 래퍼 (타이머 자동 취소 및 엔코더 위치 출력)"""
        if hasattr(self, 'homing_save_timer') and self.homing_save_timer:
            self.homing_save_timer.cancel()
            self.homing_save_timer = None
        
        # 현재 위치를 홈 위치로 저장 (실제 엔코더 절대값)
        self.home_x_encoder_position = self.current_positions['x']
        self.home_y_encoder_position = self.current_positions['y']
        self.home_yaw_encoder_position = self.current_positions['yaw']
        
        self.get_logger().info(f'📍 홈 위치 저장:')
        self.get_logger().info(f'  → X축 홈: {self.home_x_encoder_position:.2f}°')
        self.get_logger().info(f'  → Y축 홈: {self.home_y_encoder_position:.2f}°')
        self.get_logger().info(f'  → Z축: {self.current_positions["z"]:.2f}°')
        self.get_logger().info(f'  → Yaw 홈: {self.home_yaw_encoder_position:.2f}°')
        
        # 엔코더 위치 출력
        self.display_stage_encoder_positions()

    def _display_stage_encoder_wrapper(self):
        """스테이지 엔코더 출력 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'stage_encoder_timer') and self.stage_encoder_timer:
            self.stage_encoder_timer.cancel()
            self.stage_encoder_timer = None
        self.display_stage_encoder_positions()

    def _start_encoder_read_wrapper(self):
        """엔코더 읽기 시작 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'encoder_read_timer') and self.encoder_read_timer:
            self.encoder_read_timer.cancel()
            self.encoder_read_timer = None
        
        # 직접 CAN으로 멀티턴 엔코더 읽기 (0x92)
        self.get_logger().info('📍 엔코더 위치 읽는 중...')
        # 1차 엔코더 읽기 명령 전송 (트리거용)
        self.request_encoder_read(0x144)  # X축
        time.sleep(0.20)
        self.request_encoder_read(0x145)  # Y축
        time.sleep(0.20)
        self.request_encoder_read(0x146)  # Z축
        time.sleep(0.20)
        self.request_encoder_read(0x147)  # Yaw

        # 2차 읽기를 0.5초 후 스케줄 (첫 읽기 트리거 이후 값 수신을 보장)
        if hasattr(self, 'encoder_second_round_timer') and self.encoder_second_round_timer:
            self.encoder_second_round_timer.cancel()
        self.encoder_second_round_timer = self.create_timer(0.5, self._start_encoder_read_second_round_wrapper)

    def _start_encoder_read_second_round_wrapper(self):
        """엔코더 2차 읽기 래퍼 및 홈 저장 트리거"""
        if hasattr(self, 'encoder_second_round_timer') and self.encoder_second_round_timer:
            self.encoder_second_round_timer.cancel()
            self.encoder_second_round_timer = None

        self.get_logger().info('📍 엔코더 2차 읽기 시작...')

        # 재차 플래그 리셋(안전) 및 2차 읽기 전송
        self.initial_position_read['x'] = False
        self.initial_position_read['y'] = False
        self.initial_position_read['yaw'] = False
        self.initial_position_read['z'] = False

        self.request_encoder_read(0x144)
        time.sleep(0.20)
        self.request_encoder_read(0x145)
        time.sleep(0.20)
        self.request_encoder_read(0x146)
        time.sleep(0.20)
        self.request_encoder_read(0x147)

        # 2.0초 후 홈 위치 저장 (엔코더 값이 업데이트될 시간)
        if hasattr(self, 'homing_save_timer') and self.homing_save_timer:
            self.homing_save_timer.cancel()
        self.homing_save_timer = self.create_timer(2.0, self._save_home_positions_wrapper)
        time.sleep(0.2)
        self.request_encoder_read(0x147)  # Yaw
    
        # 2.5초 후 홈 위치 저장 (엔코더 값이 충분히 업데이트될 시간)
        if hasattr(self, 'homing_save_timer') and self.homing_save_timer:
            self.homing_save_timer.cancel()
        self.homing_save_timer = self.create_timer(2.5, self._save_home_positions_wrapper)

    def display_stage_encoder_positions(self):
        """스테이지 엔코더 위치 출력 (호밍 완료 후)"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('📍 ===== 스테이지 엔코더 위치 (호밍 후) =====')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  0x144 (X축): {self.current_positions["x"]:.2f}° (홈위치)')
        self.get_logger().info(f'  0x145 (Y축): {self.current_positions["y"]:.2f}° (홈위치)')
        self.get_logger().info(f'  0x146 (Z축): {self.current_positions["z"]:.2f}°')
        self.get_logger().info(f'  0x147 (Yaw): {self.current_positions["yaw"]:.2f}° (홈위치)')
        self.get_logger().info('=' * 60)

    def handle_homing(self):
        """드라이브 모터 홈잉 (S14)"""
        # 디버그: S14 상태 출력
        current = self.switch_data.get('S14', 0)
        previous = self.prev_switches.get('S14', 0)

        if current != previous:
            self.get_logger().info(f'🔍 [DEBUG] S14 상태 변화: {previous} → {current}')

        if self.switch_pressed('S14'):
            self.get_logger().info('🔘 S14 버튼 눌림 감지!')
            self.start_homing_sequence()

    def trigger_pull(self):
        """트리거 당기기 (SmcCmd 사용) - S22 작업 시퀀스에서만 호출"""
        self.get_logger().info('Trigger pull started!')
        
        try:
            import subprocess
            SMC_CMD = '/home/koceti/ros2_ws/src/smc_linux/SmcCmd'
            DEVICE = '#51FF-7406-4980-4956-3043-1287'  # Pololu 시리얼 넘버
            
            # Resume 명령으로 safe-start 해제
            subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--resume'],
                capture_output=True,
                timeout=2
            )
            
            # Forward at 100% speed (3200) - 트리거 당김
            result = subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--speed', '3200'],
                capture_output=True,
                timeout=2,
                text=True
            )
            
            if result.returncode != 0:
                self.get_logger().error(f'트리거 당김 실패: {result.stderr}')
                return
            
            self.get_logger().info('Trigger pull: forward command sent')

            # 1초 후 역방향 타이머 설정 (수동 취소 방식)
            if hasattr(self, 'trigger_pull_timer') and self.trigger_pull_timer:
                self.trigger_pull_timer.cancel()
            self.trigger_pull_timer = self.create_timer(1.0, self._trigger_reverse_wrapper)

        except Exception as e:
            self.get_logger().error(f'Trigger pull failed: {e}')

    def _trigger_reverse_wrapper(self):
        """트리거 reverse 래퍼 (타이머 자동 취소)"""
        if hasattr(self, 'trigger_pull_timer') and self.trigger_pull_timer:
            self.trigger_pull_timer.cancel()
            self.trigger_pull_timer = None
        self.trigger_reverse()
    
    def trigger_reverse(self):
        """트리거 되돌림 (1초 후 호출)"""
        try:
            import subprocess
            SMC_CMD = '/home/koceti/ros2_ws/src/smc_linux/SmcCmd'
            DEVICE = '#51FF-7406-4980-4956-3043-1287'

            # Resume 명령으로 safe-start 해제 (정지 후 재시작을 위해 필요)
            subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--resume'],
                capture_output=True,
                timeout=2
            )

            # Reverse at 100% speed (-3200) - 트리거 되돌림
            result = subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--speed', '-3200'],
                capture_output=True,
                timeout=2,
                text=True
            )
            
            if result.returncode != 0:
                self.get_logger().error(f'트리거 되돌림 실패: {result.stderr}')
                return
            
            self.get_logger().info('Trigger reverse: backward command sent')

            # 타이머로 자동 정지 설정 (0.5초 후, 수동 취소 방식)
            if self.trigger_timer:
                self.trigger_timer.cancel()
            self.trigger_timer = self.create_timer(self.trigger_duration, self._trigger_release_wrapper)

        except Exception as e:
            self.get_logger().error(f'Trigger reverse failed: {e}')

    def _trigger_release_wrapper(self):
        """트리거 release 래퍼 (타이머 자동 취소)"""
        if self.trigger_timer:
            self.trigger_timer.cancel()
            self.trigger_timer = None
        self.trigger_release()
    
    def trigger_release(self):
        """트리거 해제 (SmcCmd 사용)"""
        try:
            import subprocess
            SMC_CMD = '/home/koceti/ros2_ws/src/smc_linux/SmcCmd'
            DEVICE = '#51FF-7406-4980-4956-3043-1287'  # Pololu 시리얼 넘버
            
            # Stop motor
            result = subprocess.run(
                [SMC_CMD, '-d', DEVICE, '--speed', '0'],
                capture_output=True,
                timeout=2,
                text=True
            )
            
            if result.returncode != 0:
                self.get_logger().error(f'트리거 해제 실패: {result.stderr}')

            if self.trigger_timer:
                self.trigger_timer.cancel()
                self.trigger_timer = None

            self.get_logger().info('🔫 트리거 해제')

            # 작업 시퀀스 중이면 다음 단계(Z축 상승)로 진행
            if self.work_sequence_active and self.work_sequence_step == 3:
                self.get_logger().info('📍 트리거 완료 (1차), Z축 상승 시작')
                self._work_sequence_step4_wrapper()
            elif self.work_sequence_active and self.work_sequence_step == 7:
                self.get_logger().info('📍 트리거 완료 (2차), Z축 상승 시작')
                self._work_sequence_step8_wrapper()
            elif self.work_sequence_active and self.work_sequence_step == 11:
                self.get_logger().info('📍 트리거 완료 (3차), Z축 상승 시작')
                self._work_sequence_step12_wrapper()
            elif self.work_sequence_active and self.work_sequence_step == 15:
                self.get_logger().info('📍 트리거 완료 (4차), Z축 상승 시작')
                self._work_sequence_step16_wrapper()

        except Exception as e:
            self.get_logger().error(f'❌ 트리거 해제 실패: {e}')

    def publish_joint_position(self, joint_name, publisher, show_log=True, speed=None):
        """관절 위치 명령 발행 (degree 단위)
        
        Args:
            joint_name: 관절 이름 ('lateral', 'x', 'y', 'z', 'yaw' 등)
            publisher: ROS2 publisher
            show_log: 로그 출력 여부
            speed: 속도 값 (None이면 속도 없이 위치만 전송)
        """
        msg = Float64MultiArray()
        
        # 속도가 지정된 경우 [위치, 속도] 형식으로 전송
        if speed is not None:
            msg.data = [self.current_positions[joint_name], speed]
        else:
            msg.data = [self.current_positions[joint_name]]
        
        publisher.publish(msg)
        
        if show_log:
            if speed is not None:
                self.get_logger().info(
                    f'📍 {joint_name.upper()}: {self.current_positions[joint_name]:.2f}° @ {speed:.1f}°/s'
                )
            else:
                self.get_logger().info(
                    f'📍 {joint_name.upper()}: {self.current_positions[joint_name]:.2f}°'
                )
    
    def send_lcd_brake_status(self):
        """Iron-MD LCD에 브레이크 상태 표시 (Page 1, Line 0)"""
        try:
            # LCD 표시 메시지 생성 (8자)
            brake_status = "Brake:ON" if not self.brake_released else "Brake:OF"

            # Iron-MD LCD 프로토콜: CAN ID 0x3E4 (996)
            # Byte 0: Page (1 = Page1)
            # Byte 1: Line (0~3)
            # Byte 2-7: ASCII 문자 (6 bytes, 8 char 중 앞 6자)
            lcd_data = bytearray(8)
            lcd_data[0] = 0x01  # Page 1
            lcd_data[1] = 0x00  # Line 0

            # 메시지를 ASCII로 인코딩 (최대 6바이트)
            msg_bytes = brake_status[:6].encode('ascii')
            for i, byte in enumerate(msg_bytes):
                lcd_data[2 + i] = byte

            # CAN 메시지 전송
            msg = can.Message(
                arbitration_id=0x3E4,
                data=bytes(lcd_data),
                is_extended_id=False
            )
            self.can_bus.send(msg)  # CAN3으로 전송 (Iron-MD로 돌려보냄)
            self.get_logger().debug(f'📺 LCD 표시: {brake_status}')

        except Exception as e:
            self.get_logger().error(f'❌ LCD 표시 실패: {e}')

    def publish_zero_velocity(self):
        """모든 모터 정지"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def emergency_stop(self):
        """비상 정지"""
        self.emergency_stopped = True
        self.get_logger().error('🚨 비상 정지 활성화!')
        
        # 모든 모터 정지
        self.publish_zero_velocity()
        self.trigger_release()
        
        # 비상 정지 신호 발행
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)
    
    def print_status(self):
        """디버그 모드 상태 출력 (1Hz)"""
        if not self.debug_mode:
            return
        
        # 조이스틱 정규화 값
        an1_norm = self.normalize_joystick(self.joystick_data['AN1'])
        an2_norm = self.normalize_joystick(self.joystick_data['AN2'])
        an3_norm = self.normalize_joystick(self.joystick_data['AN3'])
        an4_norm = self.normalize_joystick(self.joystick_data['AN4'])
        
        # 간소화된 상태 출력 (주요 정보만)
        self.get_logger().info(f'--- Iron-MD Status (DEBUG) ---')
        self.get_logger().info(f'TX: {"Connected" if self.switch_data["TX_Connected"] else "Disconnected"}, '
                               f'Mode: {self.control_mode}, '
                               f'E-Stop: {self.emergency_stopped}')
        self.get_logger().info(f'Joystick: AN1={an1_norm:+.2f} AN2={an2_norm:+.2f} AN3={an3_norm:+.2f} AN4={an4_norm:+.2f}')
        self.get_logger().info(f'Position: X={self.current_positions["x"]:.2f} Y={self.current_positions["y"]:.2f} '
                               f'Z={self.current_positions["z"]:.2f} Yaw={self.current_positions["yaw"]:.1f}deg')
        self.get_logger().info(f'Brake: {"Released" if self.brake_released else "Locked"}')
    
    def destroy_node(self):
        """노드 종료"""
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()
            self.get_logger().info('CAN3 bus closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IronMDTeleopNode()
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
