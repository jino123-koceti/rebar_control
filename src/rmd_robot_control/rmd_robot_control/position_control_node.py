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
from std_msgs.msg import Float64MultiArray, Float32, Int32, Empty, String, Bool
from rebar_base_interfaces.msg import SafetyState
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import struct
import time
import threading
from typing import Dict, Optional, List

from .can_manager import CANManager
from .rmd_x4_protocol import RMDX4Protocol, CommandType
from .lateral_axes import (
    Protection, I_RATED_A, I_HARD_A, I2T_BUDGET, TEMP_STOP_C, SEV_HARD)


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
        # 2026-09-08 전진방향 재정의: ID1(0x141)=우측, ID2(0x142)=좌측
        self.declare_parameter('left_motor_id', 0x142)
        self.declare_parameter('right_motor_id', 0x141)
        self.declare_parameter('wheel_radius', 0.02912)  # 바퀴 반지름 (m) - 2026-09-08 실측
        self.declare_parameter('wheel_base', 0.5)     # 바퀴 간 거리 (m)
        self.declare_parameter('max_linear_vel', 0.25)  # 최대 선속도 (m/s) - 모터 정격 498dps 기준
        self.declare_parameter('wheel_max_dps', 498.0)  # 바퀴 출력축 속도 상한 (RMD-X4-36 정격)
        self.declare_parameter('max_angular_vel', 0.5) # 최대 각속도 (rad/s) - 2026-09-08 선회 공진 회피
        # 주행 모터 하드 차단(0x80) 임계 [A]. RMD-X4-36 피크 21.5A(rms) 를 넘기지 말 것.
        # 실측 참고: 전후진 최대 5.51A, 선회 최대 7.63A 이므로 18A 는 이미 2.4배 여유다.
        self.declare_parameter('drive_i_hard_a', 18.0)
        # cmd_vel 이 이 시간 이상 끊기면 주행을 자동 정지한다. 0 이면 비활성.
        self.declare_parameter('cmd_vel_timeout', 0.5)
        # 관절(상부 축) 속도 명령이 끊겼을 때 자동 정지하는 시간.
        # 주행에는 워치독이 있었지만 **관절에는 없었다** — 0xA2 는 마지막 속도를
        # 계속 유지하므로, 명령이 끊기면 축이 그대로 돌아간다.
        # 2026-09-30 실측: 스틱을 놓은 뒤 명령이 960ms 끊긴 구간에서 X축이 50dps 로
        # 계속 돌았다(실제속도 확인). 원인 규명과 별개로 보호가 필요하다.
        self.declare_parameter('joint_speed_timeout', 0.3)
        # L2(safety_node)의 판정을 받아 **여기서 최종 차단**한다.
        # 상위에서 막으면 "무엇이 명령했든" 을 보장할 수 없다 (아키텍처 §2 규칙 4).
        # /safety/state 가 이 시간 이상 안 오면 안전 노드가 죽은 것으로 보고 정지한다.
        # ⚠ 기본값 0 = 비활성. 안전 노드를 띄우지 않은 상태에서 갑자기 모든 명령이
        #   막히면 그게 더 위험하므로, 도입은 명시적으로 켠다.
        self.declare_parameter('safety_timeout', 0.0)
        # ⚠ 모터측 통신두절 보호 (RMD 0xB3). **주행모터에만** 건다.
        #   젯슨이 하드프리즈하면 SW 워치독도 같이 멈춰 무용지물이다 — 모터가 이 시간 안에
        #   명령을 못 받으면 스스로 출력을 끊는다. 2차년도 2026-08-06 도입 근거와 같다.
        #   정상 주행 중엔 0xA2·폴링이 하트비트가 되어 걸리지 않는다.
        #   스테이지·Yaw 에는 절대 걸지 말 것 — 유휴 시 명령이 끊겨 오작동한다.
        self.declare_parameter('drive_motor_watchdog_ms', 500)   # 0 = 비활성
        
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
        self.wheel_max_dps = self.get_parameter('wheel_max_dps').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.drive_i_hard_a = float(self.get_parameter('drive_i_hard_a').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.joint_speed_timeout = float(self.get_parameter('joint_speed_timeout').value)
        # 관절별 마지막 속도 명령: {motor_id: (speed_dps, 시각)}
        self.joint_speed_active = {}
        self.safety_timeout = float(self.get_parameter('safety_timeout').value)
        self.safety = None          # 마지막 SafetyState
        self.safety_time = 0.0
        self._safety_blocked_logged = None
        
        # CAN 매니저 및 프로토콜 초기화
        self.can_manager = CANManager(self.can_interface)
        self.protocol = RMDX4Protocol()

        self.drive_prot = {
            self.left_motor_id: Protection(self.left_motor_id, '좌측주행',
                                           i_hard=self.drive_i_hard_a),
            self.right_motor_id: Protection(self.right_motor_id, '우측주행',
                                            i_hard=self.drive_i_hard_a),
        }
        self.get_logger().info(
            f"주행 보호: 정격 {I_RATED_A}A / 즉시차단 {self.drive_i_hard_a}A / "
            f"I²t {I2T_BUDGET:.0f} A²s / 온도 {TEMP_STOP_C}°C")

        # 주행 모터 가감속 상한 설정은 기동 시 호출하지 않는다 (2026-09-08):
        #  1) 이 시점엔 can_manager.connect() 전이라 소켓이 없어 8건 전부 실패했다.
        #  2) 0x43 은 RAM+ROM 저장이라 노드가 뜰 때마다 ROM 에 쓸 이유가 없다.
        #  3) X4-10 -> X4-36 으로 감속비가 커져(토크 2.9배, 최고속도 하락)
        #     20000 dps/s 는 과한 값이다. 현재 모터 설정값은 5000 dps/s.
        # 값을 바꿔야 하면 apply_drive_accel_limits() 를 도구로 1회만 호출할 것.

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

        # 주행 모터(0x141/0x142) 전용 보호 — 2026-09-08 추가.
        # 기존 motor_current_limits 방식의 결함 세 가지:
        #   (1) send_motor_stop 이 0xA2 speed=0 이라 모터가 꺼지지 않는다 (실측 확인).
        #   (2) emergency(18A) x 2샘플이어야 발동. 그 아래 지속 전류는 무방비.
        #   (3) 정격 초과 '지속'(I²t)을 보지 않아 10A 로 계속 돌아도 조치가 없다.
        # 2차년도에 횡이동 모터를 소손시킨 것이 이 조합이다.
        # NOTE(2026-09-15): 여기서 self.drive_prot 를 {} 로 다시 초기화하고 있었다.
        # __init__ 앞부분에서 만든 Protection 2개를 덮어써서 drive_protect() 의
        # `pr = self.drive_prot.get(motor_id)` 가 항상 None → 즉시 return 이었다.
        # 즉 주행 보호가 한 번도 동작한 적이 없다. 재초기화만 제거한다.
        self.drive_tripped = False    # 래치: 해제 전까지 cmd_vel 무시
        self.last_cmd_vel_time = None # cmd_vel 워치독용 마지막 수신 시각
        self.drive_cmd_active = False # 0 이 아닌 주행 명령이 살아 있는가
        self.drive_trip_reason = ""
        self.drive_normal_since = None

        # 모터 ID → 축 이름. 안전 차단에서 "x+" 같은 방향 표기와 맞추기 위한 것이다.
        # axes.yaml 과 같은 배치 (3차년도 실측).
        self.AXIS_BY_MOTOR = {0x145: 'x', 0x146: 'y', 0x147: 'z', 0x148: 'yaw'}

        # 모터 전류 보호 시스템
        #
        # ★ 2026-09-29: **이름이 2차년도 배치였다.** 3차년도는 축이 한 칸씩 밀려 있어
        #   0x145 를 'Yaw', 0x146 을 'X축' 으로 적어 두면 로그를 보고 엉뚱한 축을
        #   의심하게 된다 (실제로 그렇게 오진했다). 실측 배치로 고쳤다:
        #     0x141 우측주행 · 0x142 좌측주행 · 0x143/0x144 횡이동 2축
        #     0x145 X · 0x146 Y · 0x147 Z(리프팅) · 0x148 Yaw
        #   좌/우 주행 이름도 뒤바뀌어 있었다 (ID1=우측, ID2=좌측 — 2026-09-08 실측).
        #
        # ⚠ Yaw(0x148)는 결속 자세전환에 필요해 새로 등록했다. 이 표에 없는 ID 는
        #   보호 로직이 조용히 건너뛰고(831줄) 다른 경로에선 KeyError 가 난다.
        #   전류 임계는 아직 3차년도 실측이 없어 상부 4축을 같은 값으로 두었다.
        # ⚠ speed_limit 은 motor_protection_state['speed_limited'] 가 True 일 때만
        #   쓰이고 그 기능은 아직 비활성이다(940줄 주석). 값은 잠정치다.
        self.motor_current_limits = {
            0x141: {'name': '우측주행', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 100},
            0x142: {'name': '좌측주행', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 100},
            # 횡이동 2축 (X4-36). 2차년도는 0x143 1축이었다.
            0x143: {'name': '횡이동#1', 'rated': 6.1, 'warning': 8.0, 'danger': 12.0, 'emergency': 18.0, 'speed_limit': 100},
            0x144: {'name': '횡이동#2', 'rated': 6.1, 'warning': 8.0, 'danger': 12.0, 'emergency': 18.0, 'speed_limit': 100},
            # 상부 스테이지
            0x145: {'name': 'X축', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 200},
            0x146: {'name': 'Y축', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 200},
            0x147: {'name': 'Z축(리프팅)', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 75},
            0x148: {'name': 'Yaw', 'rated': 7.8, 'warning': 9.0, 'danger': 12.0, 'emergency': 15.0, 'speed_limit': 67},
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
        # ⚠ 큐 깊이 1. 2026-09-30 실측: 깊이 10 이면 들어오는 36Hz 를 19Hz 로 처리하면서
        #   밀린 10개(≈280ms)를 순서대로 처리해 조작이 그만큼 늦게 반응했다.
        #   제어 명령은 **최신값만** 의미가 있으므로 오래된 것은 버리는 게 맞다.
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            1
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
                1      # 최신 속도만 쓴다 (위 cmd_vel 주석 참고)
            )
            self.joint_speed_subscriptions.append(subscription)
        
        # 주행 모터(0x141, 0x142) 위치 제어 토픽 구독 (S20 모드용)
        # 주행부 수동 해제: 0x80 으로 출력을 완전히 끊어 유지 토크를 놓는다.
        # 정지 상태에서 구동계 탄성 예하중을 계속 붙잡고 3~4A 를 소모하는 일이
        # 있어(2026-09-15 실측, 온도 51°C 까지 상승) 조작자가 즉시 풀 수 있게 한다.
        self.drive_release_sub = self.create_subscription(
            Empty, '/drive/release', self.drive_release_callback, 10)
        self.safety_sub = self.create_subscription(
            SafetyState, '/safety/state', self._on_safety, 1)

        # 축별 브레이크. 기존 서비스(`safe_brake_release`)는 **전 모터를 한꺼번에** 푼다.
        # Z 는 리프팅축이라 풀면 자중으로 내려앉을 수 있어 자동 해제에서 빼야 하고
        # (`axes.yaml` brake.never_auto_release), 호밍도 돌리는 축만 풀고 시작한다.
        #   ros2 topic pub --once /brake_cmd std_msgs/String "{data: 'release x,y'}"
        #   ros2 topic pub --once /brake_cmd std_msgs/String "{data: 'lock x,y'}"
        #   ros2 topic pub --once /brake_cmd std_msgs/String "{data: 'release z force'}"
        self.brake_cmd_sub = self.create_subscription(
            String, '/brake_cmd', self._on_brake_cmd, 10)

        # 단회전 절대 위치 발행 (0x61 mod 262144).
        # **멀티턴은 전원에 날아가지만 단회전값은 물리적으로 고정**이라, 전원 재투입
        # 후에도 "지금 어느 각도인가" 를 말해준다. 호밍 전제 검사(yaw 가 12시 근처인가)
        # 와 기구 무결성 검사(커플링이 미끄러졌나)에 쓴다.
        # 2026-09-30: yaw 감지판을 옮겼을 때 이 값이 275104→205327 로 바뀌었다.
        self.encoder_single_pubs = {
            mid: self.create_publisher(Int32, f"motor_{hex(mid)}/encoder_single", 10)
            for mid in self.motor_ids}
        # ⚠ **멀티턴도 같이 발행한다.** 위치 토픽(0x92)은 `is_moving` 인 모터만
        # 폴링해서 **멈추면 묵는다** — 그걸 믿고 각도를 역산하면 틀린다
        # (2026-10-03 에 yaw 를 27.6° 틀리게 봤다). 0x61 응답에 멀티턴이 이미
        # 들어 있으므로 CAN 트래픽이 늘지 않는다. 전원마다 영점이 달라 **절대값은
        # 세션 한정**이지만, 기준점과의 **차이**는 그 안에서 정확하다.
        self.encoder_multi_pubs = {
            mid: self.create_publisher(Int32, f"motor_{hex(mid)}/encoder_multi", 10)
            for mid in self.motor_ids}
        self.encoder_single = {}

        # 브레이크 해제 상태 — 0x9A DATA[3] (0x01 = 해제). **호밍이 이것을 기다린다.**
        # 고정 시간 대기로는 모자란다: 2026-10-03 에 0x77 이 먹기까지 **1.50초** 걸린
        # 경우를 봤다 (0.4초 시점에는 아직 잠김). 브레이크를 문 채 속도 명령을 받으면
        # 모터는 최대 전류로 밀면서 거의 안 움직인다 — 증상이 "모터 고장" 과 같다.
        self.brake_pubs = {
            mid: self.create_publisher(Bool, f"motor_{hex(mid)}/brake", 10)
            for mid in self.motor_ids
            if mid not in (self.left_motor_id, self.right_motor_id)}
        self.brake_state = {}

        # 축별 maxTorque (0xA2/0xA4 DATA[1]) — **정격 전류의 백분율**.
        # 상부축 기본 100 은 yaw 가 2·3번 자세 사이를 못 지난다. axes.yaml 에서 올린다.
        # ⚠ 상부축에는 Protection 이 없어 이 값이 유일한 상한이다 (255 금지).
        from .axis_config import load_max_torque
        self.max_torque = load_max_torque()
        if self.max_torque:
            self.get_logger().info(
                "축별 maxTorque: " + ", ".join(
                    f"0x{m:03X}={v}" for m, v in sorted(self.max_torque.items())))

        # 엔코더 명령 진단. 어떤 읽기 명령이 이 모터에서 실제로 동작하는지 확인한다.
        #   ros2 topic pub --once /encoder_probe std_msgs/String "{data: 'yaw'}"
        # 멀티턴(0x92)은 전원을 내리면 사라진다. 자세를 저장해 두려면 **싱글턴
        # 앱솔루트**가 필요해서, 0x90·0x94·0x60·0x61·0x62 중 무엇이 응답하는지 봐야 한다.
        # (횡이동에서는 0x90 이 응답하지 않아 0x61 을 썼다 — 개체마다 다르다.)
        self.encoder_probe_sub = self.create_subscription(
            String, '/encoder_probe', self._on_encoder_probe, 10)

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
        # cmd_vel 끊김 감시 (10Hz) 와 보호 래치 해제 구동 (2Hz)
        self.drive_watchdog_timer = self.create_timer(0.1, self.drive_watchdog_tick)
        self.joint_watchdog_timer = self.create_timer(0.05, self.joint_watchdog_tick)
        self.drive_latch_timer = self.create_timer(0.5, self.drive_latch_tick)
        # 1Hz 면 충분하다 — 사람이 축을 옮기는 속도에 비하면 빠르고, CAN 부담도 작다
        # 10Hz. **1Hz 는 느리다** — 호밍이 이 값으로 yaw 자세를 판별하고 로그를 찍는데,
        # 1초 묵으면 30dps 에서 모터축 30°(건 2.4°) 어긋난다. 2026-10-03 에 이 묵은
        # 값으로 "FINE 이 원점을 엉뚱한 곳에 적는다" 고 오진했다.
        # 4축 × 10Hz = 40프레임/초, 1Mbps 버스에서 무시할 수 있는 부하다.
        self.encoder_single_timer = self.create_timer(0.1, self._read_encoder_single)
        # 2Hz. 호밍의 ARM 단계가 이 값을 보고 넘어간다 — 느리면 호밍이 그만큼 기다린다.
        self.brake_state_timer = self.create_timer(0.5, self._read_brake_state)
        
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
            self._arm_drive_motor_watchdog()
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

    # 찔러볼 엔코더 읽기 명령. 이름은 참고용이고, 실제로 응답하는지가 중요하다.
    PROBE_COMMANDS = (
        (0x90, '싱글턴 엔코더 원시(위치/원위치/오프셋)'),
        (0x92, '멀티턴 각도 (전원 내리면 사라짐)'),
        (0x94, '싱글턴 각도 0.01도'),
        (0x60, '멀티턴 엔코더 위치'),
        (0x61, '멀티턴 엔코더 원위치'),
        (0x62, '멀티턴 엔코더 영점 오프셋'),
    )

    def _on_encoder_probe(self, msg):
        """축 하나에 엔코더 읽기 명령을 차례로 보내고 원시 응답을 로그로 남긴다.

        `ros2 topic pub --once /encoder_probe std_msgs/String "{data: 'yaw'}"`
        축 이름 대신 `0x148` 처럼 CAN ID 를 직접 줘도 된다.
        """
        tok = msg.data.strip().lower()
        # `yaw:0x61` 처럼 명령 하나만 지정할 수 있다. 전체를 돌리면 6개×0.25초라
        # 1.5초가 걸려, 움직임 직후를 잡아야 하는 측정에는 너무 느리다.
        only = None
        idx = None
        # `yaw:raw:A200000048F4FFFF` — 8바이트 원시 프레임을 그대로 보낸다.
        # 노드의 인코딩을 우회해야 프로토콜 해석 차이를 시험할 수 있다.
        if ':raw:' in tok:
            name, hexs = tok.split(':raw:', 1)
            by = {v: k for k, v in self.AXIS_BY_MOTOR.items()}
            mid = by.get(name.strip())
            if mid is None:
                try: mid = int(name, 0)
                except ValueError:
                    self.get_logger().error(f"원시 전송: 축을 모르겠습니다 '{name}'"); return
            hexs = hexs.strip().replace(' ', '')
            try: frame = bytes.fromhex(hexs)
            except ValueError:
                self.get_logger().error(f"원시 전송: 16진수가 아닙니다 '{hexs}'"); return
            if len(frame) != 8:
                self.get_logger().error(f"원시 전송: 8바이트여야 합니다 ({len(frame)})"); return
            self._probe_until = time.time() + 2.5
            self.get_logger().info(f"[PROBE] 0x{mid:03X} → RAW {frame.hex().upper()}")
            if not self.can_manager.send_frame(mid, frame):
                self.get_logger().warning("[PROBE] 원시 전송 실패")
            return
        if ':' in tok:
            tok, cmd_s = tok.split(':', 1)
            if ':' in cmd_s:                      # `yaw:0x30:4` — 명령 + 인덱스
                cmd_s, idx_s = cmd_s.split(':', 1)
                try:
                    idx = int(idx_s, 0)
                except ValueError:
                    self.get_logger().error(f"인덱스를 모르겠습니다 '{idx_s}'"); return
            try:
                only = int(cmd_s, 0)
            except ValueError:
                self.get_logger().error(f"엔코더 진단: 명령을 모르겠습니다 '{cmd_s}'")
                return
        by_name = {v: k for k, v in self.AXIS_BY_MOTOR.items()}
        if tok in by_name:
            mid = by_name[tok]
        else:
            try:
                mid = int(tok, 0)
            except ValueError:
                self.get_logger().error(f"엔코더 진단: 축을 모르겠습니다 '{msg.data}'")
                return

        # 응답을 INFO 로 찍는 창을 연다
        cmds = ([(only, '지정 명령')] if only is not None else list(self.PROBE_COMMANDS))
        self._probe_until = time.time() + (1.0 if only is not None else 3.0)
        self.get_logger().info(f"[PROBE] 0x{mid:03X} 엔코더 명령 진단 시작")
        for code, what in cmds:
            frame = bytearray(8)
            frame[0] = code
            # `yaw:0x30:4` 처럼 인덱스를 줄 수 있다. 0x30(PID 읽기)은 DATA[1] 에
            # 인덱스가 필요하다 (0x04=속도루프 KP, 0x05=KI 등). 안 주면 전부 0 이 온다.
            if idx is not None:
                frame[1] = idx & 0xFF
            self.get_logger().info(f"[PROBE] 0x{mid:03X} → 0x{code:02X}  {what}")
            if not self.can_manager.send_frame(mid, bytes(frame)):
                self.get_logger().warning(f"[PROBE] 0x{code:02X} 전송 실패")
            time.sleep(0.05 if only is not None else 0.25)   # 응답이 섞이지 않게 띄운다
        self.get_logger().info(
            "[PROBE] 끝. 응답이 없는 명령은 이 모터가 지원하지 않는 것이다")

    # 자동 해제 금지 축. `axes.yaml` 의 brake.never_auto_release 와 같은 정책을
    # 코드에도 둔다 — 설정 파일을 못 읽어도 Z 가 실수로 풀리면 안 된다.
    NEVER_AUTO_RELEASE = ('z',)

    ENCODER_CPR = 262144            # 18bit. 0x61 을 이 값으로 나눈 나머지가 단회전 절대값

    def _read_encoder_single(self):
        """상부 축의 0x61 을 읽어 단회전 절대값을 발행한다."""
        frame = bytearray(8)
        frame[0] = 0x61
        for mid in self.motor_ids:
            if mid in (self.left_motor_id, self.right_motor_id):
                continue                # 주행은 이 값이 의미 없다
            self.can_manager.send_frame(mid, bytes(frame))
            time.sleep(0.002)

    def _read_brake_state(self):
        """상부 축의 0x9A 를 읽어 브레이크 해제 상태를 발행한다.

        응답 처리는 `0x9A` 분기에서 한다. 주행 축은 제외한다 — 브레이크가 없다.
        """
        frame = bytearray(8)
        frame[0] = 0x9A
        for mid in self.brake_pubs:
            self.can_manager.send_frame(mid, bytes(frame))
            time.sleep(0.002)

    def _on_brake_cmd(self, msg):
        """축별 브레이크 해제/잠금.

        `release x` / `release x,y` / `lock z` / `release z force`

        Z 는 자중 낙하 위험이 있어 `force` 를 붙여야 풀린다. 잠그는 것은 언제나 허용한다
        (안전한 방향이다).
        """
        parts = msg.data.strip().lower().split()
        if not parts:
            return
        action = parts[0]
        if action not in ('release', 'lock', 'shutdown'):
            self.get_logger().error(
                f"브레이크 명령을 모르겠습니다: '{msg.data}' — "
                f"'release x,y' / 'lock z' / 'shutdown x,y' 형식")
            return

        force = 'force' in parts[1:]
        names = []
        for tok in parts[1:]:
            if tok == 'force':
                continue
            names.extend(n for n in tok.split(',') if n)
        if not names:
            self.get_logger().error("브레이크 명령에 축이 없습니다 (예: 'release x,y')")
            return

        by_name = {v: k for k, v in self.AXIS_BY_MOTOR.items()}
        if names == ['all']:
            names = list(by_name)

        cmd = self.protocol.create_system_command(
            CommandType.BRAKE_RELEASE if action == 'release' else CommandType.BRAKE_LOCK)
        # `shutdown` = 잠금 후 여자 해제. **`0x78` 만으로는 전류가 끊기지 않는다** —
        # 모터가 여자된 채 마지막 속도 명령(0 이어도)을 계속 수행해 브레이크와 반력을
        # 상대로 밀면서 발열한다 (2026-10-03 yaw 실측: -4.02A 계속, 29→43°C).
        # 순서가 중요하다: 먼저 잠그지 않고 0x80 을 보내면 **Z 가 떨어진다.**
        shutdown_cmd = (self.protocol.create_system_command(CommandType.MOTOR_SHUTDOWN)
                        if action == 'shutdown' else None)

        done, skipped, failed = [], [], []
        for name in names:
            mid = by_name.get(name)
            if mid is None:
                skipped.append(f"{name}(모르는 축)")
                continue
            if action == 'release' and name in self.NEVER_AUTO_RELEASE and not force:
                skipped.append(f"{name}(자중 낙하 위험 — 풀려면 'force')")
                continue
            ok = self.can_manager.send_frame(mid, cmd)
            if ok and shutdown_cmd is not None:
                time.sleep(0.05)                 # 잠금이 먹은 뒤에 여자를 끊는다
                ok = self.can_manager.send_frame(mid, shutdown_cmd)
            if ok:
                done.append(f"{name}(0x{mid:03X})")
            else:
                failed.append(f"{name}(0x{mid:03X})")
            time.sleep(0.05)

        verb = {'release': '해제', 'lock': '잠금', 'shutdown': '잠금+차단'}[action]
        if done:
            self.get_logger().info(f"브레이크 {verb}: {', '.join(done)}")
        if skipped:
            self.get_logger().warning(f"브레이크 {verb} 건너뜀: {', '.join(skipped)}")
        if failed:
            self.get_logger().error(f"브레이크 {verb} 실패: {', '.join(failed)}")

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
        """단일 관절 속도 명령 콜백.

        ⚠ 2026-09-30: 여기서 메시지마다 INFO 2줄(journald + 파일)을 쓰고 있었다.
        그 탓에 이 콜백 처리량이 **19 Hz** 로 묶여, 56 Hz 로 들어오는 명령이 큐에 밀려
        리모콘 입력이 체감상 느렸다 (실측 입력→구동 179 ms, 놓음→정지 252 ms).
        → 값이 실제로 바뀔 때만 로그를 남긴다. 유지 구간(같은 값 반복)은 조용히 보낸다.
        """
        if joint_index >= len(self.motor_ids):
            return
        
        motor_id = self.motor_ids[joint_index]
        speed_dps = msg.data  # degree per second

        # ── 안전 차단 ────────────────────────────────────────────────────────
        stop_reason = self._safety_full_stop()
        if stop_reason is not None:
            speed_dps = 0.0
        else:
            speed_dps = self._safety_clamp_axis(motor_id, speed_dps)

        # ── 위치 읽기가 나가게 한다 ──────────────────────────────────────────
        # 위치 읽기(0x92)는 `position_control_loop` 이 **is_moving 인 모터에만** 보낸다.
        # 그런데 is_moving 은 위치 제어(0xA4) 경로에서만 서고, 속도 제어(0xA2)로 움직일
        # 때는 아무도 세우지 않았다. 그래서 **속도로 움직이는 내내 위치가 갱신되지
        # 않았다** — 2026-09-30 실측: Y 를 y_max 에서 y_min 까지 옮겼는데
        # `/motor_0x146_position` 이 한 값에 고정이었다 (6초에 39건 수신, 값 1종).
        # 호밍이 그 값을 원점 레퍼런스로 적으면 엉뚱한 상수가 남는다.
        #
        # 멈춘 뒤 0.5초는 계속 읽는다. 감속 구간이 남아 있어서, 속도가 0 이 된 순간
        # 읽으면 실제로 멈춘 위치가 아니다.
        st = self.motor_states.setdefault(motor_id, {})
        if abs(speed_dps) > 0.01:
            st['is_moving'] = True
            st['_moving_until'] = time.time() + 0.5
        elif time.time() >= st.get('_moving_until', 0.0):
            st['is_moving'] = False

        # 값이 바뀔 때만 로그 (연속 같은 값은 초당 수십 번 들어온다)
        if not hasattr(self, '_last_joint_speed'):
            self._last_joint_speed = {}
        changed = abs(self._last_joint_speed.get(motor_id, 0.0) - speed_dps) > 0.5
        self._last_joint_speed[motor_id] = speed_dps
        if changed:
            self.get_logger().info(
                f'📥 [ROS2] /joint_{joint_index+1}/speed 수신: {speed_dps:.1f} dps (motor 0x{motor_id:03X})'
            )
        
        # 같은 값이면 굳이 다시 보내지 않는다.
        # 0xA2 를 받은 모터는 **그 속도를 계속 유지**하므로 매 메시지마다 재송신할
        # 이유가 없다. 다만 완전히 끊으면 모터측 워치독·진단이 불리해서 10Hz 로 새로 고친다.
        now = time.time()
        if not hasattr(self, '_last_speed_sent'):
            self._last_speed_sent = {}
        prev_val, prev_t = self._last_speed_sent.get(motor_id, (None, 0.0))
        # 갱신 주기 30ms (약 33Hz). 100ms 로 두면 모터 피드백(0xA2 응답)도 그만큼만 와서
        # 램프·실제속도를 관측할 수 없고, 제어 상태 추정이 늦는다.
        # 송신 간격이 2ms 이므로 이 정도 빈도는 버스에 부담이 없다.
        if prev_val is not None and abs(prev_val - speed_dps) <= 0.5 and now - prev_t < 0.03:
            return
        self._last_speed_sent[motor_id] = (speed_dps, now)
        # 워치독용: 0 이 아닌 명령이 살아 있는 축을 기록한다
        if abs(speed_dps) > 0.5:
            self.joint_speed_active[motor_id] = now
        else:
            self.joint_speed_active.pop(motor_id, None)

        # 속도 명령 전송 (0xA2)
        speed_control = int(speed_dps * 100)  # 0.01 dps/LSB
        
        if changed:
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

    def drive_protect(self, motor_id: int, current: float, speed: float, temperature: int):
        """주행 모터 보호 감시. 응답이 올 때마다 호출된다.

        SEV_HARD(18A 초과) → 두 주행 모터 모두 0x80 SHUTDOWN.
        그 외(I²t 초과, 스톨, 온도) → 속도 0 + 0x81 정지.
        어느 쪽이든 래치가 걸려 해제 전까지 cmd_vel 을 무시한다.
        바퀴는 떨어질 것이 없으므로 횡이동과 달리 '후퇴' 동작은 없다.
        """
        pr = self.drive_prot.get(motor_id)
        if pr is None:
            return
        now = time.time()
        reason, sev = pr.update(current, speed, temperature, now)

        if reason and not self.drive_tripped:
            self.drive_tripped = True
            self.drive_trip_reason = f"0x{motor_id:03X} {reason}"
            self.drive_normal_since = None
            if sev == SEV_HARD:
                self.get_logger().error(f"⛔ 주행 즉시 차단: {self.drive_trip_reason}")
                self.shutdown_drive_motors()
            else:
                self.get_logger().error(f"⛔ 주행 정지: {self.drive_trip_reason}")
                for mid in self.drive_prot:
                    self.send_speed_command(mid, 0)
                    self.can_manager.send_frame(
                        mid, self.protocol.create_system_command(CommandType.MOTOR_STOP))
            for q in self.drive_prot.values():
                self.get_logger().error(f"   {q.summary()}")
            return

        # 래치 해제: 두 축 모두 정격 이하로 3초 유지
        if self.drive_tripped:
            hot = any(abs(q.peak_a) > I_RATED_A and q.i2t > I2T_BUDGET * 0.5
                      for q in self.drive_prot.values())
            if abs(current) < I_RATED_A and not hot:
                if self.drive_normal_since is None:
                    self.drive_normal_since = now
                elif now - self.drive_normal_since > 3.0:
                    self.get_logger().info("✅ 주행 보호 해제 (3초간 정격 이하)")
                    self.drive_tripped = False
                    self.drive_trip_reason = ""
                    for q in self.drive_prot.values():
                        q.reset()
            else:
                self.drive_normal_since = None

    def _on_safety(self, msg):
        self.safety = msg
        self.safety_time = time.time()

    def _safety_active(self):
        """안전 차단을 적용할 상태인가. (safety_timeout 0 이면 비활성)"""
        if self.safety_timeout <= 0:
            return False
        if self.safety is None:
            return True             # 켜뒀는데 아직 못 받았다 → 안전측으로 막는다
        return True

    def _safety_full_stop(self):
        """전체 정지 사유가 있는가 — 비상정지·STOP·입력두절·안전노드 두절."""
        if not self._safety_active():
            return None
        if self.safety is None:
            return "안전 상태 수신 전"
        if time.time() - self.safety_time > self.safety_timeout:
            return f"/safety/state 두절 {self.safety_timeout:.1f}s 초과"
        s = self.safety
        if s.estop:
            return "비상정지"
        if s.stop_switch:
            return "STOP 스위치"
        if s.inputs_stale:
            return "안전 입력 두절"
        return None

    def _safety_clamp_drive(self, linear_vel, angular_vel):
        """방향별 차단을 적용한다. 부딪힌 방향만 막고 반대는 열어 둔다."""
        if not self._safety_active() or self.safety is None:
            return linear_vel, angular_vel
        s = self.safety
        if linear_vel > 0 and s.block_forward:
            linear_vel = 0.0
        if linear_vel < 0 and s.block_backward:
            linear_vel = 0.0
        if angular_vel > 0 and s.block_left:
            angular_vel = 0.0
        if angular_vel < 0 and s.block_right:
            angular_vel = 0.0
        return linear_vel, angular_vel

    def _safety_clamp_axis(self, motor_id, speed_dps):
        """리미트에 닿은 축의 그 방향만 막는다 (반대 방향은 빠져나올 수 있게 열어 둔다)."""
        if not self._safety_active() or self.safety is None or abs(speed_dps) < 0.01:
            return speed_dps
        axis = self.AXIS_BY_MOTOR.get(motor_id)
        if axis is None:
            return speed_dps
        want = f"{axis}{'+' if speed_dps > 0 else '-'}"
        if want in self.safety.blocked_axes:
            if self._safety_blocked_logged != want:
                self._safety_blocked_logged = want
                self.get_logger().warning(f"안전 차단 — {want} 방향 (리미트)")
            return 0.0
        return speed_dps

    def drive_release_callback(self, msg):
        """주행 모터 출력 완전 차단 (0x80).

        0xA2 speed=0 과 0x81 은 모터를 여자 상태로 남긴다 — 0x80 만이 0.00A 를
        만든다 (2026-09-15 실측). 유지 토크가 사라지므로 경사에서는 밀릴 수 있다.
        다음 cmd_vel 이 오면 모터가 다시 살아난다.
        """
        self.get_logger().warning("🔓 주행부 해제 요청 — 0x80 전송 (유지 토크 없어짐)")
        self.drive_cmd_active = False
        self.shutdown_drive_motors()

    def joint_watchdog_tick(self):
        """관절 속도 명령이 끊기면 해당 축을 정지한다.

        0xA2 를 받은 모터는 그 속도를 계속 유지한다. 그래서 상위 노드가 죽거나
        발행이 끊기면 축이 계속 돌아간다. 주행에는 워치독이 있었는데 관절에는 없었다.
        2026-09-30 실측: 스틱을 놓은 뒤 명령이 960ms 끊긴 구간에서 X축이 50dps 로
        계속 돌았다.

        0 명령은 기록하지 않으므로(joint_speed_active 에서 제거) 정지 상태에서는
        아무 것도 하지 않는다.
        """
        if self.joint_speed_timeout <= 0 or not self.joint_speed_active:
            return
        now = time.time()
        for motor_id, last in list(self.joint_speed_active.items()):
            if now - last <= self.joint_speed_timeout:
                continue
            name = self.motor_current_limits.get(motor_id, {}).get('name', f'0x{motor_id:03X}')
            self.get_logger().warning(
                f"⚠️ {name}(0x{motor_id:03X}) 속도 명령 끊김 "
                f"{self.joint_speed_timeout:.2f}s 초과 — 정지")
            self.joint_speed_active.pop(motor_id, None)
            self._last_speed_sent[motor_id] = (0.0, now)
            self.send_speed_command_single(motor_id, 0)

    def drive_watchdog_tick(self):
        """cmd_vel 이 끊기면 주행을 자동 정지한다.

        0xA2 속도 명령을 받은 모터는 **그 속도를 계속 유지한다.** 따라서
        cmd_vel 발행이 멈춰도(텔레옵 크래시, SSH 끊김, 네트워크 손실)
        백엔드가 아무 조치를 안 하면 바퀴는 마지막 속도로 계속 돈다.
        데드맨이 teleop_keyboard 쪽에만 있어 클라이언트가 죽으면 무방비였다
        (2026-09-15 "주행이 안 멈춤" 의 원인).
        """
        if self.cmd_vel_timeout <= 0 or not self.drive_cmd_active:
            return
        if self.last_cmd_vel_time is None:
            return
        if time.time() - self.last_cmd_vel_time <= self.cmd_vel_timeout:
            return
        self.drive_cmd_active = False
        self.get_logger().warning(
            f"⚠️ cmd_vel 끊김 {self.cmd_vel_timeout:.1f}s 초과 — 주행 정지")
        for mid in self.drive_prot:
            self.send_speed_command(mid, 0)

    def drive_latch_tick(self):
        """보호 래치가 걸린 동안 속도 0 을 보내 해제 판정을 돌린다.

        해제 조건은 drive_protect() 안에 있고, drive_protect() 는 0xA2 응답이
        올 때만 호출된다. 그런데 cmd_vel_callback 은 래치가 걸리면 명령을
        보내지 않으므로 — 명령이 없으면 응답이 없고, 응답이 없으면 해제
        판정이 영영 실행되지 않는다. 교착이다.
        (2026-09-15 실측: "cmd_vel 무시" 1,247회 후에도 미해제, 노드 재시작으로만 복구)
        속도 0 은 이미 멈춘 모터에 무해하면서 필요한 응답을 만들어낸다.
        """
        if not self.drive_tripped:
            return
        for mid in self.drive_prot:
            self.send_speed_command(mid, 0)

    def shutdown_drive_motors(self):
        """주행 모터 완전 차단. 0x80 만이 실제로 출력을 끊는다.

        0xA2 speed=0 과 0x81 은 모터를 여자 상태로 남긴다 (2026-09-08 실측:
        0x81 후에도 0.6~1.4A 가 계속 흘렀고 0x80 에서만 0.00A 가 되었다).
        """
        cmd = self.protocol.create_system_command(CommandType.MOTOR_SHUTDOWN)
        for _ in range(2):
            for mid in self.drive_prot:
                self.can_manager.send_frame(mid, cmd)
            time.sleep(0.02)

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

            # RAW 응답은 debug 레벨로 (터미널 로그 방지).
            # 단 `/encoder_probe` 진단 중에는 INFO 로 올린다 — 어떤 엔코더 명령이
            # 실제로 응답하는지 보려면 원시 바이트를 봐야 한다.
            if time.time() < getattr(self, '_probe_until', 0.0):
                self.get_logger().info(
                    f"[PROBE] 0x{motor_id:03X} ← 0x{command:02X}  {data.hex().upper()}")
            else:
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
            elif command == CommandType.SET_MOTOR_SPEED:
                # 0xA2 응답도 0x9C 와 같은 자리배치다: [cmd][temp][current][speed][angle]
                # **이 분기가 없어서 속도 제어 이동에는 전류 보호가 걸리지 않았다.**
                # 0xA4(위치 제어)는 걸려 있었는데 0xA2 만 비어 있었다 — 호밍의
                # SEEK/BACK_OFF/FINE 과 리모콘 조작이 전부 그 구멍에 있었다.
                # 상부축은 0x9C 를 폴링하지 않으므로, 이 응답이 유일한 전류 창구다.
                if len(data) >= 8:
                    temperature = struct.unpack('<b', data[1:2])[0]
                    current = struct.unpack('<h', data[2:4])[0] * 0.01
                    speed = struct.unpack('<h', data[4:6])[0]
                    st = self.motor_states.setdefault(motor_id, {})
                    st['velocity'] = float(speed)
                    st['torque'] = current
                    # **주행은 건드리지 않는다** — 이미 전용 경로(drive_protect)가
                    # 적산 중이고, 여기서 또 부르면 중복 집계가 된다.
                    if motor_id not in self.drive_prot:
                        lvl = self.check_motor_current_safety(
                            motor_id, current, temperature)
                        self.apply_current_protection_action(motor_id, lvl)
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
            elif command == 0x61:
                # 멀티턴 원위치. 전원마다 영점이 달라지므로 **절대값은 세션 한정**이지만,
                # `mod CPR` 한 단회전값은 물리적으로 고정이라 전원과 무관하다.
                if len(data) >= 8:
                    raw = struct.unpack('<i', data[4:8])[0]
                    single = raw % self.ENCODER_CPR
                    self.encoder_single[motor_id] = single
                    pub = self.encoder_single_pubs.get(motor_id)
                    if pub is not None:
                        m = Int32()
                        m.data = int(single)
                        pub.publish(m)
                    # 멀티턴 원값. 단회전은 28.8°(모터 1회전) 주기로 접혀 자세
                    # 판별이 뚫린다 — 멀티턴은 세션 안에서 접히지 않는다
                    mpub = self.encoder_multi_pubs.get(motor_id)
                    if mpub is not None:
                        mm = Int32()
                        mm.data = int(raw)
                        mpub.publish(mm)
            elif command == 0x9A:
                # 에러 상태 읽기 응답 (0x9A)
                if len(data) >= 4:
                    # DATA[3] = 브레이크 상태 (0x01 = 해제). 호밍 ARM 이 이걸 기다린다.
                    rel = (data[3] == 0x01)
                    if self.brake_state.get(motor_id) != rel:
                        self.get_logger().info(
                            f"브레이크 0x{motor_id:03X}: "
                            f"{'해제' if rel else '잠김'}")
                    self.brake_state[motor_id] = rel
                    pub = self.brake_pubs.get(motor_id)
                    if pub is not None:
                        b = Bool()
                        b.data = rel
                        pub.publish(b)
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
        if self.drive_tripped:
            self.get_logger().warning(
                f"주행 보호 작동 중 — cmd_vel 무시 ({self.drive_trip_reason}). "
                f"전류가 정격 이하로 3초 유지되면 자동 해제됩니다.")
            return
        self.last_cmd_vel_time = time.time()
        self.drive_cmd_active = bool(msg.linear.x or msg.angular.z)
        if self.drive_cmd_active:
            # 다음 정지 전이를 다시 찍을 수 있게 래치를 푼다
            self._drive_stop_logged = False

        # 로그: 수신한 cmd_vel
        # 값이 바뀔 때만 로그 (같은 명령이 초당 수십 번 들어온다 — 로그가 처리량을 깎는다)
        _cv = (round(msg.linear.x, 3), round(msg.angular.z, 3))
        if getattr(self, '_last_cmd_vel_log', None) != _cv:
            self._last_cmd_vel_log = _cv
            self.get_logger().info(
                f'📥 [ROS2] /cmd_vel 수신: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}'
            )
        
        # 선속도와 각속도 제한
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

        # ── 안전 차단 (L2 판정을 모터 직전에서 적용) ──────────────────────────
        stop_reason = self._safety_full_stop()
        if stop_reason is not None:
            if self._safety_blocked_logged != stop_reason:
                self._safety_blocked_logged = stop_reason
                self.get_logger().warning(f"안전 정지 — {stop_reason} (주행 명령 무시)")
            linear_vel = 0.0
            angular_vel = 0.0
        else:
            before = (linear_vel, angular_vel)
            linear_vel, angular_vel = self._safety_clamp_drive(linear_vel, angular_vel)
            if (linear_vel, angular_vel) != before:
                if self._safety_blocked_logged != 'dir':
                    self._safety_blocked_logged = 'dir'
                    self.get_logger().warning(
                        f"안전 차단 — 방향 차단 적용 ({self.safety.reason})")
            elif self._safety_blocked_logged is not None:
                self._safety_blocked_logged = None

        # 정지 명령 확인
        if abs(linear_vel) < 0.001 and abs(angular_vel) < 0.001:
            # 속도 0 명령으로 정지 (0xA2)
            if not getattr(self, '_drive_stop_logged', False):
                self._drive_stop_logged = True
                self.get_logger().info("주행 모터 정지")
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

        # 좌우 바퀴 속도 상한 — 둘 중 하나라도 정격을 넘으면 같은 비율로 축소한다.
        # linear/angular 를 따로만 제한하면 직진+회전이 겹칠 때 한쪽만 포화되어
        # 의도치 않게 휘어진다. 비율을 유지해야 주행 궤적이 보존된다.
        peak = max(abs(left_dps), abs(right_dps))
        if peak > self.wheel_max_dps:
            k = self.wheel_max_dps / peak
            self.get_logger().warning(
                f"바퀴 속도 상한 초과 {peak:.0f} dps > {self.wheel_max_dps:.0f} — "
                f"좌우 {k:.2f}배로 축소")
            left_dps *= k
            right_dps *= k

        # 속도 제어값 변환 (0.01dps/LSB)
        # 2026-09-08 전진방향 재정의 후 실물 기준:
        #   좌측 모터(0x142) = 전진 방향 그대로, 우측 모터(0x141) = 반전
        #   좌우 바퀴가 서로 마주보게 장착되어 있어 부호가 반대여야 같은 쪽으로 굴러간다.
        #   (rebar_base_control/can_sender.py 의 "오른쪽만 반전" 규약과 동일)
        left_speed_control = int(left_dps * 100)
        right_speed_control = -int(right_dps * 100)

        # 로그: CAN 명령 전송 전 (DEBUG 레벨)
        self.get_logger().debug(
            f'CAN2: 좌 0x{self.left_motor_id:03X}={left_speed_control}, '
            f'우 0x{self.right_motor_id:03X}={right_speed_control}'
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

        # maxTorque = 정격 전류의 백분율. 축별로 axes.yaml 에서 읽는다 (기본 100).
        data[1] = self.max_torque.get(motor_id, 100)

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
    
    def _arm_drive_motor_watchdog(self):
        """주행모터 통신두절 보호 무장 (RMD 0xB3).

        프레임: data[0]=0xB3, data[4:8]=uint32 ms (LE). 0 이면 비활성.
        모터가 그 시간 안에 어떤 명령도 받지 못하면 스스로 출력을 끊는다. 젯슨이
        프리즈해도 모터 내부 타이머는 독립이라 폭주를 막는다 (2차년도 실측 근거).

        ⚠ 주행모터(0x141/0x142)에만. 스테이지·Yaw 는 유휴 시 명령이 끊기는 것이 정상이라
          걸면 오작동한다.
        """
        ms = int(self.get_parameter('drive_motor_watchdog_ms').value)
        if ms <= 0:
            self.get_logger().info("모터측 통신두절 보호(0xB3) 비활성")
            return
        t = ms & 0xFFFFFFFF
        data = bytes([0xB3, 0x00, 0x00, 0x00,
                      t & 0xFF, (t >> 8) & 0xFF, (t >> 16) & 0xFF, (t >> 24) & 0xFF])
        for motor_id in (self.left_motor_id, self.right_motor_id):
            if self.can_manager.send_frame(motor_id, data):
                self.get_logger().info(
                    f"🛡️ 주행모터 0x{motor_id:03X} 통신두절 보호 무장: {ms}ms (0xB3)")
            else:
                self.get_logger().error(
                    f"❌ 0x{motor_id:03X} 통신두절 보호(0xB3) 무장 실패")

    def send_speed_command_single(self, motor_id: int, speed_control: int):
        """단일 모터 속도 명령 전송 (0xA2 - Speed Control Command)"""
        data = bytearray(8)
        data[0] = 0xA2  # Speed Control Command

        # maxTorque = 정격 전류의 백분율. 축별로 axes.yaml 에서 읽는다 (기본 100).
        data[1] = self.max_torque.get(motor_id, 100)

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

                    # 전류/열/스톨 보호 (2026-09-08 교체)
                    self.drive_protect(self.left_motor_id, torque, speed, temperature)
                    safety_level = 'n/a'

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
                    
                    self.drive_protect(self.right_motor_id, torque, speed, temperature)
                    safety_level = 'n/a'

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
