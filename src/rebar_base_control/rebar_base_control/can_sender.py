#!/usr/bin/env python3
"""
CAN Sender Node
ROS2 메시지 → CAN 메시지 전송

DriveControl 메시지를 받아 CAN2 버스로 모터 제어 명령 전송
- 속도 제어: 0x141, 0x142 (DriveControl)
- 위치 제어: 0x143~0x147 (JointControl)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rebar_base_interfaces.msg import DriveControl, GripperControl, JointControl, MotorFeedback
from std_msgs.msg import Bool
import can
import struct
import math


class CANSender(Node):
    """ROS2 메시지를 CAN 메시지로 변환하여 전송하는 노드"""

    def __init__(self):
        super().__init__('can_sender')

        # 파라미터 선언
        self.declare_parameter('can_interface', 'can2')
        self.declare_parameter('can_bitrate', 1000000)  # 1Mbps
        self.declare_parameter('left_motor_id', 0x141)
        self.declare_parameter('right_motor_id', 0x142)
        # 주행모터 토크 상한 (0xA2 DATA[1], 정격전류 %). 0 = 제한 없음(기존 동작).
        # ⚠ 정격전류 미상이라 실측 교정 필요 — 위 data 패킹부 주석 참조.
        self.declare_parameter('max_torque_pct', 0)
        self.declare_parameter('lateral_motor_id', 0x143)
        # 0x141/0x142: 1 rev = 0.18 m → radius ≈ 0.02865 m
        self.declare_parameter('wheel_radius', 0.02865)  # m
        # 정격 속도 (rpm)
        self.declare_parameter('wheel_max_speed_rpm', 238.0)   # RMD X4-10
        self.declare_parameter('lateral_max_speed_rpm', 83.0)   # RMD X4-36
        # ⚠️ [2026-08-14] 횡이동(0x143) 토크 제한. 정격 전류의 %.
        #   주행모터(0xA2)는 max_torque_pct로 제한했지만 **횡이동은 위치제어라
        #   제한이 전혀 없었다** — 0xA4 프레임엔 maxTorque 바이트가 없다.
        #   → 0xA9(Force Control Position)로 바꿔 제한을 건다. ([[robot_freeze_safety]])
        #   0 = 제한 없음(0xA4 사용). 횡이동은 원래 부하가 큰 동작이라 100%로 둔다.
        self.declare_parameter('lateral_max_torque_pct', 100)
        # ★ [2026-09-09] 주행모터 토크 상한을 **좌우 따로** 줄 수 있게 한다.
        #   왜: 0x141이 죽어 X4-36으로 교체하면서 좌우 모델이 달라졌다.
        #   max_torque_pct 는 **정격전류의 %** 라 정격이 다르면 절대 토크가 달라진다:
        #       X4-10   정격 4.0 N·m × 180% =  7.2 N·m
        #       X4-36   정격 10.5 N·m × 180% = 18.9 N·m   ← 2.6배 세다
        #   같은 %를 주면 한쪽만 세게 밀어 직진이 틀어지고 차체가 비틀린다.
        #   X4-36 쪽을 69%로 주면 7.2 N·m 로 맞는다 (7.2/10.5).
        #   -1 = max_torque_pct 를 그대로 쓴다(기존 동작 유지). 기본은 -1이라
        #   이 값을 안 건드리면 **동작이 전혀 바뀌지 않는다**.
        self.declare_parameter('left_max_torque_pct', -1)
        self.declare_parameter('right_max_torque_pct', -1)
        # ★ [2026-09-14] 직진 드리프트 **잔여 보정** — 좌우 속도지령 배율.
        #   왜: 좌우 모터를 같은 모델(X4-36)로 통일하고 가감속·토크를 다 맞춰도,
        #     궤도 장력·바퀴 마모·좌우 하중이 다르면 같은 dps에 **다른 거리**를 간다.
        #     그 잔여분을 여기서 상수배로 깎는다.
        #   ⚠ **순서를 지킬 것** — 트림은 마지막 수단이다:
        #       ① check_drive_pair.py 로 가감속·토크·최고속 좌우 일치 확인
        #       ② 궤도 장력 좌우 비교 (기계적 원인이면 이게 먼저다)
        #       ③ 그래도 남는 편차만 트림으로
        #     기계 문제를 트림으로 덮으면 부하가 한쪽에 쏠린 채 숨는다 —
        #     0x141을 죽인 게 정확히 그 패턴이었다([[drive_motor_0x141_stall_2026-09-09]]).
        #   교정법: 평탄한 곳에서 직진 지령만 주고 일정 거리(예: 3 m) 주행 →
        #     **뒤처지는 쪽**의 트림을 올리거나 앞서는 쪽을 내린다.
        #     한쪽만 건드리면 실제 속도가 바뀌므로, 크게 벗어나면 양쪽을
        #     반대로 나눠 주는 편이 최고속을 덜 깎는다 (예: 1.02 / 0.98).
        #   1.0 = 보정 없음(기본). 안 건드리면 **동작이 전혀 바뀌지 않는다**.
        #   범위는 0.8~1.2로 묶는다 — 그 밖이면 트림이 아니라 기계 문제다.
        self.declare_parameter('left_speed_trim', 1.0)
        self.declare_parameter('right_speed_trim', 1.0)

        # 파라미터 가져오기
        can_interface = self.get_parameter('can_interface').value
        can_bitrate = self.get_parameter('can_bitrate').value
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.max_torque_pct = max(0, min(255, int(self.get_parameter('max_torque_pct').value)))
        self.lateral_motor_id = self.get_parameter('lateral_motor_id').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_max_speed_rpm = self.get_parameter('wheel_max_speed_rpm').value
        self.lateral_max_speed_rpm = self.get_parameter('lateral_max_speed_rpm').value
        self.lateral_max_torque_pct = max(
            0, min(255, int(self.get_parameter('lateral_max_torque_pct').value)))

        # 모터별 토크 상한 표. 미지정(-1)이면 공통값으로 떨어진다.
        def _tq(name):
            v = int(self.get_parameter(name).value)
            return self.max_torque_pct if v < 0 else max(0, min(255, v))
        self.torque_by_id = {
            self.left_motor_id: _tq('left_max_torque_pct'),
            self.right_motor_id: _tq('right_max_torque_pct'),
        }

        # 직진 드리프트 트림. 오타로 로봇이 폭주하지 않도록 범위를 좁게 묶는다.
        def _trim(name):
            v = float(self.get_parameter(name).value)
            c = max(0.8, min(1.2, v))
            if abs(c - v) > 1e-9:
                self.get_logger().warn(
                    f"{name}={v} 는 허용범위(0.8~1.2) 밖 → {c} 로 제한. "
                    f"이 정도로 벌어지면 트림이 아니라 기계 점검 대상이다.")
            return c
        self.left_speed_trim = _trim('left_speed_trim')
        self.right_speed_trim = _trim('right_speed_trim')

        # 속도 한도 (0.01 dps 단위) 계산: rpm → dps(=rpm*6) → 0.01 dps
        self.wheel_speed_limit_cmd = int(self.wheel_max_speed_rpm * 6.0 * 100.0)
        self.lateral_speed_limit_cmd = int(self.lateral_max_speed_rpm * 6.0 * 100.0)

        # CAN 버스 초기화
        try:
            self.bus = can.Bus(
                can_interface,
                bustype='socketcan',
                bitrate=can_bitrate
            )
            self.get_logger().info(f"✅ {can_interface} 연결 성공 ({can_bitrate}bps)")
        except Exception as e:
            self.get_logger().error(f"❌ {can_interface} 연결 실패: {e}")
            self.bus = None

        # ROS2 Subscribers
        self.drive_control_sub = self.create_subscription(
            DriveControl,
            '/drive_control',
            self.drive_control_callback,
            10
        )

        self.gripper_control_sub = self.create_subscription(
            GripperControl,
            '/gripper_control',
            self.gripper_control_callback,
            10
        )

        self.joint_control_sub = self.create_subscription(
            JointControl,
            '/joint_control',
            self.joint_control_callback,
            qos_profile_system_default
        )

        self.encoder_request_sub = self.create_subscription(
            JointControl,
            '/encoder_request',
            self.encoder_request_callback,
            10
        )

        # ★ 2026-09-22 요축 최종 보호막: **홈 리미트가 눌린 상태에선 −방향(홈 너머) 명령 차단.**
        #   리미트는 홈 한쪽에만 있고(반대쪽은 범퍼) 홈은 0x92가 가장 작은 끝이다
        #   (홈 ≈ +5~15°, 좌측 작업자세 ≈ +390°). 호밍·오케스트레이터·리모콘 모두
        #   /joint_control → 여기로 오므로(joint_controller의 리미트 검사를 안 거치는 경로 포함)
        #   여기서 막아야 모든 경로가 막힌다.
        #   계기: 홈에서 0x90 오판 → 호밍 좌→우 복귀가 −220° 명령 → 리미트 너머 충돌 반복.
        #   ⚠ 우측 작업자세는 **홈보다 −13.3°** 아래이고 하드스톱은 −18.29°다
        #     (tying_orchestrator.yaml). 그래서 −방향을 전부 막으면 결속을 못 한다 →
        #     **홈 에지(OFF→ON 순간의 0x92) 기준 −yaw_below_home_max(15°)까지만 허용.**
        #     에지를 아직 못 봤으면(홈에 멈춘 채로 부팅) 기준이 없으니 −방향 전부 차단 —
        #     호밍의 BACK_OFF(+)→FINE_HOME(−)이 에지를 만들어 준다.
        self.yaw_motor_id = 0x147
        self.yaw_home_on = False
        self.yaw_angle_now = None           # 0x92 실측 (can_parser → /motor_feedback)
        self.yaw_home_edge = None           # 홈 센서가 켜진 순간의 0x92
        self.yaw_below_home_max = 15.0      # 홈 아래 허용 폭 (작업자세 13.3 < 15 < 스톱 18.29)
        self.create_subscription(Bool, '/limit_sensors/yaw_home', self._yaw_home_cb, 10)
        self.create_subscription(MotorFeedback, '/motor_feedback', self._yaw_fb_cb, 50)

        # 관절 모터 현재 위치 추적 (상대 위치 제어용)
        self.joint_positions = {
            0x143: 0.0,
            0x144: 0.0,
            0x145: 0.0,
            0x146: 0.0,
            0x147: 0.0,
        }

        # 속도 명령 버퍼 (모터별 최신값만 유지, 타이머에서 일괄 전송)
        self.pending_speed_cmds = {}  # {motor_id: speed_dps}
        self.create_timer(1.0 / 200.0, self._flush_speed_commands)  # 200Hz

        # 주행 모터 0x92 multiturn position 주기적 요청 (position 기반 오도메트리용)
        # [프리즈 진단 2026-08-06] 0으로 꺼도 프리즈 발생 → 0x92 폴링은 원인 아님(무죄 확정).
        #   범인은 하드웨어(주행 고전류/진동 → 전원·커넥터 순간정지)로 좁혀짐. → 자율주행 오도메트리 위해 원복.
        self.declare_parameter('drive_encoder_rate', 20.0)  # Hz
        drive_encoder_rate = self.get_parameter('drive_encoder_rate').value
        if drive_encoder_rate > 0:
            self.create_timer(1.0 / drive_encoder_rate, self._request_drive_encoder)
            self.get_logger().info(f"  - Drive encoder 요청: {drive_encoder_rate:.0f} Hz (0x92)")

        # ⚠️ [폭주방지 2026-08-06] 주행모터 통신두절 보호(0xB3) 무장.
        #   Jetson 하드프리즈 시 SW 워치독은 함께 멈춰 무용지물 → 모터 자체가 watchdog_ms 안에
        #   명령을 못 받으면 스스로 출력 차단(정지). 주행모터(0x141/0x142)에만 적용.
        #   정상 시엔 0x92 폴링(50ms)+0xA2가 하트비트라 절대 안 걸림. 스테이지/yaw엔 미적용.
        self.declare_parameter('drive_motor_watchdog_ms', 500)  # 0 = 비활성
        self.drive_watchdog_ms = self.get_parameter('drive_motor_watchdog_ms').value
        if self.drive_watchdog_ms > 0:
            self._arm_drive_motor_watchdog(self.drive_watchdog_ms)

        self.get_logger().info("CAN Sender 노드 초기화 완료")
        self.get_logger().info("  - 속도 제어: 0x141, 0x142 (DriveControl)")
        self.get_logger().info("  - 위치 제어: 0x143~0x147 (JointControl)")
        self.get_logger().info(
            f"  - Wheel limit: {self.wheel_max_speed_rpm:.0f} rpm ({self.wheel_speed_limit_cmd/100:.0f} dps)"
        )
        self.get_logger().info(
            f"  - Lateral limit: {self.lateral_max_speed_rpm:.0f} rpm ({self.lateral_speed_limit_cmd/100:.0f} dps)"
        )
        # 직진 트림은 조용히 먹으면 나중에 원인 못 찾는다 → 기동 시 항상 찍는다.
        self.get_logger().info(
            f"  - Drive trim: L×{self.left_speed_trim:.3f} / R×{self.right_speed_trim:.3f}"
            + ("" if self.left_speed_trim == self.right_speed_trim == 1.0 else "  ⚠ 보정 적용 중")
        )
        self.get_logger().info(
            f"  - Drive torque: L {self.torque_by_id[self.left_motor_id]}% / "
            f"R {self.torque_by_id[self.right_motor_id]}%"
        )

    def drive_control_callback(self, msg):
        """
        DriveControl 메시지를 받아 CAN 속도 명령 전송

        RMD-X4 Speed Control Command:
        - Command Type: 0xA2 (Speed Control)
        - Data: [0xA2, 0x00, 0x00, 0x00, speed_low, speed_high, 0x00, 0x00]
        - Speed: int32, 0.01 dps/LSB
        """
        if not self.bus:
            return

        try:
            # ★ 트림은 **클램프 직전**에 곱한다 — 클램프 뒤에 곱하면 포화 구간에서
            #   한쪽만 한도를 넘겨 다시 잘리며 트림이 무효가 된다.
            # 왼쪽 모터 속도 (m/s -> dps)
            left_omega = msg.left_speed / self.wheel_radius  # rad/s
            left_dps = left_omega * 180.0 / 3.14159265359  # dps
            left_speed_cmd = int(left_dps * 100 * self.left_speed_trim)  # 0.01 dps/LSB
            left_speed_cmd = self._clamp_speed_cmd(left_speed_cmd, self.wheel_speed_limit_cmd)

            # 오른쪽 모터 속도 (m/s -> dps, 반전 필요)
            right_omega = msg.right_speed / self.wheel_radius  # rad/s
            right_dps = right_omega * 180.0 / 3.14159265359  # dps
            right_speed_cmd = int(-right_dps * 100 * self.right_speed_trim)  # 반전 + 0.01 dps/LSB
            right_speed_cmd = self._clamp_speed_cmd(right_speed_cmd, self.wheel_speed_limit_cmd)

            # 횡이동 모터는 위치 제어(0xA4) 전용으로 변경
            # DriveControl에서는 0x141, 0x142만 제어
            # 0x143은 joint_controller → JointControl → 0xA4로만 제어됨

            # CAN 메시지 생성 및 전송 (0x141, 0x142만)
            self._send_speed_command(self.left_motor_id, left_speed_cmd, self.wheel_speed_limit_cmd)
            self._send_speed_command(self.right_motor_id, right_speed_cmd, self.wheel_speed_limit_cmd)

        except Exception as e:
            self.get_logger().error(f"DriveControl 전송 오류: {e}")

    def _clamp_speed_cmd(self, speed_cmd: int, limit_cmd: int) -> int:
        """0.01 dps 단위 speed_cmd를 모터 정격 한도로 클램프."""
        return max(-limit_cmd, min(limit_cmd, speed_cmd))

    def _send_speed_command(self, motor_id, speed, speed_limit_cmd=None):
        """
        속도 제어 CAN 명령 전송 (0xA2)

        Parameters:
        - motor_id: 모터 CAN ID (0x141, 0x142)
        - speed: int32, 속도 (0.01 dps/LSB)
        """
        if not self.bus:
            return

        try:
            # 속도 제한: 모터별 한도 또는 기본값 (-30000~30000) 사용
            if speed_limit_cmd is not None:
                speed = max(-speed_limit_cmd, min(speed_limit_cmd, speed))
            else:
                speed = max(-30000, min(30000, speed))

            # 데이터 패킹
            # [Command, Reserved, Reserved, Reserved, Speed_low, Speed_high, Speed_upper, Speed_sign]
            speed_bytes = struct.pack('<i', speed)  # int32 -> 4 bytes (little-endian)

            data = [
                0xA2,  # Command Type: Speed Control
                # ★ DATA[1] = maxTorque — **모터 내부 전류루프의 하드 리밋**
                #   (프로토콜 V4.3 2.20: 정격전류의 %, 1%/LSB. **0이면 제한 없음** =
                #    모터 설정의 스톨 전류까지 허용 → 실측 28.3A까지 올라갔다.)
                #   drive_controller의 소프트웨어 거버너는 20Hz 외부 루프라 µs 단위
                #   과도전류를 못 잡는다(6A 걸고도 13~16A 관측). 이건 모터 안에서
                #   걸리므로 즉시 반응한다. ([[robot_freeze_safety]] 2026-08-14)
                #   ⚠ 정격전류를 모르므로 **실측 교정**할 것:
                #     %를 걸고 주행 → motor_current_probe로 천장 확인 → 역산.
                self.torque_by_id.get(motor_id, self.max_torque_pct),
                0x00,  # Reserved
                0x00,  # Reserved
                speed_bytes[0],  # Speed byte 0 (LSB)
                speed_bytes[1],  # Speed byte 1
                speed_bytes[2],  # Speed byte 2
                speed_bytes[3],  # Speed byte 3 (MSB)
            ]

            # CAN 메시지 생성
            msg = can.Message(
                arbitration_id=motor_id,
                data=data,
                is_extended_id=False
            )

            # 전송
            self.bus.send(msg)

        except Exception as e:
            self.get_logger().error(f"CAN 속도 명령 전송 실패 (ID: 0x{motor_id:03X}): {e}")

    def gripper_control_callback(self, msg):
        """
        GripperControl 메시지를 받아 그리퍼 제어
        (현재는 CAN이 아닌 Modbus로 제어하므로 여기서는 구현하지 않음)
        """
        # Modbus Controller에서 처리
        pass

    def encoder_request_callback(self, msg):
        """
        Encoder Request 메시지를 받아 0x90 명령 전송

        control_mode가 0x90이면 Single-Turn Encoder 읽기 명령 전송
        """
        if not self.bus:
            return

        try:
            motor_id = msg.joint_id
            command = msg.control_mode

            # control_mode = 0x90/0x92/0x94: 엔코더/각도 읽기
            # 0x9C: Read Motor Status 2 (temp, torque_current, speed, angle)
            # ⚠️ [2026-08-18] 아래 둘 추가 — 횡이동 스톨 복구용.
            #   0x9A: Read Motor Status 1 — **errorState(알람 비트)** 를 읽는다.
            #        0x143이 철근을 못 넘고 버티면 드라이버가 스톨(0x0002)을 띄우고
            #        **출력을 끊는다**(실측: 그 상태에서 손으로 축이 돌아감).
            #        상위가 이걸 못 보면 완료 신호만 무한정 기다린다.
            #   0x76: System Reset — 스톨 알람 해제에 **이것만** 듣는다.
            #        0x80/0x88/0x9B 조합으로는 안 풀리는 것을 실측 확인(2026-08-18).
            #        드라이버가 재기동돼도 0x92 멀티턴 위치는 유지된다.
            #   0x9B: Clear Error Flag — 원인이 남아 있으면 즉시 재설정된다(참고용).
            if command in (0x90, 0x92, 0x94, 0x9C, 0x9A, 0x76, 0x9B):
                can_msg = can.Message(
                    arbitration_id=motor_id,
                    data=[command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    is_extended_id=False
                )

                # 전송
                self.bus.send(can_msg)

                self.get_logger().debug(
                    f"[CAN TX 0x{motor_id:03X}] 0x{command:02X} Encoder/Angle 요청"
                )

        except Exception as e:
            self.get_logger().error(f"Encoder Request 전송 오류: {e}")

    def _yaw_home_cb(self, msg):
        on = bool(msg.data)
        if on and not self.yaw_home_on and self.yaw_angle_now is not None:
            self.yaw_home_edge = self.yaw_angle_now
            self.get_logger().info(f'요축 홈 에지 기록: 0x92={self.yaw_home_edge:.2f}° '
                                   f'(허용 하한 {self.yaw_home_edge - self.yaw_below_home_max:.2f}°)')
        self.yaw_home_on = on

    def _yaw_fb_cb(self, msg):
        if msg.motor_id == (self.yaw_motor_id & 0xFF) and msg.status == 0x92:
            self.yaw_angle_now = float(msg.current_position)

    def _yaw_blocked(self, mode, value):
        """홈 리미트 ON에서 −방향이면 사유 문자열, 아니면 None.
        value: SPEED=dps / RELATIVE=delta° / ABSOLUTE=목표°"""
        if not self.yaw_home_on:
            return None
        cur = self.yaw_angle_now
        floor = (self.yaw_home_edge - self.yaw_below_home_max
                 if self.yaw_home_edge is not None else None)
        # −방향인지, 그렇다면 도착점(속도모드는 현재각)이 하한 위인지
        if mode == JointControl.MODE_SPEED:
            if value >= -0.1:
                return None
            dest = cur
        elif mode == JointControl.MODE_RELATIVE:
            if value >= -0.05:
                return None
            dest = (cur + value) if cur is not None else None
        else:
            if cur is not None and value >= cur - 0.5:
                return None
            dest = value
        if floor is None:
            return f'−방향 {value:.1f} (홈 에지 미기록 → 기준 없음)'
        if dest is None:
            return f'−방향 {value:.1f} (현재각 미수신)'
        if dest < floor:
            return f'도착 {dest:.1f}° < 허용 하한 {floor:.1f}° (홈 {self.yaw_home_edge:.1f}−{self.yaw_below_home_max:.0f})'
        return None

    def joint_control_callback(self, msg):
        """
        JointControl 메시지를 받아 위치/속도 제어 명령 전송

        RMD-X4 Position Control Command (0xA4):
        - Data: [0xA4, 0x00, max_speed_low, max_speed_high,
                 position_0, position_1, position_2, position_3]
        - Max Speed: uint16, 1 dps/LSB
        - Position: int32, 0.01 degree/LSB

        RMD-X4 Speed Control Command (0xA2):
        - Data: [0xA2, 0x00, 0x00, 0x00, speed_0, speed_1, speed_2, speed_3]
        - Speed: int32, 0.01 dps/LSB
        """
        if not self.bus:
            return

        try:
            motor_id = msg.joint_id
            target_deg = msg.position
            max_speed_dps = msg.velocity
            mode = msg.control_mode

            if motor_id == self.yaw_motor_id:
                why = self._yaw_blocked(mode, target_deg)
                if why:
                    self.get_logger().error(
                        f'⛔ 요축 차단: 홈 리미트 ON 상태에서 −방향 명령 ({why}) → 정지',
                        throttle_duration_sec=1.0)
                    self.pending_speed_cmds.pop(motor_id, None)
                    self._send_motor_stop(motor_id)
                    return

            # 속도 제어 모드 (MODE_SPEED = 2) → 버퍼에 저장, 타이머에서 일괄 전송
            if mode == JointControl.MODE_SPEED:
                speed_dps = target_deg
                self.pending_speed_cmds[motor_id] = speed_dps
                return

            self.get_logger().info(
                f"[JointControl RX] ID:0x{motor_id:03X} pos:{target_deg:.1f}° vel:{max_speed_dps:.0f}dps mode:{mode}"
            )

            # 상대 위치 모드: 현재 위치에 delta 추가
            if mode == JointControl.MODE_RELATIVE:
                if motor_id in self.joint_positions:
                    self.joint_positions[motor_id] += target_deg
                    absolute_target = self.joint_positions[motor_id]
                else:
                    self.get_logger().warn(
                        f"알 수 없는 모터 ID: 0x{motor_id:03X}, 절대 위치로 처리"
                    )
                    absolute_target = target_deg
            else:
                # 절대 위치 모드: 누적 상태도 함께 갱신
                absolute_target = target_deg
                if motor_id in self.joint_positions:
                    self.joint_positions[motor_id] = absolute_target

            # 위치 제어 명령 전송
            self._send_position_command(motor_id, absolute_target, max_speed_dps)

        except Exception as e:
            self.get_logger().error(f"JointControl 전송 오류: {e}")

    def _flush_speed_commands(self):
        """버퍼된 속도 명령을 일괄 CAN 전송 (200Hz 타이머)"""
        if not self.pending_speed_cmds or not self.bus:
            return
        cmds = dict(self.pending_speed_cmds)
        self.pending_speed_cmds.clear()
        for motor_id, speed_dps in cmds.items():
            if motor_id == self.yaw_motor_id and self._yaw_blocked(JointControl.MODE_SPEED, speed_dps):
                self._send_motor_stop(motor_id)
                continue
            if abs(speed_dps) < 0.1:
                self._send_motor_stop(motor_id)
            else:
                self._send_speed_command_joint(motor_id, speed_dps)

    def _send_speed_command_joint(self, motor_id, speed_dps):
        """
        관절 모터 속도 제어 CAN 명령 전송 (0xA2)

        Parameters:
        - motor_id: 모터 CAN ID (0x144, 0x145 등)
        - speed_dps: 속도 (dps, degree per second)
        """
        if not self.bus:
            return

        try:
            # 속도 변환: dps → 0.01 dps/LSB
            speed_cmd = int(speed_dps * 100.0)

            # 속도 제한: ±30000 (= ±300 dps)
            speed_cmd = max(-30000, min(30000, speed_cmd))

            # 속도 바이트 (int32, little-endian)
            speed_bytes = struct.pack('<i', speed_cmd)

            # 데이터 패킹
            data = [
                0xA2,  # Command Type: Speed Control
                # ⚠ 관절/스테이지 모터는 **토크 제한을 걸지 않는다** —
                #   프리즈 원인은 주행모터 과부하이고, 스테이지는 결속 정밀도가 걸려 있어
                #   토크를 깎으면 위치오차가 생긴다. (주행모터 경로에만 max_torque_pct 적용)
                0x00,  # maxTorque = 0 (제한 없음)
                0x00,  # Reserved
                0x00,  # Reserved
                speed_bytes[0],  # Speed byte 0 (LSB)
                speed_bytes[1],  # Speed byte 1
                speed_bytes[2],  # Speed byte 2
                speed_bytes[3],  # Speed byte 3 (MSB)
            ]

            # CAN 메시지 생성
            can_msg = can.Message(
                arbitration_id=motor_id,
                data=data,
                is_extended_id=False
            )

            # 전송
            self.bus.send(can_msg)

            # 조이스틱 연속 제어는 로그 최소화 (DEBUG 레벨)
            self.get_logger().debug(
                f"[CAN TX 0x{motor_id:03X}] 속도 명령: {speed_dps:.1f} dps"
            )

        except Exception as e:
            self.get_logger().error(f"CAN 속도 명령 전송 실패 (ID: 0x{motor_id:03X}): {e}")

    def _send_motor_stop(self, motor_id):
        """
        모터 정지 CAN 명령 전송 (0x81)
        관성으로 인한 오버슈트 방지를 위한 즉시 정지 명령

        Parameters:
        - motor_id: 모터 CAN ID (0x144, 0x145 등)
        """
        if not self.bus:
            return

        try:
            # 0x81: Motor Stop (clear running status)
            data = [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

            can_msg = can.Message(
                arbitration_id=motor_id,
                data=data,
                is_extended_id=False
            )

            self.bus.send(can_msg)

            self.get_logger().debug(
                f"[CAN TX 0x{motor_id:03X}] 모터 정지 (0x81)"
            )

        except Exception as e:
            self.get_logger().error(f"모터 정지 명령 전송 실패 (ID: 0x{motor_id:03X}): {e}")

    def _send_position_command(self, motor_id, target_deg, max_speed_dps):
        """
        위치 제어 CAN 명령 전송 (0xA4 / 횡이동은 0xA9)

        Parameters:
        - motor_id: 모터 CAN ID (0x143~0x147)
        - target_deg: 목표 위치 (degree, 절대 위치)
        - max_speed_dps: 최대 속도 (dps)

        ⚠️ [2026-08-14] **횡이동 모터(0x143)만 0xA9(Force Control Position)** 를 쓴다.
           0xA4와 프레임이 동일하되 DATA[1]이 Reserved가 아니라 **maxTorque**다
           (프로토콜 V4.3 §2.24). 0xA4로는 토크를 못 잡아서, 주행모터에만 제한이
           걸리고 횡이동은 무제한인 상태였다 → 횡이동 중 프리즈 재현.
           덤으로 0xA9 응답에는 **토크 전류(iq)** 가 실려 와서, 그동안 못 보던
           횡이동 전류를 계측할 수 있다.
           결속 상부축(0x144~0x147)은 정밀도 우선이라 **0xA4 그대로 둔다.**
        """
        if not self.bus:
            return

        try:
            # 위치 변환: degree → 0.01 degree/LSB
            position_cmd = int(target_deg * 100.0)  # 0.01 degree/LSB

            # 속도 변환: dps → 1 dps/LSB
            speed_cmd = int(max_speed_dps)  # 1 dps/LSB
            speed_cmd = max(0, min(65535, speed_cmd))  # uint16 범위 제한

            # 속도 바이트 (uint16, little-endian)
            speed_bytes = struct.pack('<H', speed_cmd)  # 2 bytes

            # 위치 바이트 (int32, little-endian)
            position_bytes = struct.pack('<i', position_cmd)  # 4 bytes

            # 횡이동만 토크 제한이 가능한 0xA9로 보낸다 (그 외는 기존 0xA4 유지)
            use_force = (motor_id == self.lateral_motor_id
                         and self.lateral_max_torque_pct > 0)
            cmd_byte = 0xA9 if use_force else 0xA4
            torque_byte = self.lateral_max_torque_pct if use_force else 0x00

            # 데이터 패킹
            data = [
                cmd_byte,  # 0xA4=Multi-turn Position / 0xA9=Force Control Position
                torque_byte,  # 0xA9: maxTorque(정격 대비 %) / 0xA4: Reserved
                speed_bytes[0],  # Max Speed LSB
                speed_bytes[1],  # Max Speed MSB
                position_bytes[0],  # Position byte 0 (LSB)
                position_bytes[1],  # Position byte 1
                position_bytes[2],  # Position byte 2
                position_bytes[3],  # Position byte 3 (MSB)
            ]

            # CAN 메시지 생성
            can_msg = can.Message(
                arbitration_id=motor_id,
                data=data,
                is_extended_id=False
            )

            # 전송
            self.bus.send(can_msg)

            self.get_logger().info(
                f"[CAN TX 0x{motor_id:03X}] 위치 명령(0x{cmd_byte:02X}): "
                f"{target_deg:.1f}° @ {max_speed_dps:.0f} dps"
                + (f" torque≤{torque_byte}%" if use_force else "")
            )

        except Exception as e:
            self.get_logger().error(f"CAN 위치 명령 전송 실패 (ID: 0x{motor_id:03X}): {e}")

    def _request_drive_encoder(self):
        """주행 모터(0x141, 0x142) multiturn position(0x92) 주기적 요청"""
        if not self.bus:
            return
        try:
            for motor_id in (self.left_motor_id, self.right_motor_id):
                can_msg = can.Message(
                    arbitration_id=motor_id,
                    data=[0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    is_extended_id=False
                )
                self.bus.send(can_msg)
        except Exception as e:
            self.get_logger().error(f"Drive encoder 요청 실패: {e}")

    def _arm_drive_motor_watchdog(self, timeout_ms):
        """주행모터(0x141/0x142) 통신두절 보호 무장 — RMD 0xB3.

        모터가 timeout_ms 안에 어떤 명령도 못 받으면 스스로 출력 차단(정지).
        Jetson이 하드프리즈해도 모터 내부 타이머는 독립적으로 동작하므로 폭주를 막는다.
        RMD-X V3 프로토콜 0xB3: data[0]=0xB3, data[4:8]=uint32 ms(LE). 0이면 비활성.
        정상 주행 중엔 0x92(50ms)·0xA2가 계속 하트비트가 되어 트립되지 않는다.
        ⚠️ 주행모터에만. 스테이지/yaw는 유휴 시 명령이 끊겨 오작동하므로 걸지 않는다.
        """
        if not self.bus:
            return
        t = int(timeout_ms) & 0xFFFFFFFF
        payload = [0xB3, 0x00, 0x00, 0x00,
                   t & 0xFF, (t >> 8) & 0xFF, (t >> 16) & 0xFF, (t >> 24) & 0xFF]
        for motor_id in (self.left_motor_id, self.right_motor_id):
            try:
                self.bus.send(can.Message(
                    arbitration_id=motor_id,
                    data=payload,
                    is_extended_id=False
                ))
                self.get_logger().info(
                    f"🛡️ 주행모터 0x{motor_id:03X} 통신두절 보호 무장: {timeout_ms}ms (0xB3)")
            except Exception as e:
                self.get_logger().error(
                    f"❌ 0x{motor_id:03X} 통신두절 보호(0xB3) 무장 실패: {e}")

    def destroy_node(self):
        """노드 종료 시 정리"""
        if self.bus:
            # 모터 정지 명령 전송
            self._send_speed_command(self.left_motor_id, 0)
            self._send_speed_command(self.right_motor_id, 0)
            self.bus.shutdown()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
