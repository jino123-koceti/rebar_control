#!/usr/bin/env python3
"""
CAN Parser Node
CAN 메시지 수신 → ROS2 메시지 변환

CAN2 (1Mbps): 모터 피드백 (0x241, 0x242)
CAN3 (250kbps): 리모콘 (0x1E4, 0x2E4, 0x764)
"""

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import MotorFeedback, RemoteControl
from std_msgs.msg import String
import can
import struct
import threading
import json

# RMD-X4 모터 에러 코드 (데이터시트 기준)
MOTOR_ERROR_CODES = {
    0x00: "No error",
    0x01: "Motor stall",
    0x02: "Low voltage",
    0x03: "Overvoltage",
    0x04: "Overcurrent",
    0x05: "Motor overheated",
    0x06: "MOS overheated",
    0x07: "Encoder error",
    0x08: "Phase wire break",
}

# 온도 임계값 (°C)
MOTOR_TEMP_WARNING = 70   # 경고
MOTOR_TEMP_CRITICAL = 85  # 위험


class CANParser(Node):
    """CAN 메시지를 ROS2 메시지로 변환하는 파서 노드"""

    def __init__(self):
        super().__init__('can_parser')

        # 파라미터 선언
        self.declare_parameter('motor_can_interface', 'can2')
        self.declare_parameter('remote_can_interface', 'can3')
        self.declare_parameter('motor_can_bitrate', 1000000)  # 1Mbps
        self.declare_parameter('remote_can_bitrate', 250000)  # 250kbps

        # 파라미터 가져오기
        motor_interface = self.get_parameter('motor_can_interface').value
        remote_interface = self.get_parameter('remote_can_interface').value
        motor_bitrate = self.get_parameter('motor_can_bitrate').value
        remote_bitrate = self.get_parameter('remote_can_bitrate').value

        # ROS2 Publishers
        self.motor_feedback_pub = self.create_publisher(
            MotorFeedback,
            '/motor_feedback',
            10
        )
        self.remote_control_pub = self.create_publisher(
            RemoteControl,
            '/remote_control',
            10
        )
        # 모터 에러 발행자
        self.motor_error_pub = self.create_publisher(
            String,
            '/motor_errors',
            10
        )

        # CAN 버스 초기화
        try:
            self.bus_motor = can.Bus(
                motor_interface,
                bustype='socketcan',
                bitrate=motor_bitrate
            )
            self.get_logger().info(f"✅ {motor_interface} 연결 성공 ({motor_bitrate}bps)")
        except Exception as e:
            self.get_logger().error(f"❌ {motor_interface} 연결 실패: {e}")
            self.bus_motor = None

        try:
            self.bus_remote = can.Bus(
                remote_interface,
                bustype='socketcan',
                bitrate=remote_bitrate
            )
            self.get_logger().info(f"✅ {remote_interface} 연결 성공 ({remote_bitrate}bps)")
        except Exception as e:
            self.get_logger().error(f"❌ {remote_interface} 연결 실패: {e}")
            self.bus_remote = None

        # 수신 스레드 시작
        self.running = True
        if self.bus_motor:
            self.motor_thread = threading.Thread(
                target=self._motor_receive_loop,
                daemon=True
            )
            self.motor_thread.start()

        if self.bus_remote:
            self.remote_thread = threading.Thread(
                target=self._remote_receive_loop,
                daemon=True
            )
            self.remote_thread.start()

        self.get_logger().info("CAN Parser 노드 초기화 완료")

    def _motor_receive_loop(self):
        """모터 CAN 메시지 수신 루프 (CAN2)"""
        while self.running:
            try:
                msg = self.bus_motor.recv(timeout=1.0)
                if msg:
                    self._parse_motor_feedback(msg)
            except Exception as e:
                self.get_logger().error(f"모터 CAN 수신 오류: {e}")

    def _remote_receive_loop(self):
        """리모콘 CAN 메시지 수신 루프 (CAN3)"""
        while self.running:
            try:
                msg = self.bus_remote.recv(timeout=1.0)
                if msg:
                    self._parse_remote_control(msg)
            except Exception as e:
                self.get_logger().error(f"리모콘 CAN 수신 오류: {e}")

    def _parse_motor_feedback(self, can_msg):
        """
        모터 피드백 CAN 메시지 파싱

        응답 ID:
        - 0x241: 왼쪽 모터 (0x141)
        - 0x242: 오른쪽 모터 (0x142)
        - 0x243: 횡이동 모터 (0x143)

        데이터 포맷 (8바이트):
        - Byte 0: Command Type
        - 0xA2 응답:
          - Byte 1: Motor Temperature (°C)
          - Byte 2-3: Motor Torque Current (int16, 0.01A/LSB)
          - Byte 4-5: Motor Speed (int16, 1dps/LSB)
          - Byte 6-7: Motor Angle (int16, 1°/LSB)
        - 0x90 응답 (Single-Turn Encoder):
          - Byte 2-3: Encoder position (uint16)
          - Byte 4-5: Encoder raw (uint16)
          - Byte 6-7: Encoder offset (uint16)
        """
        can_id = can_msg.arbitration_id
        data = can_msg.data

        # 모터 응답 ID 확인 (0x241~0x247: 주행+횡이동+3축스테이지+Yaw)
        if can_id not in [0x241, 0x242, 0x243, 0x244, 0x245, 0x246, 0x247]:
            return

        # 모터 ID 계산 (0x241 -> 0x41, ..., 0x247 -> 0x47)
        # uint8 범위: 0x141 = 321 (범위 초과) → 0x41 = 65 사용
        motor_id = can_id - 0x200  # 0x241 -> 0x41, 0x244 -> 0x44, etc.

        if len(data) < 8:
            return

        try:
            command_type = data[0]

            # 0x90: Single-Turn Encoder 응답
            if command_type == 0x90:
                encoder_position = struct.unpack('<H', data[2:4])[0]  # uint16
                encoder_raw = struct.unpack('<H', data[4:6])[0]  # uint16
                encoder_offset = struct.unpack('<H', data[6:8])[0]  # uint16

                # MotorFeedback 메시지 생성
                feedback_msg = MotorFeedback()
                feedback_msg.motor_id = motor_id
                feedback_msg.current_position = float(encoder_position)
                feedback_msg.current_speed = 0.0
                feedback_msg.current_current = 0
                feedback_msg.temperature = 0
                feedback_msg.error_code = 0
                feedback_msg.status = command_type

                self.motor_feedback_pub.publish(feedback_msg)

                # 관측 로그 (DEBUG 레벨로 변경 - 반복 로그 방지)
                if motor_id == 0x43:
                    self.get_logger().debug(
                        f"[PUB /motor_feedback] id:0x{motor_id:02X} status:0x{command_type:02X} enc:{encoder_position} raw:{encoder_raw} offset:{encoder_offset}"
                    )
                    self.get_logger().debug(
                        f"[0x90 RX 0x243] encoder:{encoder_position} raw:{encoder_raw} offset:{encoder_offset}"
                    )
                return

            # 0x92: Multi-Turn Angle (0.01 deg/LSB, output shaft)
            if command_type == 0x92:
                angle_raw = struct.unpack('<i', data[4:8])[0]  # int32, 0.01°/LSB
                angle_deg = angle_raw * 0.01

                feedback_msg = MotorFeedback()
                feedback_msg.motor_id = motor_id
                feedback_msg.current_position = float(angle_deg)
                feedback_msg.current_speed = 0.0
                feedback_msg.current_current = 0
                feedback_msg.temperature = 0
                feedback_msg.error_code = 0
                feedback_msg.status = command_type

                self.motor_feedback_pub.publish(feedback_msg)

                # 0x43~0x47 (횡이동 + 3축 스테이지 + Yaw) 로깅
                if motor_id in [0x43, 0x44, 0x45, 0x46, 0x47]:
                    self.get_logger().debug(
                        f"[0x92 RX 0x2{motor_id:02X}] angle:{angle_deg:.2f}° raw:{angle_raw}"
                    )
                return

            # 0x94: Single-Circle Angle (0.01 deg/LSB, 0~360°)
            if command_type == 0x94:
                angle_raw = struct.unpack('<H', data[6:8])[0]  # uint16, 0.01°/LSB
                angle_deg = angle_raw * 0.01

                feedback_msg = MotorFeedback()
                feedback_msg.motor_id = motor_id
                feedback_msg.current_position = float(angle_deg)
                feedback_msg.current_speed = 0.0
                feedback_msg.current_current = 0
                feedback_msg.temperature = 0
                feedback_msg.error_code = 0
                feedback_msg.status = command_type

                self.motor_feedback_pub.publish(feedback_msg)

                # 0x43~0x47 (횡이동 + 3축 스테이지 + Yaw) 로깅
                if motor_id in [0x43, 0x44, 0x45, 0x46, 0x47]:
                    self.get_logger().debug(
                        f"[0x94 RX 0x2{motor_id:02X}] angle:{angle_deg:.2f}° raw:{angle_raw}"
                    )
                return

            # 0xA2: Speed Control 응답 (기존)
            # 데이터 파싱 (프로토콜 정의 필드만 사용)
            temperature = data[1]  # °C
            torque_current = struct.unpack('<h', data[2:4])[0] * 0.01  # A
            speed = struct.unpack('<h', data[4:6])[0]  # dps or rpm
            angle_deg = struct.unpack('<h', data[6:8])[0]  # int16, 1°/LSB (프로토콜 정의)

            # 에러 코드 추출 (온도 기반 + 전류 기반)
            error_code = 0

            # 온도 기반 에러 감지
            if temperature >= MOTOR_TEMP_CRITICAL:
                error_code = 0x05  # Motor overheated
                self.get_logger().error(
                    f"🔥 모터 0x{motor_id + 0x100:03X} 과열 위험: {temperature}°C"
                )
            elif temperature >= MOTOR_TEMP_WARNING:
                self.get_logger().warn(
                    f"⚠️ 모터 0x{motor_id + 0x100:03X} 온도 경고: {temperature}°C",
                    throttle_duration_sec=5.0
                )

            # 과전류 감지 (모터 정격의 2배 이상)
            if abs(torque_current) > 20.0:  # 20A 초과 시 경고
                error_code = 0x04  # Overcurrent
                self.get_logger().error(
                    f"⚡ 모터 0x{motor_id + 0x100:03X} 과전류: {torque_current:.2f}A"
                )

            # MotorFeedback 메시지 생성
            feedback_msg = MotorFeedback()
            feedback_msg.motor_id = motor_id  # 0x41 or 0x42 (uint8 범위)
            feedback_msg.current_speed = float(speed)  # dps or rpm
            feedback_msg.current_position = float(angle_deg)  # 0xA2 응답의 각도 필드
            feedback_msg.current_current = int(torque_current * 1000)  # A -> mA
            feedback_msg.temperature = temperature
            feedback_msg.error_code = error_code  # 에러 코드 설정
            feedback_msg.status = command_type

            # 발행
            self.motor_feedback_pub.publish(feedback_msg)

            # 에러 발생 시 /motor_errors 토픽으로도 발행
            if error_code != 0:
                error_msg = String()
                error_msg.data = json.dumps({
                    'motor_id': f'0x{motor_id + 0x100:03X}',
                    'error_code': error_code,
                    'error_message': MOTOR_ERROR_CODES.get(error_code, 'Unknown'),
                    'temperature': temperature,
                    'current': torque_current,
                    'timestamp': self.get_clock().now().nanoseconds // 1000000  # ms
                })
                self.motor_error_pub.publish(error_msg)

            # 0x243 (0x143) 피드백 로그 (DEBUG 레벨로 변경 - 반복 로그 방지)
            if motor_id == 0x43:
                hex_data = data.hex()
                self.get_logger().debug(
                    f"[CAN RX 0x243] RAW:{hex_data} angle:{angle_deg} speed:{speed}dps current:{torque_current:.2f}A temp:{temperature}°C"
                )

        except Exception as e:
            self.get_logger().error(f"모터 피드백 파싱 오류 (ID: 0x{can_id:03X}): {e}")

    def _parse_remote_control(self, can_msg):
        """
        리모콘 CAN 메시지 파싱 (Iron-MD)

        CAN ID:
        - 0x1E4 (484): 조이스틱 아날로그 데이터
        - 0x2E4 (740): 스위치 및 상태
        - 0x764 (1892): Heartbeat
        """
        can_id = can_msg.arbitration_id
        data = can_msg.data

        if can_id == 0x1E4:
            # 조이스틱 아날로그 데이터
            self._parse_joystick_data(data)
        elif can_id == 0x2E4:
            # 스위치 상태
            self._parse_switch_data(data)
        elif can_id == 0x764:
            # Heartbeat (연결 상태 확인용)
            pass

    def _parse_joystick_data(self, data):
        """
        조이스틱 아날로그 데이터 파싱 (0x1E4)

        데이터 포맷:
        - Byte 0-1: AN1 (X축)
        - Byte 2-3: AN2 (Y축)
        - Byte 4-5: AN3 (전후진)
        - Byte 6-7: AN4 (좌우회전)
        """
        if len(data) < 8:
            return

        try:
            # 조이스틱 값 저장 (0-255 범위, 중립=127)
            self.joystick_an1 = data[0]  # X축
            self.joystick_an2 = data[1]  # Y축
            self.joystick_an3 = data[2]  # 전후진
            self.joystick_an4 = data[3]  # 좌우회전

            # RemoteControl 메시지에 포함하여 발행
            # (스위치 데이터와 결합하여 발행하므로 여기서는 저장만)

        except Exception as e:
            self.get_logger().error(f"조이스틱 데이터 파싱 오류: {e}")

    def _parse_switch_data(self, data):
        """
        스위치 데이터 파싱 (0x2E4) - Iron-MD 프로토콜

        Byte 0:
        - Bit 7: Emergency_Stop_Active
        - Bit 6: Emergency_Stop_Release

        Byte 3:
        - Bit 4: S19 (Manual 모드)
        - Bit 5: S20 (Auto 모드)
        """
        if len(data) < 8:
            return

        try:
            # Byte 0: 비상정지
            byte0 = data[0]
            emergency_stop_release = (byte0 >> 6) & 0x01
            emergency_stop_active = (byte0 >> 7) & 0x01

            # 비상정지 상태: Active=1이면 비상정지, Release=1이면 해제
            emergency_stop = (emergency_stop_active == 1)

            # Byte 3: S19/S20 모드 스위치
            byte3 = data[3]
            s19 = (byte3 >> 4) & 0x01  # Manual mode
            s20 = (byte3 >> 5) & 0x01  # Auto mode

            # RemoteControl 메시지 생성
            remote_msg = RemoteControl()
            remote_msg.switch_s10 = (s19 == 1)  # Manual mode (S10=S19)
            remote_msg.switch_s20 = (s20 == 1)  # Auto mode
            remote_msg.emergency_stop = emergency_stop

            # 조이스틱 값 포함 (이전에 저장한 값 사용)
            an1 = getattr(self, 'joystick_an1', 127)
            an2 = getattr(self, 'joystick_an2', 127)
            an3 = getattr(self, 'joystick_an3', 127)
            an4 = getattr(self, 'joystick_an4', 127)

            # float32 배열로 변환 (-1.0 ~ 1.0 범위로 정규화)
            remote_msg.joysticks = [
                float((an1 - 127) / 127.0),
                float((an2 - 127) / 127.0),
                float((an3 - 127) / 127.0),
                float((an4 - 127) / 127.0)
            ]

            # 기타 스위치들 (Byte 1, 2)
            byte1 = data[1]
            byte2 = data[2]
            s13 = (byte1 >> 5) & 0x01
            s14 = (byte1 >> 6) & 0x01

            # 버튼 재매핑: S17/S18이 byte3 상위 비트(0x40, 0x80)로 들어옴
            s17 = (byte3 >> 6) & 0x01
            s18 = (byte3 >> 7) & 0x01

            # S21/S22/S23/S24: 이전 코드(iron_md_teleop_node.py) 참고 - byte3에서 파싱
            s21 = (byte3 >> 2) & 0x01  # byte3 비트 2
            s22 = (byte3 >> 3) & 0x01  # byte3 비트 3
            s23 = (byte3 >> 0) & 0x01  # byte3 비트 0
            s24 = (byte3 >> 1) & 0x01  # byte3 비트 1

            # 기타 스위치 상태 (배열로 저장)
            remote_msg.buttons = [s13, s14, s17, s18, s21, s22, s23, s24]

            # AN command (임시로 0 설정)
            remote_msg.an_command = 0

            # 발행
            self.remote_control_pub.publish(remote_msg)

        except Exception as e:
            self.get_logger().error(f"스위치 데이터 파싱 오류: {e}")

    def destroy_node(self):
        """노드 종료 시 정리"""
        self.running = False

        if hasattr(self, 'motor_thread'):
            self.motor_thread.join(timeout=1.0)
        if hasattr(self, 'remote_thread'):
            self.remote_thread.join(timeout=1.0)

        if self.bus_motor:
            self.bus_motor.shutdown()
        if self.bus_remote:
            self.bus_remote.shutdown()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANParser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
