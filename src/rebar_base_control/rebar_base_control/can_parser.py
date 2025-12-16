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
import can
import struct
import threading


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

        데이터 포맷 (8바이트):
        - Byte 0: Command Type
        - Byte 1: Motor Temperature (°C)
        - Byte 2-3: Motor Torque Current (int16, 0.01A/LSB)
        - Byte 4-5: Motor Speed (int16, 1dps/LSB or 1rpm/LSB)
        - Byte 6-7: Encoder Position (uint16)
        """
        can_id = can_msg.arbitration_id
        data = can_msg.data

        # 모터 응답 ID 확인 (0x241, 0x242)
        if can_id not in [0x241, 0x242]:
            return

        # 모터 ID 계산 (0x241 -> 0x141, 0x242 -> 0x142)
        motor_id = can_id - 0x100

        if len(data) < 8:
            return

        try:
            # 데이터 파싱
            temperature = data[1]  # °C
            torque_current = struct.unpack('<h', data[2:4])[0] * 0.01  # A
            speed = struct.unpack('<h', data[4:6])[0]  # dps or rpm
            encoder_raw = struct.unpack('<H', data[6:8])[0]  # raw encoder

            # MotorFeedback 메시지 생성
            feedback_msg = MotorFeedback()
            feedback_msg.motor_id = motor_id
            feedback_msg.current_speed = float(speed)  # dps or rpm
            feedback_msg.current_position = float(encoder_raw)  # encoder counts
            feedback_msg.current_current = int(torque_current * 1000)  # A -> mA
            feedback_msg.temperature = temperature
            feedback_msg.error_code = 0  # TODO: 에러 코드 파싱
            feedback_msg.status = data[0]  # Command Type as status

            # 발행
            self.motor_feedback_pub.publish(feedback_msg)

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
        스위치 데이터 파싱 (0x2E4)

        주요 스위치:
        - S10 (S19): Manual 모드
        - S20: Auto 모드
        - Emergency_Stop
        """
        if len(data) < 8:
            return

        try:
            # 스위치 비트 파싱 (실제 Iron-MD 프로토콜에 맞춰 수정 필요)
            # 임시 구현: Byte 단위로 스위치 상태 추출

            # S19-S20 모드 스위치 (3단 스위치)
            s19 = (data[2] >> 3) & 0x01  # S19 (Manual)
            s20 = (data[2] >> 4) & 0x01  # S20 (Auto)

            # 비상정지
            emergency_stop = (data[7] >> 0) & 0x01

            # 기타 스위치들
            s13 = (data[1] >> 5) & 0x01  # 브레이크
            s14 = (data[1] >> 6) & 0x01  # 위치 리셋
            s17 = (data[2] >> 1) & 0x01
            s18 = (data[2] >> 2) & 0x01
            s21 = (data[2] >> 5) & 0x01
            s22 = (data[2] >> 6) & 0x01
            s23 = (data[2] >> 7) & 0x01
            s24 = (data[3] >> 0) & 0x01

            # RemoteControl 메시지 생성
            remote_msg = RemoteControl()
            remote_msg.switch_s10 = (s19 == 1)  # Manual mode
            remote_msg.switch_s20 = (s20 == 1)  # Auto mode
            remote_msg.emergency_stop = (emergency_stop == 1)

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
