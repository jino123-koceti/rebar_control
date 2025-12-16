#!/usr/bin/env python3
"""
CAN Sender Node
ROS2 메시지 → CAN 메시지 전송

DriveControl 메시지를 받아 CAN2 버스로 모터 제어 명령 전송
- 0x141: 왼쪽 모터
- 0x142: 오른쪽 모터
"""

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import DriveControl, GripperControl
import can
import struct


class CANSender(Node):
    """ROS2 메시지를 CAN 메시지로 변환하여 전송하는 노드"""

    def __init__(self):
        super().__init__('can_sender')

        # 파라미터 선언
        self.declare_parameter('can_interface', 'can2')
        self.declare_parameter('can_bitrate', 1000000)  # 1Mbps
        self.declare_parameter('left_motor_id', 0x141)
        self.declare_parameter('right_motor_id', 0x142)
        self.declare_parameter('wheel_radius', 0.1)  # m

        # 파라미터 가져오기
        can_interface = self.get_parameter('can_interface').value
        can_bitrate = self.get_parameter('can_bitrate').value
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

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

        self.get_logger().info("CAN Sender 노드 초기화 완료")

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
            # m/s -> dps 변환
            # v = ω * r  =>  ω (rad/s) = v / r
            # dps = ω (rad/s) * 180 / π

            # 왼쪽 모터 속도 (m/s -> dps)
            left_omega = msg.left_speed / self.wheel_radius  # rad/s
            left_dps = left_omega * 180.0 / 3.14159265359  # dps
            left_speed_cmd = int(left_dps * 100)  # 0.01 dps/LSB

            # 오른쪽 모터 속도 (m/s -> dps, 반전 필요)
            right_omega = msg.right_speed / self.wheel_radius  # rad/s
            right_dps = right_omega * 180.0 / 3.14159265359  # dps
            right_speed_cmd = int(-right_dps * 100)  # 반전 + 0.01 dps/LSB

            # CAN 메시지 생성 및 전송
            self._send_speed_command(self.left_motor_id, left_speed_cmd)
            self._send_speed_command(self.right_motor_id, right_speed_cmd)

        except Exception as e:
            self.get_logger().error(f"DriveControl 전송 오류: {e}")

    def _send_speed_command(self, motor_id, speed):
        """
        속도 제어 CAN 명령 전송 (0xA2)

        Parameters:
        - motor_id: 모터 CAN ID (0x141, 0x142)
        - speed: int32, 속도 (0.01 dps/LSB)
        """
        if not self.bus:
            return

        try:
            # 속도 제한 (-30000 ~ 30000, 즉 -300 ~ 300 dps)
            speed = max(-30000, min(30000, speed))

            # 데이터 패킹
            # [Command, Reserved, Reserved, Reserved, Speed_low, Speed_high, Speed_upper, Speed_sign]
            speed_bytes = struct.pack('<i', speed)  # int32 -> 4 bytes (little-endian)

            data = [
                0xA2,  # Command Type: Speed Control
                0x00,  # Reserved
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
