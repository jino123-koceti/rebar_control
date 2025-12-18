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
from rebar_base_interfaces.msg import DriveControl, GripperControl, JointControl
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
        self.declare_parameter('lateral_motor_id', 0x143)
        # 0x141/0x142: 1 rev = 0.18 m → radius ≈ 0.02865 m
        self.declare_parameter('wheel_radius', 0.02865)  # m
        # 정격 속도 (rpm)
        self.declare_parameter('wheel_max_speed_rpm', 238.0)   # RMD X4-10
        self.declare_parameter('lateral_max_speed_rpm', 83.0)   # RMD X4-36

        # 파라미터 가져오기
        can_interface = self.get_parameter('can_interface').value
        can_bitrate = self.get_parameter('can_bitrate').value
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.lateral_motor_id = self.get_parameter('lateral_motor_id').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_max_speed_rpm = self.get_parameter('wheel_max_speed_rpm').value
        self.lateral_max_speed_rpm = self.get_parameter('lateral_max_speed_rpm').value

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

        # 관절 모터 현재 위치 추적 (상대 위치 제어용)
        self.joint_positions = {
            0x143: 0.0,
            0x144: 0.0,
            0x145: 0.0,
            0x146: 0.0,
            0x147: 0.0,
        }

        self.get_logger().info("CAN Sender 노드 초기화 완료")
        self.get_logger().info("  - 속도 제어: 0x141, 0x142 (DriveControl)")
        self.get_logger().info("  - 위치 제어: 0x143~0x147 (JointControl)")
        self.get_logger().info(
            f"  - Wheel limit: {self.wheel_max_speed_rpm:.0f} rpm ({self.wheel_speed_limit_cmd/100:.0f} dps)"
        )
        self.get_logger().info(
            f"  - Lateral limit: {self.lateral_max_speed_rpm:.0f} rpm ({self.lateral_speed_limit_cmd/100:.0f} dps)"
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
            # 왼쪽 모터 속도 (m/s -> dps)
            left_omega = msg.left_speed / self.wheel_radius  # rad/s
            left_dps = left_omega * 180.0 / 3.14159265359  # dps
            left_speed_cmd = int(left_dps * 100)  # 0.01 dps/LSB
            left_speed_cmd = self._clamp_speed_cmd(left_speed_cmd, self.wheel_speed_limit_cmd)

            # 오른쪽 모터 속도 (m/s -> dps, 반전 필요)
            right_omega = msg.right_speed / self.wheel_radius  # rad/s
            right_dps = right_omega * 180.0 / 3.14159265359  # dps
            right_speed_cmd = int(-right_dps * 100)  # 반전 + 0.01 dps/LSB
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
            if command in (0x90, 0x92, 0x94):
                can_msg = can.Message(
                    arbitration_id=motor_id,
                    data=[command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                    is_extended_id=False
                )

                # 전송
                self.bus.send(can_msg)

                self.get_logger().info(
                    f"[CAN TX 0x{motor_id:03X}] 0x{command:02X} Encoder/Angle 요청"
                )

        except Exception as e:
            self.get_logger().error(f"Encoder Request 전송 오류: {e}")

    def joint_control_callback(self, msg):
        """
        JointControl 메시지를 받아 위치 제어 명령 전송

        RMD-X4 Position Control Command:
        - Command Type: 0xA4 (Multi-turn Position Control)
        - Data: [0xA4, 0x00, max_speed_low, max_speed_high,
                 position_0, position_1, position_2, position_3]
        - Max Speed: uint16, 1 dps/LSB
        - Position: int32, 0.01 degree/LSB
        """
        if not self.bus:
            return

        try:
            motor_id = msg.joint_id
            target_deg = msg.position
            max_speed_dps = msg.velocity
            mode = msg.control_mode

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

    def _send_position_command(self, motor_id, target_deg, max_speed_dps):
        """
        위치 제어 CAN 명령 전송 (0xA4)

        Parameters:
        - motor_id: 모터 CAN ID (0x143~0x147)
        - target_deg: 목표 위치 (degree, 절대 위치)
        - max_speed_dps: 최대 속도 (dps)
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

            # 데이터 패킹
            data = [
                0xA4,  # Command Type: Multi-turn Position Control
                0x00,  # Reserved
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
                f"[CAN TX 0x{motor_id:03X}] 위치 명령: {target_deg:.1f}° @ {max_speed_dps:.0f} dps"
            )

        except Exception as e:
            self.get_logger().error(f"CAN 위치 명령 전송 실패 (ID: 0x{motor_id:03X}): {e}")

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
