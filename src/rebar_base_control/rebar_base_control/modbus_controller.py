#!/usr/bin/env python3
"""
Modbus Controller Node
Modbus 통신 통합 관리

- Seengrip (Modbus RTU): 그리퍼 제어 (직접 시리얼 통신)
- EZI-IO (Modbus TCP): 센서 상태 읽기
"""

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import IOStatus, GripperControl
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import serial
import time

# Seengrip Register Definitions (매뉴얼 기반)
# Holding Registers (쓰기)
MB_GRIP_COMMAND = 9      # Grip Command
MB_LIMIT_SPEED = 8       # Limit of Speed (1~1000) [0.1%]
MB_GRIP_TORQUE = 6       # Grip Torque (0~1000) [0.1%]
MB_FAULT_CLEAR = 16      # Fault Clear

# Input Registers (읽기)
MB_PRESENT_POS = 10      # Present Position (-100~1100) [0.1%]
MB_PRESENT_STATUS = 15   # Present Gripper Status
MB_PRESENT_FAULT = 16    # Present Fault

# Grip Command: ((명령번호 << 12) & 0xF000) | (매개변수 & 0x0FFF)
CMD_HOME = 1             # Home: 원점 복귀 (매개변수: 속도 0~1000)
CMD_MOVE = 2             # Move: 위치 이동 (매개변수: 목표 위치 0~1000)
CMD_GRIP = 5             # Grip: 파지 (매개변수: 목표 속도 -1000~1000)
CMD_SPEEDY_GRIP = 6      # Speedy Grip: 빠른 파지


class ModbusController(Node):
    """Modbus 통신을 통합 관리하는 노드"""

    def __init__(self):
        super().__init__('modbus_controller')

        # 파라미터 선언
        # Seengrip (Modbus RTU - 직접 시리얼)
        self.declare_parameter('seengrip_port', '/dev/ttyUSB0')
        self.declare_parameter('seengrip_baudrate', 115200)
        self.declare_parameter('seengrip_slave_id', 1)
        self.declare_parameter('gripper_default_speed', 500)  # 0.1% 단위 (50%)

        # EZI-IO (Modbus TCP) - 현재 미사용
        self.declare_parameter('ezi_io_host', '192.168.1.100')
        self.declare_parameter('ezi_io_port', 502)
        self.declare_parameter('ezi_io_slave_id', 1)

        # 파라미터 가져오기
        self.seengrip_port = self.get_parameter('seengrip_port').value
        self.seengrip_baudrate = self.get_parameter('seengrip_baudrate').value
        self.seengrip_slave_id = self.get_parameter('seengrip_slave_id').value
        self.gripper_default_speed = self.get_parameter('gripper_default_speed').value

        # Seengrip 시리얼 연결
        self.ser = None
        self._connect_seengrip()

        # ROS2 Publishers
        self.io_status_pub = self.create_publisher(
            IOStatus,
            '/io_status',
            10
        )

        self.gripper_status_pub = self.create_publisher(
            JointState,
            '/gripper/joint_states',
            10
        )

        # ROS2 Subscribers
        self.gripper_control_sub = self.create_subscription(
            GripperControl,
            '/gripper_control',
            self.gripper_control_callback,
            10
        )

        # 그리퍼 상태 발행 타이머 (10Hz)
        self.status_timer = self.create_timer(0.1, self.publish_gripper_status)

        # Homing 상태 추적
        self.is_homing = False
        self.homing_start_time = None

        self.get_logger().info("Modbus Controller 노드 초기화 완료")
        self.get_logger().info(f"  - Seengrip: {self.seengrip_port} @ {self.seengrip_baudrate}")

    def _connect_seengrip(self):
        """Seengrip 시리얼 연결"""
        try:
            self.ser = serial.Serial(
                port=self.seengrip_port,
                baudrate=self.seengrip_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            self.get_logger().info(f"✅ Seengrip 연결 성공 ({self.seengrip_port})")
            time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"❌ Seengrip 연결 실패: {e}")
            self.ser = None

    def _calculate_crc(self, data):
        """Modbus CRC16 계산"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _write_register(self, register, value):
        """Holding 레지스터 쓰기 (Function code 0x06)"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("시리얼 포트가 열려있지 않음")
            return False

        data = bytearray([
            self.seengrip_slave_id,
            0x06,  # Write single register
            (register >> 8) & 0xFF,
            register & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
        ])
        crc = self._calculate_crc(data)
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)

        try:
            self.ser.write(data)
            time.sleep(0.05)
            response = self.ser.read(100)
            return len(response) > 0
        except Exception as e:
            self.get_logger().error(f"레지스터 쓰기 오류: {e}")
            return False

    def _read_input_register(self, register, count=1):
        """Input 레지스터 읽기 (Function code 0x04)"""
        if not self.ser or not self.ser.is_open:
            return None

        data = bytearray([
            self.seengrip_slave_id,
            0x04,  # Read input registers
            (register >> 8) & 0xFF,
            register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF
        ])
        crc = self._calculate_crc(data)
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)

        try:
            self.ser.write(data)
            time.sleep(0.05)
            response = self.ser.read(100)

            if len(response) >= 5:
                byte_count = response[2]
                if len(response) >= 3 + byte_count + 2:
                    values = []
                    for i in range(count):
                        high = response[3 + i*2]
                        low = response[4 + i*2]
                        values.append((high << 8) | low)
                    return values
        except Exception as e:
            self.get_logger().error(f"레지스터 읽기 오류: {e}")

        return None

    def _send_grip_command(self, cmd_num, param):
        """
        Grip Command 전송 (Seengrip 매뉴얼 기반)
        형식: ((명령번호 << 12) & 0xF000) | (매개변수 & 0x0FFF)
        """
        # 매개변수 범위 제한
        if param < 0:
            # 음수는 2의 보수로 변환 (12비트)
            param = (abs(param) ^ 0x0FFF) + 1
        param = param & 0x0FFF

        # 명령 조합
        command_value = ((cmd_num << 12) & 0xF000) | param

        self.get_logger().debug(f"Grip Command: cmd={cmd_num}, param={param}, value=0x{command_value:04X}")

        return self._write_register(MB_GRIP_COMMAND, command_value)

    def gripper_control_callback(self, msg: GripperControl):
        """그리퍼 제어 명령 처리 (Seengrip 프로토콜)"""
        if not self.ser:
            self.get_logger().warn("Seengrip 연결되지 않음")
            return

        # speed: 0-100% -> 0-1000 (0.1% 단위)
        speed = int(msg.speed * 10) if msg.speed > 0 else self.gripper_default_speed

        try:
            if msg.command == GripperControl.COMMAND_STOP:
                # 정지: 속도 0으로 Grip 명령
                self._send_grip_command(CMD_GRIP, 0)
                self.get_logger().info("그리퍼: STOP")

            elif msg.command == GripperControl.COMMAND_OPEN:
                # 열기: Grip 명령 + 양수 속도
                self._send_grip_command(CMD_GRIP, speed)
                self.get_logger().info(f"그리퍼: OPEN (속도: {speed})")

            elif msg.command == GripperControl.COMMAND_CLOSE:
                # 닫기: Grip 명령 + 음수 속도
                self._send_grip_command(CMD_GRIP, -speed)
                self.get_logger().info(f"그리퍼: CLOSE (속도: {speed})")

            elif msg.command == GripperControl.COMMAND_GRIP:
                # 파지 (닫기와 동일)
                self._send_grip_command(CMD_GRIP, -speed)
                self.get_logger().info(f"그리퍼: GRIP (속도: {speed})")

            else:
                self.get_logger().warn(f"알 수 없는 명령: {msg.command}")

        except Exception as e:
            self.get_logger().error(f"그리퍼 제어 오류: {e}")

    def home_gripper(self, speed=500):
        """그리퍼 홈 복귀"""
        self.get_logger().info(f"그리퍼 홈 복귀 (속도: {speed})")
        self.is_homing = True
        self.homing_start_time = time.time()
        return self._send_grip_command(CMD_HOME, speed)

    def clear_fault(self):
        """Fault 상태 클리어"""
        result = self._write_register(MB_FAULT_CLEAR, 1)
        if result:
            self.get_logger().info("Fault 클리어 완료")
        return result

    def publish_gripper_status(self):
        """그리퍼 상태 발행"""
        if not self.ser:
            return

        # 현재 위치 읽기
        pos_data = self._read_input_register(MB_PRESENT_POS, 1)

        if pos_data:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['gripper']
            # Present Position: -100~1100 [0.1%] -> 실제값으로 변환
            joint_state.position = [float(pos_data[0]) / 10.0]

            self.gripper_status_pub.publish(joint_state)

        # Homing 타임아웃 체크
        if self.is_homing:
            if self.homing_start_time and (time.time() - self.homing_start_time > 5.0):
                self.is_homing = False
                self.get_logger().info("홈 복귀 타임아웃 - 완료로 간주")

    def destroy_node(self):
        """노드 종료 시 정리"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("시리얼 포트 종료")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModbusController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
