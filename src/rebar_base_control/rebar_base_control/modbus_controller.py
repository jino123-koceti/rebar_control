#!/usr/bin/env python3
"""
Modbus Controller Node
Modbus 통신 통합 관리

- Seengrip (Modbus RTU): 그리퍼 제어
- EZI-IO (Modbus TCP): 센서 상태 읽기
"""

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import IOStatus, GripperControl
from std_msgs.msg import Bool
from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.exceptions import ModbusException


class ModbusController(Node):
    """Modbus 통신을 통합 관리하는 노드"""

    def __init__(self):
        super().__init__('modbus_controller')

        # 파라미터 선언
        # Seengrip (Modbus RTU)
        self.declare_parameter('seengrip_port', '/dev/ttyUSB0')
        self.declare_parameter('seengrip_baudrate', 115200)
        self.declare_parameter('seengrip_slave_id', 1)

        # EZI-IO (Modbus TCP)
        self.declare_parameter('ezi_io_host', '192.168.1.100')
        self.declare_parameter('ezi_io_port', 502)
        self.declare_parameter('ezi_io_slave_id', 1)

        # 파라미터 가져오기
        seengrip_port = self.get_parameter('seengrip_port').value
        seengrip_baudrate = self.get_parameter('seengrip_baudrate').value
        self.seengrip_slave_id = self.get_parameter('seengrip_slave_id').value

        ezi_io_host = self.get_parameter('ezi_io_host').value
        ezi_io_port = self.get_parameter('ezi_io_port').value
        self.ezi_io_slave_id = self.get_parameter('ezi_io_slave_id').value

        # Seengrip Modbus RTU 클라이언트 초기화
        try:
            self.seengrip_client = ModbusSerialClient(
                port=seengrip_port,
                baudrate=seengrip_baudrate,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=1.0
            )
            if self.seengrip_client.connect():
                self.get_logger().info(f"✅ Seengrip 연결 성공 ({seengrip_port})")
            else:
                self.get_logger().error(f"❌ Seengrip 연결 실패 ({seengrip_port})")
                self.seengrip_client = None
        except Exception as e:
            self.get_logger().error(f"❌ Seengrip 초기화 실패: {e}")
            self.seengrip_client = None

        # EZI-IO Modbus TCP 클라이언트 초기화
        try:
            self.ezi_io_client = ModbusTcpClient(
                host=ezi_io_host,
                port=ezi_io_port,
                timeout=1.0
            )
            if self.ezi_io_client.connect():
                self.get_logger().info(f"✅ EZI-IO 연결 성공 ({ezi_io_host}:{ezi_io_port})")
            else:
                self.get_logger().error(f"❌ EZI-IO 연결 실패 ({ezi_io_host}:{ezi_io_port})")
                self.ezi_io_client = None
        except Exception as e:
            self.get_logger().error(f"❌ EZI-IO 초기화 실패: {e}")
            self.ezi_io_client = None

        # ROS2 Publishers
        self.io_status_pub = self.create_publisher(
            IOStatus,
            '/io_status',
            10
        )

        # ROS2 Subscribers
        self.gripper_control_sub = self.create_subscription(
            GripperControl,
            '/gripper_control',
            self.gripper_control_callback,
            10
        )

        # 주기적인 센서 상태 읽기 타이머 (10Hz)
        self.io_timer = self.create_timer(0.1, self.update_io_status)

        self.get_logger().info("Modbus Controller 노드 초기화 완료")

    def update_io_status(self):
        """EZI-IO 센서 상태 읽기 및 발행"""
        if not self.ezi_io_client:
            return

        try:
            # 디지털 입력 읽기 (Holding Register 또는 Discrete Input)
            # EZI-IO 실제 주소에 맞춰 수정 필요
            result = self.ezi_io_client.read_discrete_inputs(
                address=0,
                count=8,
                slave=self.ezi_io_slave_id
            )

            if result.isError():
                self.get_logger().warn("EZI-IO 디지털 입력 읽기 실패")
                return

            digital_inputs = result.bits[:8]

            # 아날로그 입력 읽기 (Holding Register)
            result_analog = self.ezi_io_client.read_holding_registers(
                address=0,
                count=4,
                slave=self.ezi_io_slave_id
            )

            analog_inputs = []
            if not result_analog.isError():
                # Register 값을 실제 전압/전류로 변환 (스케일링 필요)
                for reg in result_analog.registers:
                    analog_inputs.append(float(reg) / 100.0)  # 예: 100배 스케일링

            # IOStatus 메시지 생성
            io_msg = IOStatus()
            io_msg.digital_inputs = digital_inputs
            io_msg.analog_inputs = analog_inputs

            # 배터리 전압 (아날로그 입력 0번)
            if len(analog_inputs) > 0:
                io_msg.battery_voltage = analog_inputs[0]
            else:
                io_msg.battery_voltage = 0.0

            # 리미트 스위치 (디지털 입력)
            if len(digital_inputs) >= 2:
                io_msg.limit_switch_front = digital_inputs[0]
                io_msg.limit_switch_rear = digital_inputs[1]
            else:
                io_msg.limit_switch_front = False
                io_msg.limit_switch_rear = False

            # 발행
            self.io_status_pub.publish(io_msg)

        except ModbusException as e:
            self.get_logger().warn(f"EZI-IO 통신 오류: {e}")
        except Exception as e:
            self.get_logger().error(f"EZI-IO 상태 읽기 오류: {e}")

    def gripper_control_callback(self, msg):
        """그리퍼 제어 명령 처리 (Seengrip Modbus RTU)"""
        if not self.seengrip_client:
            return

        try:
            # Seengrip 제어 레지스터 주소 (실제 주소에 맞춰 수정 필요)
            # 예: 레지스터 0 = 명령, 레지스터 1 = 속도, 레지스터 2 = 힘
            command_register = 0
            speed_register = 1
            force_register = 2

            # 명령 쓰기
            result = self.seengrip_client.write_register(
                address=command_register,
                value=msg.command,
                slave=self.seengrip_slave_id
            )

            if result.isError():
                self.get_logger().error("그리퍼 명령 전송 실패")
                return

            # 속도 쓰기
            self.seengrip_client.write_register(
                address=speed_register,
                value=msg.speed,
                slave=self.seengrip_slave_id
            )

            # 힘 쓰기
            self.seengrip_client.write_register(
                address=force_register,
                value=msg.force,
                slave=self.seengrip_slave_id
            )

            cmd_str = ["STOP", "OPEN", "CLOSE", "GRIP"][msg.command] if msg.command < 4 else "UNKNOWN"
            self.get_logger().info(f"그리퍼 명령: {cmd_str}, 속도: {msg.speed}, 힘: {msg.force}")

        except ModbusException as e:
            self.get_logger().error(f"그리퍼 제어 오류 (Modbus): {e}")
        except Exception as e:
            self.get_logger().error(f"그리퍼 제어 오류: {e}")

    def destroy_node(self):
        """노드 종료 시 정리"""
        # Modbus 연결 종료
        if self.seengrip_client:
            self.seengrip_client.close()
        if self.ezi_io_client:
            self.ezi_io_client.close()

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
