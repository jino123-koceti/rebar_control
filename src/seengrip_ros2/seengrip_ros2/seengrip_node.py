#!/usr/bin/env python3
"""
Seengrip ROS2 노드
Seengrip Optimum Gripper를 Modbus RTU로 제어
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import JointState
import serial
import time


# Seengrip Register Definitions (매뉴얼 기반 수정)
# Holding Registers (쓰기)
MB_LIMIT_ACCEL = 5       # Limit of Acceleration (0~1000) [0.1%]
MB_GRIP_TORQUE = 6       # Grip Torque (0~1000) [0.1%] - 전원 인가 시 25%로 초기화
MB_MOTION_TORQUE = 7     # Motion Torque (1~1000) [0.1%] - 전원 인가 시 50%로 초기화
MB_LIMIT_SPEED = 8       # Limit of Speed (1~1000) [0.1%] - 전원 인가 시 50%로 초기화
MB_GRIP_COMMAND = 9      # Grip Command
MB_GRIP_CMD = 9          # Grip Command (alias)
MB_FAULT_CLEAR = 16      # Fault Clear
MB_DETECT_SPEED = 22     # Detect Speed (0~1000) [0.1%]

# Input Registers (읽기)
MB_GRIPPER_INFO = 2      # Gripper Information
MB_PRESENT_POS = 10      # Present Position (-100~1100) [0.1%]
MB_PRESENT_SPEED = 11    # Present Speed (0~1000) [0.1%]
MB_GRIP_POS_DEV = 12     # Grip Position Deviation (0~1000) [0.1%]
MB_PRESENT_TEMP = 13     # Present Temperature (0~2000) [0.1deg]
MB_PRESENT_VOLT = 14     # Present Power Voltage (0~400) [0.1V]
MB_PRESENT_STATUS = 15   # Present Gripper Status
MB_PRESENT_FAULT = 16    # Present Fault

# Grip Command 형식: ((명령번호 << 12) & 0xF000) | (매개변수 & 0x0FFF)
CMD_HOME = 1             # Home: 원점 복귀 (매개변수: 속도 0~1000)
CMD_MOVE = 2             # Move: 위치 이동 (매개변수: 목표 위치 0~1000)
CMD_GRIP = 5             # Grip: 파지 (매개변수: 목표 속도 -1000~1000)
CMD_SPEEDY_GRIP = 6      # Speedy Grip: 빠른 파지 (매개변수: 목표 위치 0~1000)


class SeengripNode(Node):
    """ROS2 노드로 Seengrip 그리퍼 제어"""
    
    def __init__(self):
        super().__init__('seengrip_node')
        
        # 파라미터 선언
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('default_speed', 500)
        self.declare_parameter('open_position', 0)
        self.declare_parameter('close_position', 2000)
        
        # 파라미터 가져오기
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.slave_id = self.get_parameter('slave_id').value
        self.default_speed = self.get_parameter('default_speed').value
        self.open_position = self.get_parameter('open_position').value
        self.close_position = self.get_parameter('close_position').value
        
        # 시리얼 연결
        self.ser = None
        self.connect()
        
        # 구독자 생성
        self.position_sub = self.create_subscription(
            Float32,
            '/gripper/position',
            self.position_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            Int32,
            '/gripper/command',
            self.command_callback,
            10
        )
        
        # 발행자 생성
        self.status_pub = self.create_publisher(
            JointState,
            '/gripper/joint_states',
            10
        )
        
        self.fault_pub = self.create_publisher(
            Int32,
            '/gripper/fault',
            10
        )
        
        # 타이머 - 10Hz로 상태 발행
        self.status_timer = self.create_timer(0.1, self.publish_status)

        # 그리퍼 상태 추적
        self.is_homing = False
        self.homing_start_time = None

        self.get_logger().info(f'Seengrip node started on {self.serial_port}')

        # 매뉴얼: 전원 인가 시 자동 원점 복귀 수행됨
        # 초기화 대기 (자동 원점 복귀 완료 대기)
        time.sleep(2.0)

        # Fault 확인 및 필요 시 수동 원점 복귀
        fault_data = self.read_input_register(MB_PRESENT_FAULT, 1)
        if fault_data and fault_data[0] == 1:
            self.get_logger().warn('Fault 1 detected (Finger encoder error - auto-homing failed)')
            self.get_logger().info('Performing manual homing...')
            self.home(500)
            time.sleep(3.0)  # 원점 복귀 완료 대기
            self.clear_fault()
            time.sleep(0.5)
            self.is_homing = False
            self.get_logger().info('Manual homing completed')
        else:
            self.get_logger().info('Gripper initialized (auto-homing completed)')
    
    def connect(self):
        """시리얼 포트 연결"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baudrate} baud')
            time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'Failed to open port {self.serial_port}: {e}')
            raise
    
    def calculate_crc(self, data):
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
    
    def write_register(self, register, value):
        """레지스터 쓰기 (Function code 0x06)"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().error('Serial port not open')
            return False
        
        data = bytearray([
            self.slave_id,
            0x06,  # Write single register
            (register >> 8) & 0xFF,
            register & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
        ])
        crc = self.calculate_crc(data)
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)
        
        try:
            self.ser.write(data)
            time.sleep(0.05)
            response = self.ser.read(100)
            return len(response) > 0
        except Exception as e:
            self.get_logger().error(f'Write register error: {e}')
            return False
    
    def read_input_register(self, register, count=1):
        """Input 레지스터 읽기 (Function code 0x04)"""
        if not self.ser or not self.ser.is_open:
            return None

        data = bytearray([
            self.slave_id,
            0x04,  # Read input registers (매뉴얼 기반 수정)
            (register >> 8) & 0xFF,
            register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF
        ])
        crc = self.calculate_crc(data)
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)

        try:
            self.ser.write(data)
            time.sleep(0.05)
            response = self.ser.read(100)

            if len(response) >= 5:
                # Response: [ID][Func][ByteCount][Data...][CRC]
                byte_count = response[2]
                if len(response) >= 3 + byte_count + 2:
                    # Extract data (2 bytes per register)
                    values = []
                    for i in range(count):
                        high = response[3 + i*2]
                        low = response[4 + i*2]
                        values.append((high << 8) | low)
                    return values
        except Exception as e:
            self.get_logger().error(f'Read input register error: {e}')

        return None

    def read_holding_register(self, register, count=1):
        """Holding 레지스터 읽기 (Function code 0x03)"""
        if not self.ser or not self.ser.is_open:
            return None

        data = bytearray([
            self.slave_id,
            0x03,  # Read holding registers
            (register >> 8) & 0xFF,
            register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF
        ])
        crc = self.calculate_crc(data)
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)

        try:
            self.ser.write(data)
            time.sleep(0.05)
            response = self.ser.read(100)

            if len(response) >= 5:
                # Response: [ID][Func][ByteCount][Data...][CRC]
                byte_count = response[2]
                if len(response) >= 3 + byte_count + 2:
                    # Extract data (2 bytes per register)
                    values = []
                    for i in range(count):
                        high = response[3 + i*2]
                        low = response[4 + i*2]
                        values.append((high << 8) | low)
                    return values
        except Exception as e:
            self.get_logger().error(f'Read holding register error: {e}')

        return None
    
    def clear_fault(self):
        """Fault 상태 클리어 (매뉴얼: Address 16)"""
        result = self.write_register(MB_FAULT_CLEAR, 1)
        if result:
            self.get_logger().info('Fault cleared')
        return result

    def get_gripper_status(self):
        """그리퍼 상태 읽기 (Present Gripper Status)"""
        status_data = self.read_input_register(MB_PRESENT_STATUS, 1)
        if status_data:
            return status_data[0]
        return None

    def is_busy(self):
        """그리퍼가 동작 중인지 확인 (상태 비트 확인)"""
        status = self.get_gripper_status()
        if status is not None:
            # Bit 확인 (매뉴얼에 따라 조정 필요)
            # 일반적으로 bit 0 = moving, bit 1 = gripping 등
            return (status & 0x03) != 0  # 동작 중 플래그
        return False

    def send_grip_command(self, cmd_num, param):
        """
        Grip Command 전송 (매뉴얼 기반)
        형식: ((명령번호 << 12) & 0xF000) | (매개변수 & 0x0FFF)
        """
        # Homing 중이면 명령 거부
        if self.is_homing and cmd_num != CMD_HOME:
            self.get_logger().warn('Gripper is homing, command ignored')
            return False

        # 매개변수 범위 제한 (0~1000 또는 -1000~1000)
        if param < 0:
            # 음수는 2의 보수로 변환 (12비트)
            param = (abs(param) ^ 0x0FFF) + 1
        param = param & 0x0FFF

        # 명령 조합
        command_value = ((cmd_num << 12) & 0xF000) | param

        # Homing 명령이면 상태 추적
        if cmd_num == CMD_HOME:
            self.is_homing = True
            self.homing_start_time = time.time()

        return self.write_register(MB_GRIP_COMMAND, command_value)

    def home(self, speed=500):
        """그리퍼 홈 이동 (매뉴얼: 명령 1, 매개변수: 속도 0~1000)"""
        self.get_logger().info(f'Homing gripper at speed {speed}...')
        return self.send_grip_command(CMD_HOME, speed)

    def move_to_position(self, position):
        """
        그리퍼 위치 이동 (매뉴얼: Move 명령 2)
        position: 0~1000 (0=완전 닫힘, 1000=완전 열림)
        """
        position = int(max(0, min(1000, position)))
        self.get_logger().info(f'Move to position: {position}')
        return self.send_grip_command(CMD_MOVE, position)

    def grip(self, speed):
        """
        파지 명령 (매뉴얼: Grip 명령 5)
        speed: -1000~1000 (음수=닫힘, 양수=열림)
        """
        speed = int(max(-1000, min(1000, speed)))
        self.get_logger().info(f'Grip at speed: {speed}')
        return self.send_grip_command(CMD_GRIP, speed)

    def speedy_grip(self, position):
        """
        빠른 파지 명령 (매뉴얼: Speedy Grip 명령 6)
        position: 0~1000
        """
        position = int(max(0, min(1000, position)))
        self.get_logger().info(f'Speedy grip to position: {position}')
        return self.send_grip_command(CMD_SPEEDY_GRIP, position)

    def open_gripper(self):
        """그리퍼 열기 (Grip 명령 사용, 양수 속도)"""
        return self.grip(500)  # 50% 속도로 열림

    def close_gripper(self):
        """그리퍼 닫기 (Grip 명령 사용, 음수 속도)"""
        return self.grip(-500)  # 50% 속도로 닫힘
    
    def position_callback(self, msg):
        """
        위치 명령 콜백 (0.0-1.0 정규화)
        0.0 = 완전 열림 (1000), 1.0 = 완전 닫힘 (0)
        """
        normalized = max(0.0, min(1.0, msg.data))
        # 0.0 -> 1000 (open), 1.0 -> 0 (close)
        position = int((1.0 - normalized) * 1000)
        self.move_to_position(position)

    def command_callback(self, msg):
        """명령 콜백
        
        명령 형식:
        - 1 또는 CMD_HOME: 홈 복귀
        - 100: 레거시 열기 명령
        - 101: 레거시 닫기 명령
        - 그 외 값: 직접 MB_GRIP_CMD 레지스터에 쓰기 (Modbus 명령)
          예: 21080 (0x5258) = g 5 600 (Command 5, Speed +600)
              23976 (0x5DA8) = g 5 -600 (Command 5, Speed -600)
        """
        cmd = msg.data
        if cmd == CMD_HOME or cmd == 1:
            self.home()
        elif cmd == 100:  # Custom: Open (legacy)
            self.open_gripper()
        elif cmd == 101:  # Custom: Close (legacy)
            self.close_gripper()
        else:
            # 직접 Modbus 명령을 MB_GRIP_CMD 레지스터에 전송
            self.get_logger().info(f'Sending raw Modbus command to MB_GRIP_CMD: {cmd} (0x{cmd:04X})')
            success = self.write_register(MB_GRIP_CMD, cmd)
            if success:
                self.get_logger().info(f'  -> Command sent successfully')
            else:
                self.get_logger().error(f'  -> Failed to send command')
    
    def publish_status(self):
        """그리퍼 상태 발행"""
        # 현재 위치 읽기 (Input Register이므로 0x04 사용)
        pos_data = self.read_input_register(MB_PRESENT_POS, 1)

        if pos_data:
            # JointState 메시지 발행
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['gripper']
            # Present Position: -100~1100 [0.1%] -> 실제값으로 변환
            joint_state.position = [float(pos_data[0]) / 10.0]

            self.status_pub.publish(joint_state)

        # Homing 상태 확인 (타임아웃 체크)
        if self.is_homing:
            if self.homing_start_time and (time.time() - self.homing_start_time > 5.0):
                # 5초 후에도 homing 중이면 완료로 간주
                self.is_homing = False
                self.get_logger().info('Homing timeout - assumed complete')

        # 에러 상태 읽기 (주기적으로)
        if hasattr(self, '_fault_counter'):
            self._fault_counter += 1
        else:
            self._fault_counter = 0

        if self._fault_counter % 10 == 0:  # 1Hz
            fault_data = self.read_input_register(MB_PRESENT_FAULT, 1)
            if fault_data:
                # Fault 값이 0xFFFF 같은 큰 값이면 무시 (정상 상태)
                fault_value = fault_data[0]
                # 유효한 Fault 범위 (1~20)만 처리
                if 1 <= fault_value <= 20:
                    fault_msg = Int32()
                    fault_msg.data = fault_value
                    self.fault_pub.publish(fault_msg)
                    self.get_logger().warn(f'Gripper fault: {fault_value}')

                    # Fault 자동 클리어
                    if fault_value == 1:
                        # Fault 1: 핑거 엔코더 오류 - 수동 원점 복귀 수행
                        self.get_logger().info('Fault 1 detected - performing manual homing')
                        self.home(500)
                        time.sleep(3.0)
                        self.clear_fault()
                        self.is_homing = False
                    elif fault_value == 14:
                        # Fault 14: 원점 복귀 중 명령 입력
                        self.get_logger().info('Fault 14 detected - clearing fault')
                        self.clear_fault()
                        self.is_homing = False
                    elif fault_value == 15:
                        # Fault 15: 원점 복귀 전 위치 이동 명령 - 수동 원점 복귀 수행
                        self.get_logger().info('Fault 15 detected - performing manual homing')
                        self.home(500)
                        time.sleep(3.0)
                        self.clear_fault()
                        self.is_homing = False
    
    def destroy_node(self):
        """노드 종료"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SeengripNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
