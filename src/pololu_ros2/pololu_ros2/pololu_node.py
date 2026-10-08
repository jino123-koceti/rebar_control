#!/usr/bin/env python3
"""ROS2 node for controlling Pololu motor controllers"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pololu_ros2.pololu_driver import Daisy, decode_errors


class PololuNode(Node):
    """ROS2 node to control Pololu Simple Motor Controllers"""
    
    def __init__(self):
        super().__init__('pololu_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('motor_ids', [0, 1, 2])  # Device numbers
        self.declare_parameter('motor_topics', ['motor_0/vel', 'motor_1/vel', 'motor_2/vel'])
        
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        motor_ids = self.get_parameter('motor_ids').value
        motor_topics = self.get_parameter('motor_topics').value
        
        if len(motor_ids) != len(motor_topics):
            self.get_logger().error('motor_ids and motor_topics must have the same length!')
            raise ValueError('motor_ids and motor_topics length mismatch')
        
        # Initialize motors
        self.motors = {}
        for motor_id in motor_ids:
            try:
                self.motors[motor_id] = Daisy(motor_id, port=serial_port, baudrate=baudrate)
                self.get_logger().info(f'Initialized motor {motor_id} on {serial_port}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize motor {motor_id}: {e}')
                raise
        
        # Create subscribers
        self.subscribers = []
        for motor_id, topic in zip(motor_ids, motor_topics):
            sub = self.create_subscription(
                Float32,
                topic,
                lambda msg, mid=motor_id: self.velocity_callback(msg, mid),
                10
            )
            self.subscribers.append(sub)
            self.get_logger().info(f'Subscribed to {topic} for motor {motor_id}')
        
        # 기동 직후 보드 상태를 남긴다 — 전원·배선 문제를 발사 때까지 숨기지 않는다
        for motor_id, motor in self.motors.items():
            err, occ = motor.error_status(), motor.errors_occurred()
            if err is None:
                self.get_logger().warning(
                    f'Motor {motor_id}: 보드 상태 응답 없음 — 배선·전원 확인')
            else:
                msg = (f'Motor {motor_id}: 보드 에러 0x{err:04X} '
                       f'[{decode_errors(err)}]'
                       + (f', 그동안 걸렸던 것 0x{occ:04X} '
                          f'[{decode_errors(occ)}]'
                          if occ is not None else ''))
                if err:
                    self.get_logger().error(msg)
                else:
                    self.get_logger().info(msg)
        self.get_logger().info('Pololu node started successfully')
    
    def velocity_callback(self, msg, motor_id):
        """Handle velocity command (-1.0 to 1.0)"""
        try:
            # Convert -1.0 ~ 1.0 to -3200 ~ 3200
            speed = int(msg.data * 3200)
            speed = max(min(3200, speed), -3200)  # clamp
            
            motor = self.motors[motor_id]
            motor.drive(speed)

            # ⚠⚠ [2026-10-08] **보드에 에러가 걸렸는지 물어본다.**
            #   에러가 걸리면 속도 명령을 **조용히 무시한다** — 쓰기는 성공하므로
            #   로그에 흔적이 없고, 그 때문에 결속 미션이 "성공" 으로 보고하면서
            #   한 발도 안 당겼다. 정지 명령에는 묻지 않는다 (안전쪽이다).
            if speed != 0:
                err = motor.error_status()
                if err is None:
                    self.get_logger().warning(
                        f'Motor {motor_id}: 보드 상태를 못 읽었다 — '
                        '응답 없음 (배선·전원 확인)')
                else:
                    # 보드가 무엇을 하고 있다고 생각하는지 (`diagnostics` 주석 참고)
                    d = motor.diagnostics()
                    self.get_logger().info(
                        f'Motor {motor_id}: 보드 상태 — 목표 {d["target"]} / '
                        f'현재 {d["speed"]} / VIN {d["vin_mv"]}mV / '
                        f'{d["temp_c"]}°C / limit 0x'
                        + ('----' if d['limit'] is None else f'{d["limit"]:04X}'))
                if err:
                    self.get_logger().error(
                        f'Motor {motor_id}: **보드 에러 0x{err:04X} '
                        f'[{decode_errors(err)}] — 이 상태에서는 속도 명령이 '
                        '무시된다 (건이 안 움직인다)')
            
            # Log every 20th command to avoid spam
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
            
            if self._log_counter % 20 == 0:
                self.get_logger().info(f'Motor {motor_id}: vel={msg.data:.2f}, speed={speed}')
                
        except Exception as e:
            self.get_logger().error(f'Error commanding motor {motor_id}: {e}')
    
    def destroy_node(self):
        """Clean shutdown - stop all motors"""
        self.get_logger().info('Stopping all motors...')
        for motor_id, motor in self.motors.items():
            try:
                motor.stop()
                self.get_logger().info(f'Motor {motor_id} stopped')
            except Exception as e:
                self.get_logger().error(f'Error stopping motor {motor_id}: {e}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PololuNode()
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
