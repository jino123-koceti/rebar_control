#!/usr/bin/env python3
"""ROS2 node for controlling Pololu motor controllers"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from pololu_ros2.pololu_driver import Daisy, BAUD_SYNC


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
        self.serial_port, self.baudrate = serial_port, baudrate
        
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
        
        self.get_logger().info('Pololu node started successfully')
    
    def velocity_callback(self, msg, motor_id):
        """Handle velocity command (-1.0 to 1.0)"""
        try:
            # Convert -1.0 ~ 1.0 to -3200 ~ 3200
            speed = int(msg.data * 3200)
            speed = max(min(3200, speed), -3200)  # clamp
            
            try:
                self.motors[motor_id].drive(speed)
            except (OSError, serial.SerialException) as e:
                # ★ 2026-09-22 USB 허브가 EMI로 포트를 껐다 켜면(dmesg "disabled by hub
                #   (EMI?)") 장치는 새로 잡히는데 옛 핸들은 영원히 EIO → 트리거 불발인데
                #   결속 시퀀스는 '완료'로 끝난다. 포트를 다시 열고 한 번 재시도한다.
                self.get_logger().error(f'motor {motor_id} 쓰기 실패({e}) → 포트 재연결')
                self._reopen()
                self.motors[motor_id].drive(speed)
                self.get_logger().warn(f'motor {motor_id} 재연결 후 명령 성공')
            
            # Log every 20th command to avoid spam
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
            
            if self._log_counter % 20 == 0:
                self.get_logger().info(f'Motor {motor_id}: vel={msg.data:.2f}, speed={speed}')
                
        except Exception as e:
            self.get_logger().error(f'Error commanding motor {motor_id}: {e}')
    
    def _reopen(self):
        """공유 시리얼을 닫고 다시 연다(by-id 심볼릭 링크라 새 ttyACM을 따라간다)."""
        try:
            if Daisy.ser is not None:
                Daisy.ser.close()
        except Exception:
            pass
        Daisy.ser = serial.Serial(self.serial_port, baudrate=self.baudrate, timeout=1)
        Daisy.ser.write(BAUD_SYNC)
        for m in self.motors.values():
            m._exit_safe_start()

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
