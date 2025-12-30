#!/usr/bin/env python3
"""
Rebar Publisher Node
상태 정보 취합 및 발행

여러 ROS2 토픽을 구독하여 통합 상태 메시지를 생성하고 발행합니다.

구독:
- /control_mode (String) - navigator_base에서
- /robot_pose (PoseStamped) - ZED X에서
- /system_status (SystemStatus) - 하드웨어 상태
- /motor_feedback (MotorFeedback) - 모터 피드백

발행:
- /mission/status (String, JSON) - zenoh_client로 전달
"""

from typing import Dict, List, Any, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from rebar_base_interfaces.msg import IOStatus, MotorFeedback
import json
import math
from datetime import datetime


class RebarPublisher(Node):
    """상태 취합 및 발행 노드"""

    def __init__(self):
        super().__init__('rebar_publisher')

        # 파라미터 선언
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # 파라미터 가져오기
        publish_rate = self.get_parameter('publish_rate').value

        # ROS2 구독자
        self.control_mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            10
        )

        # 미션 진행 피드백 (navigator → UI)
        self.mission_feedback_sub = self.create_subscription(
            String,
            '/mission/feedback',
            self.mission_feedback_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        # 하위 계층에서 올라오는 IO/배터리 상태
        self.io_status_sub = self.create_subscription(
            IOStatus,
            '/io_status',
            self.io_status_callback,
            10
        )

        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback,
            '/motor_feedback',
            self.motor_feedback_callback,
            10
        )

        # ROS2 발행자
        self.mission_status_pub = self.create_publisher(
            String,
            '/mission/status',
            10
        )

        # 상태 저장
        self.control_mode = "idle"
        self.mission_status = "idle"
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.speed = 0.0
        self.heading = 0.0  # degrees
        self.battery = 0.0
        self.temperature = 0  # 모터 최고 온도
        self.current_waypoint = 0
        self.total_waypoints = 0
        self.errors = []

        # 모터 속도 계산용 상태 변수
        self.left_motor_speed_dps = 0.0   # 좌측 모터 속도 (dps)
        self.right_motor_speed_dps = 0.0  # 우측 모터 속도 (dps)
        self.wheel_radius = 0.02865       # m (can_sender.py와 동일)

        # 주기적 발행 타이머
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_status)

        self.get_logger().info("Rebar Publisher 노드 초기화 완료")
        self.get_logger().info(f"  - 발행 주기: {publish_rate} Hz")

    def control_mode_callback(self, msg: String) -> None:
        """제어 모드 업데이트"""
        self.control_mode = msg.data

    def pose_callback(self, msg: PoseStamped) -> None:
        """로봇 위치 업데이트"""
        # PoseStamped → dict (mm 단위)
        self.position['x'] = msg.pose.position.x * 1000.0  # m → mm
        self.position['y'] = msg.pose.position.y * 1000.0  # m → mm
        self.position['theta'] = self._quaternion_to_yaw(msg.pose.orientation)  # radian

        # Heading (degree)
        self.heading = math.degrees(self.position['theta'])

    def mission_feedback_callback(self, msg: String) -> None:
        """navigator의 미션 피드백 반영"""
        try:
            data = json.loads(msg.data)
            self.current_waypoint = data.get('current_waypoint', self.current_waypoint)
            self.total_waypoints = data.get('total_waypoints', self.total_waypoints)
            self.mission_status = data.get('state', self.mission_status)
        except Exception as e:
            self.get_logger().warn(f"미션 피드백 파싱 실패: {e}")

    def io_status_callback(self, msg: IOStatus) -> None:
        """I/O 및 배터리 상태 업데이트"""
        self.battery = msg.battery_voltage
        # IOStatus에는 온도/에러 메시지가 없으므로 그대로 유지

    def motor_feedback_callback(self, msg: MotorFeedback) -> None:
        """
        모터 피드백 업데이트 (속도 및 온도 계산)

        Motor IDs:
        - 0x41 (0x141): 좌측 바퀴 모터
        - 0x42 (0x142): 우측 바퀴 모터

        속도 계산:
        - msg.current_speed: dps (degrees per second) from 0xA2 response
        - 선속도 = 각속도 * 바퀴 반경
        - 각속도 (rad/s) = dps * (pi/180)
        """
        # 바퀴 모터 피드백만 처리 (0x41, 0x42)
        if msg.motor_id == 0x41:  # 좌측 모터 (0x141)
            self.left_motor_speed_dps = msg.current_speed
        elif msg.motor_id == 0x42:  # 우측 모터 (0x142)
            self.right_motor_speed_dps = msg.current_speed
        else:
            # 다른 모터 (0x43~0x47)는 온도만 추적
            if msg.temperature > self.temperature:
                self.temperature = msg.temperature
            return

        # 모터 온도 추적 (최고 온도)
        if msg.temperature > self.temperature:
            self.temperature = msg.temperature

        # 평균 선속도 계산 (m/s)
        # 참고: 우측 모터는 장착 방향에 따라 부호가 반대일 수 있음
        avg_dps = (abs(self.left_motor_speed_dps) + abs(self.right_motor_speed_dps)) / 2.0
        avg_rad_per_sec = avg_dps * (math.pi / 180.0)
        self.speed = avg_rad_per_sec * self.wheel_radius  # m/s

        # 에러 코드 추적
        if msg.error_code != 0:
            error_info = f"Motor 0x{msg.motor_id + 0x100:03X}: code {msg.error_code}"
            if error_info not in self.errors:
                self.errors.append(error_info)
                self.get_logger().error(f"모터 에러 감지: {error_info}")

        # 온도 경고 (70°C 이상)
        if msg.temperature > 70:
            self.get_logger().warn(
                f"모터 0x{msg.motor_id + 0x100:03X} 고온 경고: {msg.temperature}°C",
                throttle_duration_sec=5.0
            )

    def publish_status(self) -> None:
        """
        통합 상태 메시지 발행 (JSON 형식)

        형식:
        {
            "timestamp": "2025-12-17 14:22:47",
            "control_mode": "auto",
            "mission_status": "navigating",
            "position": {"x": 123.4, "y": 456.7, "theta": 1.57},
            "speed": 1.2,
            "heading": 45.5,
            "battery": 87.5,
            "temperature": 45,
            "current_waypoint": 3,
            "total_waypoints": 10,
            "errors": []
        }
        """
        try:
            status_data = {
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'control_mode': self.control_mode,
                'mission_status': self.mission_status,
                'position': self.position,
                'speed': self.speed,
                'heading': self.heading,
                'battery': self.battery,
                'temperature': self.temperature,
                'current_waypoint': self.current_waypoint,
                'total_waypoints': self.total_waypoints,
                'errors': self.errors
            }

            # JSON 변환
            json_str = json.dumps(status_data)

            # 발행
            msg = String()
            msg.data = json_str
            self.mission_status_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"상태 발행 오류: {e}")

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """Quaternion → Yaw (radian) 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = RebarPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
