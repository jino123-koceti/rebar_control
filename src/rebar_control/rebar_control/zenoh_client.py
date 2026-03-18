#!/usr/bin/env python3
"""
Zenoh Client Node
UI ↔ ROS2 브릿지

Zenoh 프로토콜을 통해 외부 UI(Laptop)와 ROS2 시스템 간 통신을 중계합니다.

Zenoh 구독 (UI → Robot):
- "rebar/command" → /mission/command (String)

Zenoh 발행 (Robot → UI):
- "rebar/status" ← /mission/status (String JSON → msgpack)
- "rebar/pose" ← /encoder_odom (PoseStamped → msgpack)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rebar_base_interfaces.msg import MotorFeedback
import zenoh
import msgpack
import json
import math


class ZenohClient(Node):
    """Zenoh ↔ ROS2 브릿지 노드"""

    def __init__(self):
        super().__init__('zenoh_client')

        # 파라미터 선언
        self.declare_parameter('zenoh_mode', 'peer')  # peer or client
        self.declare_parameter('zenoh_router', '')  # client 모드일 때만 사용
        self.declare_parameter('zenoh_port', 7447)

        self.declare_parameter('command_key', 'rebar/command')
        self.declare_parameter('status_key', 'rebar/status')
        self.declare_parameter('pose_key', 'rebar/pose')

        self.declare_parameter('mission_command_topic', '/mission/command')
        self.declare_parameter('mission_status_topic', '/mission/status')
        self.declare_parameter('robot_pose_topic', '/encoder_odom')
        self.declare_parameter('control_mode_topic', '/control_mode')

        self.declare_parameter('status_publish_rate', 10.0)  # Hz
        self.declare_parameter('pose_publish_rate', 20.0)  # Hz

        # 파라미터 가져오기
        self.zenoh_mode = self.get_parameter('zenoh_mode').value
        self.zenoh_router = self.get_parameter('zenoh_router').value

        self.command_key = self.get_parameter('command_key').value
        self.status_key = self.get_parameter('status_key').value
        self.pose_key = self.get_parameter('pose_key').value

        self.mission_command_topic = self.get_parameter('mission_command_topic').value
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.control_mode_topic = self.get_parameter('control_mode_topic').value

        # Zenoh 세션 초기화
        # NOTE: DDS(FastRTPS)가 participant 수에 따라 7446 포트를 점유할 수 있어
        #       멀티캐스트 스카우팅을 비활성화하고 TCP 리스너만 사용
        try:
            config = zenoh.Config()
            port = self.get_parameter('zenoh_port').value

            # 멀티캐스트 스카우팅 비활성화 (DDS 포트 충돌 방지)
            config.insert_json5("scouting/multicast/enabled", "false")

            # TCP 리스너 설정 (외부 노트북에서 접속 가능)
            config.insert_json5("listen/endpoints", f'["tcp/0.0.0.0:{port}"]')

            if self.zenoh_mode == 'client' and self.zenoh_router:
                # Client 모드 (Router 주소 지정)
                endpoints = f'["tcp/{self.zenoh_router}:{port}"]'
                config.insert_json5("connect/endpoints", endpoints)
                self.session = zenoh.open(config)
                self.get_logger().info(f"✅ Zenoh 세션 열림 (Client 모드): {self.zenoh_router}:{port}")
            else:
                # Peer 모드 (TCP 리스너로 대기)
                self.session = zenoh.open(config)
                self.get_logger().info(f"✅ Zenoh 세션 열림 (Peer 모드, TCP 리스너: 0.0.0.0:{port})")
        except Exception as e:
            self.get_logger().error(f"❌ Zenoh 세션 열기 실패: {e}")
            self.session = None
            return

        # Zenoh 구독 (UI 명령 수신)
        if self.session:
            self.zenoh_sub = self.session.declare_subscriber(
                self.command_key,
                self.on_command_received
            )
            self.get_logger().info(f"📡 Zenoh 구독: {self.command_key}")

        # ROS2 발행자 (명령 → ROS2)
        self.mission_cmd_pub = self.create_publisher(
            String,
            self.mission_command_topic,
            10
        )

        # ROS2 구독자 (상태 → Zenoh)
        self.status_sub = self.create_subscription(
            String,
            self.mission_status_topic,
            self.on_status_received,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.robot_pose_topic,
            self.on_pose_received,
            10
        )

        self.control_mode_sub = self.create_subscription(
            String,
            self.control_mode_topic,
            self.on_control_mode_received,
            10
        )

        # 모터 피드백 구독 (속도 계산용)
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback,
            '/motor_feedback',
            self.on_motor_feedback_received,
            10
        )

        # 주기적 상태 발행 타이머
        self.status_timer = self.create_timer(
            1.0 / float(self.get_parameter('status_publish_rate').value),
            self.publish_status
        )

        # 상태 저장
        self.latest_status = None
        self.latest_pose = None
        self.latest_control_mode = "idle"

        # 모터 속도 계산용 상태
        self.left_motor_speed_dps = 0.0
        self.right_motor_speed_dps = 0.0
        self.wheel_radius = 0.02865  # m
        self.current_speed = 0.0     # m/s

        self.get_logger().info("Zenoh Client 노드 초기화 완료")
        self.get_logger().info(f"  - Command key: {self.command_key}")
        self.get_logger().info(f"  - Status key: {self.status_key}")
        self.get_logger().info(f"  - Pose key: {self.pose_key}")

    def on_command_received(self, sample):
        """
        Zenoh에서 UI 명령 수신 → ROS2 발행

        명령 형식:
        - "E-STOP", "START_MISSION", "GO_HOME" 등
        - "WAYPOINTS:<json>"
        """
        try:
            # zenoh-python은 payload가 ZBytes 객체일 수 있으므로 안전하게 문자열로 변환
            raw_payload = sample.payload
            if hasattr(raw_payload, 'to_bytes'):
                data_bytes = raw_payload.to_bytes()
            elif isinstance(raw_payload, (bytes, bytearray, memoryview)):
                data_bytes = bytes(raw_payload)
            else:
                data_bytes = str(raw_payload).encode('utf-8')

            command = data_bytes.decode('utf-8')

            # ROS2 토픽으로 발행
            msg = String()
            msg.data = command
            self.mission_cmd_pub.publish(msg)

            self.get_logger().info(f"📥 명령 수신 (Zenoh → ROS2): {command[:50]}")

        except Exception as e:
            self.get_logger().error(f"명령 수신 오류: {e}")

    def on_status_received(self, msg):
        """
        ROS2에서 상태 수신 → Zenoh 발행 (msgpack)

        msg.data는 JSON 형식의 String
        """
        self.latest_status = msg.data

    def on_pose_received(self, msg):
        """
        ROS2에서 위치 수신 → Zenoh 발행 (msgpack)

        PoseStamped → msgpack dict
        """
        try:
            from datetime import datetime

            # PoseStamped → dict 변환
            pose_data = {
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],  # "2025-12-17 14:22:47.123"
                'x': msg.pose.position.x * 1000.0,  # m → mm
                'y': msg.pose.position.y * 1000.0,  # m → mm
                'theta': self._quaternion_to_yaw(msg.pose.orientation),  # radian
                'speed': self.current_speed  # m/s (모터 피드백에서 계산)
            }

            # MessagePack 압축
            packed = msgpack.packb(pose_data)

            # Zenoh 발행 (고주파 - 20Hz)
            if self.session:
                self.session.put(self.pose_key, packed)

        except Exception as e:
            self.get_logger().error(f"위치 발행 오류: {e}")

    def on_control_mode_received(self, msg: String) -> None:
        """제어 모드 업데이트"""
        self.latest_control_mode = msg.data

    def on_motor_feedback_received(self, msg: MotorFeedback) -> None:
        """
        모터 피드백 수신 → 속도 계산

        Motor IDs:
        - 0x41 (0x141): 좌측 바퀴 모터
        - 0x42 (0x142): 우측 바퀴 모터
        """
        if msg.motor_id == 0x41:  # 좌측 모터
            self.left_motor_speed_dps = msg.current_speed
        elif msg.motor_id == 0x42:  # 우측 모터
            self.right_motor_speed_dps = msg.current_speed
        else:
            return

        # 평균 선속도 계산 (m/s)
        avg_dps = (abs(self.left_motor_speed_dps) + abs(self.right_motor_speed_dps)) / 2.0
        avg_rad_per_sec = avg_dps * (math.pi / 180.0)
        self.current_speed = avg_rad_per_sec * self.wheel_radius

    def publish_status(self):
        """
        통합 상태를 Zenoh로 발행 (주기적 호출)

        /mission/status의 JSON을 msgpack으로 변환하여 발행
        """
        if not self.session or not self.latest_status:
            return

        try:
            # JSON String → dict
            status_dict = json.loads(self.latest_status)

            # MessagePack 압축
            packed = msgpack.packb(status_dict)

            # Zenoh 발행
            self.session.put(self.status_key, packed)

        except Exception as e:
            self.get_logger().error(f"상태 발행 오류: {e}")

    def _quaternion_to_yaw(self, q):
        """Quaternion → Yaw (radian) 변환"""
        import math

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def destroy_node(self):
        """노드 종료 시 정리"""
        if self.session:
            self.session.close()
            self.get_logger().info("Zenoh 세션 닫힘")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZenohClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
