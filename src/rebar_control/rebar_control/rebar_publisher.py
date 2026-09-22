#!/usr/bin/env python3
"""
Rebar Publisher Node
상태 정보 취합 및 발행

여러 ROS2 토픽을 구독하여 통합 상태 메시지를 생성하고 발행합니다.

구독:
- /control_mode (String) - navigator_base에서
- /encoder_odom (PoseStamped) - 엔코더 odometry
- /system_status (SystemStatus) - 하드웨어 상태
- /motor_feedback (MotorFeedback) - 모터 피드백
- /tying/status (String, JSON) - tying_orchestrator에서 결속 상태

발행:
- /mission/status (String, JSON) - zenoh_client로 전달 (tying_* 필드 포함)
"""

from typing import Dict, List, Any, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from rebar_base_interfaces.msg import IOStatus, MotorFeedback
import json
from collections import deque
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
            '/encoder_odom',
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

        # 결속 상태 피드백 (tying_orchestrator → rebar/status)
        self.tying_status_sub = self.create_subscription(
            String,
            '/tying/status',
            self.tying_status_callback,
            10
        )

        # ROS2 발행자
        self.mission_status_pub = self.create_publisher(
            String,
            '/mission/status',
            10
        )

        # 횡이동 완료 구독 → position.y 오프셋 누적
        self.lateral_complete_sub = self.create_subscription(
            String,
            '/lateral_motion_complete',
            self.lateral_complete_callback,
            10
        )
        self.lateral_y_offset = 0.0  # 횡이동 누적 오프셋 (mm)
        self.mm_per_rotation = 44.0  # 1회전 = 44mm (실측: 9회전=396mm)

        # 자율결속 진행 이벤트 구독 (rebar_drive_node) → UI 전달
        #   ⚠ 왜 링버퍼인가: UI는 이 상태를 10Hz로 폴링한다. "최신 한 줄"만 실으면
        #     스텝완료→검출→결속생략→다음스텝처럼 100ms 안에 몰리는 구간에서
        #     중간 이벤트가 통째로 사라진다. 최근 N개를 함께 실어 UI가 seq로
        #     새 것만 골라 append하게 한다.
        self.drive_event_sub = self.create_subscription(
            String,
            '/rebar_drive/event',
            self.drive_event_callback,
            50
        )
        self.drive_state_sub = self.create_subscription(
            String,
            '/rebar_drive/state',
            self.drive_state_callback,
            10
        )
        self.drive_events = deque(maxlen=self.DRIVE_EVENT_KEEP)
        self.drive_state = ''
        self.drive_message = ''

        # 자율결속 실행기 상태 — UI가 "지금 자율작업이 도는지"를 알 유일한 근거.
        # (지금까지 아무도 구독하지 않아 UI는 시작 여부조차 몰랐다)
        self.auto_tying_sub = self.create_subscription(
            String,
            '/auto_tying/status',
            self.auto_tying_callback,
            10
        )
        self.auto_tying = None

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

        # 결속 상태 (tying_orchestrator로부터 수신)
        self.tying_feedback = {}

        # 경로 웨이포인트 (navigator의 path_gen_complete에서 수신)
        self.waypoints = None  # 1회 전송 후 클리어

        # 작업영역 (navigator로부터 수신)
        self.work_area = None
        self.work_area_sub = self.create_subscription(
            String,
            '/work_area',
            self.work_area_callback,
            10
        )

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
        # 전후면 반전 보정: encoder_odom은 물리적 방향 그대로이므로
        # 부호 반전하여 전진=양수 (navigator와 동일)
        self.position['x'] = -msg.pose.position.x * 1000.0  # m → mm, 부호 반전
        self.position['y'] = -msg.pose.position.y * 1000.0 + self.lateral_y_offset  # m → mm, 부호 반전 + 횡이동 오프셋

        raw_yaw = self._quaternion_to_yaw(msg.pose.orientation)
        inv_yaw = raw_yaw + math.pi
        self.position['theta'] = math.atan2(math.sin(inv_yaw), math.cos(inv_yaw))

        # Heading (degree)
        self.heading = math.degrees(self.position['theta'])

    # UI로 함께 실어 보낼 최근 이벤트 수. UI 폴링(10Hz) 사이에 몰리는 이벤트를
    # 놓치지 않을 만큼이면 충분하고, msgpack 크기도 이 정도면 부담 없다.
    DRIVE_EVENT_KEEP = 20

    def drive_event_callback(self, msg: String) -> None:
        """rebar_drive 진행 이벤트 수신 → 링버퍼 적재."""
        try:
            ev = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.drive_events.append(ev)
        self.drive_message = ev.get('text', '')

    def auto_tying_callback(self, msg: String) -> None:
        """자율결속 실행기 상태 {"running","direction","pid","state"}."""
        try:
            self.auto_tying = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def drive_state_callback(self, msg: String) -> None:
        """rebar_drive FSM 상태 문자열 (FWD_STEP / LATERAL_MOVE / ABORT ...)."""
        self.drive_state = msg.data

    def lateral_complete_callback(self, msg: String) -> None:
        """횡이동 완료 시 position.y 오프셋 누적

        메시지 형식:
        - "MANUAL:+:9.0" → S17, 9회전
        - "MANUAL:-:9.0" → S18, 9회전
        - "COMPLETE" → Auto 모드 (rebar_controller에서 처리, 여기서는 무시)
        """
        try:
            parts = msg.data.split(':')
            if parts[0] not in ('MANUAL', 'AUTO') or len(parts) < 3:
                return
            direction = parts[1]
            turns = float(parts[2])
            dy_mm = turns * self.mm_per_rotation
            if direction == '+':
                self.lateral_y_offset += dy_mm  # S17 = +Y (좌측)
            else:
                self.lateral_y_offset -= dy_mm  # S18 = -Y (우측)
            self.get_logger().info(
                f"횡이동 오프셋 업데이트: {direction}{dy_mm:.0f}mm, 누적={self.lateral_y_offset:.0f}mm"
            )
        except Exception as e:
            self.get_logger().warn(f"횡이동 메시지 파싱 실패: {msg.data}, {e}")

    def mission_feedback_callback(self, msg: String) -> None:
        """navigator의 미션 피드백 반영"""
        try:
            data = json.loads(msg.data)
            self.current_waypoint = data.get('current_waypoint', self.current_waypoint)
            self.total_waypoints = data.get('total_waypoints', self.total_waypoints)
            self.mission_status = data.get('state', self.mission_status)

            # path_gen_complete 시 waypoints 캡처 (UI 전달용)
            if data.get('state') == 'path_gen_complete' and 'waypoints' in data:
                self.waypoints = data['waypoints']
                self.total_waypoints = data.get('waypoint_count', len(self.waypoints))
                if 'work_area' in data:
                    self.work_area = data['work_area']
                self.get_logger().info(
                    f"경로 수신: {len(self.waypoints)}개 웨이포인트 → status에 포함"
                )
        except Exception as e:
            self.get_logger().warn(f"미션 피드백 파싱 실패: {e}")

    def tying_status_callback(self, msg: String) -> None:
        """결속 오케스트레이터의 상태 피드백 수신

        JSON 필드: tying_state, tying_progress, tying_message, tying_result
        """
        try:
            self.tying_feedback = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"결속 상태 파싱 실패: {e}")

    def work_area_callback(self, msg: String) -> None:
        """작업영역 정보 수신 (navigator에서)"""
        try:
            self.work_area = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"작업영역 파싱 실패: {e}")

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

            # 결속 상태 필드 병합 (tying_orchestrator에서 수신)
            if self.tying_feedback:
                status_data.update(self.tying_feedback)

            # 자율결속 진행상황 (rebar_drive_node) — UI 로그창에 그대로 뿌릴 수 있게
            #   drive_message : 최신 한 줄 (간단 표시용)
            #   drive_events  : 최근 N개. UI는 마지막으로 표시한 seq보다 큰 것만 append
            if self.auto_tying is not None:
                status_data['auto_tying'] = self.auto_tying
            if self.drive_state:
                status_data['drive_state'] = self.drive_state
            if self.drive_events:
                status_data['drive_message'] = self.drive_message
                status_data['drive_events'] = list(self.drive_events)

            # 작업영역 정보 포함 (navigator에서 수신)
            if self.work_area:
                status_data['work_area'] = self.work_area

            # 경로 웨이포인트 포함 (path_gen_complete 시 1회)
            if self.waypoints is not None:
                status_data['waypoints'] = self.waypoints
                status_data['waypoint_count'] = len(self.waypoints)
                self.get_logger().info(
                    f"waypoints 발행: {len(self.waypoints)}개 (status에 포함)"
                )
                self.waypoints = None  # 1회 전송 후 클리어

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
