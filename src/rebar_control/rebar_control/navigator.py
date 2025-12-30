#!/usr/bin/env python3
"""
Navigator Node
미션 관리자 (웨이포인트 순회)

UI로부터 미션 명령을 수신하여 웨이포인트를 관리하고,
rebar_controller에게 목표 지점을 전달합니다.

State Machine:
- idle → planning → navigating → mission_done
- emergency_stop (언제든지)

구독:
- /mission/command (String) - UI 명령
- /control_mode (String) - navigator_base에서

발행:
- /mission/target_pose (PoseStamped) - 현재 목표
- /mission/feedback (String) - 진행 상황
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rebar_base_interfaces.msg import Waypoint, WaypointArray
from statemachine import StateMachine, State
import json
import math


class RebarMissionStateMachine(StateMachine):
    """미션 State Machine"""

    # States
    idle = State(initial=True)
    planning = State()
    navigating = State()
    paused = State()  # 일시정지 상태
    mission_done = State()
    emergency_stop = State()

    # Transitions
    plan = idle.to(planning)
    start = planning.to(navigating)
    pause = navigating.to(paused)  # 일시정지
    resume = paused.to(navigating)  # 재개
    complete = navigating.to(mission_done)
    reset = mission_done.to(idle)
    stop = planning.to(idle) | navigating.to(idle) | paused.to(idle) | mission_done.to(idle)
    estop = (
        idle.to(emergency_stop) |
        planning.to(emergency_stop) |
        navigating.to(emergency_stop) |
        paused.to(emergency_stop) |
        mission_done.to(emergency_stop)
    )
    recover = emergency_stop.to(idle)

    def __init__(self, navigator):
        self.navigator = navigator
        super().__init__()

    def on_enter_planning(self):
        self.navigator.get_logger().info("📋 State: PLANNING")

    def on_enter_navigating(self):
        self.navigator.get_logger().info("🚀 State: NAVIGATING")

    def on_enter_paused(self):
        self.navigator.get_logger().info("⏸️ State: PAUSED")
        self.navigator.publish_zero_cmd_vel()  # 즉시 정지

    def on_enter_mission_done(self):
        self.navigator.get_logger().info("✅ State: MISSION_DONE")

    def on_enter_emergency_stop(self):
        self.navigator.get_logger().info("🛑 State: EMERGENCY_STOP")

    def on_enter_idle(self):
        self.navigator.get_logger().info("💤 State: IDLE")


class Navigator(Node):
    """미션 관리자 노드"""

    def __init__(self):
        super().__init__('navigator')

        # State Machine 초기화
        self.sm = RebarMissionStateMachine(self)

        # 웨이포인트 관리
        self.waypoints = []  # [{'x': float, 'y': float, 'motion_type': int, 'max_speed': float}, ...]
        self.current_waypoint_idx = 0

        # Motion type detection threshold (mm)
        self.motion_threshold = 1.0  # 1mm threshold for auto-detection

        # ROS2 구독자
        self.mission_cmd_sub = self.create_subscription(
            String,
            '/mission/command',
            self.mission_command_callback,
            10
        )

        self.control_mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            10
        )

        # waypoint 도달 알림 수신
        self.waypoint_reached_sub = self.create_subscription(
            String,
            '/mission/waypoint_reached',
            self.waypoint_reached_callback,
            10
        )

        # ROS2 발행자
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/mission/target_pose',
            10
        )

        # Enhanced waypoint publisher (for hybrid_controller) - 개별 웨이포인트 (하위호환)
        self.enhanced_target_pub = self.create_publisher(
            Waypoint,
            '/mission/enhanced_target',
            10
        )

        # Batch waypoint array publisher (tire_roller style) - 전체 경로 한번에 전달
        self.waypoint_array_pub = self.create_publisher(
            WaypointArray,
            '/mission/waypoint_array',
            10
        )

        # 전체 경로 발행 완료 플래그
        self.path_published = False

        # 일시정지 시 주행을 멈추기 위한 cmd_vel zero 발행용
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.feedback_pub = self.create_publisher(
            String,
            '/mission/feedback',
            10
        )

        # 현재 제어 모드
        self.control_mode = "idle"

        # 주기적 업데이트 타이머 (5Hz)
        self.timer = self.create_timer(0.2, self.update_mission)

        self.get_logger().info("Navigator 노드 초기화 완료")

    def mission_command_callback(self, msg):
        """
        UI 명령 처리

        명령어:
        - "E-STOP" → emergency_stop
        - "STOP" → idle
        - "GO_HOME" → 홈 위치로 이동
        - "PLAN_PATH" → planning
        - "START_MISSION" → navigating
        - "PAUSE_MISSION" → 일시정지
        - "RESUME_MISSION" → 재개
        - "ABORT_MISSION" → 중단
        - "WAYPOINTS:<json>" → 웨이포인트 로드
        - {"waypoints": [...]} → JSON 직접 전송 (외부 PC용)
        - {"command": "PAUSE"} → JSON 명령어 (외부 PC용)
        """
        command = msg.data

        self.get_logger().info(f"📥 명령 수신: {command[:100]}")

        # JSON 형식 명령 처리 (외부 PC에서 전송)
        if command.startswith("{"):
            try:
                data = json.loads(command)

                # 1. waypoints 직접 전송
                if "waypoints" in data:
                    self.load_waypoints_from_json(command)
                    # 웨이포인트 로드 후 planning 상태로 대기 (START_MISSION 명령 대기)
                    # 자동 시작하지 않음 - 사용자가 명시적으로 START_MISSION 전송 필요
                    return

                # 2. command 필드가 있는 경우
                elif "command" in data:
                    json_command = data["command"]

                    if json_command == "START_MISSION":
                        if self.sm.current_state == self.sm.planning:
                            self.sm.start()
                        elif self.sm.current_state == self.sm.idle:
                            self.sm.plan()
                            self.sm.start()

                    elif json_command == "PAUSE":
                        if self.sm.current_state == self.sm.navigating:
                            try:
                                self.sm.pause()
                            except Exception as e:
                                self.get_logger().error(f"PAUSE 전이 실패: {e}")

                    elif json_command == "RESUME":
                        if self.sm.current_state == self.sm.paused:
                            try:
                                self.sm.resume()
                                self.get_logger().info("▶️ 미션 재개")
                            except Exception as e:
                                self.get_logger().error(f"RESUME 전이 실패: {e}")

                    elif json_command == "CANCEL":
                        self.get_logger().warn(f"🛑 CANCEL 명령 수신 - 즉시 정지")

                        # 즉시 정지
                        self.publish_zero_cmd_vel()

                        try:
                            if self.sm.current_state in (self.sm.navigating, self.sm.planning, self.sm.paused):
                                self.sm.stop()
                            elif self.sm.current_state == self.sm.mission_done:
                                self.sm.reset()
                        except Exception as e:
                            self.get_logger().warn(f"CANCEL 처리 중 상태 전이 실패: {e}")

                        self.waypoints.clear()
                        self.current_waypoint_idx = 0
                        self.path_published = False

                    else:
                        self.get_logger().warn(f"알 수 없는 JSON 명령: {json_command}")

                    return

            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON 파싱 오류: {e}")
                return

        # 기존 문자열 명령어 처리
        if command == "E-STOP":
            self.sm.estop()

        elif command == "STOP":
            self.get_logger().warn(f"🛑 STOP 명령 수신 - 즉시 정지")

            # 즉시 정지
            self.publish_zero_cmd_vel()

            # 진행 중이든 완료 상태든 즉시 idle로 복귀
            try:
                if self.sm.current_state in (self.sm.navigating, self.sm.planning):
                    self.sm.stop()
                elif self.sm.current_state == self.sm.mission_done:
                    self.sm.reset()
                else:
                    # idle 등에서는 상태만 정리
                    pass
            except Exception as e:
                self.get_logger().warn(f"STOP 처리 중 상태 전이 실패: {e}")

            self.waypoints.clear()
            self.current_waypoint_idx = 0
            self.path_published = False

        elif command == "GO_HOME":
            # 홈 위치 (0, 0)로 이동
            self.waypoints = [{'x': 0.0, 'y': 0.0, 'motion_type': Waypoint.MOTION_DIFFERENTIAL, 'max_speed': 0.3}]
            self.current_waypoint_idx = 0
            self.path_published = False
            self.sm.plan()
            self.sm.start()

        elif command == "PLAN_PATH":
            self.sm.plan()

        elif command == "START_MISSION":
            if self.sm.current_state == self.sm.planning:
                self.sm.start()
            elif self.sm.current_state == self.sm.idle:
                self.sm.plan()
                self.sm.start()

        elif command == "PAUSE_MISSION":
            if self.sm.current_state == self.sm.navigating:
                try:
                    self.sm.pause()  # navigating → paused 전이
                except Exception as e:
                    self.get_logger().error(f"PAUSE 전이 실패: {e}")
            else:
                self.get_logger().warn(f"PAUSE_MISSION: navigating 상태가 아님 (현재: {self.sm.current_state.id})")

        elif command == "RESUME_MISSION":
            if self.sm.current_state == self.sm.paused:
                try:
                    self.sm.resume()  # paused → navigating 전이
                    self.get_logger().info("▶️ 미션 재개")
                except Exception as e:
                    self.get_logger().error(f"RESUME 전이 실패: {e}")
            else:
                self.get_logger().warn(f"RESUME_MISSION: paused 상태가 아님 (현재: {self.sm.current_state.id})")

        elif command == "ABORT_MISSION":
            self.get_logger().warn(f"🛑 ABORT 명령 수신 - 즉시 정지")

            # 즉시 정지
            self.publish_zero_cmd_vel()

            try:
                # 진행 중/planning이면 stop 전이, 완료 상태면 reset
                if self.sm.current_state in (self.sm.navigating, self.sm.planning):
                    self.sm.stop()
                elif self.sm.current_state == self.sm.mission_done:
                    self.sm.reset()
                else:
                    # idle 등에서는 상태만 정리
                    pass
            except Exception as e:
                self.get_logger().warn(f"ABORT 처리 중 상태 전이 실패: {e}")

            self.waypoints.clear()
            self.current_waypoint_idx = 0
            self.path_published = False

        elif command.startswith("WAYPOINTS:"):
            # 웨이포인트 로드
            json_str = command[10:]  # "WAYPOINTS:" 제거
            self.load_waypoints_from_json(json_str)

        else:
            self.get_logger().warn(f"알 수 없는 명령: {command}")

    def control_mode_callback(self, msg):
        """제어 모드 업데이트 (navigator_base에서)"""
        old_mode = self.control_mode
        self.control_mode = msg.data

        # emergency_stop 감지
        if self.control_mode == "emergency_stop" and self.sm.current_state != self.sm.emergency_stop:
            self.sm.estop()

        # auto 모드 복구
        if old_mode == "emergency_stop" and self.control_mode == "auto":
            self.sm.recover()

    def detect_motion_type(self, prev_wp, curr_wp):
        """
        두 웨이포인트 간 motion type 자동 감지

        Args:
            prev_wp: {'x': float, 'y': float} (in mm)
            curr_wp: {'x': float, 'y': float} (in mm)

        Returns:
            motion_type: MOTION_DIFFERENTIAL or MOTION_LATERAL
        """
        dx = abs(curr_wp['x'] - prev_wp['x'])
        dy = abs(curr_wp['y'] - prev_wp['y'])

        # Pure Y motion (횡이동)
        if dy > self.motion_threshold and dx < self.motion_threshold:
            return Waypoint.MOTION_LATERAL
        # X motion or diagonal (전진/후진)
        else:
            return Waypoint.MOTION_DIFFERENTIAL

    def load_waypoints_from_json(self, json_str):
        """
        웨이포인트 JSON 로드

        형식 1 (간단 - 외부 PC, meters):
        {
            "waypoints": [
                {"x": 0.0, "y": 0.0},
                {"x": 0.2, "y": 0.0},
                ...
            ]
        }

        형식 2 (상세 - 내부, mm):
        {
            "waypoints": [
                {"x": 0.0, "y": 0.0, "motion_type": 0, "max_speed": 0.3},
                {"x": 200.0, "y": 50.0, "motion_type": 1, "max_speed": 200.0},
                ...
            ]
        }

        단위: meters (외부 PC) 또는 mm (내부 명령) - 자동 감지
        """
        try:
            data = json.loads(json_str)
            waypoints_raw = data.get('waypoints', [])

            self.waypoints = []

            # 단위 자동 감지: 첫 웨이포인트의 값이 10보다 크면 mm로 가정
            if waypoints_raw and (abs(waypoints_raw[0].get('x', 0)) > 10 or abs(waypoints_raw[0].get('y', 0)) > 10):
                unit_is_mm = True
                self.get_logger().info("📏 단위: mm 감지")
            else:
                unit_is_mm = False
                self.get_logger().info("📏 단위: meters 감지")

            # mm 단위 리스트 생성 (motion type detection용)
            waypoints_mm = []
            for wp in waypoints_raw:
                if unit_is_mm:
                    waypoints_mm.append(wp)
                else:
                    waypoints_mm.append({'x': wp['x'] * 1000.0, 'y': wp['y'] * 1000.0})

            for i, wp in enumerate(waypoints_raw):
                # 단위 변환
                if unit_is_mm:
                    x_m = wp['x'] / 1000.0  # mm → m
                    y_m = wp['y'] / 1000.0
                else:
                    x_m = wp['x']  # 이미 meters
                    y_m = wp['y']

                # Motion type: 명시적 지정 or 자동 감지
                if 'motion_type' in wp:
                    motion_type = wp['motion_type']
                else:
                    # 자동 감지 (이전 웨이포인트와 비교, mm 단위 사용)
                    if i == 0:
                        motion_type = Waypoint.MOTION_DIFFERENTIAL  # 첫 웨이포인트는 기본값
                    else:
                        motion_type = self.detect_motion_type(waypoints_mm[i-1], waypoints_mm[i])

                # Max speed: 명시적 지정 or 기본값
                if 'max_speed' in wp:
                    max_speed = wp['max_speed']
                else:
                    # 기본값: differential=0.3m/s, lateral=200dps
                    if motion_type == Waypoint.MOTION_LATERAL:
                        max_speed = 200.0  # dps
                    else:
                        max_speed = 0.3  # m/s

                waypoint_dict = {
                    'x': x_m,
                    'y': y_m,
                    'motion_type': motion_type,
                    'max_speed': max_speed
                }

                self.waypoints.append(waypoint_dict)

                # 로그 출력
                motion_str = "LATERAL" if motion_type == Waypoint.MOTION_LATERAL else "DIFFERENTIAL"
                if unit_is_mm:
                    self.get_logger().info(
                        f"  [{i}] ({wp['x']:.0f}, {wp['y']:.0f}) mm → {motion_str} @ {max_speed:.1f}"
                    )
                else:
                    self.get_logger().info(
                        f"  [{i}] ({wp['x']:.3f}, {wp['y']:.3f}) m → {motion_str} @ {max_speed:.1f}"
                    )

            self.current_waypoint_idx = 0
            self.path_published = False

            self.get_logger().info(f"✅ 웨이포인트 로드: {len(self.waypoints)}개")

            # Planning 상태로 전환 (idle 또는 mission_done 상태에서)
            if self.sm.current_state == self.sm.idle:
                self.sm.plan()
            elif self.sm.current_state == self.sm.mission_done:
                # mission_done 상태에서 새 웨이포인트를 받으면 리셋 후 planning으로 전환
                try:
                    self.sm.reset()  # mission_done → idle
                    self.sm.plan()   # idle → planning
                except Exception as e:
                    self.get_logger().warn(f"상태 리셋 실패: {e}")

        except Exception as e:
            self.get_logger().error(f"웨이포인트 로드 오류: {e}")

    def update_mission(self):
        """
        미션 업데이트 (타이머 콜백, 5Hz)

        Navigating 상태에서:
        1. 전체 경로를 한번에 발행 (WaypointArray)
        2. 이후 피드백만 발행 (rebar_controller가 내부 인덱스로 관리)

        Paused 상태에서는 피드백만 발행 (target_pose 발행 중단)
        Mission_done 상태에서도 피드백 발행 (performance_test가 다음 단계로 넘어가도록)
        """
        if self.sm.current_state == self.sm.navigating:
            if not self.path_published and len(self.waypoints) > 0:
                # 전체 경로를 한번에 발행 (tire_roller style)
                self.publish_waypoint_array()
                self.path_published = True
                self.get_logger().info(f"📤 전체 경로 발행 완료: {len(self.waypoints)}개 웨이포인트")

            # 피드백 발행 (rebar_controller에서 인덱스 업데이트 수신)
            self.publish_feedback()

        elif self.sm.current_state == self.sm.paused:
            # Paused 상태에서는 피드백만 발행 (target_pose는 발행하지 않음)
            self.publish_feedback()

        elif self.sm.current_state == self.sm.mission_done:
            # Mission_done 상태에서도 피드백 발행 (performance_test가 다음 단계로 넘어가도록)
            self.publish_feedback()

    def publish_waypoint_array(self):
        """
        전체 웨이포인트 배열을 한번에 발행 (tire_roller style)

        rebar_controller가 내부 인덱스로 진행 관리
        """
        msg = WaypointArray()

        for wp in self.waypoints:
            msg.x.append(float(wp['x']))
            msg.y.append(float(wp['y']))
            msg.motion_type.append(int(wp['motion_type']))
            msg.max_speed.append(float(wp['max_speed']))

        self.waypoint_array_pub.publish(msg)

    def publish_target(self, waypoint_dict):
        """
        목표 위치 발행 (Enhanced Waypoint + 하위호환 PoseStamped)

        NOTE: 이 함수는 하위호환용으로 유지됨
        새로운 방식에서는 publish_waypoint_array()를 사용

        Args:
            waypoint_dict: {'x': float, 'y': float, 'motion_type': int, 'max_speed': float}
        """
        x = waypoint_dict['x']
        y = waypoint_dict['y']
        motion_type = waypoint_dict['motion_type']
        max_speed = waypoint_dict['max_speed']

        # 1. Enhanced Waypoint 발행 (for hybrid_controller)
        wp_msg = Waypoint()
        wp_msg.x = x
        wp_msg.y = y
        wp_msg.motion_type = motion_type
        wp_msg.max_speed = max_speed
        self.enhanced_target_pub.publish(wp_msg)

        # 2. 기존 PoseStamped 발행 (하위호환, for rebar_controller)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)

    def publish_feedback(self):
        """
        미션 피드백 발행

        형식: JSON String
        {
            "current_waypoint": 3,
            "total_waypoints": 10,
            "state": "navigating"
        }
        """
        feedback_data = {
            'current_waypoint': self.current_waypoint_idx + 1,  # 1-based
            'total_waypoints': len(self.waypoints),
            'state': self.sm.current_state.id
        }

        json_str = json.dumps(feedback_data)

        msg = String()
        msg.data = json_str
        self.feedback_pub.publish(msg)

    def waypoint_reached_callback(self, msg):
        """
        rebar_controller에서 목표 도달 알림 처리

        msg.data 형식:
        - "x,y" (기존 방식)
        - "INDEX:n" (새 방식 - 완료된 웨이포인트 인덱스)
        - "MISSION_COMPLETE" (새 방식 - 전체 미션 완료)
        """
        if self.sm.current_state != self.sm.navigating:
            # 미션 진행 중이 아니면 무시
            return

        data = msg.data

        # 새 방식: MISSION_COMPLETE
        if data == "MISSION_COMPLETE":
            self.get_logger().info("✅ 전체 미션 완료!")
            try:
                self.sm.complete()
            except Exception as e:
                self.get_logger().warn(f"미션 완료 전이 실패: {e}")
            return

        # 새 방식: INDEX:n
        if data.startswith("INDEX:"):
            try:
                idx = int(data.split(":")[1])
                self.current_waypoint_idx = idx + 1  # 다음 웨이포인트로
                self.get_logger().info(f"✅ 웨이포인트 {idx + 1}/{len(self.waypoints)} 도달")

                if self.current_waypoint_idx >= len(self.waypoints):
                    try:
                        self.sm.complete()
                    except Exception as e:
                        self.get_logger().warn(f"미션 완료 전이 실패: {e}")
                else:
                    self.publish_feedback()
                return
            except ValueError:
                pass

        # 기존 방식: x,y 좌표
        self.get_logger().info(f"✅ 웨이포인트 {self.current_waypoint_idx + 1} 도달: {data}")

        # 다음 웨이포인트로 이동
        self.current_waypoint_idx += 1

        if self.current_waypoint_idx >= len(self.waypoints):
            # 미션 완료
            try:
                self.sm.complete()
            except Exception as e:
                self.get_logger().warn(f"미션 완료 전이 실패: {e}")
        else:
            # 진행 상황 피드백 갱신
            self.publish_feedback()

    def publish_zero_cmd_vel(self):
        """cmd_vel을 0으로 발행하여 주행을 즉시 멈춤"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
