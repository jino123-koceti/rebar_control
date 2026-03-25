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
from rebar_base_interfaces.msg import Waypoint, WaypointArray, JointControl
from statemachine import StateMachine, State
import json
import math
import sys
import os
import logging
from datetime import datetime


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

        # 파일 로거 설정
        self._setup_file_logger()

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
        self.mission_repeat = False  # repeat 모드 (핑퐁)

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

        # Mission command publisher (rebar_controller에 repeat 등 설정 전달)
        self.mission_command_pub = self.create_publisher(
            String,
            '/mission/command',
            10
        )

        # Homing 명령/상태 토픽
        self.homing_cmd_pub = self.create_publisher(
            String,
            '/homing_cmd',
            10
        )

        self.homing_status_sub = self.create_subscription(
            String,
            '/homing_status',
            self.homing_status_callback,
            10
        )

        # Homing 상태 추적
        self.homing_in_progress = False
        self.is_homed = False

        # 상부 결속부 단동 구동 (UPPER_BINDING_MOVE)
        self.joint_control_pub = self.create_publisher(
            JointControl,
            '/joint_control_cmd',
            10
        )

        # mm → degree 변환 계수 (can_devices.yaml 실측값)
        self.declare_parameter('stage_x_step_deg', 4.497)   # 1mm = 4.497° (0x144) - 실측 보정
        self.declare_parameter('stage_y_step_deg', 4.462)   # 1mm = 4.462° (0x145) - 실측 보정
        self.declare_parameter('stage_z_step_deg', 2.69)    # 1mm = 2.69°  (0x146)
        self.declare_parameter('stage_max_speed', 200.0)    # dps
        self.declare_parameter('yaw_max_speed', 134.0)      # dps

        self.stage_x_step = self.get_parameter('stage_x_step_deg').value
        self.stage_y_step = self.get_parameter('stage_y_step_deg').value
        self.stage_z_step = self.get_parameter('stage_z_step_deg').value
        self.stage_max_speed = self.get_parameter('stage_max_speed').value
        self.yaw_max_speed = self.get_parameter('yaw_max_speed').value

        self.stage_moving = False

        # 호밍 영점 기준 (homing reference angles, 0x92 deg)
        self.home_references = {'x': None, 'y': None, 'z': None, 'yaw': None}

        # 현재 제어 모드
        self.control_mode = "idle"

        # 작업영역 설정
        self.area_setup_active = False
        self.work_area = None  # {'start': {x,y,theta}, 'end': {x,y,theta}, 'corners': [...], 'width': float, 'height': float}
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # mm, mm, rad

        # 횡이동 오프셋 (encoder_odom에 반영 안 되는 Y 이동량)
        self.lateral_y_offset = 0.0  # mm
        self.mm_per_rotation = 44.0  # 1회전 = 44mm (실측: 9회전=396mm)
        self.lateral_complete_sub = self.create_subscription(
            String,
            '/lateral_motion_complete',
            self._lateral_complete_callback,
            10
        )

        # 로봇 위치 구독 - 엔코더 odometry 기반 (영역 설정/경로 추종 스케일 통일)
        self.encoder_odom_sub = self.create_subscription(
            PoseStamped,
            '/encoder_odom',
            self.encoder_odom_callback,
            10
        )

        # 작업영역 발행 (rebar_publisher → UI)
        self.work_area_pub = self.create_publisher(
            String,
            '/work_area',
            10
        )

        # 주기적 업데이트 타이머 (5Hz)
        self.timer = self.create_timer(0.2, self.update_mission)

        self.get_logger().info("Navigator 노드 초기화 완료")
        self.flog("=== Navigator 시작 ===")

    def _setup_file_logger(self):
        """파일 로거 설정 (/tmp/navigator_YYYYMMDD_HHMMSS.log)"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = f'/tmp/navigator_{timestamp}.log'
        self._file_logger = logging.getLogger('navigator_file')
        self._file_logger.setLevel(logging.DEBUG)
        self._file_logger.handlers.clear()
        fh = logging.FileHandler(self._log_path, mode='a', encoding='utf-8')
        fh.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d %(message)s', datefmt='%H:%M:%S'))
        self._file_logger.addHandler(fh)
        self._file_handler = fh
        self.get_logger().info(f"📝 파일 로그: {self._log_path}")

    def flog(self, msg):
        """파일 로거에 기록 + flush"""
        self._file_logger.info(msg)
        self._file_handler.flush()

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
        self.flog(f"CMD: {command[:200]}")

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
                        # repeat 모드 파싱 (ON/OFF)
                        self.mission_repeat = data.get("repeat", "OFF").upper() == "ON"
                        self.get_logger().info(f"START_MISSION: repeat={'ON' if self.mission_repeat else 'OFF'}")
                        self.flog(f"START_MISSION: repeat={'ON' if self.mission_repeat else 'OFF'}")

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

                    elif json_command == "GO_HOME":
                        self.start_homing()

                    elif json_command == "AREA_SETUP_START":
                        self.handle_area_setup_start(data)

                    elif json_command == "AREA_SETUP_COMPLETE":
                        self.handle_area_setup_complete(data)

                    elif json_command == "PATH_GEN":
                        self.handle_path_gen(data)

                    elif json_command == "UPPER_BINDING_MOVE":
                        self.handle_upper_biding_move(data)

                    elif json_command.startswith("CHN_POS="):
                        # tying_orchestrator가 /mission/command에서 직접 수신
                        self.get_logger().info(f"CHN_POS 명령: {json_command} (tying_orchestrator 처리)")

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
            # 하드웨어 호밍 (리밋 센서 기반 스테이지 원점복귀)
            self.start_homing()

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
                    # 기본값: differential=0.15m/s, lateral=200dps
                    if motion_type == Waypoint.MOTION_LATERAL:
                        max_speed = 200.0  # dps
                    else:
                        max_speed = 0.15  # m/s

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

        미션 시작 시 엔코더가 (0,0)으로 리셋되므로,
        현재 위치(미션 시작점)를 빼서 상대좌표로 변환하여 발행.
        WP[0]≈(0,0), 마지막 WP이 영역 시작점(도착점)이 됨.

        rebar_controller가 내부 인덱스로 진행 관리
        """
        msg = WaypointArray()

        # 현재 위치 기준 상대좌표 변환 (엔코더 리셋 후 (0,0) 기준과 일치시킴)
        origin_x = self.current_pose['x'] / 1000.0  # mm → m
        origin_y = self.current_pose['y'] / 1000.0

        self.flog(f"경로 발행: {len(self.waypoints)}개 WP (origin=({origin_x:.3f},{origin_y:.3f})m)")
        for i, wp in enumerate(self.waypoints):
            rel_x = float(wp['x']) - origin_x  # wp['x']는 이미 m 단위
            rel_y = float(wp['y']) - origin_y
            msg.x.append(rel_x)
            msg.y.append(rel_y)
            msg.motion_type.append(int(wp['motion_type']))
            msg.max_speed.append(float(wp['max_speed']))
            mt = "LAT" if wp['motion_type'] == Waypoint.MOTION_LATERAL else "DIFF"
            self.flog(f"  WP[{i}]: abs=({wp['x']:.3f},{wp['y']:.3f})m → rel=({rel_x:.3f},{rel_y:.3f})m @ {mt}")

        self.waypoint_array_pub.publish(msg)

        # repeat 모드 전달 (rebar_controller에 JSON으로)
        repeat_msg = String()
        repeat_msg.data = json.dumps({
            'command': 'SET_REPEAT',
            'repeat': self.mission_repeat
        })
        self.mission_command_pub.publish(repeat_msg)
        self.flog(f"repeat 모드 전달: {'ON' if self.mission_repeat else 'OFF'}")

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

    def start_homing(self):
        """하드웨어 호밍 시작 (joint_controller의 호밍 상태머신 트리거)"""
        if self.homing_in_progress:
            self.get_logger().warn("호밍이 이미 진행 중입니다")
            return

        self.get_logger().info("=" * 60)
        self.get_logger().info("🏠 GO_HOME: 하드웨어 호밍 시작")
        self.get_logger().info("=" * 60)

        self.homing_in_progress = True

        # joint_controller에 호밍 시작 명령 전송
        cmd_msg = String()
        cmd_msg.data = 'START'
        self.homing_cmd_pub.publish(cmd_msg)

        # 피드백 발행 (UI에 호밍 상태 알림)
        self._publish_homing_feedback('homing')

    def homing_status_callback(self, msg):
        """joint_controller로부터 호밍 상태 수신"""
        status = msg.data

        if not self.homing_in_progress:
            return

        self.get_logger().info(f"🏠 호밍 상태: {status}")

        if status.startswith('COMPLETE'):
            self.homing_in_progress = False
            self.is_homed = True

            # Reference angles 파싱: "COMPLETE:{"x":123.4,"y":456.7,...}"
            if ':' in status:
                try:
                    ref_json = status.split(':', 1)[1]
                    ref_data = json.loads(ref_json)
                    self.home_references = {
                        'x': ref_data.get('x', 0.0),
                        'y': ref_data.get('y', 0.0),
                        'z': ref_data.get('z', 0.0),
                        'yaw': ref_data.get('yaw', 0.0),
                    }
                    self.get_logger().info(f"🏠 영점 기준 설정: {self.home_references}")
                except Exception as e:
                    self.get_logger().warn(f"Reference 파싱 실패: {e}")

            self.get_logger().info("=" * 60)
            self.get_logger().info("🏠 호밍 완료!")
            self.get_logger().info("=" * 60)
            self._publish_homing_feedback('homing_complete')

        elif 'FAIL' in status or 'STOPPED' in status:
            self.homing_in_progress = False
            self.get_logger().error(f"🏠 호밍 실패: {status}")
            self._publish_homing_feedback('homing_failed')

    def _publish_homing_feedback(self, state):
        """호밍 피드백 발행 (UI 표시용)"""
        feedback_data = {
            'current_waypoint': 0,
            'total_waypoints': 0,
            'state': state
        }
        msg = String()
        msg.data = json.dumps(feedback_data)
        self.feedback_pub.publish(msg)

    def handle_upper_biding_move(self, data):
        """
        상부 결속부 단동 구동 처리 (홈 영점 기준 절대 위치)

        UI 명령 형식:
        {"command":"UPPER_BINDING_MOVE","target":{"x":100.0,"y":50.0,"z":20.0,"yaw":90.0}}

        - x, y, z: mm 단위 (홈 영점 기준 절대 위치, 0 = 홈 위치)
        - yaw: degree 단위 (홈 영점 기준)
        - 실제로는 한 축씩만 데이터가 들어옴
        """
        if not self.is_homed:
            self.get_logger().warn("호밍 미완료 - 먼저 GO_HOME을 실행하세요")
            self._publish_stage_feedback('stage_move_failed', 'homing_required')
            return

        if self.stage_moving:
            self.get_logger().warn("스테이지 이동 중 - 이전 명령 완료 대기")
            return

        target = data.get('target', {})
        if not target:
            self.get_logger().warn("UPPER_BINDING_MOVE: target 데이터 없음")
            return

        # 축별 변환 테이블: (axis_key, motor_id, mm→deg 변환계수, 최대속도, 단위)
        axis_config = {
            'x':   (0x144, self.stage_x_step, self.stage_max_speed, 'mm'),
            'y':   (0x145, self.stage_y_step, self.stage_max_speed, 'mm'),
            'z':   (0x146, self.stage_z_step, self.stage_max_speed, 'mm'),
            'yaw': (0x147, 1.0,               self.yaw_max_speed,   'deg'),
        }

        moved = False
        for axis_name, (motor_id, step_deg, max_speed, unit) in axis_config.items():
            value = target.get(axis_name)
            if value is None:
                continue

            # 영점 기준(reference angle) 확인
            ref_angle = self.home_references.get(axis_name)
            if ref_angle is None:
                self.get_logger().warn(f"[UPPER_BINDING] {axis_name} 영점 미설정")
                continue

            # mm → degree 변환 후 영점 기준 절대 위치 계산
            offset_deg = float(value) * step_deg
            absolute_deg = ref_angle + offset_deg

            # JointControl 발행 (절대 위치)
            cmd = JointControl()
            cmd.joint_id = motor_id
            cmd.position = absolute_deg
            cmd.velocity = max_speed
            cmd.control_mode = JointControl.MODE_ABSOLUTE
            self.joint_control_pub.publish(cmd)

            self.get_logger().info(
                f"[UPPER_BINDING] {axis_name.upper()}: {value}{unit} "
                f"(ref={ref_angle:.1f} + {offset_deg:.1f} = {absolute_deg:.1f}deg) "
                f"-> 0x{motor_id:03X} @ {max_speed:.0f}dps"
            )
            moved = True

        if moved:
            self.stage_moving = True
            self._publish_stage_feedback('stage_moving')
            # 이동 완료 체크 타이머 (이동 시간 추정 후 완료 처리)
            self.create_timer(0.5, self._check_stage_move_complete_once)

    def _check_stage_move_complete_once(self):
        """스테이지 이동 완료 체크 (1회성 타이머)"""
        # 단동 구동은 명령 전송 후 모터가 알아서 이동 완료
        # 간단한 타이머 기반 완료 처리 (추후 모터 피드백 기반으로 개선 가능)
        self.stage_moving = False
        self._publish_stage_feedback('stage_move_complete')
        self.get_logger().info("[UPPER_BINDING] 이동 완료")

    def _publish_stage_feedback(self, state, detail=''):
        """스테이지 이동 피드백 발행"""
        feedback_data = {
            'current_waypoint': 0,
            'total_waypoints': 0,
            'state': state,
        }
        if detail:
            feedback_data['detail'] = detail
        msg = String()
        msg.data = json.dumps(feedback_data)
        self.feedback_pub.publish(msg)

    def _lateral_complete_callback(self, msg: String):
        """횡이동 완료 시 Y 오프셋 누적"""
        try:
            parts = msg.data.split(':')
            if parts[0] not in ('MANUAL', 'AUTO') or len(parts) < 3:
                return
            direction = parts[1]
            turns = float(parts[2])
            dy_mm = turns * self.mm_per_rotation
            if direction == '+':
                self.lateral_y_offset += dy_mm
            else:
                self.lateral_y_offset -= dy_mm
            self.get_logger().info(
                f"횡이동 오프셋: {direction}{dy_mm:.0f}mm, 누적={self.lateral_y_offset:.0f}mm"
            )
        except Exception:
            pass

    def encoder_odom_callback(self, msg):
        """로봇 위치 업데이트 (엔코더 odometry - 영역설정/경로추종 스케일 통일)

        전후면 반전 보정: encoder_odom은 물리적 방향 그대로이므로
        제어용으로 부호 반전하여 cmd_vel 방향과 일치시킴
        """
        self.current_pose['x'] = -msg.pose.position.x * 1000.0  # m → mm, 부호 반전
        self.current_pose['y'] = -msg.pose.position.y * 1000.0 + self.lateral_y_offset  # m → mm, 부호 반전 + 횡이동 오프셋
        siny = 2.0 * (msg.pose.orientation.w * msg.pose.orientation.z +
                       msg.pose.orientation.x * msg.pose.orientation.y)
        cosy = 1.0 - 2.0 * (msg.pose.orientation.y ** 2 + msg.pose.orientation.z ** 2)
        raw_yaw = math.atan2(siny, cosy)
        # 전후면 반전: yaw도 π 회전
        inv_yaw = raw_yaw + math.pi
        self.current_pose['theta'] = math.atan2(math.sin(inv_yaw), math.cos(inv_yaw))

        # 작업영역 설정 중이면 로컬 좌표 기준 최대·최소값 추적
        if self.area_setup_active and hasattr(self, '_area_bounds'):
            # odom → 시작점 헤딩 기준 로컬 좌표 변환
            dx = self.current_pose['x'] - self._area_origin['x']
            dy = self.current_pose['y'] - self._area_origin['y']
            local_x = dx * self._area_cos - dy * self._area_sin
            local_y = dx * self._area_sin + dy * self._area_cos
            self._area_bounds['min_x'] = min(self._area_bounds['min_x'], local_x)
            self._area_bounds['max_x'] = max(self._area_bounds['max_x'], local_x)
            self._area_bounds['min_y'] = min(self._area_bounds['min_y'], local_y)
            self._area_bounds['max_y'] = max(self._area_bounds['max_y'], local_y)

            # 주행 중 pose 로그 (1초 간격)
            now = self.get_clock().now()
            if not hasattr(self, '_area_last_log_time') or (now - self._area_last_log_time).nanoseconds > 1e9:
                self._area_last_log_time = now
                heading_deg = math.degrees(self.current_pose['theta'])
                self.get_logger().info(
                    f'  [AREA] enc=({self.current_pose["x"]:.0f},{self.current_pose["y"]:.0f}) '
                    f'heading={heading_deg:.1f}° local=({local_x:.0f},{local_y:.0f})'
                )

    def handle_area_setup_start(self, data=None):
        """
        작업영역 설정 시작

        현재 로봇 위치를 시작 꼭지점(P1)으로 기록하고,
        사용자가 리모컨으로 대각 꼭지점까지 주행하도록 대기.

        UI 명령 형식:
        {"command": "AREA_SETUP_START"}
        {"command": "AREA_SETUP_START", "area_type": "line"}
        {"command": "AREA_SETUP_START", "area_type": "rectangle"}
        """
        if data is None:
            data = {}
        start = {
            'x': self.current_pose['x'],
            'y': self.current_pose['y'],
            'theta': self.current_pose['theta'],
        }

        self.area_setup_active = True
        self.work_area = None  # 이전 작업영역 초기화
        self._area_type_hint = data.get('area_type', None)  # UI 지정 타입

        # 시작 헤딩 기준 로컬 좌표계 (헤딩 방향 = 로컬 X, 수직 = 로컬 Y)
        self._area_origin = {'x': start['x'], 'y': start['y'], 'theta': start['theta']}
        self._area_cos = math.cos(-start['theta'])
        self._area_sin = math.sin(-start['theta'])

        # 로컬 좌표 기준 경계 추적
        self._area_bounds = {
            'min_x': 0.0, 'max_x': 0.0,
            'min_y': 0.0, 'max_y': 0.0,
        }

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"작업영역 설정 시작 - 시작점: ({start['x']:.1f}, {start['y']:.1f}) mm")
        self.get_logger().info("작업영역 경계를 따라 주행 후 AREA_SETUP_COMPLETE 전송")
        self.get_logger().info("=" * 60)

        # 시작점 임시 저장
        self._area_start = start

        # UI에 상태 피드백
        self._publish_area_feedback('area_setup_active', {
            'start': start,
        })

    def handle_area_setup_complete(self, data=None):
        """
        작업영역 설정 완료

        현재 위치를 종료 꼭지점(P2)으로 기록하고,
        시작점(P1)과 종료점(P2)로 직사각형 작업영역 생성.

        P1(start) ---- P4
        |               |
        |               |
        P3 ---- P2(end)

        좌표계: 로봇 오도메트리 기준 (mm)
        """
        if not self.area_setup_active or not hasattr(self, '_area_start'):
            self.get_logger().warn("작업영역 설정이 시작되지 않았습니다. AREA_SETUP_START를 먼저 전송하세요.")
            self._publish_area_feedback('area_setup_failed', {
                'reason': 'not_started',
            })
            return

        end = {
            'x': self.current_pose['x'],
            'y': self.current_pose['y'],
            'theta': self.current_pose['theta'],
        }

        start = self._area_start
        self.area_setup_active = False

        # 시작 헤딩 기준 로컬 좌표에서 추적된 경계값
        bounds = self._area_bounds
        min_x = bounds['min_x']
        max_x = bounds['max_x']
        min_y = bounds['min_y']
        max_y = bounds['max_y']

        width = max_x - min_x    # mm
        height = max_y - min_y   # mm

        # 영역 타입: UI 지정(area_type) 우선, 미지정 시 자동 판별
        # COMPLETE 또는 START 어디서든 area_type 수신 가능
        if data is None:
            data = {}
        complete_hint = data.get('area_type', None)
        if complete_hint in ('line', 'rectangle'):
            area_type = complete_hint
        elif hasattr(self, '_area_type_hint') and self._area_type_hint in ('line', 'rectangle'):
            area_type = self._area_type_hint
        else:
            area_type = 'line' if height < 100.0 else 'rectangle'

        if area_type == 'line':
            center_y = (min_y + max_y) / 2.0
            corners = [
                {'x': min_x, 'y': center_y},  # P1 (시작)
                {'x': max_x, 'y': center_y},  # P2 (끝)
            ]
        else:
            corners = [
                {'x': min_x, 'y': max_y},  # P1 (좌상)
                {'x': max_x, 'y': max_y},  # P2 (우상)
                {'x': max_x, 'y': min_y},  # P3 (우하)
                {'x': min_x, 'y': min_y},  # P4 (좌하)
            ]

        self.work_area = {
            'type': area_type,
            'start': start,
            'end': end,
            'corners': corners,
            'width': width,
            'height': height,
            'area': width * height,  # mm²
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y,
            'origin': self._area_origin,  # odom 원점 + 헤딩 (로컬→odom 변환용)
        }

        heading_deg = math.degrees(self._area_origin['theta'])
        self.get_logger().info("=" * 60)
        if area_type == 'line':
            self.get_logger().info(f"작업영역 설정 완료! [직선]")
            self.get_logger().info(f"  시작점(odom): ({start['x']:.1f}, {start['y']:.1f}) mm, 헤딩: {heading_deg:.1f}°")
            self.get_logger().info(f"  종료점(odom): ({end['x']:.1f}, {end['y']:.1f}) mm")
            self.get_logger().info(f"  길이(W): {width:.1f} mm, 횡편차(H): {height:.1f} mm (로컬좌표)")
        else:
            self.get_logger().info(f"작업영역 설정 완료! [직사각형]")
            self.get_logger().info(f"  시작점(odom): ({start['x']:.1f}, {start['y']:.1f}) mm, 헤딩: {heading_deg:.1f}°")
            self.get_logger().info(f"  종료점(odom): ({end['x']:.1f}, {end['y']:.1f}) mm")
            self.get_logger().info(f"  로컬 Bounds: X[{min_x:.1f} ~ {max_x:.1f}], Y[{min_y:.1f} ~ {max_y:.1f}]")
            self.get_logger().info(f"  가로(W): {width:.1f} mm, 세로(H): {height:.1f} mm")
            self.get_logger().info(f"  면적: {width * height / 1e6:.2f} m²")
        self.get_logger().info("=" * 60)

        # UI용 work_area: corners를 로컬→odom 좌표로 변환
        origin = self._area_origin
        cos_t = math.cos(origin['theta'])
        sin_t = math.sin(origin['theta'])
        ui_corners = []
        for c in corners:
            odom_x = origin['x'] + c['x'] * cos_t - c['y'] * sin_t
            odom_y = origin['y'] + c['x'] * sin_t + c['y'] * cos_t
            ui_corners.append({'x': odom_x, 'y': odom_y})

        ui_work_area = dict(self.work_area)
        ui_work_area['corners'] = ui_corners

        # 작업영역 토픽 발행 (rebar_publisher가 UI에 전달)
        area_msg = String()
        area_msg.data = json.dumps(ui_work_area)
        self.work_area_pub.publish(area_msg)

        # UI에 완료 피드백
        self._publish_area_feedback('area_setup_complete', {
            'work_area': ui_work_area,
        })

    def handle_path_gen(self, data):
        """
        RL 모델 기반 경로 생성 (PATH_GEN 커맨드)

        UI 명령 형식:
        {"command": "PATH_GEN"}
        {"command": "PATH_GEN", "obstacles": [{"x": 1500, "y": 800}]}

        work_area가 설정된 상태에서만 동작.
        경로 생성 후 자동으로 planning 상태로 전환.
        """
        if self.work_area is None:
            self.get_logger().warn("작업영역이 설정되지 않았습니다. AREA_SETUP을 먼저 수행하세요.")
            self._publish_area_feedback('path_gen_failed', {
                'reason': 'no_work_area',
            })
            return

        obstacles_mm = data.get('obstacles', None)

        self.get_logger().info("=" * 60)
        self.get_logger().info("경로 생성 시작 (PATH_GEN)")
        self.get_logger().info(f"  작업영역: {self.work_area['width']:.0f} x {self.work_area['height']:.0f} mm")
        if obstacles_mm:
            self.get_logger().info(f"  장애물: {len(obstacles_mm)}개")

        # UI에 경로 생성 시작 피드백 (추론 중 표시용)
        self._publish_area_feedback('path_gen_started', {
            'work_area': self.work_area,
            'obstacle_count': len(obstacles_mm) if obstacles_mm else 0,
        })

        try:
            # RLPathGenerator import (lazy load)
            rl_path = os.path.join(
                os.path.expanduser('~'), 'ros2_ws',
                'path_generation', 'reinforcement_learning_sim'
            )
            if rl_path not in sys.path:
                sys.path.insert(0, rl_path)

            from path_generator import RLPathGenerator

            model_path = os.path.join(rl_path, 'models', 'cnn_ppo_agent.pth')
            generator = RLPathGenerator(model_path)

            waypoints = generator.generate(self.work_area, obstacles_mm)

            if not waypoints:
                self.get_logger().error("경로 생성 실패: 웨이포인트 없음")
                self._publish_area_feedback('path_gen_failed', {
                    'reason': 'empty_path',
                })
                return

            # 로컬 좌표 → odom 좌표 변환
            origin = self.work_area.get('origin')
            if origin:
                cos_t = math.cos(origin['theta'])
                sin_t = math.sin(origin['theta'])
                for wp in waypoints:
                    lx, ly = wp['x'], wp['y']
                    wp['x'] = origin['x'] + lx * cos_t - ly * sin_t
                    wp['y'] = origin['y'] + lx * sin_t + ly * cos_t

            # 경로 시작점을 현재 위치(설정 완료 지점)에 가까운 쪽으로 정렬
            if len(waypoints) >= 2:
                cx, cy = self.current_pose['x'], self.current_pose['y']
                d_first = (waypoints[0]['x'] - cx)**2 + (waypoints[0]['y'] - cy)**2
                d_last = (waypoints[-1]['x'] - cx)**2 + (waypoints[-1]['y'] - cy)**2
                if d_last < d_first:
                    waypoints.reverse()
                    self.get_logger().info("  경로 역순 정렬 (현재 위치 → 시작점 방향)")

            # 웨이포인트를 기존 load_waypoints_from_json 형식으로 변환
            wp_json = json.dumps({'waypoints': waypoints})
            self.load_waypoints_from_json(wp_json)

            self.get_logger().info(f"  생성된 웨이포인트: {len(waypoints)}개")
            self.get_logger().info("  START_MISSION 명령으로 주행을 시작하세요")
            self.get_logger().info("=" * 60)

            # UI에 피드백 (로봇 자세/작업영역과 동일한 좌표계로 전송)
            self._publish_area_feedback('path_gen_complete', {
                'waypoint_count': len(waypoints),
                'waypoints': waypoints,
                'work_area': self.work_area,
            })

        except Exception as e:
            self.get_logger().error(f"경로 생성 오류: {e}")
            self._publish_area_feedback('path_gen_failed', {
                'reason': str(e),
            })

    def _publish_area_feedback(self, state, data=None):
        """작업영역 설정 피드백 발행"""
        feedback_data = {
            'current_waypoint': 0,
            'total_waypoints': 0,
            'state': state,
        }
        if data:
            feedback_data.update(data)
        msg = String()
        msg.data = json.dumps(feedback_data)
        self.feedback_pub.publish(msg)

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
