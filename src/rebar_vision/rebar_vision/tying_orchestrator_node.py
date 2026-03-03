#!/usr/bin/env python3
"""
결속 오케스트레이터 노드 (Tying Orchestrator Node)

비전 검출 → 스테이지 이동 → 결속 시퀀스 (S21/S22) 자동 실행
6포인트(2x3) 순회 결속 자동화

흐름:
1. /control_mode가 "tying"으로 전환 시 시작
2. /rebar/detect_crossings 서비스로 교차점 검출
3. 각 포인트에 대해:
   a. 스테이지 XY 이동 (0x144, 0x145)
   b. S21 실행 (하강+그립)
   c. S22 실행 (트리거+상승)
4. 완료 시 /rebar_motion_cmd에 "TYING_COMPLETE" 발행
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rebar_base_interfaces.msg import JointControl, MotorFeedback, RebarGrid
from rebar_base_interfaces.srv import DetectCrossings
from std_msgs.msg import String
import time
from enum import Enum, auto


class TyingState(Enum):
    """오케스트레이터 상태"""
    IDLE = auto()
    DETECTING = auto()
    MOVING_TO_POINT = auto()
    WAITING_STAGE = auto()
    EXECUTING_S21 = auto()
    WAITING_S21 = auto()
    EXECUTING_S22 = auto()
    WAITING_S22 = auto()
    ADVANCING = auto()
    RETURNING_HOME = auto()
    COMPLETE = auto()
    ERROR = auto()


class TyingOrchestratorNode(Node):
    """비전 기반 6포인트 자동 결속 오케스트레이터"""

    def __init__(self):
        super().__init__('tying_orchestrator')

        # ============================================
        # 파라미터 선언
        # ============================================
        self.declare_parameter('deg_per_mm', 36.0)
        self.declare_parameter('stage_max_speed_dps', 200.0)
        self.declare_parameter('stage_settling_time', 1.5)
        self.declare_parameter('max_stage_timeout', 10.0)
        self.declare_parameter('position_tolerance_deg', 5.0)
        self.declare_parameter('max_stage_x_mm', 500.0)
        self.declare_parameter('max_stage_y_mm', 500.0)
        self.declare_parameter('inter_point_delay', 0.5)
        self.declare_parameter('detection_retry_count', 3)
        self.declare_parameter('detection_retry_delay', 1.0)

        # 파라미터 가져오기
        self.deg_per_mm = self.get_parameter('deg_per_mm').value
        self.stage_max_speed = self.get_parameter('stage_max_speed_dps').value
        self.stage_settling_time = self.get_parameter('stage_settling_time').value
        self.max_stage_timeout = self.get_parameter('max_stage_timeout').value
        self.position_tolerance = self.get_parameter('position_tolerance_deg').value
        self.max_stage_x_mm = self.get_parameter('max_stage_x_mm').value
        self.max_stage_y_mm = self.get_parameter('max_stage_y_mm').value
        self.inter_point_delay = self.get_parameter('inter_point_delay').value
        self.detection_retry_count = self.get_parameter('detection_retry_count').value
        self.detection_retry_delay = self.get_parameter('detection_retry_delay').value

        # ============================================
        # 상태 변수
        # ============================================
        self.state = TyingState.IDLE
        self.state_start_time = None
        self.control_mode = 'idle'
        self.prev_control_mode = 'idle'

        # 검출 결과
        self.grid = None
        self.current_point_index = 0

        # 스테이지 위치 추적 (누적 이동량, deg)
        self.stage_x_accumulated = 0.0
        self.stage_y_accumulated = 0.0

        # 스테이지 목표
        self.target_x_deg = 0.0
        self.target_y_deg = 0.0

        # 서비스 호출 상태
        self.detection_future = None
        self.detection_retry = 0

        # ============================================
        # Callback Group (서비스 호출과 타이머 분리)
        # ============================================
        self.cb_group = ReentrantCallbackGroup()

        # ============================================
        # 서비스 클라이언트
        # ============================================
        self.detect_client = self.create_client(
            DetectCrossings,
            '/rebar/detect_crossings',
            callback_group=self.cb_group
        )

        # ============================================
        # 구독자
        # ============================================
        self.control_mode_sub = self.create_subscription(
            String, '/control_mode',
            self._control_mode_callback, 10
        )

        self.sequence_status_sub = self.create_subscription(
            String, '/sequence_status',
            self._sequence_status_callback, 10
        )

        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, '/motor_feedback',
            self._motor_feedback_callback, 10
        )

        # ============================================
        # 발행자
        # ============================================
        self.joint_pub = self.create_publisher(
            JointControl, '/joint_control', 10
        )

        self.sequence_cmd_pub = self.create_publisher(
            String, '/sequence_cmd', 10
        )

        self.motion_cmd_pub = self.create_publisher(
            String, '/rebar_motion_cmd', 10
        )

        self.status_pub = self.create_publisher(
            String, '/tying/status', 10
        )

        # ============================================
        # 상태 머신 타이머 (20Hz)
        # ============================================
        self.timer = self.create_timer(0.05, self._state_machine_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Tying Orchestrator 노드 초기화 완료')
        self.get_logger().info(f'  스테이지: {self.deg_per_mm} deg/mm')
        self.get_logger().info(f'  스테이지 속도: {self.stage_max_speed} dps')
        self.get_logger().info(f'  안정화 시간: {self.stage_settling_time}s')
        self.get_logger().info(f'  X 최대: {self.max_stage_x_mm}mm, '
                               f'Y 최대: {self.max_stage_y_mm}mm')
        self.get_logger().info('=' * 60)

    # ============================================
    # 콜백
    # ============================================
    def _control_mode_callback(self, msg: String):
        """제어 모드 변경 감지 → tying 상태 시 자동 시작"""
        self.prev_control_mode = self.control_mode
        self.control_mode = msg.data

        # tying 상태로 전환 시 자동 시작
        if (self.control_mode == 'tying' and
                self.prev_control_mode != 'tying' and
                self.state == TyingState.IDLE):
            self.get_logger().info('=' * 60)
            self.get_logger().info('[AUTO] Tying 모드 진입 - 자동 결속 시작')
            self.get_logger().info('=' * 60)
            self._start_detection()

    def _sequence_status_callback(self, msg: String):
        """시퀀스 완료 신호 처리"""
        if msg.data == 'S21_COMPLETE' and self.state == TyingState.WAITING_S21:
            self.get_logger().info(
                f'  S21 완료 (포인트 {self.current_point_index + 1}/'
                f'{len(self.grid.detections)})'
            )
            self._transition_to(TyingState.EXECUTING_S22)

        elif msg.data == 'S22_COMPLETE' and self.state == TyingState.WAITING_S22:
            self.get_logger().info(
                f'  S22 완료 (포인트 {self.current_point_index + 1}/'
                f'{len(self.grid.detections)})'
            )
            self._transition_to(TyingState.ADVANCING)

    def _motor_feedback_callback(self, msg: MotorFeedback):
        """모터 피드백 (스테이지 위치 확인용)"""
        # 필요 시 0x144, 0x145 엔코더 피드백으로 위치 확인 가능
        pass

    # ============================================
    # 상태 머신
    # ============================================
    def _state_machine_loop(self):
        """20Hz 상태 머신 메인 루프"""
        elapsed = self._elapsed()

        if self.state == TyingState.IDLE:
            return

        elif self.state == TyingState.DETECTING:
            self._handle_detecting()

        elif self.state == TyingState.MOVING_TO_POINT:
            self._handle_moving_to_point()

        elif self.state == TyingState.WAITING_STAGE:
            if elapsed >= self.stage_settling_time:
                self.get_logger().info(
                    f'  스테이지 이동 완료 ({elapsed:.1f}s)'
                )
                self._transition_to(TyingState.EXECUTING_S21)
            elif elapsed >= self.max_stage_timeout:
                self.get_logger().warn('  스테이지 이동 타임아웃')
                self._transition_to(TyingState.EXECUTING_S21)

        elif self.state == TyingState.EXECUTING_S21:
            self._send_sequence_cmd('AUTO_S21')
            self._transition_to(TyingState.WAITING_S21)

        elif self.state == TyingState.WAITING_S21:
            # 30초 타임아웃
            if elapsed >= 30.0:
                self.get_logger().error('  S21 타임아웃 - 건너뜀')
                self._transition_to(TyingState.ADVANCING)

        elif self.state == TyingState.EXECUTING_S22:
            self._send_sequence_cmd('AUTO_S22')
            self._transition_to(TyingState.WAITING_S22)

        elif self.state == TyingState.WAITING_S22:
            # 30초 타임아웃
            if elapsed >= 30.0:
                self.get_logger().error('  S22 타임아웃 - 건너뜀')
                self._transition_to(TyingState.ADVANCING)

        elif self.state == TyingState.ADVANCING:
            self._handle_advancing()

        elif self.state == TyingState.RETURNING_HOME:
            if elapsed >= self.stage_settling_time:
                self._transition_to(TyingState.COMPLETE)

        elif self.state == TyingState.COMPLETE:
            self._handle_complete()

        elif self.state == TyingState.ERROR:
            if elapsed >= 3.0:
                self._transition_to(TyingState.IDLE)

    # ============================================
    # 상태 핸들러
    # ============================================
    def _start_detection(self):
        """검출 시작"""
        self.current_point_index = 0
        self.stage_x_accumulated = 0.0
        self.stage_y_accumulated = 0.0
        self.detection_retry = 0
        self._transition_to(TyingState.DETECTING)

    def _handle_detecting(self):
        """검출 서비스 호출 처리"""
        if self.detection_future is None:
            # 서비스 가용 확인
            if not self.detect_client.service_is_ready():
                if self._elapsed() >= 5.0:
                    self.get_logger().error('검출 서비스 미가용')
                    self._handle_error('검출 서비스에 연결할 수 없음')
                return

            # 서비스 호출
            request = DetectCrossings.Request()
            request.camera_selection = 0  # 양쪽 카메라
            request.confidence_threshold = 0.0  # 노드 기본값 사용
            request.expected_count = 6

            self.detection_future = self.detect_client.call_async(request)
            self.get_logger().info('  교차점 검출 서비스 호출...')
            return

        # 비동기 결과 확인
        if not self.detection_future.done():
            return

        try:
            result = self.detection_future.result()
            self.detection_future = None

            if result.success and result.grid.valid:
                self.grid = result.grid
                self.get_logger().info(
                    f'  검출 성공: {len(self.grid.detections)}개 교차점 '
                    f'({result.detection_time_ms:.1f}ms)'
                )
                for i, det in enumerate(self.grid.detections):
                    self.get_logger().info(
                        f'    [{i}] X={det.x:.1f}mm Y={det.y:.1f}mm '
                        f'conf={det.confidence:.2f}'
                    )
                self._transition_to(TyingState.MOVING_TO_POINT)
            else:
                self.detection_retry += 1
                msg = result.message if result else '응답 없음'
                self.get_logger().warn(
                    f'  검출 실패 ({self.detection_retry}/'
                    f'{self.detection_retry_count}): {msg}'
                )
                if self.detection_retry >= self.detection_retry_count:
                    self._handle_error(f'검출 {self.detection_retry}회 실패')
                else:
                    # 재시도 대기 후 다시 호출
                    time.sleep(self.detection_retry_delay)
                    self.detection_future = None

        except Exception as e:
            self.detection_future = None
            self.get_logger().error(f'  검출 서비스 오류: {e}')
            self._handle_error(f'검출 서비스 예외: {e}')

    def _handle_moving_to_point(self):
        """현재 포인트로 스테이지 이동"""
        if self.current_point_index >= len(self.grid.detections):
            self._transition_to(TyingState.RETURNING_HOME)
            return

        det = self.grid.detections[self.current_point_index]

        # 첫 포인트는 원점에서, 이후는 이전 포인트에서의 상대 이동
        if self.current_point_index == 0:
            dx_mm = det.x
            dy_mm = det.y
        else:
            prev = self.grid.detections[self.current_point_index - 1]
            dx_mm = det.x - prev.x
            dy_mm = det.y - prev.y

        # 안전 제한 확인
        new_x = self.stage_x_accumulated + dx_mm
        new_y = self.stage_y_accumulated + dy_mm

        if abs(new_x) > self.max_stage_x_mm or abs(new_y) > self.max_stage_y_mm:
            self.get_logger().warn(
                f'  스테이지 범위 초과: X={new_x:.1f}mm, Y={new_y:.1f}mm - 건너뜀'
            )
            self._transition_to(TyingState.ADVANCING)
            return

        # mm → degree 변환
        dx_deg = dx_mm * self.deg_per_mm
        dy_deg = dy_mm * self.deg_per_mm

        self.get_logger().info(
            f'  [{self.current_point_index + 1}/{len(self.grid.detections)}] '
            f'스테이지 이동: dX={dx_mm:.1f}mm({dx_deg:.0f}deg) '
            f'dY={dy_mm:.1f}mm({dy_deg:.0f}deg)'
        )

        # X축 이동 (0x144)
        if abs(dx_deg) > 0.1:
            x_msg = JointControl()
            x_msg.joint_id = 0x144
            x_msg.position = dx_deg
            x_msg.velocity = self.stage_max_speed
            x_msg.control_mode = JointControl.MODE_RELATIVE
            self.joint_pub.publish(x_msg)

        # Y축 이동 (0x145)
        if abs(dy_deg) > 0.1:
            y_msg = JointControl()
            y_msg.joint_id = 0x145
            y_msg.position = dy_deg
            y_msg.velocity = self.stage_max_speed
            y_msg.control_mode = JointControl.MODE_RELATIVE
            self.joint_pub.publish(y_msg)

        # 누적 위치 업데이트
        self.stage_x_accumulated = new_x
        self.stage_y_accumulated = new_y

        self._transition_to(TyingState.WAITING_STAGE)

    def _handle_advancing(self):
        """다음 포인트로 이동 또는 완료"""
        self.current_point_index += 1

        if self.current_point_index >= len(self.grid.detections):
            self.get_logger().info('  전체 결속 포인트 완료 - 스테이지 복귀')
            self._return_stage_home()
            self._transition_to(TyingState.RETURNING_HOME)
        else:
            # 포인트 간 대기
            time.sleep(self.inter_point_delay)
            self._transition_to(TyingState.MOVING_TO_POINT)

        self._publish_status()

    def _handle_complete(self):
        """결속 완료 처리"""
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'[AUTO] 자동 결속 완료! '
            f'{len(self.grid.detections)}개 포인트 결속'
        )
        self.get_logger().info('=' * 60)

        # navigator_base에 완료 신호
        msg = String()
        msg.data = 'TYING_COMPLETE'
        self.motion_cmd_pub.publish(msg)

        self._publish_status()
        self._transition_to(TyingState.IDLE)

    def _handle_error(self, error_msg):
        """에러 처리"""
        self.get_logger().error(f'[ERROR] {error_msg}')

        # 에러 시에도 완료 신호 (navigator가 멈추지 않도록)
        msg = String()
        msg.data = 'TYING_COMPLETE'
        self.motion_cmd_pub.publish(msg)

        self._transition_to(TyingState.ERROR)

    # ============================================
    # 유틸리티
    # ============================================
    def _return_stage_home(self):
        """스테이지를 원위치로 복귀"""
        if abs(self.stage_x_accumulated) > 0.1:
            x_msg = JointControl()
            x_msg.joint_id = 0x144
            x_msg.position = -self.stage_x_accumulated * self.deg_per_mm
            x_msg.velocity = self.stage_max_speed
            x_msg.control_mode = JointControl.MODE_RELATIVE
            self.joint_pub.publish(x_msg)

        if abs(self.stage_y_accumulated) > 0.1:
            y_msg = JointControl()
            y_msg.joint_id = 0x145
            y_msg.position = -self.stage_y_accumulated * self.deg_per_mm
            y_msg.velocity = self.stage_max_speed
            y_msg.control_mode = JointControl.MODE_RELATIVE
            self.joint_pub.publish(y_msg)

        self.get_logger().info(
            f'  스테이지 복귀: X={-self.stage_x_accumulated:.1f}mm, '
            f'Y={-self.stage_y_accumulated:.1f}mm'
        )

        self.stage_x_accumulated = 0.0
        self.stage_y_accumulated = 0.0

    def _send_sequence_cmd(self, command):
        """시퀀스 명령 발행"""
        msg = String()
        msg.data = command
        self.sequence_cmd_pub.publish(msg)
        self.get_logger().info(f'  시퀀스 명령: {command}')

    def _transition_to(self, new_state: TyingState):
        """상태 전환"""
        if self.state != new_state:
            self.get_logger().debug(
                f'  상태 전환: {self.state.name} -> {new_state.name}'
            )
        self.state = new_state
        self.state_start_time = time.monotonic()

    def _elapsed(self):
        """현재 상태 경과 시간"""
        if self.state_start_time is None:
            return 0.0
        return time.monotonic() - self.state_start_time

    def _publish_status(self):
        """상태 발행"""
        msg = String()
        if self.grid:
            msg.data = (
                f'{self.state.name}|'
                f'{self.current_point_index}/{len(self.grid.detections)}'
            )
        else:
            msg.data = self.state.name
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TyingOrchestratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
