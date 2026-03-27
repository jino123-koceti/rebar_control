#!/usr/bin/env python3
"""
Drive Controller Node
리모콘(RemoteControl) 또는 cmd_vel → DriveControl 변환

Tire Roller 방식:
- Manual 모드: 리모콘 조이스틱 값 직접 변환
- Auto 모드: cmd_vel (Twist) 사용
- Differential drive kinematics 적용
"""

from typing import Optional, List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rebar_base_interfaces.msg import DriveControl, RemoteControl
from std_msgs.msg import String, Bool
import math
import time

from rebar_base_control.velocity_profiler import VelocityProfiler


class DriveController(Node):
    """리모콘과 cmd_vel을 DriveControl로 변환하는 통합 노드 (Tire Roller 방식)"""

    def __init__(self):
        super().__init__('drive_controller')

        # 파라미터 선언
        self.declare_parameter('wheel_base', 0.5)  # m, 바퀴 간 거리
        self.declare_parameter('max_linear_vel', 10.0)  # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s
        self.declare_parameter('speed_scale_factor', 1.0)  # 속도 스케일링 (1.0 = 100%)
        self.declare_parameter('joystick_deadzone', 20)  # 조이스틱 데드존
        self.declare_parameter('joystick_center', 127)  # 조이스틱 중립값
        self.declare_parameter('publish_frequency', 20)  # Hz
        # 횡이동(lateral) 제어는 joint_controller.py에서 처리 (0xA4 위치 제어)

        # 파라미터 가져오기
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.speed_scale = self.get_parameter('speed_scale_factor').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.joystick_center = self.get_parameter('joystick_center').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        # 상태 변수
        self.control_mode = 'idle'  # 'idle', 'manual', 'auto', 'emergency_stop', 'obstacle_pause'
        self.obstacle_paused = False
        self.pre_pause_mode = None  # PAUSE 전 모드 저장
        self.cmd_vel_msg = Twist()
        self.remote_control_msg = RemoteControl()
        self.last_drive_msg = DriveControl()

        # 속도 측정 변수 (AN3 전진 시 1m 달성 측정)
        self.speed_test_active = False
        self.speed_test_start_time = None
        self.speed_test_start_pose = None
        self.speed_test_target_distance = 1.0  # 목표 거리 (m)
        self.current_pose = None
        self.speed_samples = []  # 속도 샘플 저장

        # ROS2 Subscribers
        # Auto 모드용: cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Manual 모드용: 리모콘
        self.remote_control_sub = self.create_subscription(
            RemoteControl,
            '/remote_control',
            self.remote_control_callback,
            10
        )

        # 제어 모드 구독
        self.control_mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            10
        )

        # Emergency stop 구독 (즉시 정지용)
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # 장애물 PAUSE 구독
        self.obstacle_pause_sub = self.create_subscription(
            Bool,
            '/obstacle_pause',
            self.obstacle_pause_callback,
            10
        )

        # ZED 카메라 pose 구독 (속도 측정용)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        # 횡이동(lateral) 위치 피드백은 joint_controller.py에서 처리

        # ROS2 Publisher
        self.drive_control_pub = self.create_publisher(
            DriveControl,
            '/drive_control',
            10
        )

        # 주기적 발행 타이머 (Tire Roller 방식)
        self.timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_drive_control
        )

        self.get_logger().info("Drive Controller 노드 초기화 완료 (Tire Roller 방식)")
        self.get_logger().info(f"  - Wheel base: {self.wheel_base} m")
        self.get_logger().info(f"  - Max linear: {self.max_linear} m/s")
        self.get_logger().info(f"  - Max angular: {self.max_angular} rad/s")
        self.get_logger().info(f"  - Speed scale: {self.speed_scale * 100:.0f}%")
        self.get_logger().info(f"  - Publish frequency: {self.publish_frequency} Hz")

    def control_mode_callback(self, msg: String) -> None:
        """제어 모드 업데이트"""
        old_mode = self.control_mode
        self.control_mode = msg.data

        if old_mode != self.control_mode:
            self.get_logger().info(f"제어 모드 변경: {old_mode} → {self.control_mode}")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """cmd_vel 저장 (Auto 모드에서 사용)"""
        self.cmd_vel_msg = msg

    def emergency_stop_callback(self, msg: Bool) -> None:
        """E-STOP 시 즉시 정지"""
        if msg.data:
            self.obstacle_paused = False  # ESTOP이 우선
            self.control_mode = 'emergency_stop'
            stop_msg = DriveControl()
            stop_msg.left_speed = 0.0
            stop_msg.right_speed = 0.0
            stop_msg.lateral_speed = 0.0
            self.drive_control_pub.publish(stop_msg)
            self.last_drive_msg = stop_msg

    def obstacle_pause_callback(self, msg: Bool) -> None:
        """장애물 감지 → 주행 일시정지/재개"""
        if self.control_mode == 'emergency_stop':
            return  # ESTOP 중에는 무시

        if msg.data and not self.obstacle_paused:
            # PAUSE: 현재 모드 저장 후 정지
            self.obstacle_paused = True
            self.pre_pause_mode = self.control_mode
            self.control_mode = 'obstacle_pause'
            self.get_logger().warn('장애물 감지 → 주행 PAUSE')
            stop_msg = DriveControl()
            stop_msg.left_speed = 0.0
            stop_msg.right_speed = 0.0
            stop_msg.lateral_speed = 0.0
            self.drive_control_pub.publish(stop_msg)
            self.last_drive_msg = stop_msg

        elif not msg.data and self.obstacle_paused:
            # RESUME: 이전 모드 복귀
            self.obstacle_paused = False
            self.control_mode = self.pre_pause_mode or 'idle'
            self.pre_pause_mode = None
            self.get_logger().info('장애물 해제 → 주행 RESUME')

    def remote_control_callback(self, msg: RemoteControl) -> None:
        """리모콘 신호 저장 (Manual 모드에서 사용)"""
        self.remote_control_msg = msg
        # S17/S18 횡이동 제어는 joint_controller.py에서 처리 (0xA4 위치 제어)

    def pose_callback(self, msg: PoseStamped) -> None:
        """ZED 카메라 pose 수신 (속도 측정용)"""
        self.current_pose = msg

        # 속도 측정 중이면 거리 체크
        if self.speed_test_active and self.speed_test_start_pose is not None:
            self._check_speed_test_progress()

    def publish_drive_control(self) -> None:
        """
        타이머 콜백: 현재 모드에 따라 DriveControl 발행

        - manual: 리모콘 조이스틱 → DriveControl
        - auto: cmd_vel → DriveControl
        - 기타: 정지
        """
        drive_msg = DriveControl()
        drive_msg.lateral_speed = 0.0

        if self.control_mode == 'manual':
            # Manual 모드: 리모콘 조이스틱 변환
            drive_msg = self._convert_remote_to_drive()

            # 속도 측정: AN3 전진 시작/종료 감지
            self._handle_speed_test(drive_msg)

            # Manual 모드 조이스틱 로그 (1초 throttle, 움직일 때만)
            if abs(drive_msg.left_speed) > 0.01 or abs(drive_msg.right_speed) > 0.01:
                self.get_logger().info(
                    f"[Manual] left:{drive_msg.left_speed:.2f}m/s right:{drive_msg.right_speed:.2f}m/s "
                    f"(AN3:{self.remote_control_msg.joysticks[2]:.2f} AN4:{self.remote_control_msg.joysticks[3]:.2f})",
                    throttle_duration_sec=1.0
                )

        elif self.control_mode == 'auto' or self.control_mode == 'navigating':
            # Auto 모드: cmd_vel 변환
            drive_msg = self._convert_cmd_vel_to_drive()
            drive_msg.lateral_speed = 0.0
            # Auto 모드 로그 (1초 throttle, 움직일 때만)
            if abs(drive_msg.left_speed) > 0.01 or abs(drive_msg.right_speed) > 0.01:
                self.get_logger().info(
                    f"[Auto] left:{drive_msg.left_speed:.2f}m/s right:{drive_msg.right_speed:.2f}m/s "
                    f"(cmd_vel linear:{self.cmd_vel_msg.linear.x:.2f} angular:{self.cmd_vel_msg.angular.z:.2f})",
                    throttle_duration_sec=1.0
                )

        else:
            # idle, emergency_stop, obstacle_pause 등: 정지
            drive_msg.left_speed = 0.0
            drive_msg.right_speed = 0.0
            drive_msg.lateral_speed = 0.0

        # 횡이동(lateral)은 joint_controller.py에서 JointControl로 제어 (0xA4 위치 제어)

        # 발행
        self.drive_control_pub.publish(drive_msg)
        self.last_drive_msg = drive_msg

    def _convert_remote_to_drive(self) -> DriveControl:
        """
        리모콘 조이스틱 → DriveControl 변환

        조이스틱 매핑 (iron_md_teleop_node 참조):
        - joysticks[2] (AN3): 전후진 (중립=0.0, 전진=-1.0~0.0, 후진=0.0~1.0)
        - joysticks[3] (AN4): 좌우회전 (중립=0.0, CCW=0.0~1.0, CW=-1.0~0.0)
        """
        msg = self.remote_control_msg
        drive_msg = DriveControl()

        # 조이스틱 값 가져오기 (-1.0 ~ 1.0 범위)
        if len(msg.joysticks) >= 4:
            joy_linear = msg.joysticks[2]   # AN3: 전후진
            joy_angular = msg.joysticks[3]  # AN4: 좌우회전
        else:
            joy_linear = 0.0
            joy_angular = 0.0

        # 데드존 + soft ramp 적용
        # 데드존 내: 0, 데드존 밖: 0부터 점진적으로 증가 (경계에서 뚝 끊김 방지)
        deadzone_normalized = self.joystick_deadzone / 127.0
        joy_linear = self._apply_deadzone_ramp(joy_linear, deadzone_normalized)
        joy_angular = self._apply_deadzone_ramp(joy_angular, deadzone_normalized)

        # 조이스틱 값 → 선속도/각속도 변환
        # AN3+: 전진, AN3-: 후진 (방향 반전)
        linear_velocity = joy_linear * self.max_linear

        # AN4+: CCW (왼쪽), AN4-: CW (오른쪽)
        angular_velocity = joy_angular * self.max_angular

        # Differential drive kinematics
        left_speed = linear_velocity - (angular_velocity * self.wheel_base / 2.0)
        right_speed = linear_velocity + (angular_velocity * self.wheel_base / 2.0)

        # 속도 스케일링 적용 (테스트용 속도 제한)
        drive_msg.left_speed = left_speed * self.speed_scale
        drive_msg.right_speed = right_speed * self.speed_scale
        drive_msg.lateral_speed = 0.0

        return drive_msg

    def _convert_cmd_vel_to_drive(self) -> DriveControl:
        """
        cmd_vel (Twist) → DriveControl 변환

        Differential Drive Kinematics:
        - v_left = v_linear - (omega * wheel_base / 2)
        - v_right = v_linear + (omega * wheel_base / 2)
        """
        msg = self.cmd_vel_msg
        drive_msg = DriveControl()

        try:
            # 입력 제한 (전후면 반전: cmd_vel 부호 반전)
            linear = -max(-self.max_linear, min(self.max_linear, msg.linear.x))
            angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

            # Differential drive 변환
            left_speed = linear - (angular * self.wheel_base / 2.0)
            right_speed = linear + (angular * self.wheel_base / 2.0)

            # 속도 스케일링 적용 (테스트용 속도 제한)
            drive_msg.left_speed = left_speed * self.speed_scale
            drive_msg.right_speed = right_speed * self.speed_scale
            drive_msg.lateral_speed = 0.0

        except Exception as e:
            self.get_logger().error(f"cmd_vel 변환 오류: {e}")
            drive_msg.left_speed = 0.0
            drive_msg.right_speed = 0.0

        return drive_msg

    @staticmethod
    def _apply_deadzone_ramp(value: float, deadzone: float) -> float:
        """데드존 + soft ramp: 데드존 밖에서 0부터 점진적으로 증가.

        입력 범위 [-1, 1] → 출력 범위 [-1, 1]
        |value| < deadzone → 0
        |value| >= deadzone → 부호 유지, 0 ~ ±1로 리매핑
        """
        if abs(value) < deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        # deadzone~1.0 구간을 0~1.0으로 리매핑
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    # ========== 속도 측정 관련 메서드 ==========

    def _handle_speed_test(self, drive_msg: DriveControl) -> None:
        """AN3 전진 시 속도 측정 시작/종료 처리"""
        # 전진 여부 판단 (left/right 모두 양수이고 일정 속도 이상)
        is_forward = (drive_msg.left_speed > 0.05 and drive_msg.right_speed > 0.05)

        if is_forward and not self.speed_test_active:
            # 전진 시작 - 속도 측정 시작
            self._start_speed_test()
        elif not is_forward and self.speed_test_active:
            # 전진 종료 - 속도 측정 중단
            self._abort_speed_test()

        # 측정 중 속도 샘플 수집
        if self.speed_test_active:
            avg_speed = (drive_msg.left_speed + drive_msg.right_speed) / 2.0
            self.speed_samples.append(avg_speed)

    def _start_speed_test(self) -> None:
        """속도 측정 시작"""
        if self.current_pose is None:
            self.get_logger().warn("⚠️ 속도 측정 시작 불가 - /robot_pose 수신 안됨")
            return

        self.speed_test_active = True
        self.speed_test_start_time = time.time()
        self.speed_test_start_pose = self.current_pose
        self.speed_samples = []

        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y

        self.get_logger().info("=" * 60)
        self.get_logger().info("🏁 [속도 측정 시작] AN3 전진 감지")
        self.get_logger().info(f"   시작 위치: ({start_x:.3f}, {start_y:.3f}) m")
        self.get_logger().info(f"   목표 거리: {self.speed_test_target_distance:.1f} m")
        self.get_logger().info("=" * 60)

    def _check_speed_test_progress(self) -> None:
        """속도 측정 진행 상황 체크 (1m 달성 확인)"""
        if not self.speed_test_active or self.current_pose is None:
            return

        start_x = self.speed_test_start_pose.pose.position.x
        start_y = self.speed_test_start_pose.pose.position.y
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y

        # 이동 거리 계산
        distance = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)

        # 0.25m 단위로 진행 상황 출력
        if hasattr(self, '_last_reported_distance'):
            if distance - self._last_reported_distance >= 0.25:
                elapsed = time.time() - self.speed_test_start_time
                avg_speed = distance / elapsed if elapsed > 0 else 0
                self.get_logger().info(
                    f"   📍 진행: {distance:.2f}m / {self.speed_test_target_distance:.1f}m "
                    f"(경과: {elapsed:.1f}s, 평균속도: {avg_speed:.2f}m/s)"
                )
                self._last_reported_distance = distance
        else:
            self._last_reported_distance = 0.0

        # 목표 거리 달성
        if distance >= self.speed_test_target_distance:
            self._complete_speed_test(distance)

    def _complete_speed_test(self, distance: float):
        """속도 측정 완료 (1m 달성)"""
        elapsed = time.time() - self.speed_test_start_time
        avg_speed = distance / elapsed if elapsed > 0 else 0

        # 명령 속도 평균
        cmd_avg_speed = sum(self.speed_samples) / len(self.speed_samples) if self.speed_samples else 0

        start_x = self.speed_test_start_pose.pose.position.x
        start_y = self.speed_test_start_pose.pose.position.y
        end_x = self.current_pose.pose.position.x
        end_y = self.current_pose.pose.position.y

        self.get_logger().info("=" * 60)
        self.get_logger().info("🎉 [속도 측정 완료] 1m 달성!")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"   시작 위치: ({start_x:.3f}, {start_y:.3f}) m")
        self.get_logger().info(f"   종료 위치: ({end_x:.3f}, {end_y:.3f}) m")
        self.get_logger().info(f"   이동 거리: {distance:.3f} m")
        self.get_logger().info(f"   소요 시간: {elapsed:.2f} 초")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"   ⭐ 실제 평균 속도: {avg_speed:.3f} m/s ({avg_speed*100:.1f} cm/s)")
        self.get_logger().info(f"   📊 명령 평균 속도: {cmd_avg_speed:.3f} m/s")
        self.get_logger().info(f"   📈 속도 효율: {(avg_speed/cmd_avg_speed*100):.1f}%" if cmd_avg_speed > 0 else "")
        self.get_logger().info("=" * 60)

        # 측정 종료
        self.speed_test_active = False
        self.speed_test_start_time = None
        self.speed_test_start_pose = None
        self.speed_samples = []
        if hasattr(self, '_last_reported_distance'):
            del self._last_reported_distance

    def _abort_speed_test(self) -> None:
        """속도 측정 중단 (전진 정지)"""
        if self.speed_test_start_pose is not None and self.current_pose is not None:
            start_x = self.speed_test_start_pose.pose.position.x
            start_y = self.speed_test_start_pose.pose.position.y
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            distance = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)
            elapsed = time.time() - self.speed_test_start_time if self.speed_test_start_time else 0

            self.get_logger().info("=" * 60)
            self.get_logger().info("⏹️ [속도 측정 중단] 전진 정지됨")
            self.get_logger().info(f"   이동 거리: {distance:.3f} m (목표: {self.speed_test_target_distance:.1f} m)")
            self.get_logger().info(f"   소요 시간: {elapsed:.2f} 초")
            if elapsed > 0 and distance > 0:
                avg_speed = distance / elapsed
                self.get_logger().info(f"   평균 속도: {avg_speed:.3f} m/s")
            self.get_logger().info("=" * 60)

        self.speed_test_active = False
        self.speed_test_start_time = None
        self.speed_test_start_pose = None
        self.speed_samples = []
        if hasattr(self, '_last_reported_distance'):
            del self._last_reported_distance


def main(args=None):
    rclpy.init(args=args)
    node = DriveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
