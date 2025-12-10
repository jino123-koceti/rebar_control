#!/usr/bin/env python3
"""
Precision Navigation Controller Node
정밀 주행 제어 노드 - 직진 Heading 제어 + Visual Odometry 거리 제어

Author: Koceti Robotics Team
Date: 2025-12-10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rebar_msgs.msg import PrecisionNavGoal, PrecisionNavFeedback, PrecisionNavStatus
import math
import time
from enum import Enum


class State(Enum):
    """상태 머신"""
    IDLE = 0
    VALIDATE = 1
    MOVING = 2
    REACHED = 3
    ERROR = 4


class PIDController:
    """간단한 PID 제어기"""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        """PID 상태 리셋"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, current_time=None):
        """
        PID 계산
        Args:
            error: 오차 (목표 - 현재)
            current_time: 현재 시간 (초)
        Returns:
            제어 출력
        """
        if current_time is None:
            current_time = time.time()

        # 첫 호출
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return self.kp * error

        # 시간 간격
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 0.001  # 최소값

        # PID 계산
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (self.kp * error +
                  self.ki * self.integral +
                  self.kd * derivative)

        # Anti-windup
        self.integral = max(-self.output_limit, min(self.output_limit, self.integral))

        # 출력 제한
        output = max(-self.output_limit, min(self.output_limit, output))

        # 상태 업데이트
        self.prev_error = error
        self.prev_time = current_time

        return output


class PrecisionNavigationNode(Node):
    """정밀 주행 제어 노드 (직진만)"""

    def __init__(self):
        super().__init__('precision_navigation_node')

        # 파라미터 선언
        self.declare_parameter('heading_kp', 0.5)
        self.declare_parameter('heading_ki', 0.0)
        self.declare_parameter('heading_kd', 0.1)
        self.declare_parameter('distance_kp', 0.003)  # 거리 제어 (mm 단위이므로 작은 값)
        self.declare_parameter('distance_ki', 0.0)
        self.declare_parameter('distance_kd', 0.0001)
        self.declare_parameter('distance_threshold_mm', 10.0)
        self.declare_parameter('yaw_threshold_deg', 3.0)
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('min_linear_velocity', 0.1)
        self.declare_parameter('slowdown_distance_mm', 100.0)

        # 파라미터 가져오기
        heading_kp = self.get_parameter('heading_kp').value
        heading_ki = self.get_parameter('heading_ki').value
        heading_kd = self.get_parameter('heading_kd').value
        distance_kp = self.get_parameter('distance_kp').value
        distance_ki = self.get_parameter('distance_ki').value
        distance_kd = self.get_parameter('distance_kd').value
        self.distance_threshold = self.get_parameter('distance_threshold_mm').value
        self.yaw_threshold = self.get_parameter('yaw_threshold_deg').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.min_linear_vel = self.get_parameter('min_linear_velocity').value
        self.slowdown_distance = self.get_parameter('slowdown_distance_mm').value

        # PID 제어기
        self.heading_pid = PIDController(heading_kp, heading_ki, heading_kd, output_limit=0.5)
        self.distance_pid = PIDController(distance_kp, distance_ki, distance_kd, output_limit=self.max_linear_vel)

        # 구독자
        self.imu_sub = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/zed/zed_node/odom', self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PrecisionNavGoal, '/precision_nav/goal', self.goal_callback, 10
        )

        # 발행자
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_precise', 10)
        self.feedback_pub = self.create_publisher(PrecisionNavFeedback, '/precision_nav/feedback', 10)
        self.status_pub = self.create_publisher(PrecisionNavStatus, '/precision_nav/status', 10)

        # 상태 변수
        self.state = State.IDLE
        self.current_imu = None
        self.current_odom = None
        self.start_odom = None
        self.goal = None
        self.target_yaw = None  # 목표 Yaw 각도 (라디안)
        self.start_time = None

        # 제어 루프 타이머 (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('Precision Navigation Node initialized')
        self.get_logger().info(f'  Heading PID: Kp={heading_kp}, Ki={heading_ki}, Kd={heading_kd}')
        self.get_logger().info(f'  Distance PID: Kp={distance_kp}, Ki={distance_ki}, Kd={distance_kd}')
        self.get_logger().info(f'  Thresholds: Distance={self.distance_threshold}mm, Yaw={self.yaw_threshold}deg')

    def imu_callback(self, msg):
        """IMU 데이터 수신"""
        self.current_imu = msg

    def odom_callback(self, msg):
        """Odometry 데이터 수신"""
        self.current_odom = msg

    def goal_callback(self, msg):
        """목표 명령 수신"""
        if self.state != State.IDLE:
            self.get_logger().warn(f'Goal rejected: Already in state {self.state.name}')
            return

        self.get_logger().info(f'Received goal: distance={msg.distance}mm, max_vel={msg.max_velocity}m/s')

        # 목표 저장
        self.goal = msg

        # 상태 전환
        self.state = State.VALIDATE

    def control_loop(self):
        """제어 루프 (50Hz)"""
        # 상태 발행
        self.publish_status()

        # 상태 머신
        if self.state == State.IDLE:
            self.handle_idle()
        elif self.state == State.VALIDATE:
            self.handle_validate()
        elif self.state == State.MOVING:
            self.handle_moving()
        elif self.state == State.REACHED:
            self.handle_reached()
        elif self.state == State.ERROR:
            self.handle_error()

    def handle_idle(self):
        """IDLE 상태 처리"""
        # 정지 명령 발행
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def handle_validate(self):
        """VALIDATE 상태 처리 - 목표 유효성 검증"""
        # 센서 데이터 확인
        if self.current_imu is None or self.current_odom is None:
            self.get_logger().error('Sensors not ready (IMU or Odom missing)')
            self.state = State.ERROR
            return

        # 목표 거리 확인
        if abs(self.goal.distance) < 1.0:
            self.get_logger().warn(f'Goal distance too small: {self.goal.distance}mm')
            self.state = State.IDLE
            return

        if abs(self.goal.distance) > 10000.0:  # 10m 제한
            self.get_logger().warn(f'Goal distance too large: {self.goal.distance}mm')
            self.state = State.IDLE
            return

        # 시작 위치 저장
        self.start_odom = self.current_odom

        # 목표 Yaw 각도 설정 (현재 Yaw 유지)
        _, _, current_yaw = self.get_yaw_from_imu(self.current_imu)
        self.target_yaw = current_yaw

        # PID 리셋
        self.heading_pid.reset()
        self.distance_pid.reset()

        # 시작 시간 기록
        self.start_time = time.time()

        # 상태 전환
        self.state = State.MOVING
        self.get_logger().info(f'Start moving: target_distance={self.goal.distance}mm, target_yaw={math.degrees(self.target_yaw):.2f}deg')

    def handle_moving(self):
        """MOVING 상태 처리 - PID 제어 실행"""
        # 센서 데이터 확인
        if self.current_imu is None or self.current_odom is None:
            self.get_logger().error('Sensor data lost during movement')
            self.state = State.ERROR
            return

        # 현재 이동 거리 계산 (mm)
        current_distance_mm = self.calculate_distance_traveled()

        # 거리 오차 계산
        distance_error_mm = self.goal.distance - current_distance_mm

        # Yaw 오차 계산
        _, _, current_yaw = self.get_yaw_from_imu(self.current_imu)
        yaw_error_deg = math.degrees(self.normalize_angle(self.target_yaw - current_yaw))

        # 목표 도달 판정
        if abs(distance_error_mm) < self.distance_threshold and abs(yaw_error_deg) < self.yaw_threshold:
            self.get_logger().info(f'Goal reached! distance_error={distance_error_mm:.1f}mm, yaw_error={yaw_error_deg:.2f}deg')
            self.state = State.REACHED
            return

        # 타임아웃 체크 (30초)
        if time.time() - self.start_time > 30.0:
            self.get_logger().error('Movement timeout (30sec)')
            self.state = State.ERROR
            return

        # PID 제어 계산
        current_time = time.time()

        # Heading 제어 (angular.z)
        angular_z = self.heading_pid.compute(math.radians(yaw_error_deg), current_time)

        # Distance 제어 (linear.x)
        linear_x = self.distance_pid.compute(distance_error_mm, current_time)

        # 속도 제한 및 감속
        remaining_distance = abs(distance_error_mm)
        if remaining_distance < self.slowdown_distance:
            # 감속 구간
            speed_factor = remaining_distance / self.slowdown_distance
            speed_factor = max(0.3, speed_factor)  # 최소 30%
            linear_x *= speed_factor

        # 방향 결정 (전진/후진)
        if self.goal.distance < 0:
            linear_x = -abs(linear_x)
        else:
            linear_x = abs(linear_x)

        # 최대/최소 속도 제한
        linear_x = max(self.min_linear_vel, min(self.max_linear_vel, abs(linear_x)))
        if self.goal.distance < 0:
            linear_x = -linear_x

        # cmd_vel 발행
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)

        # 피드백 발행
        self.publish_feedback(current_distance_mm, yaw_error_deg, linear_x, angular_z)

        # 디버그 로그 (1Hz)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0

        if self._debug_counter % 50 == 0:  # 50Hz → 1Hz
            progress = (current_distance_mm / self.goal.distance) * 100.0 if self.goal.distance != 0 else 0.0
            self.get_logger().info(
                f'Progress: {progress:.1f}% | '
                f'Distance: {current_distance_mm:.1f}/{self.goal.distance:.1f}mm | '
                f'Yaw Error: {yaw_error_deg:.2f}deg | '
                f'Vel: linear={linear_x:.3f}, angular={angular_z:.3f}'
            )

    def handle_reached(self):
        """REACHED 상태 처리 - 목표 도달"""
        # 정지 명령
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

        # 1초 대기 후 IDLE로 복귀
        if not hasattr(self, '_reached_time'):
            self._reached_time = time.time()

        if time.time() - self._reached_time > 1.0:
            self.get_logger().info('Returning to IDLE state')
            del self._reached_time
            self.state = State.IDLE
            self.goal = None

    def handle_error(self):
        """ERROR 상태 처리"""
        # 정지 명령
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

        # 2초 대기 후 IDLE로 복귀
        if not hasattr(self, '_error_time'):
            self._error_time = time.time()
            self.get_logger().error('Entering ERROR state')

        if time.time() - self._error_time > 2.0:
            self.get_logger().info('Recovering from ERROR, returning to IDLE')
            del self._error_time
            self.state = State.IDLE
            self.goal = None

    def calculate_distance_traveled(self):
        """이동 거리 계산 (mm)"""
        if self.start_odom is None or self.current_odom is None:
            return 0.0

        dx = self.current_odom.pose.pose.position.x - self.start_odom.pose.pose.position.x
        dy = self.current_odom.pose.pose.position.y - self.start_odom.pose.pose.position.y

        distance_m = math.sqrt(dx**2 + dy**2)
        distance_mm = distance_m * 1000.0

        return distance_mm

    def get_yaw_from_imu(self, imu_msg):
        """IMU quaternion에서 Yaw 각도 추출 (라디안)"""
        q = imu_msg.orientation

        # Quaternion to Euler angles conversion
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def normalize_angle(self, angle):
        """각도 정규화 (-pi ~ pi)"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_feedback(self, current_distance, yaw_error, linear_vel, angular_vel):
        """피드백 메시지 발행"""
        feedback = PrecisionNavFeedback()
        feedback.progress = abs(current_distance / self.goal.distance) if self.goal and self.goal.distance != 0 else 0.0
        feedback.current_distance = current_distance
        feedback.target_distance = self.goal.distance if self.goal else 0.0
        feedback.yaw_error = yaw_error
        feedback.linear_velocity = linear_vel
        feedback.angular_velocity = angular_vel

        self.feedback_pub.publish(feedback)

    def publish_status(self):
        """상태 메시지 발행"""
        status = PrecisionNavStatus()
        status.state = self.state.value
        status.active = (self.state == State.MOVING)
        status.error_msg = ''

        if self.state == State.ERROR:
            status.error_msg = 'Sensor data lost or timeout'

        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PrecisionNavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
