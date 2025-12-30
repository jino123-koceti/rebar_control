#!/usr/bin/env python3
"""
Performance Test Node
600mm 전진/후진 반복 테스트 및 위치 정확도 측정

자동으로 웨이포인트를 발행하여 전진/후진을 반복하고,
각 지점에서의 위치 오차를 측정합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import csv
import time
import json
import numpy as np
from datetime import datetime
import os


class PerformanceTest(Node):
    """성능 테스트 노드"""

    def __init__(self):
        super().__init__('performance_test')

        # 테스트 설정
        self.declare_parameter('num_cycles', 10)  # 반복 횟수
        self.declare_parameter('target_distance', 0.6)  # 목표 거리 (m)
        self.declare_parameter('position_tolerance', 0.05)  # 도달 판정 (m)
        self.declare_parameter('output_csv', 'performance_test_results.csv')
        self.declare_parameter('wait_time', 2.0)  # 각 단계 대기 시간 (초)

        self.num_cycles = self.get_parameter('num_cycles').value
        self.target_distance = self.get_parameter('target_distance').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.output_csv = self.get_parameter('output_csv').value
        self.wait_time = self.get_parameter('wait_time').value

        # 상태 변수
        self.current_pose = None
        self.test_running = False
        self.current_cycle = 0
        self.current_phase = 'idle'  # idle, forward, backward
        self.waiting_for_waypoint = False
        self.results = []  # [(cycle, phase, target_x, actual_x, actual_y, error), ...]

        # ROS2 통신
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10
        )

        self.feedback_sub = self.create_subscription(
            String, '/mission/feedback', self.feedback_callback, 10
        )

        self.command_pub = self.create_publisher(
            String, '/mission/command', 10
        )

        # 타이머 (상태 체크, 1Hz)
        self.timer = self.create_timer(1.0, self.update_test)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Performance Test 노드 초기화 완료")
        self.get_logger().info(f"  테스트 횟수: {self.num_cycles} cycles")
        self.get_logger().info(f"  이동 거리: {self.target_distance} m ({self.target_distance * 1000:.0f} mm)")
        self.get_logger().info(f"  도달 허용 오차: {self.position_tolerance} m ({self.position_tolerance * 1000:.0f} mm)")
        self.get_logger().info(f"  단계별 대기: {self.wait_time} 초")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        self.get_logger().info("3초 후 자동으로 테스트를 시작합니다...")
        self.get_logger().info("")

        # 3초 후 자동 시작
        self.start_timer = self.create_timer(3.0, self.auto_start)

    def auto_start(self):
        """자동 시작 (1회만)"""
        self.start_timer.cancel()
        self.start_test()

    def pose_callback(self, msg):
        """현재 위치 업데이트"""
        self.current_pose = msg

    def feedback_callback(self, msg):
        """미션 피드백 처리"""
        try:
            feedback = json.loads(msg.data)
            state = feedback.get('state', '')

            # 웨이포인트 도달 확인
            if state == 'mission_done' and self.waiting_for_waypoint:
                self.waiting_for_waypoint = False
                self.on_waypoint_reached()

        except Exception as e:
            self.get_logger().error(f"피드백 파싱 오류: {e}")

    def start_test(self):
        """테스트 시작"""
        self.test_running = True
        self.current_cycle = 1
        self.current_phase = 'forward'
        self.results = []

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"===== 성능 테스트 시작 ({self.num_cycles} cycles) =====")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")

        time.sleep(1.0)
        self.send_waypoint([self.target_distance, 0.0])

    def send_waypoint(self, target):
        """웨이포인트 전송"""
        waypoints_json = {
            "waypoints": [{"x": target[0] * 1000, "y": target[1] * 1000}]  # m → mm
        }

        command = f"WAYPOINTS:{json.dumps(waypoints_json)}"

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        # 미션 시작
        time.sleep(0.5)
        msg.data = "START_MISSION"
        self.command_pub.publish(msg)

        self.waiting_for_waypoint = True

        self.get_logger().info(
            f"[Cycle {self.current_cycle}/{self.num_cycles}] "
            f"{self.current_phase.upper()}: 목표 ({target[0]:.3f}, {target[1]:.3f}) m 전송"
        )

    def on_waypoint_reached(self):
        """웨이포인트 도달 처리"""
        if not self.current_pose:
            self.get_logger().warn("위치 정보 없음")
            return

        # 결과 기록
        actual_x = self.current_pose.pose.position.x
        actual_y = self.current_pose.pose.position.y

        if self.current_phase == 'forward':
            target_x = self.target_distance
            error = np.sqrt((actual_x - target_x)**2 + actual_y**2)

            self.results.append({
                'cycle': self.current_cycle,
                'phase': 'forward',
                'target_x': target_x,
                'target_y': 0.0,
                'actual_x': actual_x,
                'actual_y': actual_y,
                'error': error
            })

            self.get_logger().info(
                f"[Cycle {self.current_cycle}/{self.num_cycles}] FORWARD 완료: "
                f"실제=({actual_x:.4f}, {actual_y:.4f}) m, "
                f"오차={error*1000:.1f} mm"
            )

            # 다음 단계: 후진
            self.current_phase = 'backward'
            time.sleep(self.wait_time)  # 안정화
            self.send_waypoint([0.0, 0.0])

        elif self.current_phase == 'backward':
            target_x = 0.0
            error = np.sqrt(actual_x**2 + actual_y**2)

            self.results.append({
                'cycle': self.current_cycle,
                'phase': 'backward',
                'target_x': target_x,
                'target_y': 0.0,
                'actual_x': actual_x,
                'actual_y': actual_y,
                'error': error
            })

            self.get_logger().info(
                f"[Cycle {self.current_cycle}/{self.num_cycles}] BACKWARD 완료: "
                f"실제=({actual_x:.4f}, {actual_y:.4f}) m, "
                f"오차={error*1000:.1f} mm"
            )
            self.get_logger().info("")

            # 다음 사이클
            if self.current_cycle < self.num_cycles:
                self.current_cycle += 1
                self.current_phase = 'forward'
                time.sleep(self.wait_time)
                self.send_waypoint([self.target_distance, 0.0])
            else:
                # 테스트 완료
                self.finish_test()

    def finish_test(self):
        """테스트 완료 및 결과 저장"""
        self.test_running = False
        self.current_phase = 'idle'

        self.get_logger().info("=" * 60)
        self.get_logger().info("===== 성능 테스트 완료 =====")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")

        # CSV 저장
        csv_path = self.save_results_csv()

        # 통계 계산 및 출력
        self.print_statistics()

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"결과 파일: {csv_path}")
        self.get_logger().info("")
        self.get_logger().info("시각화 명령:")
        self.get_logger().info(f"  python3 src/rebar_control/scripts/visualize_results.py {csv_path}")
        self.get_logger().info("=" * 60)

        # 노드 종료
        time.sleep(2.0)
        raise KeyboardInterrupt

    def save_results_csv(self):
        """결과를 CSV 파일로 저장"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"{timestamp}_{self.output_csv}"

        # 현재 디렉토리에 저장
        filepath = os.path.join(os.getcwd(), filename)

        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'cycle', 'phase', 'target_x', 'target_y',
                'actual_x', 'actual_y', 'error'
            ])
            writer.writeheader()
            writer.writerows(self.results)

        self.get_logger().info(f"✅ CSV 저장 완료: {filepath}")
        return filepath

    def print_statistics(self):
        """통계 출력"""
        if not self.results:
            return

        errors = [r['error'] for r in self.results]

        mean_error = np.mean(errors) * 1000  # mm
        std_error = np.std(errors) * 1000
        max_error = np.max(errors) * 1000
        min_error = np.min(errors) * 1000

        self.get_logger().info("=" * 60)
        self.get_logger().info("===== 통계 결과 =====")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  평균 오차:   {mean_error:6.2f} mm")
        self.get_logger().info(f"  표준편차:    {std_error:6.2f} mm")
        self.get_logger().info(f"  최대 오차:   {max_error:6.2f} mm")
        self.get_logger().info(f"  최소 오차:   {min_error:6.2f} mm")
        self.get_logger().info("")

        # 전진/후진 분리 통계
        forward_errors = [r['error'] for r in self.results if r['phase'] == 'forward']
        backward_errors = [r['error'] for r in self.results if r['phase'] == 'backward']

        if forward_errors:
            self.get_logger().info(
                f"  전진 평균:   {np.mean(forward_errors)*1000:6.2f} mm "
                f"(std: {np.std(forward_errors)*1000:.2f} mm)"
            )

        if backward_errors:
            self.get_logger().info(
                f"  후진 평균:   {np.mean(backward_errors)*1000:6.2f} mm "
                f"(std: {np.std(backward_errors)*1000:.2f} mm)"
            )

        self.get_logger().info("=" * 60)

    def update_test(self):
        """타이머 콜백 (상태 모니터링)"""
        # 현재는 피드백 기반으로 동작하므로 추가 로직 불필요
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("테스트 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
