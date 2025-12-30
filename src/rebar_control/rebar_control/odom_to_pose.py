#!/usr/bin/env python3
"""
Odometry to PoseStamped Converter Node

ZED X 카메라의 Odometry 메시지를 PoseStamped로 변환하여 발행합니다.
단일 카메라 환경에서 사용하며, 듀얼 카메라 환경에서는 pose_mux를 사용합니다.

구독:
- /zed/zed_node/odom (nav_msgs/Odometry) - ZED X 카메라에서

발행:
- /robot_pose (geometry_msgs/PoseStamped) - rebar_controller, zenoh_client로
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPose(Node):
    """Odometry 메시지를 PoseStamped로 변환하는 노드"""

    def __init__(self):
        super().__init__('odom_to_pose')

        # 파라미터 선언
        self.declare_parameter('input_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('output_pose_topic', '/robot_pose')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('queue_size', 10)

        # 파라미터 값 가져오기
        self.input_topic = self.get_parameter('input_odom_topic').value
        self.output_topic = self.get_parameter('output_pose_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.queue_size = int(self.get_parameter('queue_size').value)

        # 발행자 생성
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.output_topic,
            self.queue_size
        )

        # 구독자 생성
        self.odom_sub = self.create_subscription(
            Odometry,
            self.input_topic,
            self.odom_callback,
            self.queue_size
        )

        # 통계 정보
        self.msg_count = 0

        # 초기화 완료 로그
        self.get_logger().info('=' * 50)
        self.get_logger().info('Odometry to Pose Converter 초기화 완료')
        self.get_logger().info(f'  입력 토픽: {self.input_topic}')
        self.get_logger().info(f'  출력 토픽: {self.output_topic}')
        self.get_logger().info(f'  출력 프레임: {self.output_frame}')
        self.get_logger().info(f'  큐 크기: {self.queue_size}')
        self.get_logger().info('=' * 50)

    def odom_callback(self, msg: Odometry):
        """
        Odometry 메시지 수신 및 변환

        Args:
            msg: nav_msgs/Odometry 메시지
        """
        try:
            # PoseStamped 메시지 생성
            pose_msg = PoseStamped()

            # 헤더 설정 (타임스탬프 유지)
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = self.output_frame

            # Pose 정보 복사 (PoseWithCovariance.pose → Pose)
            pose_msg.pose = msg.pose.pose

            # 발행
            self.pose_pub.publish(pose_msg)

            # 통계 업데이트 (100번마다 로그)
            self.msg_count += 1
            if self.msg_count % 100 == 0:
                self.get_logger().debug(
                    f'Odometry → Pose 변환 완료 (총 {self.msg_count}개 메시지)'
                )

        except Exception as e:
            self.get_logger().error(f'Odometry 변환 중 오류 발생: {e}')


def main(args=None):
    """노드 메인 함수"""
    rclpy.init(args=args)
    node = OdomToPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료됨 (Ctrl+C)')
    except Exception as e:
        node.get_logger().error(f'노드 실행 중 오류 발생: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
