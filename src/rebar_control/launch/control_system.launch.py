#!/usr/bin/env python3
"""
Control System Launch File
상위 제어 계층 (rebar_control) 전체 실행

실행 노드:
1. pose source (듀얼 ZED: pose_mux / 단일 ZED: odom_to_pose)
2. zenoh_client - UI 통신 (Zenoh ↔ ROS2)
3. navigator - 미션 관리 (웨이포인트 순회)
4. rebar_controller - 경로 추종 제어 (PID)
5. rebar_publisher - 상태 취합/발행
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # 패키지 경로
    pkg_share = FindPackageShare('rebar_control')

    # Config 파일 경로
    zenoh_config = PathJoinSubstitution([pkg_share, 'config', 'zenoh_config.yaml'])

    # Launch Arguments
    use_dual_zed_arg = DeclareLaunchArgument(
        'use_dual_zed',
        default_value='true',
        description='Set to false to use single ZED camera with odom_to_pose converter'
    )

    single_zed_topic_arg = DeclareLaunchArgument(
        'single_zed_odom_topic',
        default_value='/zed/zed_node/odom',
        description='Odometry topic for single ZED camera (used when use_dual_zed=false)'
    )

    return LaunchDescription([
        # Launch Arguments
        use_dual_zed_arg,
        single_zed_topic_arg,

        # Pose Source: Dual ZED (pose_mux)
        Node(
            package='rebar_control',
            executable='pose_mux',
            name='pose_mux',
            output='screen',
            parameters=[{
                # [2026-08-06] 주행 카메라 교체: zed_front/zed_back(ZED X) → zedxmini1/2.
                #   zedxmini2 = 물리 전방, zedxmini1 = 물리 후방 (시야 스냅샷으로 확인).
                #   슬롯 이름은 '진행방향' 기준이고, 진행방향 반대편 카메라의 VSLAM을 쓴다
                #   (기존 front_odom_topic=/zed_back 매핑과 동일한 규칙 유지).
                #     전진(front 슬롯) → 물리 후방 카메라 zedxmini1
                #     후진(back  슬롯) → 물리 전방 카메라 zedxmini2
                # ⚠ zedxmini는 현재 depth_mode:=NONE + pos_tracking_enabled:=false 라
                #   odom이 발행되지 않는다. 사용하려면 full_system.launch.py에서
                #   depth_mode를 PERFORMANCE 이상 + pos_tracking_enabled:=true 로 올릴 것.
                'front_odom_topic': '/zedxmini1/zed_node/odom',
                'back_odom_topic': '/zedxmini2/zed_node/odom',
                'output_topic': '/robot_pose',
                'direction_source': 'topic',
                'direction_topic': '/travel_direction',
                'velocity_deadband': 0.02,
                'prefer_front_when_stationary': False,
                'output_frame': 'odom',
                'front_camera_offset': -0.49,   # front cam offset from robot center
                'back_camera_offset': 0.49,     # back cam offset from robot center
            }],
            condition=IfCondition(LaunchConfiguration('use_dual_zed')),
            emulate_tty=True
        ),

        # Pose Source: Single ZED (odom_to_pose)
        Node(
            package='rebar_control',
            executable='odom_to_pose',
            name='odom_to_pose',
            output='screen',
            parameters=[{
                'input_odom_topic': LaunchConfiguration('single_zed_odom_topic'),
                'output_pose_topic': '/robot_pose',
                'output_frame': 'odom',
                'queue_size': 10,
            }],
            condition=UnlessCondition(LaunchConfiguration('use_dual_zed')),
            emulate_tty=True
        ),

        # Zenoh Client Node
        Node(
            package='rebar_control',
            executable='zenoh_client',
            name='zenoh_client',
            output='screen',
            parameters=[zenoh_config],
            emulate_tty=True
        ),

        # Navigator Node
        Node(
            package='rebar_control',
            executable='navigator',
            name='navigator',
            output='screen',
            emulate_tty=True
        ),

        # Rebar Controller Node (supports lateral + differential motion)
        Node(
            package='rebar_control',
            executable='rebar_controller',
            name='rebar_controller',
            output='screen',
            emulate_tty=True
        ),

        # Rebar Publisher Node
        Node(
            package='rebar_control',
            executable='rebar_publisher',
            name='rebar_publisher',
            output='screen',
            emulate_tty=True
        ),
    ])
