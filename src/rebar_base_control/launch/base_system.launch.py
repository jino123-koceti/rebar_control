#!/usr/bin/env python3
"""
Base System Launch File
하드웨어 추상화 계층 (rebar_base_control) 전체 실행

실행 노드:
1. can_parser - CAN 메시지 파싱
2. can_sender - CAN 메시지 전송 (속도 + 위치 제어)
3. drive_controller - cmd_vel/리모콘 → DriveControl 변환 (속도 제어 전용)
4. joint_controller - 리모콘 S17/S18/S23/S24 → JointControl (위치 제어 0xA4)
5. sequence_controller - S21/S22 작업 시퀀스 (Z축+그리퍼+트리거)
6. modbus_controller - Modbus 통신 (Seengrip)
7. ezi_io_controller - EZI-IO 리밋 센서 모니터링
8. authority_controller - 제어권한 관리
9. navigator_base - State Machine 상태 관리
10. pololu_node - 트리거 모터 제어 (Pololu)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 패키지 경로
    pkg_share = FindPackageShare('rebar_base_control')

    # Config 파일 경로
    can_config = PathJoinSubstitution([pkg_share, 'config', 'can_devices.yaml'])
    modbus_config = PathJoinSubstitution([pkg_share, 'config', 'modbus_devices.yaml'])

    return LaunchDescription([
        # CAN Parser Node
        Node(
            package='rebar_base_control',
            executable='can_parser.py',
            name='can_parser',
            output='screen',
            parameters=[can_config],
            emulate_tty=True
        ),

        # CAN Sender Node
        Node(
            package='rebar_base_control',
            executable='can_sender.py',
            name='can_sender',
            output='screen',
            parameters=[can_config],
            emulate_tty=True
        ),

        # Drive Controller Node (속도 제어)
        Node(
            package='rebar_base_control',
            executable='drive_controller.py',
            name='drive_controller',
            output='screen',
            parameters=[can_config],
            emulate_tty=True
        ),

        # Joint Controller Node (위치 제어 0xA4 - S17/S18/S23/S24)
        Node(
            package='rebar_base_control',
            executable='joint_controller.py',
            name='joint_controller',
            output='screen',
            parameters=[can_config],
            emulate_tty=True
        ),

        # Sequence Controller Node (S21/S22 작업 시퀀스)
        Node(
            package='rebar_base_control',
            executable='sequence_controller.py',
            name='sequence_controller',
            output='screen',
            parameters=[{
                'z_down_deg': 900.0,
                'z_up_deg': 3600.0,
                'z_speed': 200.0,
                'gripper_speed': 50,
                'gripper_force': 50,
                'wait_home': 2.0,
                'wait_z_down': 6.0,
                'wait_trigger': 3.0,
                'wait_open': 2.0,
                'trigger_speed': 1.0,  # Pololu 트리거 속도 (-1.0 ~ 1.0)
            }],
            emulate_tty=True
        ),

        # Modbus Controller Node
        Node(
            package='rebar_base_control',
            executable='modbus_controller.py',
            name='modbus_controller',
            output='screen',
            parameters=[modbus_config],
            emulate_tty=True
        ),

        # EZI-IO Controller Node (리밋 센서)
        Node(
            package='rebar_base_control',
            executable='ezi_io_controller.py',
            name='ezi_io_controller',
            output='screen',
            parameters=[can_config],
            emulate_tty=True
        ),

        # Authority Controller Node
        Node(
            package='rebar_base_control',
            executable='authority_controller.py',
            name='authority_controller',
            output='screen',
            emulate_tty=True
        ),

        # Navigator Base Node (State Machine)
        Node(
            package='rebar_base_control',
            executable='navigator_base.py',
            name='navigator_base',
            output='screen',
            emulate_tty=True
        ),

        # Pololu Node (트리거 모터)
        Node(
            package='pololu_ros2',
            executable='pololu_node',
            name='pololu_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baudrate': 9600,
                'motor_ids': [0],
                'motor_topics': ['motor_0/vel']
            }],
            emulate_tty=True
        ),
    ])
