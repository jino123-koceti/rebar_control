#!/usr/bin/env python3
"""
Base System Launch File
하드웨어 추상화 계층 (rebar_base_control) 전체 실행

실행 노드:
1. can_parser - CAN 메시지 파싱
2. can_sender - CAN 메시지 전송
3. drive_controller - cmd_vel → DriveControl 변환
4. modbus_controller - Modbus 통신 (Seengrip + EZI-IO)
5. authority_controller - 제어권한 관리
6. navigator_base - State Machine 상태 관리
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
            executable='can_parser',
            name='can_parser',
            output='screen',
            parameters=[can_config],
            emulate_tty=True
        ),

        # CAN Sender Node
        Node(
            package='rebar_base_control',
            executable='can_sender',
            name='can_sender',
            output='screen',
            parameters=[can_config],
            emulate_tty=True
        ),

        # Drive Controller Node
        Node(
            package='rebar_base_control',
            executable='drive_controller',
            name='drive_controller',
            output='screen',
            parameters=[can_config],
            emulate_tty=True
        ),

        # Modbus Controller Node
        Node(
            package='rebar_base_control',
            executable='modbus_controller',
            name='modbus_controller',
            output='screen',
            parameters=[modbus_config],
            emulate_tty=True
        ),

        # Authority Controller Node
        Node(
            package='rebar_base_control',
            executable='authority_controller',
            name='authority_controller',
            output='screen',
            emulate_tty=True
        ),

        # Navigator Base Node (State Machine)
        Node(
            package='rebar_base_control',
            executable='navigator_base',
            name='navigator_base',
            output='screen',
            emulate_tty=True
        ),
    ])
