#!/usr/bin/env python3
"""
로봇 제어 통합 런치 파일
CMD_VEL 제어와 위치제어를 동시에 실행
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """런치 파일 생성"""
    
    # 패키지 경로 가져오기
    pkg_share = FindPackageShare(package='rmd_robot_control').find('rmd_robot_control')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'robot_control.yaml'])
    
    # 런치 인수 선언
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can2',
        description='CAN 인터페이스 이름'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    # 통합 제어 노드 (CMD_VEL + 위치제어)
    position_control_node = Node(
        package='rmd_robot_control',
        executable='position_control_node',
        name='position_control_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('joint_states', '/joint_states'),
            ('motor_status', '/motor_status')
        ]
    )
    
    # Joint State Publisher (통합된 관절 상태)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('joint_states', '/joint_states')
        ]
    )
    
    # 시작 메시지
    start_message = LogInfo(
        msg="RMD 로봇 제어 시스템 시작"
    )
    
    return LaunchDescription([
        can_interface_arg,
        use_sim_time_arg,
        start_message,
        position_control_node,
        joint_state_publisher
    ])



