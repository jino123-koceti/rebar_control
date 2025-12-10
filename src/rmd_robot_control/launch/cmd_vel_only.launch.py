#!/usr/bin/env python3
"""
CMD_VEL 제어만 실행하는 런치 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    # 통합 제어 노드 (CMD_VEL 포함)
    position_control_node = Node(
        package='rmd_robot_control',
        executable='position_control_node',
        name='position_control_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('joint_states', '/joint_states'),
            ('motor_status', '/motor_status')
        ]
    )
    
    # 시작 메시지
    start_message = LogInfo(
        msg="CMD_VEL 제어 시스템 시작 (통합 노드 사용)"
    )
    
    return LaunchDescription([
        can_interface_arg,
        use_sim_time_arg,
        start_message,
        position_control_node
    ])



