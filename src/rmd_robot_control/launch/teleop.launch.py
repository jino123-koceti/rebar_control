#!/usr/bin/env python3
"""키보드 텔레옵용 런치: 주행(position_control_node) + 횡이동(lateral_node)

텔레옵 자체는 키 입력을 받아야 하므로 별도 터미널에서 직접 실행한다:
  ros2 run rmd_robot_control teleop_keyboard
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='rmd_robot_control').find('rmd_robot_control')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'robot_control.yaml'])

    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can2', description='CAN 인터페이스')
    lateral_speed_arg = DeclareLaunchArgument(
        'lateral_speed_dps', default_value='50', description='횡이동 속도 (dps)')
    lateral_max_torque_arg = DeclareLaunchArgument(
        'lateral_max_torque', default_value='255',
        description='횡이동 0xA2/0xA4 DATA[1] maxTorque. 0 = 힘 제어 비활성')
    lateral_i_hard_arg = DeclareLaunchArgument(
        'lateral_i_hard_a', default_value='20.0',
        description='횡이동 하드 차단 전류 [A]. 데이터시트 피크 21.5A 를 넘기지 말 것')

    drive_node = Node(
        package='rmd_robot_control',
        executable='position_control_node',
        name='position_control_node',
        output='screen',
        parameters=[config_file],
        remappings=[('cmd_vel', '/cmd_vel'),
                    ('joint_states', '/joint_states'),
                    ('motor_status', '/motor_status')],
    )

    lateral_node = Node(
        package='rmd_robot_control',
        executable='lateral_node',
        name='lateral_node',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'speed_dps': LaunchConfiguration('lateral_speed_dps'),
            'i_hard_a': ParameterValue(
                LaunchConfiguration('lateral_i_hard_a'), value_type=float),
            'max_torque': ParameterValue(
                LaunchConfiguration('lateral_max_torque'), value_type=int),
        }],
    )

    return LaunchDescription([
        can_interface_arg,
        lateral_speed_arg,
        lateral_i_hard_arg,
        lateral_max_torque_arg,
        LogInfo(msg='텔레옵 백엔드 시작 — 조작은 별도 터미널에서 '
                    '`ros2 run rmd_robot_control teleop_keyboard`'),
        drive_node,
        lateral_node,
    ])
