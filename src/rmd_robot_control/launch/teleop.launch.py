#!/usr/bin/env python3
"""3차년도 수동 조작 구성 — 리모콘으로 조작한다.

systemd `rebar-teleop.service` 가 이 파일을 띄운다. 전원만 넣으면 리모콘이 먹는 상태가
목표다 (2차년도 integrated_control.sh 와 같은 역할).

  L1  position_control_node   can2 소유자. 모터 명령·보호
      lateral_node            횡이동 2축
      remote_bridge           can3 → /remote_control  (can3 소유자)
      ezi_io_node             EZIO 2보드 → 리미트·범퍼·스위치
  L2  safety_node             안전 판정 (차단은 L1 이 한다)
  L3  remote_teleop_node      리모콘 입력 → 축 명령

2026-09-30: 키보드 텔레옵(`teleop_keyboard`)을 제거하고 리모콘으로 통합했다.
리모콘으로 주행·상부 축이 동작하는 것을 실장비에서 확인한 뒤 정리했다.

## 인자

  use_safety:=true|false   안전 차단을 켤지 (기본 false)
      ⚠ 기본값이 false 인 이유: 차단을 켜면 safety_node·ezi_io_node 가 죽거나
        EZIO 가 안 붙을 때 **모든 명령이 막힌다.** 의도한 동작이지만, 검증 전에
        기본으로 켜두면 "갑자기 아무것도 안 움직인다" 가 된다.
        실장비에서 범퍼·STOP·비상정지 차단을 확인한 뒤 true 로 바꿀 것.
  lateral_speed_dps, lateral_i_hard_a, lateral_max_torque   횡이동 파라미터
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rmd_share = get_package_share_directory('rmd_robot_control')
    params = os.path.join(rmd_share, 'config', 'robot_control.yaml')
    try:
        ezi_params = os.path.join(
            get_package_share_directory('ezi_io_ros2'), 'config', 'ezi_io.yaml')
    except Exception:
        ezi_params = None

    use_safety = LaunchConfiguration('use_safety')
    # 차단을 켤 때만 의미 있는 값. 0 이면 position_control_node 가 차단을 적용하지 않는다.
    safety_timeout = PythonExpression(["'1.0' if '", use_safety, "' == 'true' else '0.0'"])

    args = [
        DeclareLaunchArgument('use_safety', default_value='false',
                              description='안전 차단 활성화 (검증 후 true 로)'),
        DeclareLaunchArgument('lateral_speed_dps', default_value='50'),
        DeclareLaunchArgument('lateral_i_hard_a', default_value='24.0'),
        DeclareLaunchArgument('lateral_max_torque', default_value='255'),
    ]

    motor = Node(
        package='rmd_robot_control', executable='position_control_node',
        name='position_control_node', output='screen',
        parameters=[params, {'safety_timeout': safety_timeout}],
        remappings=[('cmd_vel', '/cmd_vel'), ('joint_states', '/joint_states'),
                    ('motor_status', '/motor_status')],
    )
    lateral = Node(
        package='rmd_robot_control', executable='lateral_node',
        name='lateral_node', output='screen',
        parameters=[{
            'speed_dps': LaunchConfiguration('lateral_speed_dps'),
            'i_hard_a': LaunchConfiguration('lateral_i_hard_a'),
            'max_torque': LaunchConfiguration('lateral_max_torque'),
        }],
    )
    remote_bridge = Node(
        package='rebar_base_control', executable='remote_bridge.py',
        name='remote_bridge', output='screen',
    )
    remote_teleop = Node(
        package='rmd_robot_control', executable='remote_teleop_node',
        name='remote_teleop_node', output='screen',
    )
    ezi_io = Node(
        package='ezi_io_ros2', executable='ezi_io_node',
        name='ezi_io_node', output='screen',
        parameters=[ezi_params] if ezi_params else [],
    )
    safety = Node(
        package='rebar_base_control', executable='safety_node.py',
        name='safety_node', output='screen',
    )

    return LaunchDescription(args + [
        LogInfo(msg=['리모콘 조작 구성 기동 — 안전 차단: ', use_safety]),
        motor, lateral, remote_bridge, remote_teleop, ezi_io, safety,
    ])
