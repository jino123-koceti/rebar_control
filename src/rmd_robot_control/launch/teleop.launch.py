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
  L4  mode_arbiter            제어 권한 중재 (수동/호밍/자율)

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
        respawn=True, respawn_delay=2.0,
    )
    lateral = Node(
        package='rmd_robot_control', executable='lateral_node',
        name='lateral_node', output='screen',
        parameters=[{
            'speed_dps': LaunchConfiguration('lateral_speed_dps'),
            'i_hard_a': LaunchConfiguration('lateral_i_hard_a'),
            'max_torque': LaunchConfiguration('lateral_max_torque'),
        }],
        respawn=True, respawn_delay=2.0,
    )
    remote_bridge = Node(
        package='rebar_base_control', executable='remote_bridge.py',
        name='remote_bridge', output='screen',
        respawn=True, respawn_delay=2.0,
    )
    remote_teleop = Node(
        package='rmd_robot_control', executable='remote_teleop_node',
        name='remote_teleop_node', output='screen',
        respawn=True, respawn_delay=2.0,
    )
    ezi_io = Node(
        package='ezi_io_ros2', executable='ezi_io_node',
        name='ezi_io_node', output='screen',
        parameters=[ezi_params] if ezi_params else [],
        respawn=True, respawn_delay=2.0,
    )
    safety = Node(
        package='rebar_base_control', executable='safety_node.py',
        name='safety_node', output='screen',
        respawn=True, respawn_delay=2.0,
    )
    # L4 권한 중재. 이게 없으면 호밍과 리모콘이 같은 축 토픽에 동시에 써서
    # 축이 툭툭 끊긴다 (2026-09-30 실측, mode_arbiter.py 주석 참고).
    mode_arbiter = Node(
        package='rebar_base_control', executable='mode_arbiter.py',
        name='mode_arbiter', output='screen',
        respawn=True, respawn_delay=2.0,
    )

    # ── 상부 스테이지 (L3·L4) ────────────────────────────────────────────
    # ⚠ **호밍 원점의 보유자는 `homing_node` 하나다.** refs 를 2Hz 로 계속
    #   재발행하므로 `stage_node`·`tying_sequence` 는 죽어도 0.5초 안에 원점을
    #   다시 받는다. 반대로 `homing_node` 가 죽으면 원점이 사라져 **재호밍이
    #   필요하다** — 멀티턴은 전원 세션 안에서만 유효하지만 refs 자체는 그
    #   노드의 메모리에만 있기 때문이다.
    # ⚠⚠ **자동 호밍은 넣지 않는다.** 호밍은 전 축을 리미트까지 크게 쓸어가는
    #   동작이라, 되살아난 즉시 자동으로 돌면 아무도 시키지 않았는데 장비가
    #   움직인다. `respawn` 과 겹치면 크래시 루프가 재호밍 루프가 된다.
    #   원점이 없으면 `stage_node` 가 모든 이동을 거부하는 것이 안전망이다
    #   ("호밍 원점이 없다 — 먼저 호밍하세요"). 호밍은 `/homing_cmd` 로만 시작한다.
    # 파라미터: robot_control.yaml 에는 이 세 노드 섹션이 없다. 넘기면 조용히
    #   무시되지만(섹션 이름이 안 맞으면 적용 안 된다) 혼동을 피해 안 넘긴다.
    homing = Node(
        package='rmd_robot_control', executable='homing_node',
        name='homing_node', output='screen',
        respawn=True, respawn_delay=2.0,
    )
    stage = Node(
        package='rmd_robot_control', executable='stage_node',
        name='stage_node', output='screen',
        respawn=True, respawn_delay=2.0,
    )
    tying = Node(
        package='rmd_robot_control', executable='tying_sequence',
        name='tying_sequence', output='screen',
        respawn=True, respawn_delay=2.0,
    )

    return LaunchDescription(args + [
        LogInfo(msg=['리모콘 조작 구성 기동 — 안전 차단: ', use_safety,
                     ' / 호밍은 자동으로 돌지 않는다 (/homing_cmd 로 시작)']),
        motor, lateral, remote_bridge, remote_teleop, ezi_io, safety, mode_arbiter,
        homing, stage, tying,
    ])
