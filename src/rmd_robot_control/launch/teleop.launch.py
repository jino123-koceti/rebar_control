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
        parameters=[{
            # [2026-10-04] 30 → 60 dps. 30dps 는 출력축 8.7mm/s 라 X 전 행정
            # (453mm)에 52초다. 리모콘이 이미 100dps 로 도는 축이므로 60 은 보수적이다.
            'move_speed_dps': 60.0,
            # ⚠ [2026-10-04] **선속도 기준으로 바꿨다.** 같은 dps 는 축마다 다른
            #   mm/s 다 (X 0.2906 / Y 0.2227 / Z 0.0906 mm/도) — 60dps 면
            #   X 17.4 · Y 13.4 · **Z 5.4** mm/s 라 Z 가 사이클을 지배했다
            #   (6점 4분 17초 중 한 점 41초에서 Z 하강·상승이 32초).
            #   30mm/s 실측: X 25.4 · Y 25.5 · Z 17.3 mm/s, 도달오차 ≤0.76mm.
            #   Z 80mm 가 15.8초 → 4.7초로 줄었다.
            #   40mm/s 도 돌아간다 (X 32.9 · Y 31.0) 그러나 X 정상 전류가
            #   4.08A 까지 올라 충돌 문턱 4.8A 와 여유가 0.7A 뿐이다. 30 을 쓴다.
            'move_speed_mm_s': 30.0,
            # ⚠ [2026-10-04] **축별로 다르게 둔다.** X·Y 는 80mm/s 까지 올려도
            #   전류가 4.45→4.72A 로 거의 안 늘고 도달오차도 ≤0.89mm 다
            #   (속도가 아니라 정지마찰·하중이 전류를 정한다). 실측 X 58 · Y 47mm/s.
            #   Z 는 행정이 80mm 뿐이라 **올려도 안 빨라진다** (30→11.4, 60→10.3mm/s
            #   가속 한계). 그런데 올리면 **충돌이 세진다** — 같은 접촉이 30mm/s 에서
            #   4.67A, 60mm/s 에서 5.93A 였다. 철근을 향해 내려가는 축이라 느린 쪽이다.
            'move_speed_mm_s_x': 80.0,
            'move_speed_mm_s_y': 80.0,
            'move_speed_mm_s_z': 20.0,
            'max_speed_dps': 450.0,
            # ── 충돌 감지 ────────────────────────────────────────────────
            # 교차점이 아닌 곳에서 Z 를 내리면 건이 철근을 찍는다. 작업영역
            # 검사(X·Y)로는 막을 수 없다 — 작업영역 안이어도 그 자리에 철근이 있다.
            # ⚠⚠ **문턱은 축마다 다르다.** 실측 정상 이동 전류(30mm/s):
            #   X 최대 3.86A · Y 2.92A · Z 1.88A. Z 충돌 때는 3.1~3.9A 가 떠서
            #   정상의 두 배다. Z 기준 3.0A 를 세 축에 같이 쓰니 X 가 헛걸렸다.
            # 실장비 검증(2026-10-04): Z -63mm 에서 3.10A 감지 → 이동 취소 →
            #   -47.9mm 로 후퇴, 거부수 +1 로 상위가 그 점을 건너뛴다.
            'collide_detect': True,
            'collide_current_a_x': 4.8,
            'collide_current_a_y': 3.8,
            'collide_current_a_z': 2.5,
            'collide_backoff_mm': 15.0,
            # ⚠⚠ **상승분으로 본다.** 절대 문턱만 쓰면 늦다 — 접촉 후 전류가
            #   1.19→2.03A 로 오르는 데 250ms, 거기서 문턱을 3표본 넘는 데
            #   100ms 가 더 걸렸다. 상승분을 넣으니 2.77A → **1.39A** 에서 끊었다.
            #   문턱은 실측 정상 변동폭 위여야 한다 (Z 하강 0.9A · X 0.8A).
            'collide_rise_a_x': 1.5,
            'collide_rise_a_y': 1.2,
            'collide_rise_a_z': 1.0,
            # ⚠⚠ Z 는 **내려갈 때만** 판정한다. 올릴 때도 걸려서 결속점 3개가
            #   전부 실패했다 — 위에는 부딪힐 것이 없고, 건이 철근에 걸려
            #   빠져나올 때 전류가 오르는 것은 정상이다.
            'collide_down_only_axes': ['z'],
            # ⚠ 브레이크: 도착해도 X·Y 는 곧바로 잠그지 않는다. 다음 구간이 바로
            #   오면 해제 대기가 없어져 동작이 이어진다 — 구간마다 고정 1초를
            #   기다려서 "너무 시퀀스처럼 움직인다" 는 지적을 받았다.
            #   Z 는 넣지 않는다 (자중 낙하).
            'brake_hold_axes': ['x', 'y'],
            'brake_hold_sec': 2.5,
            # [2026-10-04] 30 → 120초. X 405mm 이동이 30초에 끊겨 153mm 에서
            # 멈췄다. 전 행정을 60dps 로 가도 26초이고, 브레이크 해제 대기와
            # 리미트 접근 감속까지 보면 여유가 필요하다.
            'move_timeout_sec': 120.0,
        }],
        respawn=True, respawn_delay=2.0,
    )
    # ⚠⚠ **결속건은 기본 꺼짐이다.** 켜는 것은 명시적 결정이어야 한다 —
    #   되돌릴 수 없고, 작업영역 안에 사람이 있을 수 있다. 꺼져 있으면 Z 하강·
    #   상승은 그대로 하고 건만 건너뛴다 (예행연습이 그 상태다).
    #   켤 때: gun_enabled:=true. Pololu 노드(/motor_0/vel)가 떠 있어야 한다.
    # ⚠ Z 는 목표에 z 가 실려 올 때만 움직인다 (`tying_planner` 가 모델에서
    #   계산해 싣는다). 손으로 x·y 만 보내면 Z 단계는 건너뛴다.
    tying = Node(
        package='rmd_robot_control', executable='tying_sequence',
        name='tying_sequence', output='screen',
        parameters=[{
            'gun_enabled': False,
            'z_safe_mm': 0.0,
            # 모델 잔차가 Z 2.7mm 다. 실측 결속깊이는 -58~-83mm 였으니
            # 그 바깥은 모델이 틀린 것으로 보고 거부한다.
            'z_tie_min_mm': -95.0,
            'z_tie_max_mm': -40.0,
        }],
        respawn=True, respawn_delay=2.0,
    )
    # ⚠ `plan_only` 를 **켜 둔다.** /plan/start 는 검출→계획까지만 하고 멈추고,
    #   실행은 /plan/execute 로 따로 받는다. 검출이 틀리면 장비가 철근을 향해
    #   그대로 가므로, 사람이 계획을 보고 한 번 끊는 지점이 있어야 한다.
    #   한 번에 돌리려면 plan_only:=false.
    planner = Node(
        package='rmd_robot_control', executable='tying_planner',
        name='tying_planner', output='screen',
        parameters=[{'plan_only': True}],
        respawn=True, respawn_delay=2.0,
    )

    return LaunchDescription(args + [
        LogInfo(msg=['리모콘 조작 구성 기동 — 안전 차단: ', use_safety,
                     ' / 호밍은 자동으로 돌지 않는다 (/homing_cmd 로 시작)']),
        motor, lateral, remote_bridge, remote_teleop, ezi_io, safety, mode_arbiter,
        homing, stage, tying, planner,
    ])
