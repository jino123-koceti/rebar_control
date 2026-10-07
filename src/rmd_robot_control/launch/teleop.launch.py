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
  use_camera:=true|false   작업영역 카메라(Gemini 2L) 를 같이 띄울지 (기본 true)
      yaw 자세 **별칭 판정**에 쓴다 — 1번·4번은 단회전만으로 못 가려서, X 호밍
      뒤 영상 한 장으로 후보 둘 중 하나를 고른다. 이게 있어야 사용자가 전원
      투입 때마다 yaw 를 12시로 맞추지 않아도 호밍이 돈다.
      ⚠ 카메라가 없어도 **호밍 자체는 돈다** — 비전은 별칭일 때만 부르는 보조
        수단이고, 없으면 기존 거부 경로(12시로 옮기거나 선언)로 간다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        # ⚠ 기본 true 다. yaw 별칭 판정이 이 카메라에 달려 있고, 그게 "전원
        #   투입 때마다 사람이 12시로 맞추는 일" 을 없애는 수단이기 때문이다.
        #   카메라가 말썽이면 false 로 끄면 되고, 꺼도 호밍은 돈다 (별칭일 때만
        #   거부되어 기존 경로로 간다).
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='작업영역 카메라(Gemini 2L) 동시 기동'),
    ]

    motor = Node(
        package='rmd_robot_control', executable='position_control_node',
        name='position_control_node', output='screen',
        # ⚠ `stage_accel_dpss` 는 **쓰지 않는다**(0). 전 축에 같은 값을 넣으면
        #   X 가 망가진다 — 축별 값은 `stage_node` 의 `accel_dpss_*` 가 보낸다.
        parameters=[params, {'safety_timeout': safety_timeout,
                             'stage_accel_dpss': 0.0}],
        remappings=[('cmd_vel', '/cmd_vel'), ('joint_states', '/joint_states'),
                    ('motor_status', '/motor_status')],
        # ⚠⚠ 상부축 가감속. **0x43 은 ROM 에 남지 않아 매 기동 재적용이 필수**라
        #   노드가 기동 3초 뒤 한 번 넣는다. 2026-10-04 실측(Z 80mm 왕복):
        #       가속    하강        상승       전류max
        #         200   5.7mm/s    5.2mm/s    2.06A   ← 낮추면 이만큼 느려진다
        #        1000  12.4       10.8        1.87
        #        3000  16.0       13.5        1.80   ← 손대기 전 수준
        #       12000  18.4       14.9        1.86   ← 여기서 포화
        #       30000  18.9       15.1        1.98
        #       60000  18.5       15.4        1.67
        #   12000 위로는 안 빨라진다. 남은 한계는 가속도 속도도 아니라 **Z 의
        #   감속비가 촘촘한 것**(0.0906mm/도, X 의 1/3)이라 기구 문제다.
        #   yaw 는 가속에 둔감하다 (12000·30000·60000 모두 2.7~3.4초, 노이즈 안).
        #   전류는 전 구간 1.0~2.4A 로 상한(11.7A)과 멀다.
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
        parameters=[{
            # ⚠ [2026-10-06] 40 → 90 초. **세 선형축 모두 전 행정이 51~52초**인데
            #   축당 타임아웃이 40초였다 — 리미트에서 멀면 도달 전에 실패한다.
            #     탐색 30dps(출력축) × mm_per_deg →
            #       X 8.72mm/s × 453.7mm = 52초
            #       Y 6.68mm/s × 349.1mm = 52초
            #       Z 2.72mm/s × 139.2mm = 51초
            #   지금까지 안 터진 건 보통 리미트 근처에서 시작했기 때문이다
            #   (2026-10-06 Z 통과도 z_min 이 이미 True 라 후퇴만 했다).
            #   런치 아래쪽 `move_speed_dps` 주석에 이 52초 계산이 이미 적혀 있었는데
            #   `stage_node` 만 30→60 으로 올리고 homing_node 는 놓쳤다.
            # 속도를 올리는 대신 타임아웃을 올린 이유: **잼 감시가 별개로 걸려 있다.**
            #   `stall_sec 4.0` / `stall_deg 0.8°` 로 축이 막히면 4초에 잡히므로
            #   타임아웃은 중복 안전망이고, 늘려도 잼 보호는 그대로다. 반대로 탐색
            #   속도를 올리면 리미트를 더 세게 지나친다 (X 는 감속이 139dps/s 라
            #   90dps 에서 8.4mm 오버슈트 — 가속을 올린 뒤에 손대야 한다).
            'axis_timeout_sec': 90.0,
        }],
        respawn=True, respawn_delay=2.0,
    )
    stage = Node(
        package='rmd_robot_control', executable='stage_node',
        name='stage_node', output='screen',
        parameters=[{
            # [2026-10-04] 30 → 60 dps. 30dps 는 출력축 8.7mm/s 라 X 전 행정
            # (453mm)에 52초다. 리모콘이 이미 100dps 로 도는 축이므로 60 은 보수적이다.
            # ⚠ [2026-10-04] yaw 60 → **400 dps**. XY 를 80mm/s 로 올리니 회전이
            #   발목을 잡았다 (건 각도로 3.2 °/s). 실측:
            #       dps   회전시간   전류max
            #        60    6.5초       —
            #       150    4.0초     3.09A
            #       250    3.2초     1.88A
            #       400    2.3초     1.87A   ← 여기서 포화
            #       600    2.3초     1.11A
            #       900    2.5~2.9초 2.06A   (더 느려진다)
            #   **가속이 한계다** — 명령 400dps 에 실측 115dps 다. 267°(자세 1↔3)를
            #   2.3초에 가는 삼각 프로파일로 역산하면 약 200 dps/s 인데,
            #   axes.yaml 의 `rom_accel: 5000` 은 [미확정]이고 적용돼 있지 않아 보인다.
            #   더 줄이려면 속도가 아니라 **모터 ROM 가감속**을 봐야 한다.
            #   토크는 문제가 아니다 (1.9A, 상한 11.7A). 도착 오차 0.04°.
            'yaw_speed_dps': 400.0,
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
            # ⚠ [2026-10-07] Z 1.0 → 0.6A. 실장비에서 **실제 접촉이 있었는데 감지가
            #   늦었다**(사용자 확인). Z 는 `down_only` 라 상승 중엔 판정하지 않고
            #   평상시 전류가 0.46A(호밍 실측)로 낮아 여유가 있다.
            'collide_rise_a_z': 0.6,
            # ⚠ [2026-10-07] 감지 지연을 줄인 두 값.
            #   전류 표본은 `0xA4` 응답으로만 갱신되고 `tick` 이 20Hz(50ms)라,
            #   연속 2회를 요구하면 **최소 100ms** 가 걸린다. Z 가 20mm/s 면 그동안
            #   2mm 더 들어가고, 거기에 기준선 EMA 가 접촉의 완만한 상승을 따라가며
            #   상승분을 깎아 실제로는 더 늦는다.
            #     rise_samples 2 → 1   : 지연 100ms → 50ms
            #     base_tau 0.6 → 0.3s : 기준선이 접촉을 덜 따라간다
            #   헛트립이 나면 rise_samples 를 2 로 되돌린다 (그게 먼저 조절할 값).
            'collide_rise_samples': 1,
            'collide_base_tau_sec': 0.3,
            # ⚠⚠ Z 는 **내려갈 때만** 판정한다. 올릴 때도 걸려서 결속점 3개가
            #   전부 실패했다 — 위에는 부딪힐 것이 없고, 건이 철근에 걸려
            #   빠져나올 때 전류가 오르는 것은 정상이다.
            # ⚠⚠ **리스트가 아니라 문자열이다.** ros2 launch 가 ['x','y'] 를
            #   YAML 수열로 쓰면 **YAML 1.1 이 `y` 를 불리언으로 읽어**
            #   `Sequence should be of same type. Value type 'bool' do not
            #   belong` 으로 노드가 아예 못 뜬다 (2026-10-04 크래시 루프).
            'collide_down_only_axes': 'z',
            # ⚠ 브레이크: 도착해도 X·Y 는 곧바로 잠그지 않는다. 다음 구간이 바로
            #   오면 해제 대기가 없어져 동작이 이어진다 — 구간마다 고정 1초를
            #   기다려서 "너무 시퀀스처럼 움직인다" 는 지적을 받았다.
            #   Z 는 넣지 않는다 (자중 낙하).
            'brake_hold_axes': 'x,y',
            'brake_hold_sec': 2.5,
            # ⚠ 이동 중 목표 **합치기**가 켜져 있다 (코드 기본). 상위가 구간
            #   완료를 매번 기다리지 않아도 되니 동작이 이어진다.
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
    # ⚠ [2026-10-04] `plan_only` 를 **끈다** (전에는 True 였다). 검출이 틀렸을 때
    #   사람이 끊는 지점을 두려 했는데, 미션 자체가 `/mission/start` 로 사람이
    #   시작하는 구조라 매 `tie` 마다 또 확인을 받는 것은 과했다 — 미션이 단계마다
    #   멈춰서 "결속을 안 한다" 로 보였다.
    #   계획만 보고 싶으면 plan_only:=true 로 띄우면 된다 (/plan/execute 로 이어서).
    planner = Node(
        package='rmd_robot_control', executable='tying_planner',
        name='tying_planner', output='screen',
        parameters=[{'plan_only': False}],
        respawn=True, respawn_delay=2.0,
    )

    # ── 작업영역 카메라 (Gemini 2L) ──────────────────────────────────────
    # ⚠⚠ **serial_number 가 없으면 안 된다.** Orbbec 305 두 대도 같은 벤더라
    #   serial 없이 띄우면 SDK 가 305 를 열고, 305 에는 이 프로파일이 없어
    #   거부한다. 그때 메시지가 "USB 2.0 으로 연결된 것 같다" 라서 대역폭 문제로
    #   오진하기 쉽다 (2026-10-04 에 그렇게 두 번 헛짚었다 — 허브는 멀쩡했다).
    # ⚠ 해상도·fps 는 USB 2.0 에서 **둘 다 흐르는** 조합이다. 모자라면 fps 가
    #   떨어지는 게 아니라 **스트림이 아예 안 열린다** (UVC 가 대역폭을 미리
    #   예약한다). depth 640x400@10 + color 1280x800@10 → 둘 다 약 9.9fps.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('orbbec_camera'),
                                  'launch', 'gemini2L.launch.py'])
        ]),
        launch_arguments={
            'serial_number': 'CPAV563008Y',
            'depth_registration': 'true',
            'depth_width': '640', 'depth_height': '400', 'depth_fps': '10',
            'color_width': '1280', 'color_height': '800', 'color_fps': '10',
            'enable_ir': 'false',
            'enable_left_ir': 'false', 'enable_right_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    return LaunchDescription(args + [
        LogInfo(msg=['리모콘 조작 구성 기동 — 안전 차단: ', use_safety,
                     ' / 호밍은 자동으로 돌지 않는다 (/homing_cmd 로 시작)']),
        motor, lateral, remote_bridge, remote_teleop, ezi_io, safety, mode_arbiter,
        homing, stage, tying, planner, camera,
    ])
