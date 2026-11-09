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
  use_gun:=true|false      결속건 트리거(Pololu) (기본 true)
      끄면 Pololu 노드가 안 뜨고 결속 시퀀스가 **발사만 건너뛴다** — 이동·자세
      변경·Z 하강/상승은 그대로 돈다. 결속건을 떼고 동작만 볼 때 쓴다.
      ⚠ 발사 방향은 **수축**이다 (2026-10-07 실측) → `gun_speed: -1.0`.
        2차년도 코드는 `+1.0` 이라 그대로 쓰면 발사와 원복이 뒤바뀐다.
  use_mission:=true|false  경로 미션(drive_node + navigator) (기본 true)
  use_camera:=true|false   작업영역 카메라(Gemini 2L) 를 같이 띄울지 (기본 true)
      yaw 자세 **별칭 판정**에 쓴다 — 1번·4번은 단회전만으로 못 가려서, X 호밍
      뒤 영상 한 장으로 후보 둘 중 하나를 고른다. 이게 있어야 사용자가 전원
      투입 때마다 yaw 를 12시로 맞추지 않아도 호밍이 돈다.
      ⚠ 카메라가 없어도 **호밍 자체는 돈다** — 비전은 별칭일 때만 부르는 보조
        수단이고, 없으면 기존 거부 경로(12시로 옮기거나 선언)로 간다.
  use_ui:=true|false       웹 UI 브릿지 (기본 true), ui_port:=8080
      노트북 브라우저에서 `http://<젯슨IP>:8080` 으로 호밍·미션·정지·상태를 본다.
      ⚠ **UI 정지를 1차 수단으로 삼지 말 것** — 가장 빠른 정지는 리모콘 비상정지다.
        UI 는 와이파이→HTTP→ROS 를 거치고, 통신이 끊기면 눌러도 안 간다.
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
        # 결속건을 쓸지. 끄면 Pololu 노드도 안 뜨고 결속 시퀀스도 발사를 건너뛴다
        # (이동·하강·상승은 그대로 돈다 — 동작 확인용으로 유용하다).
        # ⚠ [2026-10-07] 기본 **true**. 리허설 흐름이 결속까지 포함하므로 켜 둔다.
        #   Pololu 가 안 붙어 있어도 노드만 못 뜨고 **나머지는 정상 동작한다**
        #   (respawn 이 붙어 재시도한다). 결속건을 떼고 동작만 볼 때 false 로.
        DeclareLaunchArgument('use_gun', default_value='true',
                              description='결속건 트리거(Pololu) 사용'),
        # 경로 미션(주행 + 결속). 끄면 drive_node·navigator 가 안 뜬다 —
        # 상부 축만 쓰는 작업에서는 주행 노드가 CAN 을 쓰지 않게 둘 수 있다.
        DeclareLaunchArgument('use_mission', default_value='true',
                              description='경로 미션(drive_node + navigator) 기동'),
        # 웹 UI. 명령을 **중계만** 하므로 띄워 둬도 장비는 움직이지 않는다 —
        # 사람이 브라우저에서 눌러야 나간다.
        DeclareLaunchArgument('use_ui', default_value='true',
                              description='웹 UI 브릿지(HTTP) 기동'),
        DeclareLaunchArgument('ui_port', default_value='8080'),
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
            # ⚠ [2026-11-09] 336L 교체로 `yaw_pose_vision` 상수가 무효가 돼
            #   한때 이 토픽을 없는 이름으로 돌려 게이트를 꺼뒀다. 상수를 336L 로
            #   재측정해 **되살렸다** (DARK 75→60, ANG_NEG 20→65, ANG_POS -15→-30,
            #   MIN_AREA 2500→8000, rail_mask 재생성). 자세별 14장 ×4자세 검증에서
            #   판정 대상 42장 중 **틀림 0 · 거부 1**. 끄는 방법은 이 값을 없는
            #   토픽으로 바꾸는 것이다 (`_frame is None` → 점검이 None 을 반환).
            'vision_topic': '/camera/color/image_raw',
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
            # ⚠ [2026-10-07] 400 → **200**. 사용자 지시로 늦췄다.
            #   400dps 는 건 32°/s 라 10Hz 피드백의 0.1초 지연이 건 3.2° 다 —
            #   정착 재보정이 그 지연 위에서 돌면 오버슈트가 커진다. 느리면
            #   한 회차로 들어오고, 자세 전환은 전체 시간에서 작은 몫이다.
            'yaw_speed_dps': 200.0,
            # ⚠⚠ [2026-10-08] **자세 레퍼런스 추종을 재서 조였다.**
            #   순회 실측(1→2→3→4→1, 레퍼런스 대비): +0.212 / +0.483 / +0.277 /
            #   -0.051° — 전부 **진행 방향**이라 차단 오버슈트다. 그런데 정착
            #   재보정이 한 번도 안 돌았다: 판정이 `yaw_gun_tol_deg` 1.0° 였고
            #   관측 오차가 전부 그 안이었다. 1.0° 는 **건끝 11mm** 로, 캘리브레이션
            #   모델 오차(2.6mm)의 4배다 — 모델을 다듬어도 여기서 다 잃는다.
            #   0.10° = 건끝 1.1mm 로 내린다. 회차 전용 판정·속도
            #   (`yaw_fine_*`)와 함께여야 뜻이 있다 — 재보정이 `yaw_tol_deg`
            #   3.0 모터축(건 0.240°)을 물려받으면 그 아래로 못 내려간다.
            # ⚠⚠ [2026-11-06 실측] 0.10 → **0.30**. 0.10°(건끝 1.1mm)는
            #   캘리 모델 오차(2.6mm)보다 **더 아래**라 거기서 더 조여도
            #   최종 정확도가 안 좋아진다. 그런데 대가가 크다 — 관측
            #   스프링백 0.13°/0.26° 가 매번 문턱을 넘어 **5.2초짜리
            #   정착 재보정**을 불렀다.
            #   실측: yaw 회전 **7.95초 → 3.55초** (−55%),
            #         자세변경 총합 14.40 → 10.35초.
            #   0.30° = 건끝 3.2mm 로 모델 오차와 같은 자릿수다.
            'yaw_gun_tol_deg': 0.30,
            # ⚠⚠ [2026-11-05] 0.6 → **1.0**. L1(`position_control_node`)의 yaw
            #   도달 판정이 `position_tolerance` 1.0° 인데 여기가 0.6° 였다.
            #   L1 이 1.0° 에서 "도달" 이라며 손을 떼는데 L4 는 아직 아니라고
            #   보니, **같은 명령을 재전송하며 목표 주변을 왕복**했다 —
            #   실측 재보정 1회차가 2.5초를 거기에 썼다 (전체 5.2초 중).
            #     16.727 명령 현재 4.3° → 17.533 현재 -0.1°(지나침)
            #     18.749 모터 "도달 오차 1.0°" → 19.209 또 명령
            #   건끝으로 환산하면 감속비 12.59 기준
            #     모터축 0.6° = 건 0.048° = 0.53mm   (필요 이상)
            #     모터축 1.0° = 건 0.079° = 0.87mm   ← 정착 임계 0.10°(1.1mm) 안쪽
            #   전체 정밀도를 지배하는 것은 어차피 `yaw_gun_tol_deg` 0.10° 이지
            #   이 값이 아니다. 두 계층 임계를 맞춰 헛도는 시간을 없앤다.
            #   ⚠ 올리면 회전이 덜 정확해진 만큼 정착 검사에서 떨어질 여지가
            #     커진다 — **재보정 회차 수가 늘지 않는지** 다음 주행에서 볼 것.
            'yaw_fine_tol_deg': 1.0,      # 모터축 = 건 0.079° (0.87mm)
            'yaw_fine_speed_dps': 60.0,
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
            # ⚠⚠ [2026-11-09] 80 → **160**. 결속 점당 8.55초(자세 유지)로
            #   0.45초 줄었고 **전류는 안 늘었다** (X 2.58A · Y 3.20A 최대,
            #   정격 6.1A 의 절반). 실측 근거는 아래 '속도 포화' 참고.
            # ⚠ **160 이상은 의미가 없다.** 명령 160·200·250 을 모두 재봤는데
            #   300mm 이동이 전부 4.10초로 같았고, 모터 위치로 실제 속도를
            #   분리해 보면 **88mm/s 에서 포화**한다:
            #       명령  80 → 실측 67~69mm/s (이동 4.4초)
            #       명령 160 → 실측 82~89mm/s (이동 3.4~3.7초)
            #       명령 250 → 실측 88~89mm/s (이동 3.4초)   ← 더 안 빨라진다
            #   `max_speed_dps 700` 도 가속 한계도 아니다 (X 는 88mm/s 에
            #   303dps 면 된다). 모터 ROM 속도 제한이나 명령 단위 환산 배율이
            #   의심된다 — **풀면 점당 1.3초**로 오늘 시도한 어떤 것보다 크다.
            'move_speed_mm_s_x': 160.0,
            'move_speed_mm_s_y': 160.0,
            # ⚠⚠ [2026-11-09] XY 이동의 진짜 병목은 **가속**이었다. 기본값
            #   12000 에서는 명령 속도(160·250)에 **애초에 도달하지 못한다** —
            #   삼각 프로파일이라 평균 = √(a·d)/2 로 결정된다. 300mm 실측:
            #       12000 → 이동 3.45초 · 평균  86.9mm/s
            #       24000 →      2.71초 ·      110.4
            #       40000 →      2.45초 ·      122.7   ← 채택
            #       60000 →      2.29초 ·      131.5  (Y 전류가 3.82A 로 오른다)
            #   결속 점당 8.55 → 8.10초, 4점 회차 44.7 → 39.4초.
            #   전류는 정격 6.1A 대비 X 2.99A(49%) · Y 3.22A(53%) · Z 2.09A(34%)
            #   로 여유가 크다. 가장 큰 Y 3.22A 도 가속 구간이 아니라
            #   **yaw 회전 중 버티는** 몫이다.
            # ⚠⚠ **런타임 `ros2 param set` 으로는 안 바뀐다.** stage_node 의
            #   `_apply_accel` 이 기동 4초 뒤 **한 번만** 돌고 타이머를 끈다.
            #   시험하려면 `/motor_accel` 에 "0x145 40000" 형식으로 직접 보내라.
            #   (그걸 모르고 파라미터만 바꿔가며 재다가 "가속은 원인이 아니다" 라고
            #    잘못 결론 낼 뻔했다 — 12000~36000 이 전부 같은 값으로 나왔다.)
            'accel_dpss_x': 40000.0,
            'accel_dpss_y': 40000.0,
            # ⚠ Z 는 올리지 않는다. 가속을 3배 올려도 거의 안 변하고(이동 90mm 라
            #   가속 구간이 금방 끝난다), **속도를 올리면 충돌 감지가 무력화된다**:
            #   `collide_grace_max_sec 1.5` 동안 기준선을 잡느라 눈을 감는데
            #   실측 확정 시간이 0.30~1.11초다. 지금 z_down 이 1.55초라 겨우
            #   1.0초가 보호되는데, 150mm/s 로 올리면 0.90초가 되어 기준선이
            #   느리게 잡힌 회차에서는 **무장 전에 하강이 끝난다**.
            #   → 기준선 확정을 안정화하는 것이 먼저다 (오감지 원인과 같은 자리).
            # ⚠⚠ [2026-10-08 시험] 20 → **45**. 결속 깊이를 -85mm 로 내리면서
            #   왕복이 5.8s → 8.4s 로 늘어 사용자가 올려보자고 했다.
            #   ⚠ 위 10-04 실측은 **반대**를 가리킨다 — 올려도 가속 한계로 안 빨라지고
            #     (30→11.4, 60→10.3mm/s) 충돌만 세진다 (4.67A → 5.93A).
            #     그런데 오늘 관측은 20 설정에서 50mm 를 3초(≈17mm/s)로, 10-04 기록과
            #     맞지 않는다 (하중·행정이 달랐을 것). 그래서 **재서 판단한다.**
            #   되돌리는 기준 둘 중 하나라도 해당하면 20 으로 복귀:
            #     ① 실제 mm/s 가 안 늘어난다 (가속 한계 확인)
            #     ② 하강 중 Z 전류가 올라 `collide_rise_a_z`(0.6A) 여유가 깎인다
            #   ⚠ Z 는 철근을 향해 내려가는 축이고 `down_only` 로 하강만 판정한다 —
            #     세게 박으면 트립은 똑같이 걸리고 건·철근에 가는 힘만 커진다.
            # ⚠⚠⚠ [2026-10-16 미검증 위험] **Z 속도는 충돌 유예와 경쟁한다.**
            #   감지는 축별로 `arm_sec`(1.0s) + `collide_grace_sec`(0.7s) = **1.7초**
            #   동안 아예 판정하지 않는다 (가속 전류를 피하려고). 그 사이에 Z 가
            #   접촉 깊이까지 가 버리면 **감지가 무장되기 전에 박는다.**
            #     설정 30mm/s 이던 2026-10-04: 접촉(-62~-69mm)까지 약 2.1초
            #       → 유예가 먼저 끝나 실제로 -63mm 에서 3.10A 로 잡혔다
            #     설정 45mm/s 인 지금: 명목 1.38초 → **유예 안에 접촉한다**
            #   다만 Z 는 실측이 설정보다 한참 낮다(설정 30 → 실측 11.4mm/s,
            #   설정 20 → 약 17mm/s). 그래서 실제로는 유예가 먼저 끝날 것으로 본다.
            #   **재 보지 않았다.** 재는 법: Z 를 -30mm(접촉보다 30mm 위)까지만
            #   내리며 전류와 시간을 찍어, ① 가속 전류 피크 ② 1.7초 동안 내려간 mm
            #   를 본다. 둘 중 하나라도 나쁘면 속도를 내리거나 유예를 줄여야 한다.
            # ⚠⚠ [2026-11-06 실측] Z 45 → **60**, 상한 450 → **700**.
            #   450dps 는 Z 를 40.8mm/s 로 잘랐다 (설정 45 가 안 나왔다).
            #   100mm 무접촉 하강: 2.91초/34.4mm/s/0.71A → **2.35초/42.5mm/s/1.08A**
            #   정격 7.8A(X4-10) 대비 여유 **7배**. 결속 1점에 Z 왕복이
            #   180mm 들어가므로 점당 약 1초, 6점이면 6초를 번다.
            # ⚠ 상한은 **전 축 공통**이다. X·Y 는 설정 80mm/s 가 각자 상한
            #   (130.8 / 100.2mm/s) 아래라 영향받지 않는다 — 바뀌는 건 Z 뿐.
            'move_speed_mm_s_z': 60.0,
            'max_speed_dps': 700.0,
            # ── 충돌 감지 ────────────────────────────────────────────────
            # 교차점이 아닌 곳에서 Z 를 내리면 건이 철근을 찍는다. 작업영역
            # 검사(X·Y)로는 막을 수 없다 — 작업영역 안이어도 그 자리에 철근이 있다.
            # ⚠⚠ **문턱은 축마다 다르다.** 실측 정상 이동 전류(30mm/s):
            #   X 최대 3.86A · Y 2.92A · Z 1.88A. Z 충돌 때는 3.1~3.9A 가 떠서
            #   정상의 두 배다. Z 기준 3.0A 를 세 축에 같이 쓰니 X 가 헛걸렸다.
            # 실장비 검증(2026-10-04): Z -63mm 에서 3.10A 감지 → 이동 취소 →
            #   -47.9mm 로 후퇴, 거부수 +1 로 상위가 그 점을 건너뛴다.
            'collide_detect': True,
            # ⚠⚠ [2026-10-07 사용자 결정] **감지는 Z 만.** 수평면은 자세별
            #   작업영역 강제가 막고, X 문턱은 X4-36 교체로 감속비가 2.88배가
            #   되어 근거를 잃었다. 틀린 문턱은 보호가 아니라 헛트립이고 결속점을
            #   건너뛴다. X 의 진짜 위험은 충돌이 아니라 지속 과전류다(10모터가
            #   죽은 원인으로 의심되는 쪽) — 그건 전류 감시 몫이다 [미구현].
            'collide_axes': 'z',
            # ↓ X·Y 값은 `collide_axes` 에 넣지 않는 한 **쓰이지 않는다.**
            #   다시 켜려면 최종 속도·가감속에서 정상 전류를 재서 넣을 것.
            'collide_current_a_x': 4.8,   # [무효 — X4-10 기준]
            'collide_current_a_y': 3.8,
            'collide_current_a_z': 2.5,
            'collide_backoff_mm': 15.0,
            # ⚠⚠ **상승분으로 본다.** 절대 문턱만 쓰면 늦다 — 접촉 후 전류가
            #   1.19→2.03A 로 오르는 데 250ms, 거기서 문턱을 3표본 넘는 데
            #   100ms 가 더 걸렸다. 상승분을 넣으니 2.77A → **1.39A** 에서 끊었다.
            #   문턱은 실측 정상 변동폭 위여야 한다 (Z 하강 0.9A · X 0.8A).
            'collide_rise_a_x': 1.5,      # [무효 — X4-10 기준]
            'collide_rise_a_y': 1.2,      # [무효]
            # ⚠ [2026-10-07] Z 1.0 → 0.6A. 실장비에서 **실제 접촉이 있었는데 감지가
            #   늦었다**(사용자 확인). Z 는 `down_only` 라 상승 중엔 판정하지 않고
            #   평상시 전류가 0.46A(호밍 실측)로 낮아 여유가 있다.
            # ⚠ [2026-10-15 사용자 요청] 0.6 → **0.45A**. "조금 더 일찍 감지"
            #   ⚠⚠ 여유가 넉넉하지 않다 — 위에 적힌 실측 정상 변동폭이 **Z 하강 0.9A**
            #     (평균 0.4 · 최대 1.3A)인데 0.6 도 이미 그 밑이었다. 지금 안 헛걸리는
            #     것은 기준선 EMA 가 받쳐주기 때문이고, 0.45 는 그 여유를 더 깎는다.
            #   **헛트립이 한 번이라도 나면 0.6 으로 되돌린다** (그게 먼저 되돌릴 값).
            #   이 값은 `_begin()` 이 이동마다 다시 읽으므로 재시작 없이
            #   `ros2 param set /stage_node collide_rise_a_z <값>` 으로 즉시 바뀐다.
            # ⚠⚠ [2026-11-06 실측] 0.45 → **0.8A**. 예고된 헛트립이 실제로 났다 —
            #   결속 4점 중 1점이 배근에 닿기도 전(z -50.6mm, 목표 -85.9)에 중단됐고,
            #   그때 전류가 기준선 0.34 → 0.79A 로 **정확히 +0.45A** 였다. 같은 회차의
            #   진짜 충돌도 +0.45A 라 **둘을 구분할 수 없는 값**이었다.
            #   CAN 으로 Z(0x247) 응답을 디코드해 실측한 분포가 근거다:
            #       정상 하강 0.3~0.8A   ·   실제 충돌 1.5~2.1A
            #   +0.8 이면 발동점이 약 1.15A 로 **그 사이 한가운데**다. 바꾼 뒤
            #   4점 중 3점이 목표 깊이 -85.9mm 에 도달(결속 성공)했고, 남은 1점은
            #   1.44A 로 걸린 **진짜 충돌**이었다.
            # ⚠⚠ [2026-11-09] **0.8 로 올린 것을 0.45 로 되돌린다.** 위 11-06 기록은
            #   틀린 처방이었다 — 그날 헛트립의 원인은 임계가 아니라 **결속 깊이**였다.
            #   상수 -96.0 이 실제보다 10mm 깊어 공구가 배근을 눌렀고 그래서 전류가
            #   올랐다. 깊이를 자세별 실측값으로 고치자(11-06/11-09) **임계 0.45 그대로**
            #   5점 중 헛트립이 1건뿐이고 결속 4/5 가 됐다.
            #   그리고 0.8 은 **위험하다**: 2026-11-09 의 실제 충돌이 +0.74A(기준선
            #   0.49 → 1.23A)였다 — 0.8 이었으면 **못 잡았다.** 11-06 에 근거로 삼은
            #   '진짜 충돌 1.5~2.1A' 는 깊이가 틀려 세게 눌렸을 때의 값이라, 깊이를
            #   고친 뒤에는 그만큼 안 오른다.
            #   ⚠ 참고: 0.8 은 **실제로 적용된 적이 없다.** 런치를 고치고 빌드를 안 해
            #     install 공간이 0.45 로 남아 있었다 (소스 18:37 / 설치본 15:43).
            #     런치는 install 에서 읽히므로 고친 뒤 반드시 colcon build 를 해야 한다.
            'collide_rise_a_z': 0.45,
            # ⚠ [2026-10-07] 감지 지연을 줄인 두 값.
            #   전류 표본은 `0xA4` 응답으로만 갱신되고 `tick` 이 20Hz(50ms)라,
            #   연속 2회를 요구하면 **최소 100ms** 가 걸린다. Z 가 20mm/s 면 그동안
            #   2mm 더 들어가고, 거기에 기준선 EMA 가 접촉의 완만한 상승을 따라가며
            #   상승분을 깎아 실제로는 더 늦는다.
            #     rise_samples 2 → 1   : 지연 100ms → 50ms
            #     base_tau 0.6 → 0.3s : 기준선이 접촉을 덜 따라간다
            #   ⚠⚠ **[2026-10-15 정정] 위 base_tau 설명은 방향이 반대다.**
            #     코드는 `k = min(1, dt/tau)` 로 기준선을 갱신한다 — tau 가 **짧으면**
            #     k 가 커져 기준선이 전류를 **더 빨리 따라간다**. 즉 0.6 → 0.3 은
            #     접촉 초기(상승분 절반 미만, 즉 +0.3A 까지)의 완만한 상승을 기준선이
            #     더 많이 흡수해 **감지를 늦추는** 쪽으로 작용한다.
            #     일찍 감지하려면 tau 를 **올려야**(0.6~1.0) 한다. 대가는 가속 전류를
            #     오래 물고 있어 헛트립이 늘 수 있다는 것.
            #     ⚠ 이 값은 기동 때만 읽는다(`__init__`) — `ros2 param set` 이 안 먹고
            #       재시작이 필요하다. 그래서 시연 전에는 건드리지 않았다.
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
            # ⚠⚠ [2026-11-06 실측] **yaw 도 유예 대상이다.** 빼 두었더니 자세변경마다
            #   잠그고 다시 푸는 왕복에 **3.5초**가 갔다 (yaw CAN 실측: 실제 회전
            #   0.82초 + 재보정 0.62초 = 1.44초인데 전체 5.1초). yaw 는 중력축이
            #   아니고 서보가 켜져 있으면(0x80 을 보내지 않는다) 위치 루프가 잡는다.
            #   Z 는 그대로 제외한다 — 중력축이라 유예 중 자중으로 내려간다.
            #   유예는 결속 한 주기(약 11초)를 덮어야 효과가 있다.
            'brake_hold_axes': 'x,y,yaw',
            'brake_hold_sec': 12.0,
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
    # ── 결속건 트리거 (Pololu Simple Motor Controller 18v7) ──────────────
    # ⚠ 전에는 `base_system.launch.py` 에만 있어 **수동으로 띄워야** 했다. 그러면
    #   상부 전원이 내려갈 때 USB 가 빠져 노드가 죽고 **프로세스 껍데기만 남는다**
    #   — 2026-10-07 에 실제로 그랬고, 조용히 죽어 있어서 결속만 안 되고 원인을
    #   찾기 어려웠다. 카메라와 같은 현상이라 같이 올린다.
    # ⚠ 포트는 udev 심링크(`/dev/pololu_trigger`)를 쓴다. `ttyACM*` 번호는 꽂는
    #   순서에 따라 바뀐다.
    pololu = Node(
        package='pololu_ros2', executable='pololu_node',
        name='pololu_node', output='screen',
        parameters=[{
            'serial_port': '/dev/pololu_trigger',
            'baudrate': 9600,
            'motor_ids': [0],
            'motor_topics': ['motor_0/vel'],
        }],
        condition=IfCondition(LaunchConfiguration('use_gun')),
        respawn=True, respawn_delay=3.0,
    )

    tying = Node(
        package='rmd_robot_control', executable='tying_sequence',
        name='tying_sequence', output='screen',
        parameters=[{
            # ⚠ `LaunchConfiguration` 을 그대로 넣으면 **문자열 'false' 가 전달**되고
            #   파이썬에서 빈 문자열이 아니므로 **참으로 평가된다** — 껐는데 켜진다.
            #   저장소의 기존 방식(`safety_timeout`)대로 명시 변환한다.
            'gun_enabled': PythonExpression(
                ["'", LaunchConfiguration('use_gun'), "' == 'true'"]),
            # ⚠⚠ **음수다.** 2026-10-07 실장비 확인: **수축 = 트리거 당김(발사)**,
            #   팽창 = 놓음. `tying_sequence` 는 `gun_speed` 로 발사하고
            #   `-gun_speed` 로 원복하므로 부호가 반대면 **발사와 원복이 뒤바뀐다.**
            #   2차년도 코드(`trigger_speed: +1.0`)는 당시 배선 기준이라 지금과 반대다.
            'gun_speed': -1.0,
            # 2차년도 검증값과 같다. ⚠ 실제 스트로크가 짧아 트리거가 안 걸리는
            #   문제는 **미해결**이다 (기구 쪽). 시간을 늘려도 끝(하드스토퍼)에
            #   닿으면 더 안 간다 — 3초·5회 끊어보기 전부 같은 거리였다.
            # ⚠⚠ [2026-11-06 실측] 1.0 → **0.35**. 1.0 은 측정이 아니라
            #   넉넉히 잡은 값이었고, 건 1회 2.4초가 결속 1점(12.2초)의
            #   **20%** 를 먹었다. 당김 시간을 줄여가며 공중에서 작동시켜
            #   **0.25초에도 스트로크가 끝까지 간다**는 것을 확인했다
            #   (1.00/0.70/0.50/0.35/0.25 전부 완주, 사용자 육안 판정).
            #   0.35 는 그 검증값에 여유 1.4배를 얹은 것이다.
            #   건 1회 = 0.35+0.2+0.35+0.2 = **1.10초** (2.40초에서).
            # ⚠ 육안 판정뿐이다 — SMC 18v7 은 전류 센싱이 없고
            #   `pololu_node` 는 발행 토픽이 없어 **스트로크 완료를
            #   소프트웨어가 알 방법이 없다.** 더 줄이려면 다시 눈으로 재라.
            'gun_fire_sec': 0.35,
            'gun_return_sec': 0.35,
            'z_safe_mm': 0.0,
            # ⚠⚠ [2026-10-12] **40 → 20mm.** XY 가 목표에서 이만큼 안으로 들면 Z
            #   하강을 미리 보낸다(`_do_move_xy`). 판정이 `hypot` 이라 **방향을 안
            #   본다** — 거의 순수 Y 이동이면 남은 40mm 가 사실상 전부 Y 라서,
            #   Y 가 40mm 남은 시점에 건이 내려가기 시작해 **깊은 데서 Y 로 쓸린다.**
            #   실측 2026-10-12: (78.1, 178.5)→(92.0, 264.6) (X 13.9 / Y 86.1mm) 점에서
            #   `Z_DOWN→FIRE` 가 0.50s 였다 — 다른 점은 1.60/1.70/3.65s 였으니
            #   XY 도착 시점에 Z 가 이미 거의 다 내려가 있었다는 뜻이다.
            #   ⚠ 40 은 Z 가 20mm/s 일 때 잡은 값이다. 오늘 Z 를 45mm/s 로 올려
            #     같은 40mm 구간에서 Z 가 두 배 넘게 내려간다.
            #   20mm 면 XY 80mm/s 로 0.25s, 그 사이 Z 는 11mm 만 내려간다
            #   (건 약 -23mm — 철근 접촉 깊이 -62~-69mm 보다 한참 위다).
            #   ⚠ `self.blend` 는 **기동 때 한 번만 읽는다** — `ros2 param set` 으로는
            #     안 먹고 서비스 재시작이 필요하다.
            'xy_blend_mm': 20.0,
            # 모델 잔차가 Z 2.7mm 다. 실측 결속깊이는 -58~-83mm 였으니
            # 그 바깥은 모델이 틀린 것으로 보고 거부한다.
            # ⚠ [2026-10-17] -95 → **-115**. 바닥이 데크플레이트에서 **RC** 로
            #   바뀌면서 철근 상면이 27mm 낮아졌다 — 실측 결속 깊이 -96.1mm
            #   (리모콘으로 내려 확인). -95 로는 그 값이 **거부**된다
            #   (자르는 게 아니라 거부한다). 여유를 두어 -115 로 넓힌다.
            #   ⚠ 이 범위는 "모델 예측이 터무니없으면 막는" 안전망이다 —
            #     바닥이 또 바뀌면 결속 깊이와 **함께** 다시 잡을 것.
            'z_tie_min_mm': -115.0,
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
        parameters=[{
            'plan_only': False,
            # ⚠ [사용자 지시 2026-10-12] **0.5 → 0.6.** 계획에 쓸 교차점의 최소
            #   신뢰도다 (`tying_planner` 가 `/rebar/crossings` 를 걸러낸다).
            #   모델 임계(`crossing_detector` 의 `threshold` 0.3)는 그대로 둔다 —
            #   낮은 것도 발행은 하게 두고 **쓸지 말지는 여기서** 정하는 구조다.
            #   왜 올리나: 오검출 점으로 내려가면 철근 위를 눌러 "얕고 센" 충돌이
            #   난다 (2026-10-12 31건 중 5건이 그 종류였고 한 번은 리모콘
            #   리시버까지 죽였다). 자세·XY 오차와 달리 이건 점을 **안 고르면**
            #   사라진다.
            #   ⚠ 미션 시작 때마다 다시 읽는다 — `ros2 param set` 으로 바로 먹고
            #     서비스 재시작이 필요 없다 (`_start()` 에서 다시 읽는다).
            'min_confidence': 0.6,
        }],
        respawn=True, respawn_delay=2.0,
    )

    # ── 작업영역 카메라 (Gemini 336L) ────────────────────────────────────
    # ⚠⚠ [2026-11-09] **Gemini 2L → 336L 로 교체했다.** 336L 은 액티브+패시브
    #   스테레오에 IR 패스 필터가 있어 밝은 환경·반사면에서 깊이가 안정적이고,
    #   2L 로는 잘 안 잡히던 얇은 배근이 보인다고 한다(사용자).
    #   교체하며 **serial_number 를 같이 바꿔야 한다** — 안 바꾸면 드라이버가
    #   그 시리얼 장치를 못 찾아 **아무 스트림도 안 열고 조용히 끝난다.**
    #   실제로 그랬다: USB 에는 336L(2bc5:0807)이 보이고 토픽도 떠 있는데
    #   발행이 0 이었다. 2L 백업: data/calibration/gemini2l_20261109/
    #     2L  CPAV563008Y (2bc5:0673)
    #     336L CPC8563000RF (2bc5:0807)
    #   ⚠ 아래 해상도·fps 는 **2L 에서 잡은 값**이다. 336L 이 같은 조합을
    #     지원하는지 확인되지 않았다 — 안 열리면 드라이버 로그의 지원 목록을 볼 것.
    # ⚠⚠ **serial_number 가 없으면 안 된다.** Orbbec 305 두 대도 같은 벤더라
    #   serial 없이 띄우면 SDK 가 305 를 열고, 305 에는 이 프로파일이 없어
    #   거부한다. 그때 메시지가 "USB 2.0 으로 연결된 것 같다" 라서 대역폭 문제로
    #   오진하기 쉽다 (2026-10-04 에 그렇게 두 번 헛짚었다 — 허브는 멀쩡했다).
    # ⚠ 해상도·fps 는 USB 2.0 에서 **둘 다 흐르는** 조합이다. 모자라면 fps 가
    #   떨어지는 게 아니라 **스트림이 아예 안 열린다** (UVC 가 대역폭을 미리
    #   예약한다). depth 640x400@10 + color 1280x720@10 MJPG 조합이다.
    # ⚠ 인자는 **여기 한 곳에만** 둔다 — 아래 워치독의 복구 명령을 이 dict 로
    #   만들기 때문이다. 두 군데에 적으면 되살릴 때 다른 설정으로 뜬다.
    CAM_ARGS = {
        'serial_number': 'CPC8563000RF',
        'depth_registration': 'true',
        # ⚠ 336L + **USB 3.0** 조합이다 (2026-11-09). 아래 이력을 꼭 읽어라.
        #   1) 2L 값(color 1280x800 / depth 640x400)을 그대로 쓰면 336L 이
        #      USB 2.0 에 물려 있을 때 1280x800 프로파일이 아예 없어서
        #      "Failed to configure the requested stream profile" 로 죽는다.
        #   2) 그래서 1280x720 **MJPG** 로 내렸더니 뜨기는 하는데, 2~3 프레임
        #      묶음 뒤 **정확히 3.75초 공백**이 반복돼 실효 0.4fps 였다.
        #      USB 2.0 에서도, 3.0 으로 옮긴 뒤에도 같은 주기였다 — 즉
        #      대역폭이 아니라 **MJPG 디코드**가 범인이다. format 강제 금지.
        #   3) USB 3.0 (2-3.2.x, 5Gbps) 에서는 1280x800 네이티브가 살아난다.
        #      depth 도 640x400 → 1280x800 로 올린다: ~500mm 에서 깊이 1픽셀
        #      풋프린트가 1mm → 0.5mm 가 되어 지름 10~13mm 철근을 10여 px
        #      대신 20여 px 로 덮는다. 층 구분(진짜/가짜 교차점)이 여기 걸려 있다.
        #   4) color 1280x800 은 RF-DETR 학습 데이터(2L, 16:10)와 종횡비가
        #      같다. 1280x720 은 화각이 잘려 모델에 불리하다.
        # ⚠⚠ [2026-11-09] **336L 은 305 두 대와 같은 허브 가지에 두지 마라.**
        #   USB3 로 옮긴 뒤 스트림이 열리고 **0.4초** 만에 장치가 버스에서
        #   사라지고 3.7~4.0초 주기로 재열거가 반복됐다. 배제한 것들:
        #     · 자동절전 — 재열거마다 control=on·runtime_suspended_time=0 (한 번도 안 잠)
        #     · 발열 — 본체가 차가웠다
        #     · 호스트 설정 — 런치를 정상 동작하던 커밋과 동일하게 되돌려도 그대로
        #     · 허브·포트 공통 문제 — 같은 허브의 305 두 대는 끊김 0건
        #   결정적 근거는 커널 로그의 **27분 공백(13:46~14:13)** 이다. 그 구간에만
        #   끊김이 0 이었고 8.3fps 가 나왔는데, 그때 **305 두 대가 죽어 있었다.**
        #   305 가 되살아난 14:13 부터 다시 끊기기 시작했다.
        #   세 대가 선언한 소비전류가 각 896mA(합 2.7A)이고 336L 은 레이저까지
        #   켜는 가장 큰 소비자다. "스트림 0.4초 뒤" 가 레이저 점등 타이밍이다.
        #   self-powered 허브라도 총 전류는 유한하다 — 그걸 근거로 전원을
        #   배제했던 것이 오진이었다.
        'depth_width': '1280', 'depth_height': '800', 'depth_fps': '10',
        'color_width': '1280', 'color_height': '800', 'color_fps': '10',
        #   5) ⚠ align_mode 는 **건드리지 마라** (기본값 SW 가 맞다).
        #      HW 로 주면 "Current stream profile is not support hardware d2c
        #      process" 로 파이프라인이 아예 안 뜬다 — 336L 의 HW D2C 표에
        #      1280x800+1280x800 조합이 없다.
        #   6) ⚠⚠ **이 해상도는 `usbfs_memory_mb` 를 올려놔야 흐른다.**
        #      커널 기본값 16MB 로는 libuvc 가 전송 버퍼를 못 잡아서
        #      "2~4 프레임 묶음 → 3.7초 공백" 이 반복되고 실효 0.3fps 가 된다.
        #      USB 2.0→3.0, MJPG→raw, 720→800 을 다 바꿔도 주기가 안 변했고,
        #      305 두 대만 멀쩡했던 건 먼저 떠서 버퍼를 선점했기 때문이다.
        #      확인:  cat /sys/module/usbcore/parameters/usbfs_memory_mb   → 1000
        #      (대역폭이 아니라 **호스트 메모리** 문제다. 전에 "USB 2.0 천장"
        #       으로 진단했던 2L depth 0fps 도 실은 이것일 수 있다.)
        # ⚠⚠ **자동노출 측광 영역을 데크 중앙으로 좁힌다** (2026-11-09).
        #   336L 은 2L 보다 화각이 넓어 화면 좌우 끝에 **데크 밖 바닥**이 들어온다.
        #   그 바닥이 데크보다 2.5배 밝아서(좌끝 101 vs 데크 40) AE 가 거기에
        #   맞추고, 정작 배근 데크가 깔린다 — 데크 중앙 밝기가 28~40 까지 떨어져
        #   자세 판별용 건 실루엣이 아예 안 보였다 (1번·4번 화소차 p90 5.6).
        #   노출을 **고정**하면 해결되지만(실측: color_exposure 50 → 데크 106.8,
        #   포화 0.0%) 현장은 야외·그늘이 섞이므로 AE 를 끄면 안 된다. 그래서
        #   AE 는 켜둔 채 **측광 영역만** 데크로 제한한다.
        # ⚠ 런타임 `ros2 param set` 으로는 안 먹는다 — 기동 때만 장치에 내려간다.
        #   (그렇게 시도했다가 색 노출이 3000 에 묶여 0.3fps 가 됐고 재시작으로만
        #    풀렸다. 노출 파라미터를 런타임에 만지지 말 것.)
        # ⚠ [2026-11-09 보류] color_ae_roi_* 는 **일단 빼둔다.** 이 값을 런치에
        #   넣고 재시작한 뒤부터 336L 이 **3.7~4.0초 주기로 USB 재열거**를 반복해
        #   0.4fps 가 됐다 (같은 허브의 305 두 대는 끊김 0건, 허브는 self-powered
        #   이므로 전원·허브 문제가 아니다). 드라이버가 기동 때 ROI 를 장치에
        #   내리는데 이 센서가 그 값을 안 받으면 초기화 실패 → 리셋 → 재열거가
        #   된다. 원인 확정 전까지는 넣지 않는다.
        #   다시 시도할 때는 값을 센서가 받는 범위로 확인한 뒤 **한 번에 하나만**.
        # 'color_ae_roi_left': '300', 'color_ae_roi_right': '1000',
        # 'color_ae_roi_top': '150', 'color_ae_roi_bottom': '700',
        # ⚠⚠ **AE 노출 상한을 푼다.** 기본값(-1)에서는 AE 가 어두워져도 노출을
        #   못 올린다 — 2026-11-09 실측: 데크에 직사광이 들던 14:20 과 사라진
        #   14:31 사이에 **모든 구역이 똑같이 3.9배** 어두워졌다. 장면만 바뀐 게
        #   아니라 AE 가 전혀 보상하지 않은 것이다. 수동으로 color_exposure 50 만
        #   줘도 데크가 106.8 이 되니, AE 는 50 보다 낮은 값에 묶여 있었다.
        # ⚠ 너무 크게 잡으면 fps 가 죽는다 (3000 에서 0.3fps 가 됐다).
        #   300 이면 지금 조도에서 데크 ~200 이라 4배 어두워져도 따라간다.
        # 'color_ae_max_exposure': '300',   ← [2026-11-09] 끊김 원인 배제를 위해
        #   잠시 뺐다. 밝기 문제(AE 가 노출을 못 올려 데크가 28까지 깔림)의 해법은
        #   이것이 맞으니, USB 재열거가 잡히면 **다시 넣고 한 번에 하나만** 본다.
        'enable_ir': 'false',
        'enable_left_ir': 'false', 'enable_right_ir': 'false',
        'enable_point_cloud': 'false',
        'enable_colored_point_cloud': 'false',
    }
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('orbbec_camera'),
                                  'launch', 'gemini_330_series.launch.py'])
        ]),
        launch_arguments=CAM_ARGS.items(),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    # ── 카메라 워치독 ────────────────────────────────────────────────────
    # ⚠⚠ [2026-10-17] **카메라는 죽지 않고 살아서 멈춘다.** 서비스 재시작 때
    #   USB 가 재열거되는 와중에 노드가 뜨면 SDK 가 그때의 장치 목록을 들고
    #   `Device ... not found` 를 5.5초마다 영원히 찍는다. 크래시가 아니라
    #   **`respawn` 으로 안 잡힌다.** 재시작 3번 중 1번 걸렸다.
    #   상류 `gemini2L.launch.py` 는 respawn 인자를 받지 않고 단독 노드 경로는
    #   Foxy 전용이라, 이 워치독이 컨테이너를 죽이고 다시 띄우는 수밖에 없다.
    #   현장에서 USB 를 뽑을 수는 없으니 자동이어야 한다.
    cam_watch = Node(
        package='rebar_vision', executable='camera_watchdog',
        name='camera_watchdog', output='screen',
        parameters=[{
            'topic': '/camera/color/image_raw',
            'grace_sec': 75.0,      # 카메라 기동에 30초쯤 걸린다
            'stale_sec': 20.0,      # 정상 10fps — 20초 없으면 멈춘 것
            'cooldown_sec': 120.0,
            'max_restarts': 3,      # 넘으면 포기하고 크게 알린다 (물리 문제)
            # ⚠⚠ [2026-11-06] **`__ns:=/camera` 를 반드시 같이 준다.**
            #   측면 305 컨테이너의 명령줄에도 `__node:=camera_container`
            #   토큰이 그대로 있다 — 노드 이름만으로는 **안 좁다.**
            #   2026-11-05 에 노드 기본값은 고쳤는데 여기가 옛 값으로
            #   덮어써서 수정이 **운용에 안 걸려 있었다.** 지금 컨테이너가
            #   셋(교차점·좌·우)이라, 2L 스트림이 20초 끊기면 워치독이
            #   **셋 다 죽인다.** 워치독은 교차점만 되살리므로 측면은
            #   영구 정지다.
            'match': '__node:=camera_container __ns:=/camera',
            'usb_serials': CAM_ARGS.get('serial_number', ''),
            'relaunch': 'ros2 launch orbbec_camera gemini_330_series.launch.py '
                        + ' '.join(f'{k}:={v}' for k, v in CAM_ARGS.items()),
            # ⚠ 되살리기 **전에** USB 권한을 고친다 — udev 가 빠른 재열거에서
            #   이벤트를 흘리면 노드가 0600 root:root 로 남아 SDK 가 못 연다.
            #   이 한 줄만 sudoers 에 열려 있다 (tools/sudoers/rebar-udev-retrigger).
            'pre_cmd': 'sudo -n /usr/bin/udevadm trigger '
                       '--subsystem-match=usb --attr-match=idVendor=2bc5',
        }],
        respawn=True, respawn_delay=5.0,
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    # ── 측면 305 워치독 ─────────────────────────────────────────────────
    # ⚠⚠ [2026-11-06] 측면 305 는 `orbbec_side.launch.py` 로 **서비스 밖에서**
    #   떠 있었다. 그래서 ① 전원을 넣을 때마다 손으로 띄워야 했고 ② 죽어도
    #   아무도 몰랐다. 오늘 우측이 한 시간 넘게 죽은 채였는데 `deck_edge` 가
    #   얼어붙은 프레임으로 "좌측 열림" 을 계속 발행했다.
    #
    # 왜 쌍으로 다루나 — `orbbec_side.launch.py` 가 좌·우를 **한 런치에서**
    #   띄운다. 한쪽만 죽이고 그 런치를 다시 돌리면 다른 쪽이 두 개가 된다.
    #   그래서 토픽 둘을 함께 보고(어느 하나라도 끊기면 발동) 둘을 함께
    #   죽인 뒤 런치를 다시 돌린다.
    #
    # 왜 `exclude` 가 필요한가 — 좌·우를 함께 고를 공통 토큰이
    #   `__node:=camera_container` 뿐이고 그건 **2L 에도 있다.** 그래서
    #   "컨테이너 전부 중 2L 만 제외" 로 고른다. 이게 없으면 측면을
    #   되살리다가 **2L 을 죽이고**, 그 재열거 반복이 USB PHY 를 래치시킨다
    #   (2026-11-05 에 좌측 305 를 그렇게 잃고 전원 재투입으로야 살렸다).
    #   실측 검증: match 만 → 3개(2L 포함) / exclude 포함 → 2개(측면만).
    #
    # 유예 75초의 근거 — 전원 재투입 뒤 실측하면 열거는 즉시 되는데 **스트림이
    #   106초 동안 안 열렸다.** 우측이 제 발로 재열거(+114.5s)하고서야 양쪽이
    #   풀렸다. 초기화 자체는 2.5~7.5초다. 즉 기다릴 게 아니라 **재열거를
    #   재촉**해야 한다 — 그게 이 워치독이 하는 일이다.
    cam_watch_side = Node(
        package='rebar_vision', executable='camera_watchdog',
        name='camera_watchdog_side', output='screen',
        parameters=[{
            'topic': '/camera_left/color/image_raw '
                     '/camera_right/color/image_raw',
            'grace_sec': 75.0,
            'stale_sec': 20.0,
            'cooldown_sec': 120.0,
            'max_restarts': 3,
            'match': '__node:=camera_container',
            'exclude': '__ns:=/camera',      # ← 2L 을 절대 건드리지 않는다
            # 버스에 없으면 재시작을 건너뛰고 크게 알린다 (전원 재투입 필요).
            #   시리얼은 `orbbec_side.launch.py` 의 LEFT_SN/RIGHT_SN 과 같다.
            'usb_serials': 'CV2L3600003P CV2856D000D0',
            'relaunch': 'ros2 launch rebar_vision orbbec_side.launch.py',
            'pre_cmd': 'sudo -n /usr/bin/udevadm trigger '
                       '--subsystem-match=usb --attr-match=idVendor=2bc5',
            'status_topic': '/camera/watchdog_side',
        }],
        respawn=True, respawn_delay=5.0,
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    # ── 데크끝·주행가능 판정 (L2) ────────────────────────────────────────
    # ⚠⚠ [2026-11-06] 이것도 서비스 **밖에서** `ros2 run` 으로 떠 있었다.
    #   자율 고리는 `/deck_edge/status` 가 없으면 15초 뒤 멈춘다 — 즉 판정
    #   노드가 죽으면 주행이 그 자리에서 끝난다. 측면 305 와 **같은 종류의
    #   구멍**이었다 (서비스 밖 = 아무도 안 지킨다 = 죽어도 모른다).
    #
    # ⚠ 파라미터를 주지 않는다. 기본값이 곧 현장값이고, 런치가 값을 따로
    #   들고 있으면 노드 기본값을 고쳐도 **운용에 안 걸린다** — 오늘
    #   `camera_watchdog` 의 `match` 가 정확히 그렇게 안 걸려 있었다.
    deck_edge = Node(
        package='rebar_vision', executable='deck_edge',
        name='deck_edge_node', output='screen',
        respawn=True, respawn_delay=5.0,
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    # ── 주행 거리 제어 ───────────────────────────────────────────────────
    # `/drive/step` 으로 mm 단위 전후진. 바퀴 각도로 닫힌 루프를 돈다.
    # ⚠ 전진은 **우측 +, 좌측 −** 다 (모터가 마주 보게 달렸다). 직진 보정 부호는
    #   **방향에 따라 뒤집으면 안 된다** — 2026-10-04 에 후진에서 뒤집었다가
    #   양의 되먹임이 되어 장비가 돌았고 사용자가 비상정지했다.
    drive = Node(
        package='rmd_robot_control', executable='drive_node',
        name='drive_node', output='screen',
        condition=IfCondition(LaunchConfiguration('use_mission')),
        respawn=True, respawn_delay=2.0,
    )

    # ── 경로 미션 ────────────────────────────────────────────────────────
    # `path_file` 을 읽어 `drive`/`tie`/`lateral` 단계로 풀고 순서대로 돌린다.
    # 시작은 `/mission/start`. ⚠ 비워 두면 경로를 안 읽고 `mission` 파라미터
    # (기본 `['tie']`)만 돈다 — 리허설에서 "주행을 안 한다" 로 보인다.
    # ⚠ `rmd_share` 는 `<ws>/install/rmd_robot_control/share/rmd_robot_control`
    #   이라 **4단계** 올라가야 워크스페이스 뿌리다. 3단계만 올라갔다가
    #   `install/src/...` 가 나와 파일을 못 찾았다 — 그러면 navigator 가 조용히
    #   `mission` 기본값(`['tie']`)만 돌아 **"주행을 안 한다"** 로 보인다.
    _paths = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(rmd_share)))),
        'src', 'rebar_control', 'path_generation')
    navigator = Node(
        package='rmd_robot_control', executable='navigator',
        name='navigator', output='screen',
        parameters=[{
            # ⚠ `rmd_share` 는 `<ws>/install/rmd_robot_control/share/rmd_robot_control`
            #   이라 **4단계** 올라가야 워크스페이스 뿌리다. 3단계만 올라갔다가
            #   `install/src/...` 가 나와 파일을 못 찾았다 — 그러면 navigator 가
            #   조용히 `mission` 기본값(`['tie']`)만 돌아 **"주행을 안 한다"** 로 보인다.
            'path_file': os.path.join(_paths, 'test_path.txt'),
            # ⚠⚠ [2026-10-08] **이름 → 파일 맵.** 웹 UI 가 `/mission/path` 로
            #   이름만 보내고 navigator 가 이 맵으로 바꿔 읽는다. 맵이 비면
            #   UI 선택이 "파일이 없다" 로 거부된다.
            #   순서가 짝이다 — `path_named[i]` 가 `path_files[i]` 다.
            'path_named': ['line', 'rect'],
            'path_files': [os.path.join(_paths, 'test_path.txt'),
                           os.path.join(_paths, 'test_path_rect.txt')],
            # 반복은 기본 꺼짐 — 켜고 끄는 것은 UI(`/mission/repeat`) 다.
            'repeat': False,
        }],
        condition=IfCondition(LaunchConfiguration('use_mission')),
        respawn=True, respawn_delay=2.0,
    )

    # ── 교차점 검출 ──────────────────────────────────────────────────────
    # ⚠ `rate: 0.0` 은 **트리거 전용**이다 (`/rebar/detect`). 0 이 아니면 그 주기로
    #   계속 추론해 GPU 를 먹는다. 결속은 정지 상태에서 한 번만 찍으면 된다.
    # ⚠ 첫 트리거는 **모델 적재 중이라 버려진다** (RF-DETR 적재에 40초쯤). 기동
    #   직후 바로 미션을 걸면 검출이 안 온 것처럼 보인다.
    detector = Node(
        package='rebar_vision', executable='crossing_detector',
        name='crossing_detector', output='screen',
        parameters=[{'rate': 0.0}],
        condition=IfCondition(LaunchConfiguration('use_camera')),
        respawn=True, respawn_delay=5.0,
    )

    # ── 웹 UI ───────────────────────────────────────────────────────────
    # 왜 HTTP 중계인가: ROS 2 기본 탐색은 **멀티캐스트**다. 공유기·AP 를 거치면
    # 막히거나 흘리지 않는 경우가 흔해 "같은 네트워크인데 토픽이 안 보인다" 가
    # 된다. 리허설 당일에 그걸 디버깅하게 된다. HTTP 는 유니캐스트라 라우터만
    # 통과하면 되고, 노트북에 ROS 2 를 깔 필요도 없다.
    # respawn 이 안전한 이유: 이 노드는 뜨면서 아무 명령도 쏘지 않는다. 호밍은
    # 사람이 눌러야 돈다 — 안 그러면 크래시 루프가 재호밍 루프가 된다.
    ui = Node(
        package='rmd_robot_control', executable='ui_bridge',
        name='ui_bridge', output='screen',
        parameters=[{'port': LaunchConfiguration('ui_port')}],
        condition=IfCondition(LaunchConfiguration('use_ui')),
        respawn=True, respawn_delay=3.0,
    )

    return LaunchDescription(args + [
        LogInfo(msg=['리모콘 조작 구성 기동 — 안전 차단: ', use_safety,
                     ' / 호밍은 자동으로 돌지 않는다 (/homing_cmd 로 시작)']),
        motor, lateral, remote_bridge, remote_teleop, ezi_io, safety, mode_arbiter,
        homing, stage, tying, planner, camera, cam_watch, cam_watch_side,
        deck_edge, pololu,
        drive, navigator, detector, ui,
    ])
