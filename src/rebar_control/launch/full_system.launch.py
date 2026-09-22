#!/usr/bin/env python3
"""
Full System Launch File
Phase 2 (하드웨어 추상화) + Phase 3 (상위 제어) + ZED Cameras 전체 실행

실행되는 노드:
Phase 2 (rebar_base_control):
- can_parser, can_sender
- drive_controller
- modbus_controller
- authority_controller
- navigator_base

Phase 3 (rebar_control):
- zenoh_client
- navigator
- rebar_controller
- rebar_publisher
- pose_mux

ZED Cameras (optional, use_zed:=true by default):
- zed_front: ZEDX cam3 (SN: 45320958) for forward motion
- zed_back: ZEDX cam2 (SN: 46674448) for backward motion

ZED X Mini Cameras (optional, use_zedxmini:=true by default):
- zedxmini2: ZED X Mini (SN: 54946194) 전방 카메라 (2026-08-06 측면→전방 물리 이동, zed_front 대체)
- zedxmini1: ZED X Mini (SN: 56755054) 후방 카메라 (2026-08-06 측면→후방 물리 이동, zed_back 대체)
  ※ 실제 시야 스냅샷으로 방향 확인 완료 (zedxmini2가 배근 진행방향을 봄)

Vision (optional, use_vision:=true by default):
- rebar_detection_node: YOLO-based rebar crossing detection service
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 주행용 zedxmini 파라미터 override (부하 감축: HD720@15, depth NONE 등).
    # zed_camera.launch.py는 grab_resolution 같은 개별 인자를 받지 않고 이 경로만 받는다.
    # 적용 순서상 common_stereo.yaml → zedxm.yaml → **이 파일**(마지막=우선)이라 확실히 덮는다.
    zedxmini_override = os.path.join(
        get_package_share_directory('rebar_vision'),
        'config', 'zedxmini_drive_override.yaml')
    # ZED X One(우측 횡이동 판정) override — HD1200 필수(SVGA는 open 실패).
    zedxone_override = os.path.join(
        get_package_share_directory('rebar_vision'),
        'config', 'zedxone_params.yaml')

    # Launch Arguments
    use_zed_arg = DeclareLaunchArgument(
        'use_zed',
        default_value='true',
        description='Set to false to disable ZED camera nodes (for simulation or testing)'
    )

    use_dual_zed_arg = DeclareLaunchArgument(
        'use_dual_zed',
        default_value='true',
        description='Use dual ZED cameras (pose_mux) vs single ZED (odom_to_pose)'
    )

    single_zed_topic_arg = DeclareLaunchArgument(
        'single_zed_odom_topic',
        default_value='/zed/zed_node/odom',
        description='Odometry topic for single ZED camera (used when use_dual_zed=false)'
    )

    # Phase 2: 하드웨어 추상화 계층
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rebar_base_control'),
                'launch',
                'base_system.launch.py'
            ])
        ])
    )

    # Phase 3: 상위 제어 계층
    control_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rebar_control'),
                'launch',
                'control_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_dual_zed': LaunchConfiguration('use_dual_zed'),
            'single_zed_odom_topic': LaunchConfiguration('single_zed_odom_topic'),
        }.items()
    )

    # ZED Front Camera (ZEDX cam3, SN: 45320958) - for forward motion
    # Using zed_wrapper standard launch
    zed_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zed_front',
            'camera_model': 'zedx',
            'serial_number': '45320958',
            # [2026-08-06] GPU/GMSL 부하 감축(프리즈 완화): GPU 85%→↓ 목표.
            #   HD1080→HD720(검출은 512 다운스케일이라 충분), depth NONE(주행 deck_edge는 RGB seg만
            #   씀, obstacle 거리 미사용), depth NONE이면 PCD도 자동 off. pos_tracking off.
            'grab_resolution': 'HD720',
            'grab_frame_rate': '15',
            'depth_mode': 'NONE',
            'pos_tracking_enabled': 'false',
            'publish_tf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zed'))
    )

    # ZED Back Camera (ZEDX cam2, SN: 46674448) - for backward motion
    # Using zed_wrapper standard launch
    zed_back = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zed_back',
            'camera_model': 'zedx',
            'serial_number': '46674448',
            # [2026-08-06] GPU/GMSL 부하 감축(프리즈 완화): front와 동일 설정.
            'grab_resolution': 'HD720',
            'grab_frame_rate': '15',
            'depth_mode': 'NONE',
            'pos_tracking_enabled': 'false',
            'publish_tf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zed'))
    )

    # ZED X Mini 1 (SN: 56755054) - 후방 카메라 (주행 deck_edge back). 이전 역할: 좌측 결속검출(P4-P6)
    # Topic pattern: /zedxmini1/zed_node/left/image_rect_color
    zedxmini1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zedxmini1',
            # ★ [2026-09-08] 전·후방 카메라를 **ZED X Mini → ZED X** 로 교체.
            #   네임스페이스(zedxmini1/2)는 **일부러 그대로 둔다** — deck_edge·수집도구·
            #   기록 등 여러 곳이 이 이름을 쓰고 있어 지금 바꾸면 위험하다. 이름은
            #   식별자일 뿐이고 실제 기종은 아래 camera_model이 정한다.
            'camera_model': 'zedx',
            # 2026-09-08 실물 화면으로 확인: 46674448 = 후방
            'serial_number': '46674448',
            'publish_tf': 'false',
            # ⚠ grab_resolution/grab_frame_rate/depth_mode/pos_tracking_enabled 를 여기에
            #   직접 넘기면 **조용히 무시된다** — zed_camera.launch.py가 받지 않는 인자다.
            #   실제로 HD1200@30으로 돌고 있었다(2026-08-06 확인). override yaml로 넘긴다.
            'ros_params_override_path': zedxmini_override,
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zedxmini'))
    )

    # ZED X Mini 2 (SN: 54946194) - 전방 카메라 (주행 deck_edge front). 이전 역할: 우측 결속검출(P1-P3)
    # Topic pattern: /zedxmini2/zed_node/left/image_rect_color
    zedxmini2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zedxmini2',
            'camera_model': 'zedx',          # 2026-09-08 ZED X 교체 (위 주석 참조)
            'serial_number': '45320958',     # 실물 화면 확인: 45320958 = 전방
            'publish_tf': 'false',
            'ros_params_override_path': zedxmini_override,   # 위 zedxmini1 주석 참조
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zedxmini'))
    )

    # ZED X One UHD (mono, SN 319647570) — **우측** 횡이동 주행가능 판정용 (2026-08-13 복귀).
    #   좌측은 USB Orbbec 305, 우측은 이 GMSL 모노. 'ㄹ' 커버리지의 첫 횡이동 방향 결정에 필요.
    # ⚠ 알아둘 것 (2026-08-13 실측):
    #   · **해상도 HD1200 필수** — SVGA/HD1080은 open 실패. override yaml에 지정.
    #   · **DISPLAY가 설정돼 있으면 Argus BadParameter로 segfault** — SSH X11 포워딩
    #     (DISPLAY=localhost:10.0)이 있으면 원격 X에 GPU 버퍼를 만들려다 죽는다.
    #     systemd 서비스는 DISPLAY가 없어 정상. 수동 실행 시엔 `env -u DISPLAY` 필요.
    #   · 토픽: /zedxone/zed_node/rgb/color/rect/image (래퍼 v5.1 경로)
    #   · GMSL 3대(zedxmini2 + zedxmini1 + 이것) 동시 스트리밍 확인됨(13~15Hz, load 12).
    zedxone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zedxone',
            'camera_model': 'zedxone4k',
            'serial_number': '319647570',
            'publish_tf': 'false',
            'ros_params_override_path': zedxone_override,
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zedxone'))
    )

    # ★★ [2026-09-14] 기본값 true → **false**. 측면 카메라(좌·우 ZED X One)는 이제
    #   `deck_edge_node` 가 **판정 직전에 띄웠다 내린다**(side_manage_nodes).
    #   이유: GMSL 4대 동시 스트리밍이 실측으로 전방 스테레오를 죽였다(SIGSEGV, 2026-09-14).
    #     상시 = 스테레오 2대(전/후, 주행 안전판정에 필수)
    #     요청 시 +1 = 모노 1대  ⟹ 동시 최대 3대(검증된 구성)
    #   여기를 true 로 되돌리면 우측이 상시로 떠서 **좌측을 띄우는 순간 4대가 된다.**
    #   디버깅 목적으로 잠깐 켤 때만 쓸 것(그때는 deck_edge `side_manage_nodes:=false`).
    # ── 좌측 ZED X One 4K (SN 319430526) — **GMSL 4대 시험용** ──────────────
    # ★ [2026-09-14] 기존 Orbbec 305(좌측)를 이 카메라로 교체했다. 다만 4대 동시
    #   스트리밍이 검증되지 않아 **기본은 꺼둔다**(use_zedxone_left:=false).
    #
    # ## 지금까지 확인된 것 (둘 다 "돌아가는 중에 추가로 열기" 였다)
    #   · 3대 가동 중 4번째를 임시 기동 → 전방 스테레오 사망(exit -11)
    #   · deck_edge on-demand 여닫기 → 스테레오 **2대** 사망
    #   두 경우 모두 직접 원인은 대수가 아니라 **자식의 비정상 종료가 Argus 소켓을
    #   끊은 것**이었다(`Error EndOfFile: reading socket` → 다른 클라이언트 SIGSEGV).
    #   ⟹ **런치 시점에 한꺼번에 여는 경로는 아직 안 해봤다.** 그게 이 인자다.
    #
    # ## 시험 방법 (반드시 **정지 상태**에서)
    #   ros2 launch ... use_zedxone:=true use_zedxone_left:=true
    #   확인: ① 4개 토픽 전부 프레임이 오는가 ② 몇 분 버티는가
    #         ③ CORRUPTED FRAME / Argus EndOfFile 이 안 뜨는가 ④ load average
    #   ⚠ 실패해도 정지 상태면 재시작이면 끝이다. 주행 중에 죽으면 눈이 먼다.
    zedxone_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zedxone_left',
            'camera_model': 'zedxone4k',
            'serial_number': '319430526',
            'publish_tf': 'false',
            # ⚠ HD1200 필수 — SVGA/HD1080은 open 실패하고 재시도가 GMSL 버스를 교란한다
            'ros_params_override_path': zedxone_override,
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zedxone_left'))
    )

    use_zedxone_left_arg = DeclareLaunchArgument(
        'use_zedxone_left',
        default_value='false',
        description='좌측 ZED X One 상시 기동 — **GMSL 4대 시험용. 정지 상태에서만 켤 것**'
    )

    use_zedxone_arg = DeclareLaunchArgument(
        'use_zedxone',
        # ★ [2026-09-14 저녁] on-demand 가 실패해 **검증된 3대 상시 구성으로 복귀**.
        #   (스테레오 2 + 이 모노 1 = 수 주간 무사고. deck_edge 의 side_manage_nodes 주석 참조)
        default_value='true',
        description='ZED X One 우측 상시 기동. 좌측 모노는 아직 소프트웨어에 없다(3대 구성)'
    )

    # USB 캠 (UVC) - 결속 미세보정 top-view (GMSL과 별개라 충돌 없음)
    # Topic: /usbcam/image_raw
    usbcam = Node(
        package='rebar_vision',
        executable='usbcam_publisher',
        name='usbcam_publisher',
        output='screen',
        parameters=[{
            'device': '/dev/video9',
            'width': 1920,
            'height': 1080,
            'fps': 15,
        }],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_zedxmini'))
    )

    # Orbbec Gemini 2L - 상단 정면 교차점 검출 (호모그래피 자율결속). USB3, GMSL과 별개라 충돌 없음.
    # Topic: /camera/color/image_raw (1280x800), /camera/depth/image_raw
    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'), 'launch', 'gemini2L.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_orbbec'))
    )

    use_orbbec_arg = DeclareLaunchArgument(
        'use_orbbec',
        default_value='true',
        description='Set to false to disable Orbbec Gemini 2L (교차점 검출)'
    )

    # Launch argument for ZED X Mini cameras
    use_zedxmini_arg = DeclareLaunchArgument(
        'use_zedxmini',
        default_value='true',
        description='Set to false to disable ZED X Mini cameras (rebar detection)'
    )

    use_vision_arg = DeclareLaunchArgument(
        'use_vision',
        default_value='true',
        description='Set to false to disable rebar detection node'
    )

    # 데크끝(주행불가) 감지: 철근배근 기반. 켜면 drive_controller가 데크 밖으로 나가는
    # 방향의 cmd_vel을 차단한다. 미검증 단계라 기본 false — 명시적으로 켤 것.
    use_deck_edge_arg = DeclareLaunchArgument(
        'use_deck_edge',
        default_value='false',
        description='Enable deck-edge (rebar drivability) detection + direction block'
    )

    # 비전 자율주행[B]: 리모콘 S20(auto)+S23 시작 / S24 정지. deck_edge가 함께 켜져야 함.
    # ⚠ 기본 dry-run(모션 없음). 실제 구동은 rebar_drive_arm:=true (궤도 공중부터!)
    use_rebar_drive_arg = DeclareLaunchArgument(
        'use_rebar_drive',
        default_value='false',
        description='Enable vision autonomous drive node (needs use_deck_edge:=true)'
    )
    rebar_drive_arm_arg = DeclareLaunchArgument(
        'rebar_drive_arm',
        default_value='false',
        description='Actually move (false = dry-run, no motion)'
    )
    # UI 버튼으로 비전 자율결속을 띄우는 실행기. 노드 자체는 아무것도 안 하고,
    # /mission/command 의 {"command":"AUTO_TYING","direction":"FWD"|"REV"} 를 받으면
    # 그때 rebar_drive 프로세스를 대신 실행한다(매번 파라미터 20개를 손으로 붙이지
    # 않게). ⚠ use_rebar_drive 와 **동시에 켜지 말 것** — rebar_drive가 둘이 되어
    # /cmd_vel 이 섞인다.
    use_auto_tying_arg = DeclareLaunchArgument(
        'use_auto_tying',
        default_value='false',
        description='Enable UI-driven autonomous tying launcher (AUTO_TYING command)'
    )

    # UI 버튼으로 현장 학습데이터 수집(field_collect.py)을 띄우는 실행기.
    # DATA_ACQ_START/STOP 만 듣는다. 도구가 구독만 하므로 주행·결속과 동시에
    # 켜도 안전하다.
    use_data_acq_arg = DeclareLaunchArgument(
        'use_data_acq',
        default_value='false',
        description='Enable UI-driven field data acquisition launcher'
    )

    # 'ㄹ'자 커버리지: 데크 끝마다 옆 레인으로 70mm씩 횡이동(1회마다 측면 재판정).
    # 측면 카메라(좌=Orbbec 305, 우=ZED X One)가 떠 있어야 판정이 나온다.
    rebar_drive_lateral_arg = DeclareLaunchArgument(
        'rebar_drive_lateral',
        default_value='false',
        description="Enable zigzag lateral lane change ('ㄹ' coverage)"
    )
    rebar_drive_tying_arg = DeclareLaunchArgument(
        'rebar_drive_tying',
        default_value='false',
        description='Actually tie at each step (via tying_orchestrator)'
    )

    # Rebar Detection Node (from rebar_vision package)
    rebar_vision_share = get_package_share_directory('rebar_vision')
    camera_config = os.path.join(rebar_vision_share, 'config', 'camera_extrinsics.yaml')
    tying_config = os.path.join(rebar_vision_share, 'config', 'tying_orchestrator.yaml')

    detection_node = Node(
        package='rebar_vision',
        executable='rebar_detection',
        name='rebar_detection_node',
        output='screen',
        parameters=[camera_config],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_vision'))
    )

    # Tying Orchestrator Node (from rebar_vision package)
    orchestrator_node = Node(
        package='rebar_vision',
        executable='tying_orchestrator',
        name='tying_orchestrator',
        output='screen',
        parameters=[tying_config],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_vision'))
    )

    # Obstacle Detector Node (person/obstacle detection with front/back cameras)
    obstacle_detector_node = Node(
        package='rebar_vision',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_vision'))
    )

    # Deck Edge Node: 철근배근 기반 주행가능 판정 → /deck_edge_block (방향별 차단)
    # obstacle_detector와 같은 위치의 하부 안전망. 미션 흐름(navigator)과 무관.
    deck_edge_node = Node(
        package='rebar_vision',
        executable='deck_edge',
        name='deck_edge_node',
        output='screen',
        emulate_tty=True,
        # [2026-08-06] GPU 부하 감축: seg_hz 8→4 + active_only(진행방향만 추론, 노드 기본 True).
        #   + 주행 카메라를 zedxmini로 교체(격리 테스트): front=zedxmini2, back=zedxmini1.
        #     (스냅샷으로 시야 확인 — zedxmini2가 배근 진행방향)
        parameters=[{
            'seg_hz': 4.0,
            'front_topic': '/zedxmini2/zed_node/rgb/color/rect/image/compressed',
            'back_topic': '/zedxmini1/zed_node/rgb/color/rect/image/compressed',
        }],
        condition=IfCondition(LaunchConfiguration('use_deck_edge'))
    )

    # Rebar Drive Node: 비전 자율주행[B]. 리모콘 S20+S23 시작 / S24 정지.
    # navigator 웨이포인트 주행[A]과 /cmd_vel을 공유하므로 동시 사용 금지.
    rebar_drive_node = Node(
        package='rebar_vision',
        executable='rebar_drive',
        name='rebar_drive_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'arm': LaunchConfiguration('rebar_drive_arm'),
            'lateral_enabled': LaunchConfiguration('rebar_drive_lateral'),
            'do_tying': LaunchConfiguration('rebar_drive_tying'),
        }],
        condition=IfCondition(LaunchConfiguration('use_rebar_drive'))
    )

    # Auto Tying Launcher: UI 버튼 → rebar_drive 프로세스 실행/중지.
    # 실제 주행 파라미터는 여기서 정한다(명령줄로 매번 넘기던 값들).
    auto_tying_launcher = Node(
        package='rebar_vision',
        executable='auto_tying_launcher',
        name='auto_tying_launcher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'arm': LaunchConfiguration('rebar_drive_arm'),
            'lateral_enabled': LaunchConfiguration('rebar_drive_lateral'),
            'do_tying': LaunchConfiguration('rebar_drive_tying'),
        }],
        condition=IfCondition(LaunchConfiguration('use_auto_tying'))
    )

    # Data Acquisition Launcher: UI 버튼 → field_collect.py 실행/정지.
    data_acq_launcher = Node(
        package='rebar_vision',
        executable='data_acq_launcher',
        name='data_acq_launcher',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_data_acq'))
    )

    return LaunchDescription([
        # Launch Arguments
        use_zed_arg,
        use_dual_zed_arg,
        single_zed_topic_arg,
        use_zedxmini_arg,
        use_zedxone_arg,
        use_zedxone_left_arg,
        use_orbbec_arg,
        use_vision_arg,
        use_deck_edge_arg,
        use_rebar_drive_arg,
        rebar_drive_arm_arg,
        rebar_drive_lateral_arg,
        rebar_drive_tying_arg,
        use_auto_tying_arg,
        use_data_acq_arg,

        # Control Systems (먼저 시작)
        base_system,
        control_system,

        # ZED Cameras - odom/navigation
        # 주행 영상 캡처/주행제어 개발 위해 활성화 (2026-07-14):
        # 전면 철근격자 인식 데이터 확보 — zedxmini1/2와 GMSL 대역폭 공존 위해
        # 결속카메라(zedxmini1/2)는 아래에서 임시 비활성.
        # [2026-08-06] zed_front/back(ZED X) 둘 다 비활성 — 프리즈 원인 의심(zedxmini는 과거 3일 안정).
        #   주행 카메라를 zedxmini로 전량 교체: 좌측 zedxmini1→전방, 우측 zedxmini2→후방(물리 이동).
        #   deck_edge front/back_topic도 zedxmini로 재지정. 이걸로 "ZED X가 범인인가" 최종 격리.
        # zed_front,
        # zed_back,

        # ZED X Mini Cameras - [2026-08-06] 주행 전/후방 관측 (zedxmini2=전방, zedxmini1=후방)
        # 작업영역 교차점검출용→orbbec 전환으로 비활성했다가, 카메라를 외부 좌우로
        # 재배치해 주행 데이터 수집용으로 복귀 (2026-07-22). quad capture card라 8대까지 대역폭 OK.
        # → 다시 좌/우 측면에서 전/후방으로 물리 이동 (2026-08-06, ZED X 프리즈 격리).
        # ⚠️ [2026-08-06] 좌우 zedxmini(GMSL) 물리 제거 → GMSL을 front/back 2대만 남김.
        #   이유: L4T 36.4.4 nvargus 멀티카메라 버그가 GMSL 3대에서도 재발(24분 프리즈) → 2대로 추가 감축.
        #   좌측=USB Orbbec 305(camera_left)로 이전. 우측 zedxmini2도 GMSL 케이블 제거(전후진만 하므로 측면 불필요).
        #   복귀 시 주석 해제 + GMSL 케이블 연결. 근본해결은 front/back도 USB Orbbec 이전(GMSL 0대).
        zedxmini1,     # [2026-08-06] 후방 카메라 (측면→후방 물리 이동) → zed_back 대체.
        zedxmini2,     # [2026-08-06] 전방 카메라 (측면→전방 물리 이동) → zed_front 대체(프리즈 격리 테스트).
        zedxone,       # [2026-08-13] 우측 횡이동 판정 (GMSL 3대째). 부하 문제 시 use_zedxone:=false
        zedxone_left,  # [2026-09-14] 좌측 (GMSL 4대째) — 기본 off, 시험용
                       #   좌측 측면은 Orbbec 305(USB)가 커버. deck_edge front/back_topic 재지정.

        # Orbbec Gemini 2L - ZED와 컨테이너명(camera_container) 충돌로 same-launch 불가.
        # → robot_control_service.sh에서 별도 프로세스로 실행 (독립 컨테이너)
        # orbbec_camera,

        # Vision - rebar detection service & tying orchestrator
        # detection_node: zedxmini 기반 교차점 검출 → orbbec_mode 전환으로 미사용,
        # zedxmini 비활성 중 유휴/에러만 내므로 임시 주석 (2026-07-14). 결속 zedxmini 복귀 시 해제.
        # detection_node,
        orchestrator_node,
        # [2026-08-06] obstacle_detector(사람감지 YOLO+depth) 제거 — depth NONE으로 거리판정
        #   불가 + 미사용. 사람/장애물 정지는 물리 범퍼 IO 신호로 대체 예정([[bumper_plan]]).
        #   복귀 시 주석 해제 + front/back depth를 PERFORMANCE 이상으로 되살릴 것.
        # obstacle_detector_node,

        # 데크끝 감지 (use_deck_edge:=true 로 켤 것). rebar_drive(비전 자율주행)는
        # 아직 서비스에 넣지 않는다 — 단독 실행으로 검증 후 추가.
        deck_edge_node,
        rebar_drive_node,
        auto_tying_launcher,
        data_acq_launcher,
    ])
