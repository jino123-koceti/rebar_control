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
- zedxmini1: ZED X Mini (SN: 56755054) left side camera
- zedxmini2: ZED X Mini (SN: 54946194) right side camera

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
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zed'))
    )

    # ZED X Mini 1 (SN: 56755054) - left side, for rebar detection (P4-P6)
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
            'camera_model': 'zedxm',
            'serial_number': '56755054',
            'publish_tf': 'false',
            'pos_tracking_enabled': 'false',
            'depth_mode': 'ULTRA',
            'grab_resolution': 'HD720',
            'grab_frame_rate': '15',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zedxmini'))
    )

    # ZED X Mini 2 (SN: 54946194) - right side, for rebar detection (P1-P3)
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
            'camera_model': 'zedxm',
            'serial_number': '54946194',
            'publish_tf': 'false',
            'pos_tracking_enabled': 'false',
            'depth_mode': 'ULTRA',
            'grab_resolution': 'HD720',
            'grab_frame_rate': '15',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zedxmini'))
    )

    # ZED X One (mono, SN: 313997679, 4K) - 결속건 부착, 미세보정(top-view)용
    # pipe_rover(jino123-koceti) 검증 방식: ZED 래퍼 + depth NONE override config로
    #  모노 구동. startup에 스테레오와 함께 열려 GMSL 코디네이트 → 공존 (커스텀 pyzed는
    #  나중에 따로 open하며 GMSL 재스캔 → 스테레오 크래시했음).
    # Topic: /zedxone/zed_node/left/image_rect_color
    zedxone_config = os.path.join(
        get_package_share_directory('rebar_vision'), 'config', 'zedxone_params.yaml')
    zedxone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zedxone',
            'camera_model': 'zedxone4k',
            'ros_params_override_path': zedxone_config,
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'serial_number': '313997679',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_zedxmini'))
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

    return LaunchDescription([
        # Launch Arguments
        use_zed_arg,
        use_dual_zed_arg,
        single_zed_topic_arg,
        use_zedxmini_arg,
        use_orbbec_arg,
        use_vision_arg,

        # Control Systems (먼저 시작)
        base_system,
        control_system,

        # ZED Cameras - odom/navigation
        # 주행 영상 캡처/주행제어 개발 위해 활성화 (2026-07-14):
        # 전면 철근격자 인식 데이터 확보 — zedxmini1/2와 GMSL 대역폭 공존 위해
        # 결속카메라(zedxmini1/2)는 아래에서 임시 비활성.
        zed_front,
        zed_back,

        # ZED X Mini Cameras - rebar detection
        # 주행 캡처 중 GMSL 대역폭 확보 위해 임시 비활성 (2026-07-14). 결속 복귀 시 주석 해제.
        # zedxmini1,
        # zedxmini2,

        # ZED X One - Orbbec 결속 전환으로 미사용 + 물리적 제거(313997679 not found)
        # → 무한재시도로 컨테이너 막혀 orbbec 스트리밍 방해 → 비활성 (2026-07-04)
        # zedxone,
        # usbcam,

        # Orbbec Gemini 2L - ZED와 컨테이너명(camera_container) 충돌로 same-launch 불가.
        # → robot_control_service.sh에서 별도 프로세스로 실행 (독립 컨테이너)
        # orbbec_camera,

        # Vision - rebar detection service & tying orchestrator
        # detection_node: zedxmini 기반 교차점 검출 → orbbec_mode 전환으로 미사용,
        # zedxmini 비활성 중 유휴/에러만 내므로 임시 주석 (2026-07-14). 결속 zedxmini 복귀 시 해제.
        # detection_node,
        orchestrator_node,
        obstacle_detector_node,
    ])
