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

    detection_node = Node(
        package='rebar_vision',
        executable='rebar_detection',
        name='rebar_detection_node',
        output='screen',
        parameters=[camera_config],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_vision'))
    )

    return LaunchDescription([
        # Launch Arguments
        use_zed_arg,
        use_dual_zed_arg,
        single_zed_topic_arg,
        use_zedxmini_arg,
        use_vision_arg,

        # Control Systems (먼저 시작)
        base_system,
        control_system,

        # ZED Cameras - odom/navigation
        zed_front,
        zed_back,

        # ZED X Mini Cameras - rebar detection
        zedxmini1,
        zedxmini2,

        # Vision - rebar detection service
        detection_node,
    ])
