#!/usr/bin/env python3
"""
데이터 수집 런치 파일

실행:
  ros2 launch rebar_vision data_collection.launch.py

수동 모드 (기본):
  ros2 launch rebar_vision data_collection.launch.py save_mode:=manual

자동 모드 (2초마다):
  ros2 launch rebar_vision data_collection.launch.py save_mode:=auto auto_interval:=2.0

연속 모드 (10 FPS):
  ros2 launch rebar_vision data_collection.launch.py save_mode:=continuous fps_limit:=10.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch Arguments
    save_mode_arg = DeclareLaunchArgument(
        'save_mode',
        default_value='manual',
        description='Capture mode: manual, auto, continuous'
    )
    
    auto_interval_arg = DeclareLaunchArgument(
        'auto_interval',
        default_value='2.0',
        description='Auto capture interval (seconds)'
    )
    
    fps_limit_arg = DeclareLaunchArgument(
        'fps_limit',
        default_value='10.0',
        description='Continuous mode FPS limit'
    )
    
    rebar_spacing_arg = DeclareLaunchArgument(
        'rebar_spacing',
        default_value='200',
        description='Rebar spacing in mm'
    )
    
    rebar_pattern_arg = DeclareLaunchArgument(
        'rebar_pattern',
        default_value='orthogonal',
        description='Rebar pattern: orthogonal, diagonal'
    )
    
    session_name_arg = DeclareLaunchArgument(
        'session_name',
        default_value='',
        description='Custom session name (optional)'
    )
    
    save_depth_arg = DeclareLaunchArgument(
        'save_depth',
        default_value='true',
        description='Save depth images'
    )
    
    # =============================================================
    # ZED 듀얼 카메라 런치 (Depth 최적화 설정)
    # =============================================================
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_launch_file = os.path.join(
        zed_wrapper_dir, 'launch', 'multicam', 'two_zedxmini.launch.py'
    )
    
    # Depth 근거리 최적화 파라미터
    zed_dual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            # Depth 모드: NEURAL (최고 품질), ULTRA, QUALITY, PERFORMANCE
            'depth_mode': 'ULTRA',
            # 깊이 신뢰도 (낮을수록 더 많은 픽셀, 노이즈 증가)
            'depth_confidence': '50',
            # 질감 신뢰도
            'depth_texture_conf': '50',
            # OpenNI 모드 (16-bit mm 단위)
            'openni_depth_mode': 'true',
            # 해상도
            'resolution': 'HD720',
            # FPS
            'grab_frame_rate': '15',
        }.items()
    )
    
    # =============================================================
    # 듀얼 카메라 녹화 노드
    # =============================================================
    recorder_node = Node(
        package='rebar_vision',
        executable='dual_camera_recorder',
        name='dual_camera_recorder',
        output='screen',
        parameters=[{
            'save_path': '/home/test/dataset',
            'save_mode': LaunchConfiguration('save_mode'),
            'auto_interval': LaunchConfiguration('auto_interval'),
            'fps_limit': LaunchConfiguration('fps_limit'),
            'rebar_spacing': LaunchConfiguration('rebar_spacing'),
            'rebar_pattern': LaunchConfiguration('rebar_pattern'),
            'session_name': LaunchConfiguration('session_name'),
            'save_depth': LaunchConfiguration('save_depth'),
        }]
    )
    
    return LaunchDescription([
        # Arguments
        save_mode_arg,
        auto_interval_arg,
        fps_limit_arg,
        rebar_spacing_arg,
        rebar_pattern_arg,
        session_name_arg,
        save_depth_arg,
        
        # Nodes
        zed_dual_launch,
        recorder_node,
    ])
