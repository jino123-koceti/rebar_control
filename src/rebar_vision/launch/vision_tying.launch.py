#!/usr/bin/env python3
"""
비전 기반 자동 결속 런치 파일

ZED 듀얼 카메라 + 교차점 검출 + 결속 오케스트레이터 통합 실행

실행:
  ros2 launch rebar_vision vision_tying.launch.py

카메라 없이 검출/오케스트레이터만:
  ros2 launch rebar_vision vision_tying.launch.py launch_zed:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('rebar_vision')

    # ============================================
    # Launch Arguments
    # ============================================
    launch_zed_arg = DeclareLaunchArgument(
        'launch_zed',
        default_value='true',
        description='ZED 듀얼 카메라 런치 여부'
    )

    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO 신뢰도 임계값'
    )

    # ============================================
    # 설정 파일 경로
    # ============================================
    camera_config = os.path.join(pkg_share, 'config', 'camera_extrinsics.yaml')
    tying_config = os.path.join(pkg_share, 'config', 'tying_orchestrator.yaml')

    # ============================================
    # ZED 듀얼 카메라 (선택적)
    # ============================================
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_launch_file = os.path.join(
        zed_wrapper_dir, 'launch', 'multicam', 'two_zedxmini.launch.py'
    )

    zed_dual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            'depth_mode': 'ULTRA',
            'depth_confidence': '50',
            'depth_texture_conf': '50',
            'openni_depth_mode': 'true',
            'resolution': 'HD720',
            'grab_frame_rate': '15',
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_zed'))
    )

    # ============================================
    # 검출 노드
    # ============================================
    detection_node = Node(
        package='rebar_vision',
        executable='rebar_detection',
        name='rebar_detection_node',
        output='screen',
        parameters=[camera_config],
        emulate_tty=True
    )

    # ============================================
    # 결속 오케스트레이터
    # ============================================
    orchestrator_node = Node(
        package='rebar_vision',
        executable='tying_orchestrator',
        name='tying_orchestrator',
        output='screen',
        parameters=[tying_config],
        emulate_tty=True
    )

    return LaunchDescription([
        # Arguments
        launch_zed_arg,
        confidence_arg,

        # Nodes
        zed_dual_launch,
        detection_node,
        orchestrator_node,
    ])
