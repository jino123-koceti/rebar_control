#!/usr/bin/env python3
"""
ZED Minimal Odometry Launch
Starts ZED camera with minimal topics (odom + IMU only)
Uses custom config to disable image/depth topic publishing while keeping VSLAM active

토픽 최소화:
- odom, imu/data 토픽만 발행
- depth/image/pointcloud 토픽 발행 비활성화 (내부 연산은 VSLAM용으로 동작)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        description='Camera name (e.g., zed_front, zed_back)'
    )

    serial_number_arg = DeclareLaunchArgument(
        'serial_number',
        description='Camera serial number'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='',
        description='Base frame name'
    )

    # Custom config file path for minimal odom publishing
    config_path = PathJoinSubstitution([
        FindPackageShare('rebar_vision'),
        'config',
        'zed_odom_only.yaml'
    ])

    # Include standard ZED wrapper launch with custom config override
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_model': 'zedx',
            'serial_number': LaunchConfiguration('serial_number'),

            # Override config with minimal odom settings
            'ros_params_override_path': config_path,

            # TF settings
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'true',
            'publish_urdf': 'true',
        }.items()
    )

    return LaunchDescription([
        camera_name_arg,
        serial_number_arg,
        base_frame_arg,
        zed_wrapper_launch,
    ])
