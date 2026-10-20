#!/usr/bin/env python3
"""전후방 ZED X 두 대를 띄운다 — 주행 중 장애물 판단용.

토픽 이름은 `obstacle_detector_node` 가 이미 기대하는 규약을 따른다:

    /zed_front/zed_node/rgb/color/rect/image
    /zed_front/zed_node/depth/depth_registered
    /zed_back/zed_node/...

시리얼로 카메라를 고른다. **ID 로 고르면 안 된다** — ZED X 의 카메라 ID 는
열거 순서가 아니라 GMSL 포트에 묶여 있어서, 케이블을 옮기면 번호가 바뀐다.

    ros2 launch rebar_vision zed_dual.launch.py              # 둘 다
    ros2 launch rebar_vision zed_dual.launch.py back:=false  # 전방만

한 대씩 켤 수 있게 해 둔 이유: 두 대가 동시에 NEURAL depth 를 돌리면 GPU
부담이 작지 않다. 부하를 재거나 한쪽을 격리해 볼 때 쓴다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# 2026-10-20 실측으로 확정. 전방이 보정이 틀어진 개체다
# (data/calibration/zedx/README.md).
FRONT_SN = '43764735'
BACK_SN = '43615762'


def _zed(name, serial, config, enable_flag):
    """ZED 래퍼를 한 대 띄운다.

    `camera_name` 이 네임스페이스가 되고 노드 이름은 기본값 `zed_node` 라,
    토픽이 `/<name>/zed_node/...` 로 나온다 — 위 규약과 맞는다.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration(enable_flag)),
        launch_arguments={
            'camera_name': name,
            'camera_model': 'zedx',
            'serial_number': serial,
            'ros_params_override_path': PathJoinSubstitution([
                FindPackageShare('rebar_vision'), 'config', config
            ]),
            # URDF 는 한 번만 내보내도 되지만, 두 대의 광학 프레임이 각각
            # 필요하므로 양쪽 다 켠다. 지도 TF 는 VSLAM 을 껐으니 불필요.
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'publish_map_tf': 'false',
        }.items()
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'front', default_value='true',
            description=f'전방 ZED X (S/N {FRONT_SN}) 를 띄운다'),
        DeclareLaunchArgument(
            'back', default_value='true',
            description=f'후방 ZED X (S/N {BACK_SN}) 를 띄운다'),

        _zed('zed_front', FRONT_SN, 'zed_front.yaml', 'front'),
        _zed('zed_back', BACK_SN, 'zed_back.yaml', 'back'),
    ])
