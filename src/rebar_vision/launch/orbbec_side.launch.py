#!/usr/bin/env python3
"""좌·우 Orbbec Gemini 305 두 대를 띄운다 — 측면 판정·데크끝 감지용.

    /camera_left/color/image_raw   ·  /camera_left/depth/image_raw
    /camera_right/...

이 토픽들을 쓰기로 돼 있는 곳 (YEAR3_FEATURE_INVENTORY §카메라):

    deck_edge     데크 끝을 보고 **그 방향 주행을 차단**한다.
                  `/deck_edge_block` 의 발행자가 지금 아무도 없다 —
                  `safety_node` 가 구독만 하고 있다 (R3_PLANNED 에 기록).
    side_judge    측면 판정. 2차년도 `usb_side_cam_node` 를 대체한다.

## 좌/우 매핑 — 영상으로 확인함 (2026-11-05)

    좌측 = CV2L3600003P      우측 = CV2856D000D0

⚠ **추측으로 두면 안 되는 값이다.** 이 카메라가 맡는 일이 "데크 끝 방향의
  주행을 막는 것" 이라, 좌우가 바뀌면 **차단이 반대로 걸려 떨어지는 쪽으로
  밀어준다.** 2차년도에 횡이동 부호가 반대라 안전장치가 통째로 무력화된
  전례가 있다. 카메라를 옮겨 달면 **영상으로 다시 확인할 것** —
  한 장씩 찍어 눈으로 보는 데 1분이면 된다.

⚠ 교차점 카메라(Gemini 2L)와 **다른 USB 가지**에 있다. 305 둘은 USB3(5000M),
  2L 은 USB2(480M) 다 — 대역폭을 나눠 쓰지 않는다
  (`rebar-camera-usb-bandwidth-ceiling` 참고).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

LEFT_SN = 'CV2L3600003P'      # 좌측 (2026-11-05 영상 확인)
RIGHT_SN = 'CV2856D000D0'     # 우측

# 측면 판정·데크끝은 멀리 볼 필요도 빠를 필요도 없다. 작게 띄워 USB·GPU 를
# 아낀다 — 교차점 검출과 ZED depth 가 같은 젯슨을 쓴다.
COMMON = {
    'enable_color': 'true', 'color_width': '640', 'color_height': '480',
    # ⚠ fps 를 15 → 5 로 내렸다 (2026-11-09). 측면·데크끝 판정은 횡이동 때
    #   한두 장만 보면 되는데, 336L 이 1280x800 으로 올라가면서 같은 젯슨에서
    #   DDS·USB 를 같이 쓰다 보니 15fps 는 그냥 낭비였다. 교차점 카메라가
    #   프레임을 잃는 상황이라 여기서 먼저 양보한다.
    # ⚠ depth 는 **끄지 마라.** deck_edge_node 가 깊이로 데크끝 거리를 잰다
    #   (평면 투영으로 바꾸면 35% 멀게 나온다). 쓰는 데가 없어 보여도 쓴다.
    'color_fps': '5',
    'enable_depth': 'true', 'depth_width': '640', 'depth_height': '480',
    'depth_fps': '5',
    'depth_registration': 'true',     # 컬러 화소에서 바로 깊이를 읽기 위해
    'enable_point_cloud': 'false',    # 쓰는 데가 아직 없다
    'enable_colored_point_cloud': 'false',
    'enable_left_ir': 'false', 'enable_right_ir': 'false',
    'enable_accel': 'false', 'enable_gyro': 'false',
}


def _cam(name, serial, flag):
    args = dict(COMMON)
    args.update({'camera_name': name, 'serial_number': serial})
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('orbbec_camera'), 'launch',
                                  'gemini_330_series.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration(flag)),
        launch_arguments=args.items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('left', default_value='true',
                              description=f'좌측 305 ({LEFT_SN})'),
        DeclareLaunchArgument('right', default_value='true',
                              description=f'우측 305 ({RIGHT_SN})'),

        _cam('camera_left', LEFT_SN, 'left'),
        _cam('camera_right', RIGHT_SN, 'right'),
    ])
