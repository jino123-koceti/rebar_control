from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('seengrip_ros2')
    config_file = os.path.join(pkg_share, 'config', 'seengrip.yaml')
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Seengrip gripper'
    )
    
    # Seengrip 노드
    seengrip_node = Node(
        package='seengrip_ros2',
        executable='seengrip_node',
        name='seengrip',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': 115200,
            'slave_id': 1,
            'default_speed': 500,
            'open_position': 0,
            'close_position': 2000,
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        seengrip_node,
    ])
