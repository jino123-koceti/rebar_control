from setuptools import setup
import os
from glob import glob

package_name = 'rmd_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 control package for 7-motor robot with RMD-X4 actuators',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_control_node = rmd_robot_control.position_control_node:main',
            'lateral_node = rmd_robot_control.lateral_node:main',
            'remote_teleop_node = rmd_robot_control.remote_teleop_node:main',
            'homing_node = rmd_robot_control.homing_node:main',
            # L3 — 상부 스테이지를 mm 목표로 보낸다 (검출점 이동용)
            'stage_node = rmd_robot_control.stage_node:main',
            # L4 — 결속 지점 하나를 자세 선택 → 회전 → 이동으로 묶는다.
            # axis_config(단일 소스)를 읽어야 해서 이 패키지에 둔다. 축·CAN 은
            # 건드리지 않고 /stage/* 로만 명령하므로 계층은 지켜진다.
            'tying_sequence = rmd_robot_control.tying_sequence:main',
        ],
    },
)
