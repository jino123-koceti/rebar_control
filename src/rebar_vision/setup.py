from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rebar_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config 파일
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='robot@local',
    description='Rebar crossing detection and vision system',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dual_camera_recorder = rebar_vision.dual_camera_recorder_node:main',
            'rebar_detection = rebar_vision.rebar_detection_node:main',
            'tying_orchestrator = rebar_vision.tying_orchestrator_node:main',
            'obstacle_detector = rebar_vision.obstacle_detector_node:main',
        ],
    },
)
