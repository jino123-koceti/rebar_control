from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rebar_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koceti',
    maintainer_email='koceti@example.com',
    description='Rebar upper control layer',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'zenoh_client = rebar_control.zenoh_client:main',
            'navigator = rebar_control.navigator:main',
            'rebar_controller = rebar_control.rebar_controller:main',
            'rebar_publisher = rebar_control.rebar_publisher:main',
            'pose_mux = rebar_control.pose_mux:main',
            'odom_to_pose = rebar_control.odom_to_pose:main',
            'performance_test = rebar_control.performance_test:main',
        ],
    },
)
