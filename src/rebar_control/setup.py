from setuptools import setup
import os
from glob import glob

package_name = 'rebar_control'

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
    maintainer='koceti',
    maintainer_email='koceti@todo.todo',
    description='Integrated control system for rebar tying robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'iron_md_teleop = rebar_control.iron_md_teleop_node:main',
            'safety_monitor = rebar_control.safety_monitor:main',
            'precise_speed_logger = rebar_control.precise_speed_logger:main',
            'precision_navigation_node = rebar_control.precision_navigation_node:main',
            'cmd_vel_relay = rebar_control.cmd_vel_relay:main',
        ],
    },
)
