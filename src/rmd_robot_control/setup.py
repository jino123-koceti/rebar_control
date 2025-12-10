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
            'robot_control_gui = rmd_robot_control.robot_control_gui:main',
        ],
    },
)
