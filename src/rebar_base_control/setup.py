from setuptools import setup
import os
from glob import glob

package_name = 'rebar_base_control'

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
    maintainer='Koceti',
    maintainer_email='koceti@example.com',
    description='Hardware abstraction layer for Rebar control system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_parser = rebar_base_control.can_parser:main',
            'can_sender = rebar_base_control.can_sender:main',
            'drive_controller = rebar_base_control.drive_controller:main',
            'modbus_controller = rebar_base_control.modbus_controller:main',
            'authority_controller = rebar_base_control.authority_controller:main',
            'navigator_base = rebar_base_control.navigator_base:main',
        ],
    },
)
