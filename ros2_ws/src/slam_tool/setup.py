#!/usr/bin/env python3
from setuptools import setup

package_name = 'slam_tool'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package.xml, launch, and config files
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_slam.py']),
        ('share/' + package_name + '/config', ['config/slam_toolbox_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='A ROS 2 ament_python package for launching slam_toolbox.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # If you have executable Python nodes, add them here.
            # 'node_name = slam_tool.node_module:main',
        ],
    },
)
