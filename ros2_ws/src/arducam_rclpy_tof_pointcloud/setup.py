import os
from glob import glob
from setuptools import setup

package_name = 'arducam_rclpy_tof_pointcloud'

setup(
    name=package_name,
    version='0.0.2',  # incremented version for new features
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # include config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='arducam',
    author_email='dennis@arducam.com',
    maintainer='arducam',
    maintainer_email='dennis@arducam.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Arducam TOF Camera Examples with additional laser scan filtering and interpolation.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_pointcloud = ' + package_name + '.tof_pointcloud:main',
        ],
    },
)
