from setuptools import setup, find_packages

package_name = 'mower_knife_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS 2 node for controlling a mower knife motor via serial interface',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'knife_controller_node = mower_knife_controller.knife_controller_node:main',
        ],
    },
)