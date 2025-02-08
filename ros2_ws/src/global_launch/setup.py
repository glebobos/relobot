from setuptools import setup

package_name = 'global_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Launch package for managing multiple nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
    data_files=[
        ('share/' + package_name + '/launch', ['launch/launch.py']),
    ],
)