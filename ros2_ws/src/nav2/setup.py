from setuptools import setup, find_packages

package_name = 'nav2'

setup(
    name=package_name,
    version='0.0.0',
    # Automatically find python packages (e.g. frontier_explorer)
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/explore.launch.py', 'launch/navigation_launch.py']),
        ('share/' + package_name + '/config', ['config/explore.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A package to control the nav2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)