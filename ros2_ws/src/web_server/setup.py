from setuptools import setup

package_name = 'web_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('lib/' + package_name + '/templates', ['web_server/templates/index.html']),
    ],
    py_modules=[],
    install_requires=[
        'setuptools',
        'Flask',  # Non-ROS dependency
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A web server ROS 2 package using Flask',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server_node = web_server.web_server_node:main',
        ],
    },
)