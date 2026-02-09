# Copyright 2025 ReloBot Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import setup, find_packages
import os

package_name = 'web_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    include_package_data=True,  # Ensure package data is included
    package_data={
        package_name: ['templates/*.html', 'templates/*.js', 'templates/*.css'],  # Include all HTML files in templates
    },
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
            'web_server_node = web_server.main:main',
        ],
    },
)
