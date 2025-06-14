from setuptools import setup, find_packages

package_name = 'web_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
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
            'web_server_node = web_server.web_server_node:main',
        ],
    },
)
