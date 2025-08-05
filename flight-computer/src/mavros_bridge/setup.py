from setuptools import setup

package_name = 'mavros_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI Rover Team',
    maintainer_email='rover@example.com',
    description='Bridge between ROS2 navigation stack and MAVROS for ArduPilot communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_mavlink.py = mavros_bridge.nav_to_mavlink:main',
        ],
    },
)