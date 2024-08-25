from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'ros2_gazebo_robot_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rexilius',
    maintainer_email='onoel2050@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'monitor_computer_node = ros2_gazebo_robot_monitor.monitor_computer:main',
        'monitor_robot_node = ros2_gazebo_robot_monitor.monitor_robot:main',
        ],
    },
)
