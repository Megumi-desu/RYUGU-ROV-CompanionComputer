from setuptools import setup
from glob import glob
import os

package_name = 'ryugu_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Ament index resource marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahmat',
    maintainer_email='rachmatgifari99@gmail.com',
    description='ROS2 control package for RYUGU ROV — underwater vehicle control via MAVROS and ArduSub v4.5.7',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Production nodes
            'gcs_bridge_node = ryugu_control.gcs_bridge_node:main',
            'webcam_streamer = ryugu_control.webcam_streamer:main',
            # Test / debug nodes
            'test_arming_mode = ryugu_control.test_arming_mode:main',
            'test_thrusters_gripper = ryugu_control.test_thrusters_gripper:main',
            'test_sensor_reader = ryugu_control.test_sensor_reader:main',
        ],
    },
)
