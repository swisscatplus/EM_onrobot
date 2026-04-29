import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'em_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    py_modules=['tf_transformations'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'profiles'), glob(os.path.join('config', 'profiles', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=[
        'setuptools',
        'dynamixel_sdk',
        'opencv-python',
        'python3-picamera2',
        'python3-transforms3d',
        'pyyaml',
    ],

    zip_safe=True,
    maintainer='mariano',
    maintainer_email='edy.mariano@epfl.com',
    description='On-robot localization and base control for EM Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_controller = em_robot.base_controller_node:main',
            'marker_map_publisher = em_robot.marker_map_publisher:main',
            'localization = em_robot.localization_node:main',
            'localization_capture = em_robot.localization_capture:main',
            'aruco_camera_test = em_robot.aruco_camera_test_node:main',
            'marker_survey = em_robot.marker_survey_node:main',
            'imu_mock = em_robot.imu_mock_node:main',
            'robot_diagnostics = em_robot.robot_diagnostics_node:main',
            'rgb_led_controller = em_robot.rgb_led_controller_node:main',
            'robot_state_manager = em_robot.robot_state_manager_node:main',
        ],
    },
)
