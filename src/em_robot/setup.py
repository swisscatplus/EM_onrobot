import os
from glob import glob
from setuptools import setup

package_name = 'em_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.submodules'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=[
        'setuptools',
        'dynamixel_sdk',
    ],
    zip_safe=True,
    maintainer='mariano',
    maintainer_email='edy.mariano@epfl.com',
    description='Manage Localization and Movement of EM Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'em_movement = em_robot.em_movement:main',
            'em_localization = em_robot.em_localization:main',
        ],
    },
)
