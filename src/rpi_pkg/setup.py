import os
from glob import glob
from setuptools import setup

package_name = 'rpi_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.submodules'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swisscat',
    maintainer_email='swisscat@todo.todo',
    description='TODO: Package description',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi_com_motors = rpi_pkg.motors:main',
            'rpi_cam = rpi_pkg.cam:main',
        ],
    },
)
