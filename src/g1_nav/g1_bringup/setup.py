import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charu',
    maintainer_email='cs.sharma2112@gmail.com',
    description='For Launch files',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'check_sensors = g1_bringup.check_sensors:main',
            'check_imu_axes = g1_bringup.check_imu_axes:main',
            'calibrate_camera = g1_bringup.calibrate_camera:main',
            'nav2_watch = g1_bringup.nav2_watch:main',
        ],
    },
)
