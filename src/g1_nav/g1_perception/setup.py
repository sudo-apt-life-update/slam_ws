import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Without these three, FindPackageShare('g1_perception') resolves but
        # the launch/, config/ and rviz/ directories are missing from the
        # install space, so every launch include fails at runtime.
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charu',
    maintainer_email='cs.sharma2112@gmail.com',
    description='Package for sensors',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ], 
    },
    entry_points={
        'console_scripts': [
            'g1_camera_bridge = g1_perception.g1_camera_bridge:main',
            'g1_state_bridge = g1_perception.g1_state_bridge:main',
            'apriltag_localizer = g1_perception.apriltag_localizer:main',
        ],
    },
)
