from setuptools import find_packages, setup

package_name = 'g1_locomotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charu',
    maintainer_email='cs.sharma2112@gmail.com',
    description='Odometry and velocity control for the G1.',
    license='MIT License',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_bridge = g1_locomotion.odom_bridge:main',
            'base_stabilizer = g1_locomotion.base_stabilizer:main',
            'loco_bridge = g1_locomotion.loco_bridge:main',
            'loco_cli = g1_locomotion.loco_cli:main',
            'fake_loco_server = g1_locomotion.fake_loco_server:main',
        ],
    },
)
