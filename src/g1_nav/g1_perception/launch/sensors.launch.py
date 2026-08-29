#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    rgbd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("g1_perception"),
                "launch",
                "rgbd_camera.launch.py"
            ])
        )
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("g1_perception"),
                "launch",
                "imu.launch.py"
            ])
        )
    )

    return LaunchDescription([
        rgbd,
        imu,
    ])