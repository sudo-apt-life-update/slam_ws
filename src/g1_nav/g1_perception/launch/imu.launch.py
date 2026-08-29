#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_namespace = LaunchConfiguration("imu_namespace")

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "imu_namespace",
            default_value="imu"
        ),

    ])