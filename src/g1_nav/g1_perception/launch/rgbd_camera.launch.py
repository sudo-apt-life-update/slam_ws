#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    camera_namespace = LaunchConfiguration("camera_namespace")

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "camera_namespace",
            default_value="camera"
        ),

    ])