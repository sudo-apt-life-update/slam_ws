#!/usr/bin/env python3
"""
description.launch.py

Publishes:
    - /robot_description
    - Complete TF tree from the G1 URDF

This is the canonical robot description for the project.

Joint states are expected to come from:
    - Isaac Sim
    - joint_state_publisher
    - the real robot
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition

def generate_launch_description():

    pkg = get_package_share_directory("g1_description")

    urdf_path = os.path.join(
        pkg,
        "urdf",
        "g1_29dof.urdf",
    )

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim = LaunchConfiguration("use_sim")  

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),

        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use Isaac Sim joint states instead of joint_state_publisher",
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=UnlessCondition(use_sim), 
            # Making joint_state_publisher conditional to only run when ISAAC Sim is off
            # ros2 launch g1_description description.launch.py use_sim:=true
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_desc,
                    "use_sim_time": use_sim_time,
                    # use_sim_time=True while in simulation 
                    # on the physical G1 set use_sim_time to False
                    # ros2 launch g1_description description.launch.py use_sim_time:=false
                }
            ],
        ),

    ])