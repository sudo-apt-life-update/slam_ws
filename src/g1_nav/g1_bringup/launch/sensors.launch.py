"""Bring up every G1 sensor on the real robot.

    ros2 launch g1_bringup sensors.launch.py

Two helper processes must already be running, because neither can live inside a
ROS node:

  1. On this PC -- the DDS side (joints + IMU):
         cd ~/workspaces/unitree/unitree_sdk2/build
         ./bin/g1_state_server eno1
     unitree_sdk2 and ROS 2 each link their own CycloneDDS and corrupt the heap
     in one address space, hence the socket bridge.

  2. On the robot -- the camera:
         ssh unitree@192.168.123.164 'cd ~/image_server && ./run.sh'

Then check everything with:
        ros2 run g1_bringup check_sensors
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('g1_description')
    bringup_share = get_package_share_directory('g1_bringup')
    urdf_path = os.path.join(description_share, 'urdf', 'g1_29dof.urdf')

    with open(urdf_path, 'r') as handle:
        robot_description = handle.read()

    # rviz:=true implies cloud:=true -- an RViz layout with an empty
    # PointCloud2 display is a confusing way to start a verification session.
    cloud_enabled = PythonExpression([
        "'", LaunchConfiguration('cloud'), "' == 'true' or '",
        LaunchConfiguration('rviz'), "' == 'true'"])

    camera_address = LaunchConfiguration('camera_address')
    state_address = LaunchConfiguration('state_address')
    publish_frequency = LaunchConfiguration('publish_frequency')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_address', default_value='192.168.123.164',
            description='host running image_server (the robot)'),
        DeclareLaunchArgument(
            'state_address', default_value='127.0.0.1',
            description='host running g1_state_server (this PC)'),
        DeclareLaunchArgument(
            # [IMPORTANT] robot_state_publisher does NOT simply follow
            # /joint_states -- it republishes TF at publish_frequency, whose
            # default is 20 Hz. That default, not a slow joint stream, is why
            # earlier frames_*.gv dumps recorded 20.5 Hz.
            'publish_frequency', default_value='200.0',
            description='TF republish rate for robot_state_publisher'),
        DeclareLaunchArgument(
            # Real robot: wall clock. There is no /clock publisher here, and
            # true would make every TF lookup hang.
            'use_sim_time', default_value='false',
            description='true only under Isaac Sim'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='also start RViz2 with the sensor-check layout'),
        DeclareLaunchArgument(
            'cloud', default_value='false',
            description='turn depth+colour into /camera/cloud (PointCloud2). '
                        'Implied by rviz:=true, since RViz cannot render a '
                        'depth Image as a cloud on its own.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': publish_frequency,
                'use_sim_time': use_sim_time,
            }],
        ),

        # joints + both IMUs, via g1_state_server over ZMQ/TCP
        Node(
            package='g1_perception',
            executable='g1_state_bridge',
            name='g1_state_bridge',
            output='screen',
            parameters=[{
                'server_address': state_address,
                'port': 5557,
                'use_sim_time': use_sim_time,
            }],
        ),

        # colour + aligned depth + CameraInfo, via image_server over ZMQ
        Node(
            package='g1_perception',
            executable='g1_camera_bridge',
            name='g1_camera_bridge',
            output='screen',
            parameters=[{
                'server_address': camera_address,
                'port': 5556,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Depth image -> PointCloud2. RViz cannot do this itself.
        # depth_image_proc/point_cloud_xyzrgb would be the conventional choice
        # but is not installed; rtabmap_util is already built and comes from
        # the same family as the mapper we will use later, so no new dependency.
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyzrgb',
            name='point_cloud_xyzrgb',
            output='screen',
            condition=IfCondition(cloud_enabled),
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('cloud', '/camera/cloud'),
            ],
            parameters=[{
                # Exact sync, not approximate: the bridge stamps colour, depth
                # and both CameraInfos with one timestamp per frame (verified
                # 300/300 identical), so there is nothing to approximate. If
                # this ever starves, the stamps have drifted apart.
                'approx_sync': False,
                # The image topics are RELIABLE (best-effort dropped 8% of the
                # 1.84 MB depth frames to fragmentation), so match it here or
                # the subscription is incompatible and receives nothing.
                'qos': 1,
                'qos_camera_info': 1,
                # D435 depth is only trustworthy to a few metres, and the
                # camera is pitched 47.6 deg down so most of the frame is floor.
                'min_depth': 0.2,
                'max_depth': 6.0,
                # 848x480 is ~407k points per frame at 30 Hz. Halving each axis
                # keeps RViz responsive; drop to 1 if you need full density.
                'decimation': 2,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=[
                '-d', os.path.join(bringup_share, 'rviz', 'sensor_check.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
