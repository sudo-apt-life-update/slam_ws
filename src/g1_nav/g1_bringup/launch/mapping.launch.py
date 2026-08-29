"""Build a map by walking the robot.

    ros2 launch g1_bringup mapping.launch.py

Everything navigation.launch.py brings up EXCEPT the map server and AMCL --
those match against a prior map, and here we are making one. slam_toolbox
supplies map -> odom instead, and publishes the growing grid on /map.

PREREQUISITES, same as navigation.launch.py:

    ~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1
    ssh unitree@192.168.123.164 'cd ~/image_server && ./run_slam.sh'

and set the DDS config, or the ros2 CLI will hang and DDS will flood eno1:

    export CYCLONEDDS_URI=<install>/g1_bringup/share/g1_bringup/config/cyclonedds_local.xml

HOW TO WALK IT

Loop closure is what keeps the map straight, and it needs actual loops:

  * Walk CLOSED circuits and return to where you started. An out-and-back
    corridor gives the solver almost nothing to close on.
  * Turn slowly. With a 70 degree FOV a fast turn moves the whole scene out of
    view between scans, and consecutive scans stop overlapping.
  * Pause a moment at corners and doorways -- they are the features loop
    closure keys on.
  * Revisit the start before finishing.

SAVING THE RESULT

    ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -p save_map_timeout:=10000

Then point navigation.launch.py at it:

    ros2 launch g1_bringup navigation.launch.py map:=$HOME/my_map.yaml
"""

import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            GroupAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('g1_description')
    bringup_share = get_package_share_directory('g1_bringup')
    navigation_share = get_package_share_directory('g1_navigation')
    perception_share = get_package_share_directory('g1_perception')

    urdf_path = os.path.join(description_share, 'urdf', 'g1_29dof.urdf')
    with open(urdf_path, 'r') as handle:
        robot_description = handle.read()

    scan_params = os.path.join(perception_share, 'config', 'scan.yaml')
    slam_params = os.path.join(navigation_share, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_address = LaunchConfiguration('camera_address')
    iface = LaunchConfiguration('iface')

    arguments = [
        DeclareLaunchArgument('iface', default_value='eno1'),
        DeclareLaunchArgument('camera_address',
                              default_value='192.168.123.164'),
        DeclareLaunchArgument('publish_frequency', default_value='200.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='on by default here -- you need to '
                                          'see the map as it builds'),
        DeclareLaunchArgument('use_loco_server', default_value='true'),
        DeclareLaunchArgument(
            'use_cmd_vel', default_value='false',
            description='start loco_bridge. Off for mapping: you drive with '
                        'the joystick, and a second instance cannot start '
                        'anyway.'),
        DeclareLaunchArgument(
            'start_enabled', default_value='false',
            description='open the /cmd_vel gate. Mapping is normally driven by '
                        'joystick, so this stays closed.'),
    ]

    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'use_sim_time': use_sim_time,
        }],
    )

    state_bridge = Node(
        package='g1_perception', executable='g1_state_bridge',
        name='g1_state_bridge', output='screen',
        parameters=[{'server_address': '127.0.0.1', 'port': 5557,
                     'use_sim_time': use_sim_time}],
    )

    camera = GroupAction(actions=[
        Node(
            package='g1_perception', executable='g1_camera_bridge',
            name='g1_camera_bridge', output='screen',
            parameters=[{'server_address': camera_address, 'port': 5556,
                         'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb',
            name='point_cloud_xyzrgb', output='screen',
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('cloud', '/camera/cloud'),
            ],
            parameters=[{
                'approx_sync': False, 'qos': 1, 'qos_camera_info': 1,
                'min_depth': 0.2, 'max_depth': 6.0, 'decimation': 2,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])

    # Not a ROS node: launch_ros would append --ros-args, which this binary's
    # own parser rejects. CYCLONEDDS_URI is cleared because ROS traffic is
    # pinned to loopback and that would hide eno1 from the SDK.
    loco_server = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('g1_loco_server'),
                          'lib', 'g1_loco_server', 'g1_loco_server'),
             ['--iface=', iface]],
        name='g1_loco_server', output='screen',
        condition=IfCondition(LaunchConfiguration('use_loco_server')),
        additional_env={'CYCLONEDDS_URI': ''},
    )

    odom_bridge = Node(
        package='g1_locomotion', executable='odom_bridge',
        name='g1_odom_bridge', output='screen',
        parameters=[{'server_address': '127.0.0.1', 'port': 5560,
                     'odom_frame': 'odom', 'base_frame': 'base_footprint',
                     'publish_tf': True, 'use_sim_time': use_sim_time}],
    )

    # Off by default. Mapping is driven by joystick, so nothing here publishes
    # /cmd_vel -- and loco_bridge holds a single-instance lock, so starting a
    # second one just dies with "another g1_loco_bridge is already running".
    loco_bridge = Node(
        package='g1_locomotion', executable='loco_bridge',
        name='g1_loco_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('use_cmd_vel')),
        parameters=[{'server_address': '127.0.0.1', 'port': 5558,
                     'start_enabled': LaunchConfiguration('start_enabled'),
                     'use_sim_time': use_sim_time}],
    )

    scan_group = GroupAction(actions=[
        Node(
            package='g1_locomotion', executable='base_stabilizer',
            name='base_stabilizer', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan', output='screen',
            remappings=[('cloud_in', '/camera/cloud'), ('scan', '/scan')],
            parameters=[scan_params, {'use_sim_time': use_sim_time}],
        ),
    ])

    # Publishes map -> odom and the growing /map.
    #
    # slam_toolbox IS a lifecycle node on Jazzy, contrary to what its docs
    # imply for the async variant. Started without a manager it sits in
    # `unconfigured` forever: it never subscribes to /scan, never publishes a
    # map, and logs nothing after "Node using stack size" -- which reads like a
    # silent hang rather than a node waiting to be told to start.
    # Check with:  ros2 lifecycle get /slam_toolbox
    slam_group = GroupAction(actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['slam_toolbox'],
                # DISABLE THE BOND. nav2_lifecycle_manager expects each managed
                # node to maintain a bond heartbeat; slam_toolbox does not, so
                # the manager decides it "was unable to be reached after 4.00s"
                # and TEARS DOWN the node it has just activated:
                #
                #   Server slam_toolbox was unable to be reached after 4.00s by bond
                #   Failed to bring up all requested nodes. Aborting bringup.
                #
                # The failure is quiet in the worst way -- slam_toolbox logs a
                # clean "Activating", then simply stops mapping. A whole walk
                # was lost to this. 0.0 disables the check.
                'bond_timeout': 0.0,
                'attempt_respawn_reconnection': False,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(bringup_share, 'rviz',
                                      'localization.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(arguments + [
        robot_state, state_bridge, camera,
        loco_server, odom_bridge, loco_bridge,
        scan_group, slam_group, rviz,
    ])
