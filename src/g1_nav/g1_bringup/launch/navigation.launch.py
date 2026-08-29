"""Everything the robot needs to be navigated, short of Nav2 itself.

    ros2 launch g1_bringup navigation.launch.py

Brings up, in one place, what used to be three terminals and two workspaces:

    robot_state_publisher   URDF -> TF, base_footprint down to the camera
    g1_state_bridge         /joint_states, /imu/data, /imu_torso/data
    g1_camera_bridge        colour + aligned depth + CameraInfo
    g1_loco_server          the DDS side: odometry out, velocity in
    odom_bridge             /odom and the odom -> base_footprint transform
    loco_bridge             /cmd_vel -> the robot, behind a closed gate
    map_server              the prior map on /map, lifecycle-managed
    base_stabilizer         odom -> base_stabilized, a LEVEL frame
    pointcloud_to_laserscan /scan, height-filtered for AMCL
    amcl                    map -> odom, localising against the prior map

What is still missing is Nav2 itself -- costmaps, planner, controller. This
brings the robot to the point where a planner has everything it needs.

AMCL publishes NO transform until it has processed a scan, and with a 70 degree
FOV it will not find itself from scratch. Give it a pose with RViz's
"2D Pose Estimate".

ONE PREREQUISITE this launch cannot cover: g1_state_server. It lives outside
both ROS workspaces, in ~/workspaces/unitree, because it links unitree_sdk2.
Start it first:

    ~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1

and the camera server on the robot:

    ssh unitree@192.168.123.164 'cd ~/image_server && ./run_slam.sh'

Useful arguments:
    use_camera:=false     bring up locomotion only
    use_map:=false        skip the map server
    rviz:=true            open the sensor-check layout
    cloud:=true           also produce /camera/cloud
    scan:=false           skip /scan (and therefore AMCL)
    localization:=false    map only, no AMCL
    start_enabled:=true   open the /cmd_vel gate at startup (see the warning
                          in loco_bridge -- default is CLOSED for a reason)
"""

import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            GroupAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('g1_description')
    bringup_share = get_package_share_directory('g1_bringup')
    navigation_share = get_package_share_directory('g1_navigation')

    urdf_path = os.path.join(description_share, 'urdf', 'g1_29dof.urdf')
    with open(urdf_path, 'r') as handle:
        robot_description = handle.read()

    default_map = os.path.join(navigation_share, 'maps', 'map.yaml')
    perception_share = get_package_share_directory('g1_perception')
    scan_params = os.path.join(perception_share, 'config', 'scan.yaml')
    amcl_params = os.path.join(navigation_share, 'config', 'amcl.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_address = LaunchConfiguration('camera_address')
    iface = LaunchConfiguration('iface')
    map_yaml = LaunchConfiguration('map')

    # AMCL is useless without a map to match against and a scan to match with,
    # so localisation implies both rather than failing quietly.
    localization_enabled = PythonExpression([
        "'", LaunchConfiguration('localization'), "' == 'true' and '",
        LaunchConfiguration('use_map'), "' == 'true'"])

    # The cloud is needed by anything that wants it directly (rviz) AND by the
    # scan, which is built from it. One cloud node serves both.
    cloud_enabled = PythonExpression([
        "'", LaunchConfiguration('cloud'), "' == 'true' or '",
        LaunchConfiguration('rviz'), "' == 'true' or '",
        LaunchConfiguration('scan'), "' == 'true'"])

    arguments = [
        DeclareLaunchArgument(
            'iface', default_value='eno1',
            description='network interface to the robot, for the DDS side'),
        DeclareLaunchArgument(
            'camera_address', default_value='192.168.123.164',
            description='host running slam_image_server (the robot)'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='occupancy grid to localise against'),
        DeclareLaunchArgument(
            # robot_state_publisher does NOT follow /joint_states -- it
            # republishes TF at this rate, default 20 Hz. That default is why
            # the earliest frame dumps recorded 20.5 Hz.
            'publish_frequency', default_value='200.0',
            description='TF republish rate'),
        DeclareLaunchArgument(
            # Real robot: wall clock. Nothing publishes /clock here, and true
            # would make every TF lookup hang.
            'use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_map', default_value='true'),
        DeclareLaunchArgument('cloud', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'scan', default_value='true',
            description='produce /scan from the depth cloud, for AMCL. '
                        'Implies cloud:=true, since the scan is built from it.'),
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='run AMCL against the map, producing map -> odom. '
                        'Needs use_map:=true and scan:=true. Give the initial '
                        'pose with RViz\'s 2D Pose Estimate -- with a 70 deg '
                        'FOV, global localisation from scratch does not '
                        'converge reliably.'),
        DeclareLaunchArgument(
            'start_enabled', default_value='false',
            description='open the /cmd_vel gate immediately. Leave false '
                        'unless you want the robot to move the moment '
                        'something publishes a Twist.'),
        DeclareLaunchArgument(
            'use_loco_server', default_value='true',
            description='start the real DDS-side g1_loco_server. Set false to '
                        'test the whole stack with the robot off, having '
                        'started `ros2 run g1_locomotion fake_loco_server` '
                        'first -- it binds the same 5558/5560 and stands in '
                        'for the robot.'),
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
        package='g1_perception',
        executable='g1_state_bridge',
        name='g1_state_bridge',
        output='screen',
        parameters=[{'server_address': '127.0.0.1', 'port': 5557,
                     'use_sim_time': use_sim_time}],
    )

    camera = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_camera')),
        actions=[
            Node(
                package='g1_perception',
                executable='g1_camera_bridge',
                name='g1_camera_bridge',
                output='screen',
                parameters=[{'server_address': camera_address, 'port': 5556,
                             'use_sim_time': use_sim_time}],
            ),
            Node(
                # depth_image_proc is not installed; rtabmap_util is, and comes
                # from the same family as the mapper we will use later.
                package='rtabmap_util',
                executable='point_cloud_xyzrgb',
                name='point_cloud_xyzrgb',
                output='screen',
                condition=IfCondition(cloud_enabled),
                remappings=[
                    ('rgb/image', '/camera/color/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('depth/image',
                     '/camera/aligned_depth_to_color/image_raw'),
                    ('cloud', '/camera/cloud'),
                ],
                parameters=[{
                    # Exact sync: the bridge stamps colour, depth and both
                    # CameraInfos with one timestamp per frame, verified
                    # 300/300 identical. Nothing to approximate.
                    'approx_sync': False,
                    # The image topics are RELIABLE (best-effort dropped 8% of
                    # the 1.84 MB depth frames to DDS fragmentation), so match
                    # it here or the subscription is incompatible.
                    'qos': 1,
                    'qos_camera_info': 1,
                    'min_depth': 0.2,
                    'max_depth': 6.0,
                    'decimation': 2,
                    'use_sim_time': use_sim_time,
                }],
            ),
        ],
    )

    # The DDS side of locomotion. Separate process because unitree_sdk2 and
    # ROS 2 each bundle a CycloneDDS and corrupt the heap in one address space.
    # ExecuteProcess, NOT Node. g1_loco_server is a plain binary that links
    # unitree_sdk2 -- it is not a ROS node and has its own argument parser.
    # launch_ros's Node action always appends `--ros-args -r __node:=...`,
    # which that parser rejects: it prints its usage and exits 1. The offline
    # test never caught this because it runs with use_loco_server:=false.
    loco_server = ExecuteProcess(
        cmd=[os.path.join(
                get_package_prefix('g1_loco_server'),
                'lib', 'g1_loco_server', 'g1_loco_server'),
             ['--iface=', iface]],
        name='g1_loco_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_loco_server')),
        # CLEAR CYCLONEDDS_URI FOR THIS PROCESS.
        #
        # ROS nodes here are pinned to loopback (config/cyclonedds_local.xml)
        # to stop CycloneDDS flooding eno1 with retries and wedging the ros2
        # CLI. But this binary is NOT a ROS node -- it is unitree_sdk2 talking
        # to the robot over eno1, and it inherits the same environment.
        #
        # It does read it, despite what the source comment in
        # g1_loco_server.cpp implies. Pinning DDS to `lo` made eno1 invisible
        # and the server aborted on startup with
        #     "eno1: does not match an available interface"
        #     "Failed to create domain explicitly"
        # taking odometry and /cmd_vel with it. Setting the variable empty
        # restores CycloneDDS's default interface discovery for this process
        # only.
        additional_env={'CYCLONEDDS_URI': ''},
    )

    odom_bridge = Node(
        package='g1_locomotion',
        executable='odom_bridge',
        name='g1_odom_bridge',
        output='screen',
        parameters=[{
            'server_address': '127.0.0.1',
            'port': 5560,
            'odom_frame': 'odom',
            # Chains onto the URDF, whose root is base_footprint. The rest of
            # the tree (base_link, pelvis, camera) hangs below it.
            'base_frame': 'base_footprint',
            'publish_tf': True,
            'use_sim_time': use_sim_time,
        }],
    )

    loco_bridge = Node(
        package='g1_locomotion',
        executable='loco_bridge',
        name='g1_loco_bridge',
        output='screen',
        parameters=[{
            'server_address': '127.0.0.1',
            'port': 5558,
            'start_enabled': LaunchConfiguration('start_enabled'),
            'use_sim_time': use_sim_time,
        }],
    )

    # Depth cloud -> /scan, for AMCL. Two nodes, because the height filter
    # needs a level frame and the robot's tree does not contain one: the
    # stabilizer supplies odom -> base_stabilized, and the converter filters
    # in it. See g1_perception/config/scan.yaml for why the band is 0.3-1.5 m.
    scan_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('scan')),
        actions=[
            Node(
                package='g1_locomotion',
                executable='base_stabilizer',
                name='base_stabilizer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                output='screen',
                remappings=[('cloud_in', '/camera/cloud'),
                            ('scan', '/scan')],
                parameters=[scan_params, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    # map_server and amcl are both LIFECYCLE nodes: they publish nothing until
    # configured and activated, and report no error while idle. The lifecycle
    # manager does those transitions, which is why it is here rather than left
    # to the user.
    #
    # Two managers rather than one, chosen with the opposite condition, because
    # the managed list has to be known when the launch file is written and
    # `localization` is not resolved until run time. A manager that waits for a
    # node which was never started blocks forever.
    map_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_map')),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_yaml,
                             'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                condition=IfCondition(localization_enabled),
                parameters=[amcl_params, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                condition=IfCondition(localization_enabled),
                parameters=[{
                    'autostart': True,
                    # Order matters: amcl asks map_server for the map on
                    # activation, so the map has to be up first.
                    'node_names': ['map_server', 'amcl'],
                    'use_sim_time': use_sim_time,
                }],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                condition=UnlessCondition(localization_enabled),
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server'],
                    'use_sim_time': use_sim_time,
                }],
            ),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(
            bringup_share, 'rviz',
            # Localisation layout when AMCL is running, sensor-check layout
            # otherwise. They need different fixed frames, so one config
            # cannot serve both.
            'localization.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(arguments + [
        robot_state, state_bridge, camera,
        loco_server, odom_bridge, loco_bridge,
        scan_group, map_group, rviz,
    ])
