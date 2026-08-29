"""Build a map by walking the robot, using RGB-D graph SLAM.

    ros2 launch g1_bringup rtabmap_mapping.launch.py

The replacement for mapping.launch.py. That one used slam_toolbox, which closes
loops by geometric scan matching and could not do it with a 70 degree FOV whose
returns start at 1.5 m -- four walks, four open horseshoes, zero closures.
RTAB-Map recognises places by RGB appearance instead. See rtabmap.yaml for the
full argument and the measured numbers behind it.

Everything upstream of the SLAM node is unchanged and already proven: the same
camera bridge, the same calibrated extrinsic, the same leg odometry, the same
base_stabilized level frame.

PREREQUISITES, same as before:

    ~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1
    ssh unitree@192.168.123.164 'cd ~/image_server && ./run_slam.sh --fps 15'

and set the DDS config, or the ros2 CLI will hang and DDS will flood eno1:

    export CYCLONEDDS_URI=<install>/g1_bringup/share/g1_bringup/config/cyclonedds_local.xml

HOW TO WALK IT -- DIFFERENT ADVICE FROM slam_toolbox

The old guidance was "walk closed circuits". That was right for scan matching
and is only half right here. What RTAB-Map needs is for the camera to SEE the
same thing twice, and a 70 degree FOV means facing the same way is as important
as standing in the same place:

  * When you return to a spot, RETURN FACING THE SAME DIRECTION. Walking back
    down a corridor the other way shows the camera a completely different
    scene, and appearance matching has nothing to work with. This is the single
    biggest difference from the previous four walks.
  * Point the camera at TEXTURE. Machinery, signage, pillars, clutter. A blank
    white wall at 2 m yields almost no features. Bare walls are where this
    method is weakest, exactly as scan matching is weakest in open space.
  * Turn slowly, and pause a beat when you arrive somewhere you have been.
  * Watch the terminal. Every accepted closure logs a line containing
    "Loop closure" with the two node ids. If none appear on a return visit,
    stop and say so rather than walking further -- more walking will not fix it,
    and that was the mistake the previous attempts kept repeating.

WATCHING IT WORK

    ros2 topic echo /rtabmap/info --field loopClosureId
    ros2 run rtabmap_viz rtabmap_viz            # graph + closures, live

SAVING THE RESULT

The 2D grid, for Nav2 and AMCL:

    ros2 run nav2_map_server map_saver_cli -f ~/g1_map_rtab --ros-args -p save_map_timeout:=20.0

The 3D cloud -- the other layer of the layered map -- is exported straight from
the database written during the walk. Nothing extra needs to run for this:

    rtabmap-export --cloud --output ~/g1_cloud ~/.ros/rtabmap.db

CONTINUING AN EARLIER SESSION

By default this DELETES the database and starts clean, because silently
appending to a session with a broken graph wastes a walk. To extend instead:

    ros2 launch g1_bringup rtabmap_mapping.launch.py delete_db:=false
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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('g1_description')
    bringup_share = get_package_share_directory('g1_bringup')
    navigation_share = get_package_share_directory('g1_navigation')

    urdf_path = os.path.join(description_share, 'urdf', 'g1_29dof.urdf')
    with open(urdf_path, 'r') as handle:
        robot_description = handle.read()

    rtabmap_params = os.path.join(navigation_share, 'config', 'rtabmap.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_address = LaunchConfiguration('camera_address')
    iface = LaunchConfiguration('iface')

    arguments = [
        DeclareLaunchArgument('iface', default_value='eno1'),
        DeclareLaunchArgument('camera_address',
                              default_value='192.168.123.164'),
        DeclareLaunchArgument('publish_frequency', default_value='200.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_loco_server', default_value='true'),
        DeclareLaunchArgument(
            'delete_db', default_value='true',
            description='wipe the database and map from scratch. false '
                        'appends to the existing session.'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/.ros/rtabmap.db'),
            description='where the RGB-D graph is written. This file IS the '
                        'map -- the 3D cloud is exported from it afterwards.'),
        DeclareLaunchArgument(
            'apriltag', default_value='true',
            description='run the tag localiser. It cannot feed a landmark '
                        'constraint into the graph, but it gives an '
                        'independent measurement of how far the map has '
                        'drifted whenever tag 10 is in view.'),
        DeclareLaunchArgument(
            'use_cmd_vel', default_value='false',
            description='start loco_bridge. Off for mapping: you drive with '
                        'the joystick, and a second instance cannot start '
                        'anyway.'),
        DeclareLaunchArgument('start_enabled', default_value='false'),
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

    # NO point_cloud_xyzrgb and NO pointcloud_to_laserscan here. RTAB-Map
    # consumes the depth IMAGE directly and builds its own cloud internally at
    # Grid/DepthDecimation -- running ours as well would duplicate the work at
    # full resolution for nothing. Add point_cloud_xyzrgb back only if you want
    # a live cloud in RViz, and expect it to cost a core.
    camera = Node(
        package='g1_perception', executable='g1_camera_bridge',
        name='g1_camera_bridge', output='screen',
        parameters=[{'server_address': camera_address, 'port': 5556,
                     'use_sim_time': use_sim_time}],
    )

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

    loco_bridge = Node(
        package='g1_locomotion', executable='loco_bridge',
        name='g1_loco_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('use_cmd_vel')),
        parameters=[{'server_address': '127.0.0.1', 'port': 5558,
                     'start_enabled': LaunchConfiguration('start_enabled'),
                     'use_sim_time': use_sim_time}],
    )

    # Supplies odom -> base_stabilized, the level frame RTAB-Map builds the
    # occupancy grid in. Without it the grid tilts with the robot's 7.5 deg
    # stance lean and the floor starts registering as an obstacle.
    stabilizer = Node(
        package='g1_locomotion', executable='base_stabilizer',
        name='base_stabilizer', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    apriltag = Node(
        package='g1_perception', executable='apriltag_localizer',
        name='apriltag_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('apriltag')),
        parameters=[{'publish_initialpose': False,
                     'use_sim_time': use_sim_time}],
    )

    # Publishes map -> odom, /map, and /cloud_map.
    #
    # NOT a lifecycle node, unlike slam_toolbox -- no nav2_lifecycle_manager
    # here, and therefore none of the bond-timeout trouble that silently tore
    # slam_toolbox down mid-walk.
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        parameters=[rtabmap_params, {
            'database_path': LaunchConfiguration('database_path'),
            # RTAB-Map also accepts '-d' / '--delete_db_on_start' as a plain
            # argv flag, but that path cannot express "false" -- the flag is
            # either present or absent. The ROS parameter takes a real boolean,
            # so the launch argument maps onto it directly. value_type=bool is
            # required: a LaunchConfiguration is a STRING, and the node declares
            # this parameter as a bool -- without the cast it fails at startup
            # with a type-mismatch on an otherwise valid command line.
            'delete_db_on_start': ParameterValue(
                LaunchConfiguration('delete_db'), value_type=bool),
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            # NAME COLLISION, not a functional need. RTAB-Map subscribes to
            # `tag_detections` expecting apriltag_msgs/AprilTagDetectionArray;
            # apriltag_localizer has published a geometry_msgs/PoseStamped on
            # that name since before RTAB-Map was in the picture. Same name,
            # different type, so ROS logs an alarming "incompatible QoS. No
            # messages will be sent to it" on every startup that means nothing.
            #
            # The tag reaches RTAB-Map via /landmark_detection instead, which is
            # the typed interface built for it. Moving RTAB-Map's unused
            # subscription out of the way keeps the log honest -- warnings that
            # are always present are warnings nobody reads.
            ('tag_detections', '/rtabmap_unused_tag_detections'),
        ],
    )

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
        GroupAction(actions=[stabilizer, apriltag]),
        rtabmap, rviz,
    ])
