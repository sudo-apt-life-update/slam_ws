"""Localise against the prior map using RTAB-Map itself, not AMCL.

    ros2 launch g1_bringup rtabmap_localization.launch.py

WHY THIS EXISTS

AMCL matches the live /scan (a 141-ray, 70 degree wedge) against the map
geometrically. That works well near distinctive geometry -- corners, the
AprilTag -- but along a long, straight, feature-poor corridor the wedge
cannot tell one metre of travel from the next: walls parallel to the direction
of travel look the same from every point along them. Measured directly: AMCL's
position sigma grew past 1.7 m over a 6 m walk down exactly such a corridor.

RTAB-Map's own localisation mode recognises PLACES by visual appearance --
doors, signage, floor markings, equipment -- the same mechanism that closed
loops when this map was built. A blank corridor wall defeats geometric
matching and visual matching alike, but the moment the robot passes a door, a
sign, or a piece of equipment (all visible in the corridor's own colour feed),
appearance matching gets a real fix that scan geometry never could.

If this ALSO fails to hold through the corridor, that points to a genuinely
under-observed stretch rather than a matcher problem, and is the trigger to
revisit AMCL (see g1_navigation/config/amcl.yaml) or add a second AprilTag
partway down the corridor as a periodic hard re-lock.

HOW THIS DIFFERS FROM rtabmap_mapping.launch.py

    Mem/IncrementalMemory: false   -- localise against the existing map,
                                       never add new nodes to it
    Mem/InitWMWithAllNodes: true   -- load the WHOLE prior map into Working
                                       Memory, not just its last session
    publish_tf: true               -- RTAB-Map now owns map -> odom (nothing
                                       else may publish it)
    delete_db_on_start: FIXED false -- never overridable here. Deleting the
                                       database out from under a localisation
                                       run destroys the only map that exists.

The AprilTag stays wired in as a landmark (publish_landmark, on by default).
In mapping mode a bad landmark reading could permanently warp the graph --
that is what caused the walk 5 rotation bug. In localisation mode the graph is
frozen (Mem/IncrementalMemory: false), so a tag sighting can only correct the
CURRENT pose estimate, not the map. That is the safe way to use it.

PREREQUISITES

    ~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1
    ssh unitree@192.168.123.164 'cd ~/image_server && ./run_slam.sh --fps 15'
    export CYCLONEDDS_URI=<install>/g1_bringup/share/g1_bringup/config/cyclonedds_local.xml

WATCHING IT

    ros2 topic echo /localization_pose      # RTAB-Map's equivalent of /amcl_pose
    ros2 run rtabmap_viz rtabmap_viz         # see which node it's currently matching
"""

import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            'database_path',
            default_value=os.path.expanduser('~/g1_maps/walk5_nolandmark.db'),
            description='the prior map to localise against -- the corrected, '
                        'landmark-free reprocessing of walk 5.'),
        DeclareLaunchArgument(
            'apriltag', default_value='true',
            description='feed the AprilTag in as a landmark correction. Safe '
                        'here: Mem/IncrementalMemory is false, so a sighting '
                        'can only pull the current pose estimate, never warp '
                        'the map.'),
        DeclareLaunchArgument(
            'use_cmd_vel', default_value='false',
            description='start loco_bridge with the gate closed. Enable and '
                        'call the /enable service once localisation is '
                        'confirmed, same as before.'),
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

    camera = Node(
        package='g1_perception', executable='g1_camera_bridge',
        name='g1_camera_bridge', output='screen',
        parameters=[{'server_address': camera_address, 'port': 5556,
                     'use_sim_time': use_sim_time}],
    )

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

    stabilizer = Node(
        package='g1_locomotion', executable='base_stabilizer',
        name='base_stabilizer', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    apriltag = Node(
        package='g1_perception', executable='apriltag_localizer',
        name='apriltag_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('apriltag')),
        parameters=[{
            # tag_id defaults to 10 in the node itself -- this is the
            # original tag, at the original map position (see rtabmap.yaml /
            # INSTRUCTIONS.md section 11).
            #
            # Landmark only -- /initialpose has no meaning here, AMCL is not
            # running and RTAB-Map does not consume that topic.
            'publish_initialpose': False,
            'publish_landmark': True,
            'use_sim_time': use_sim_time,
        }],
    )

    # SECOND tag, partway down the corridor that broke AMCL (measured
    # 2026-08-25, INSTRUCTIONS.md section 11b: "8 genuine confirmed matches"
    # once both tags were anchoring the pose). Ran as a manual `ros2 run`
    # side-process for most of that session -- moved into the launch tree so
    # the next run gets it for free instead of relying on someone remembering
    # to start it by hand.
    apriltag_20 = Node(
        package='g1_perception', executable='apriltag_localizer',
        name='apriltag_localizer_20', output='screen',
        condition=IfCondition(LaunchConfiguration('apriltag')),
        parameters=[{
            'tag_id': 20,
            'tag_map_x': 2.91, 'tag_map_y': 12.01, 'tag_map_yaw': 0.0,
            'publish_initialpose': False,
            'publish_landmark': True,
            'use_sim_time': use_sim_time,
        }],
    )

    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        parameters=[rtabmap_params, {
            'database_path': LaunchConfiguration('database_path'),
            # HARD-CODED, not a launch argument. This node's only job is to
            # localise against an existing map -- there is no scenario here
            # where deleting it on start is the right default, and no
            # scenario worth a footgun for.
            'delete_db_on_start': False,
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            # The reprocessed database was built with LandmarksIgnored=true
            # (deliberately, to keep the AprilTag from warping the MAPPING
            # graph -- see walk 5's rotation bug). RTAB-Map persists that
            # setting INTO the database and reloads it on every future launch,
            # silently discarding every tag correction here too unless
            # overridden. Safe to re-enable in localisation mode: the map is
            # frozen (Mem/IncrementalMemory=false), so a landmark can only
            # correct the CURRENT pose estimate, never touch the graph again.
            'Optimizer/LandmarksIgnored': 'false',
            'publish_tf': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            # Same name collision as in mapping: apriltag_localizer's
            # PoseStamped vs. RTAB-Map's expected AprilTagDetectionArray. The
            # tag reaches RTAB-Map via /landmark_detection instead.
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
        GroupAction(actions=[stabilizer, apriltag, apriltag_20]),
        rtabmap, rviz,
    ])
