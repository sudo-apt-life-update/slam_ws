"""Autonomous navigation: RTAB-Map localisation + Nav2 costmaps/planner/controller.

    ros2 launch g1_bringup nav2_navigation.launch.py

Adds three things on top of rtabmap_localization.launch.py:

    point_cloud_xyzrgb + pointcloud_to_laserscan   -- /scan, for the costmaps
    nav2_bringup's navigation_launch.py            -- controller, planner,
                                                       behaviours, bt_navigator
    g1_locomotion loco_bridge, gate OPEN by default -- Nav2 cannot drive the
                                                       robot through a closed
                                                       gate, and there is no
                                                       joystick safety net once
                                                       it is planning its own
                                                       paths

See g1_navigation/config/nav2_params.yaml for the costmap/planner/controller
reasoning, and INSTRUCTIONS.md section 11b for why this sits on RTAB-Map
rather than AMCL.

PREREQUISITES, same as rtabmap_localization.launch.py:

    ~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1
    ssh unitree@192.168.123.164 'cd ~/image_server && ./run_slam.sh --fps 15'
    export CYCLONEDDS_URI=<install>/g1_bringup/share/g1_bringup/config/cyclonedds_local.xml

PLUS: localisation should already be holding a real lock (position sigma in
the few-cm range, not the "bootstrapped from the database's last pose"
state described in section 11b) before sending a goal. Nav2 has no way to
know the difference between a confident pose and an unconfirmed guess.

SENDING A GOAL

    ros2 run rviz2 rviz2 -d <this package>/rviz/localization.rviz
    -- use RViz's "Nav2 Goal" tool, or:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
        "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}}}}"

STOPPING IT SAFELY

    ros2 service call /g1_loco_bridge/disable std_srvs/srv/Trigger {}

closes the cmd_vel gate immediately without killing the stack -- the fastest
way to stop the robot mid-navigation if something looks wrong.
"""

import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share = get_package_share_directory('g1_bringup')
    navigation_share = get_package_share_directory('g1_navigation')
    perception_share = get_package_share_directory('g1_perception')

    scan_params = os.path.join(perception_share, 'config', 'scan.yaml')
    nav2_params = os.path.join(navigation_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.expanduser('~/g1_maps/walk5_nolandmark.db'),
        ),
        DeclareLaunchArgument('apriltag', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'start_enabled', default_value='true',
            description='open the /cmd_vel gate at startup. Different default '
                        'from every other launch file here on purpose: with '
                        'no operator driving by joystick, Nav2 needs the gate '
                        'open to do anything at all. Close it fast with '
                        '`ros2 service call /g1_loco_bridge/disable '
                        'std_srvs/srv/Trigger {}` if something looks wrong.'),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch',
                        'rtabmap_localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'database_path': LaunchConfiguration('database_path'),
            'apriltag': LaunchConfiguration('apriltag'),
            'rviz': LaunchConfiguration('rviz'),
            # Localisation brings its OWN loco_bridge up with the gate CLOSED
            # (use_cmd_vel default false) -- this launch file starts its own
            # instance below instead, with start_enabled wired to Nav2's
            # needs rather than the joystick-driving default.
            'use_cmd_vel': 'false',
        }.items(),
    )

    loco_bridge = Node(
        package='g1_locomotion', executable='loco_bridge',
        name='g1_loco_bridge', output='screen',
        parameters=[{'server_address': '127.0.0.1', 'port': 5558,
                     'start_enabled': LaunchConfiguration('start_enabled'),
                     'use_sim_time': use_sim_time}],
    )

    # RTAB-Map subscribes to depth directly and does not publish a cloud or
    # scan on its own. Nav2's costmaps need SOMETHING; reuse the pipeline and
    # config already validated for AMCL rather than inventing a second one.
    scan_group = [
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
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan', output='screen',
            remappings=[('cloud_in', '/camera/cloud'), ('scan', '/scan')],
            parameters=[scan_params, {'use_sim_time': use_sim_time}],
        ),
    ]

    # HAND-ROLLED, not nav2_bringup's navigation_launch.py. That file's
    # lifecycle_nodes list is a hardcoded Python literal -- not a launch
    # argument -- and unconditionally includes docking_server, which refuses
    # to activate without at least one real charging-dock plugin ("Charging
    # dock plugins not given!"). This robot has no docking hardware, and
    # there is no valid empty/no-op config for that requirement -- so
    # including that file meant one unusable, unconfigurable node aborting
    # the ENTIRE stack's bringup every time. Reproduces the same node set
    # (package/executable/remappings) as that file, minus docking_server,
    # smoother_server and route_server -- the latter two are harmless but
    # genuinely unconfigured here (no section for either in nav2_params.yaml)
    # and this project would rather not run a node on faith that its defaults
    # are fine.
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key='',
        param_rewrites={'autostart': LaunchConfiguration('autostart')},
        convert_types=True,
    )
    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nav2_lifecycle_nodes = [
        'controller_server', 'planner_server', 'behavior_server',
        'velocity_smoother', 'collision_monitor', 'bt_navigator',
        'waypoint_follower',
    ]
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
            output='screen', parameters=[configured_params],
            remappings=tf_remap + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[configured_params], remappings=tf_remap),
        Node(package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen',
            parameters=[configured_params],
            remappings=tf_remap + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[configured_params], remappings=tf_remap),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[configured_params], remappings=tf_remap),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[configured_params],
            remappings=tf_remap + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', output='screen',
            parameters=[configured_params], remappings=tf_remap),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'autostart': LaunchConfiguration('autostart'),
                        'node_names': nav2_lifecycle_nodes}]),
    ]

    return LaunchDescription(
        arguments + [localization, loco_bridge] + scan_group + nav2_nodes)
