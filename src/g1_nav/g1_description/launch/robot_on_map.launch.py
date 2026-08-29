# robot_on_map.launch.py
# ----------------------------------------------------------------------------
# Run this ALONGSIDE the FAST-LIO localization (or mapping). It adds the full
# G1 body to the live TF tree so the robot shows up at its localized pose and
# the LiDAR cloud fits the robot's head (mid360_link).
#
# It publishes:
#   1. robot_state_publisher   -> pelvis -> all body links (from the URDF)
#   2. a JOINT SOURCE (chosen by the `joints` arg, see below)
#   3. static TF base_link -> pelvis  -> hangs the robot under the LiDAR point
#
# It does NOT publish map/odom/base_link or open RViz by default -- localization
# already does that. To SEE the robot, add a "RobotModel" display (topic
# /robot_description) in your existing localization RViz, or pass rviz:=true.
#
# joints:=  (where the joint angles come from)
#   zeros  (default) -> joint_state_publisher: all joints 0 = nominal standing.
#                       The whole body slides to the localized pose but limbs
#                       don't move.
#   gui              -> joint_state_publisher_gui sliders, to pose limbs by hand.
#   live             -> g1_joint_state_bridge.py: REAL joint angles from the
#                       robot, so the legs/arms animate as it walks. Requires the
#                       host-side reader running:
#                         ~/unitree_localization/g1_lowstate_reader/run_lowstate_reader.sh --network eno1
#
# Frame tree (combined with localization):
#   map -> odom -> ... -> base_link(LiDAR) -> pelvis -> torso -> mid360_link (== base_link)
#                                                    -> legs/arms/head
#
# Usage (inside the Humble container, after sourcing /ws/install/setup.bash):
#   ros2 launch g1_description robot_on_map.launch.py                 # standing
#   ros2 launch g1_description robot_on_map.launch.py joints:=live    # live walk
#   ros2 launch g1_description robot_on_map.launch.py joints:=gui     # sliders
#   ros2 launch g1_description robot_on_map.launch.py rviz:=true      # also open RViz
# ----------------------------------------------------------------------------
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("g1_description")
    urdf_path = os.path.join(pkg, "urdf", "g1_29dof.urdf")
    rviz_path = os.path.join(pkg, "rviz", "g1_29dof.rviz")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    joints = LaunchConfiguration("joints")
    rviz = LaunchConfiguration("rviz")

    # one IfCondition per joint-source mode (mutually exclusive)
    is_zeros = IfCondition(PythonExpression(["'", joints, "' == 'zeros'"]))
    is_gui = IfCondition(PythonExpression(["'", joints, "' == 'gui'"]))
    is_live = IfCondition(PythonExpression(["'", joints, "' == 'live'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "joints", default_value="zeros",
            choices=["zeros", "gui", "live"],
            description="joint source: zeros=standing, gui=sliders, live=real robot"),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="true = also open RViz with the bundled config"),

        # 1) URDF -> TF (pelvis and below) + /robot_description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
        ),

        # 2a) joints = 0 (nominal standing) -- default
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=is_zeros,
        ),
        # 2b) sliders to pose the joints by hand
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=is_gui,
        ),
        # 2c) LIVE joint angles from the real robot (needs the host reader)
        Node(
            package="g1_description",
            executable="g1_joint_state_bridge.py",
            name="g1_joint_state_bridge",
            output="screen",
            condition=is_live,
        ),

        # 3) [REMOVED] the static base_link -> pelvis transform.
        #    It placed base_link at the head-mounted MID360 LiDAR, which was
        #    correct for the old FAST-LIO stack but is now wrong twice over:
        #    the URDF defines base_link itself (coincident with pelvis), so
        #    this would be a second publisher for the same edge, and it would
        #    put every sensor ~0.46 m too high.

        # optional RViz (normally you use the localization's RViz instead)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_path],
            condition=IfCondition(rviz),
        ),
    ])
