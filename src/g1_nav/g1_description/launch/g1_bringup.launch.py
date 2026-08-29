# g1_bringup.launch.py
# ----------------------------------------------------------------------------
# ONE launch that starts the FULL live stack INSIDE the humble_loc container:
#   1. Livox MID360 driver           -> /livox/lidar + /livox/imu
#   2. FAST-LIO localization          -> localization_3d_g1 (fast_lio + open3d_loc + RViz)
#   3. G1 robot model with LIVE joints -> robot_on_map.launch.py joints:=live
#
# The pieces that live on the HOST (the network IP and the g1_lowstate_reader
# that feeds joint angles over UDP) are started by start_g1.sh, which runs THIS
# launch via `docker exec`. Together that's the single-command bringup.
#
# Start is staggered so FAST-LIO has LiDAR data before it initialises, and the
# robot model attaches after localization's TF (map->...->base_link) exists.
#
# Run (normally via start_g1.sh; or by hand inside the container):
#   export ROS_DOMAIN_ID=42
#   source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
#   ros2 launch g1_description g1_bringup.launch.py
# ----------------------------------------------------------------------------
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    livox_launch = os.path.join(
        get_package_share_directory("livox_ros_driver2"),
        "launch_ROS2", "msg_MID360_launch.py")
    loc_launch = os.path.join(
        get_package_share_directory("open3d_loc"),
        "launch", "localization_3d_g1.launch.py")
    robot_launch = os.path.join(
        get_package_share_directory("g1_description"),
        "launch", "robot_on_map.launch.py")

    livox = IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch))
    localization = IncludeLaunchDescription(PythonLaunchDescriptionSource(loc_launch))
    # NOTE: pass rviz:=false EXPLICITLY. ROS2 launch does not scope launch args
    # between includes, and FAST-LIO's mapping.launch.py (pulled in by
    # `localization`) declares `rviz` with default 'true'. That value leaks into
    # the shared context, so robot_on_map's own `rviz` (default 'false') would be
    # overridden to 'true' and pop a SECOND, redundant RViz (g1_29dof.rviz).
    # Forcing it here keeps the ONE localization window (which already shows the
    # robot via its RobotModel display).
    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={"joints": "live", "rviz": "false"}.items())

    return LaunchDescription([
        # 1) LiDAR driver immediately
        livox,
        # 2) localization (fast_lio + open3d + RViz) after the LiDAR is publishing
        TimerAction(period=3.0, actions=[localization]),
        # 3) robot model + live joints once localization's TF exists
        TimerAction(period=5.0, actions=[robot_model]),
    ])
